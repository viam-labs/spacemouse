// Package jog jogs an arm's end effector from an input_controller's six absolute axes
// via the motion-service teleop pipeline (teleop_start / teleop_move / teleop_stop).
package jog

import (
	"context"
	"fmt"
	"math"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

const referenceFrame = "world"

// Model is the registered Viam model for the jog service.
var Model = resource.NewModel("viam", "spacemouse", "jog")

const (
	defaultDeadband            = 0.05
	defaultTranslationMMPerSec = 100.0
	defaultRotationDegPerSec   = 45.0
	defaultTickHz              = 50.0
	defaultLookaheadMs         = 300
	defaultMotionName          = "motion"
	defaultPosDeadbandMM       = 1.0
	defaultRotDeadbandDeg      = 1.0
	defaultErrorCooldownMs     = 1000

	teleopCmdTimeout = 500 * time.Millisecond
)

// Config wires an arm to an input_controller via the motion-service teleop pipeline.
type Config struct {
	ArmName    string `json:"arm"`
	InputName  string `json:"input_controller"`
	MotionName string `json:"motion,omitempty"`

	Deadband            *float64 `json:"deadband,omitempty"`
	TranslationMMPerSec *float64 `json:"translation_mm_per_sec,omitempty"`
	RotationDegPerSec   *float64 `json:"rotation_deg_per_sec,omitempty"`
	TickHz              *float64 `json:"tick_hz,omitempty"`

	// LookaheadMs is how far into the future each goal pose is placed.
	LookaheadMs *float64 `json:"lookahead_ms,omitempty"`

	// Pose-change thresholds below which we skip the teleop_move send.
	// Prevents planner thrash from sub-mm/sub-degree input jitter.
	PosDeadbandMM  *float64 `json:"pos_deadband_mm,omitempty"`
	RotDeadbandDeg *float64 `json:"rot_deadband_deg,omitempty"`

	// ErrorCooldownMs is how long to skip teleop_move sends after a failure.
	ErrorCooldownMs *float64 `json:"error_cooldown_ms,omitempty"`
}

// Validate requires arm + input_controller + motion service and returns them as implicit deps.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.ArmName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if c.InputName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "input_controller")
	}
	motionName := c.MotionName
	if motionName == "" {
		motionName = defaultMotionName
	}
	return []string{
		arm.Named(c.ArmName).String(),
		input.Named(c.InputName).String(),
		motion.Named(motionName).String(),
	}, nil, nil
}

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newJog,
		},
	)
}

type jog struct {
	resource.Named
	resource.AlwaysRebuild

	logger    logging.Logger
	armRes    arm.Arm
	armName   string
	motionSvc motion.Service

	deadband       float64
	transPerSec    float64
	rotPerSec      float64
	tickInterval   time.Duration
	lookaheadSec   float64
	posDeadbandMM  float64
	rotDeadbandDeg float64
	errorCooldown  time.Duration

	velMu      sync.RWMutex
	velocities [6]float64

	// movePending gates a single in-flight teleop_move; ticks drop otherwise.
	movePending atomic.Bool

	moveMu       sync.Mutex
	moving       bool
	lastSentPose spatialmath.Pose
	errorUntil   time.Time

	cancelCtx  context.Context
	cancelFunc context.CancelFunc
	wg         sync.WaitGroup
}

func newJog(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	armRes, err := arm.FromProvider(deps, cfg.ArmName)
	if err != nil {
		return nil, fmt.Errorf("arm %q not found: %w", cfg.ArmName, err)
	}
	inRes, err := input.FromProvider(deps, cfg.InputName)
	if err != nil {
		return nil, fmt.Errorf("input_controller %q not found: %w", cfg.InputName, err)
	}
	motionName := cfg.MotionName
	if motionName == "" {
		motionName = defaultMotionName
	}
	motionSvc, err := motion.FromProvider(deps, motionName)
	if err != nil {
		return nil, fmt.Errorf("motion service %q not found: %w", motionName, err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	j := &jog{
		Named:          conf.ResourceName().AsNamed(),
		logger:         logger,
		armRes:         armRes,
		armName:        cfg.ArmName,
		motionSvc:      motionSvc,
		deadband:       firstNonNil(cfg.Deadband, defaultDeadband),
		transPerSec:    firstNonNil(cfg.TranslationMMPerSec, defaultTranslationMMPerSec),
		rotPerSec:      firstNonNil(cfg.RotationDegPerSec, defaultRotationDegPerSec),
		tickInterval:   tickInterval(firstNonNil(cfg.TickHz, defaultTickHz)),
		lookaheadSec:   firstNonNil(cfg.LookaheadMs, defaultLookaheadMs) / 1000,
		posDeadbandMM:  firstNonNil(cfg.PosDeadbandMM, defaultPosDeadbandMM),
		rotDeadbandDeg: firstNonNil(cfg.RotDeadbandDeg, defaultRotDeadbandDeg),
		errorCooldown:  time.Duration(firstNonNil(cfg.ErrorCooldownMs, defaultErrorCooldownMs)) * time.Millisecond,
		cancelCtx:      cancelCtx,
		cancelFunc:     cancelFunc,
	}

	if err := j.subscribe(ctx, inRes); err != nil {
		cancelFunc()
		return nil, fmt.Errorf("subscribe to input_controller: %w", err)
	}

	if err := j.teleopStart(ctx); err != nil {
		cancelFunc()
		return nil, fmt.Errorf("teleop_start failed: %w", err)
	}

	j.wg.Add(1)
	go j.drainLoop()

	logger.CInfow(ctx, "jog service started",
		"arm", cfg.ArmName,
		"input_controller", cfg.InputName,
		"motion", motionName,
		"tick_interval_ms", j.tickInterval.Milliseconds(),
		"lookahead_ms", j.lookaheadSec*1000,
		"pos_deadband_mm", j.posDeadbandMM,
		"rot_deadband_deg", j.rotDeadbandDeg,
	)
	return j, nil
}

// teleopStart kicks off the motion-service teleop pipeline with the arm
// parked at its current pose. The JSON string unmarshals into pb.MoveRequest.
func (j *jog) teleopStart(ctx context.Context) error {
	currentPose, err := j.armRes.EndPosition(ctx, nil)
	if err != nil {
		return fmt.Errorf("read current pose: %w", err)
	}
	pt := currentPose.Point()
	ov := currentPose.Orientation().OrientationVectorDegrees()

	startReq := fmt.Sprintf(
		`{"name":"builtin","component_name":"%s","destination":{"reference_frame":"%s","pose":{"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}}}`,
		j.armName, referenceFrame, pt.X, pt.Y, pt.Z, ov.OX, ov.OY, ov.OZ, ov.Theta,
	)

	cmdCtx, cancel := context.WithTimeout(ctx, teleopCmdTimeout)
	defer cancel()
	_, err = j.motionSvc.DoCommand(cmdCtx, map[string]interface{}{
		"teleop_start": startReq,
	})
	return err
}

func (j *jog) teleopMove(ctx context.Context, pose spatialmath.Pose) error {
	pt := pose.Point()
	ov := pose.Orientation().OrientationVectorDegrees()
	moveReq := fmt.Sprintf(
		`{"reference_frame":"%s","pose":{"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}}`,
		referenceFrame, pt.X, pt.Y, pt.Z, ov.OX, ov.OY, ov.OZ, ov.Theta,
	)

	err := j.doTeleopMove(ctx, moveReq)
	// If the pipeline died (motion service reconfigured, teleop_stop called
	// elsewhere, etc.), lazily restart it and retry once.
	if err != nil && strings.Contains(err.Error(), "not running") {
		if restartErr := j.teleopStart(ctx); restartErr != nil {
			return fmt.Errorf("teleop pipeline restart failed: %w (original: %v)", restartErr, err)
		}
		err = j.doTeleopMove(ctx, moveReq)
	}
	return err
}

func (j *jog) doTeleopMove(ctx context.Context, moveReq string) error {
	cmdCtx, cancel := context.WithTimeout(ctx, teleopCmdTimeout)
	defer cancel()
	_, err := j.motionSvc.DoCommand(cmdCtx, map[string]interface{}{
		"teleop_move":    moveReq,
		"component_name": j.armName,
	})
	return err
}

func (j *jog) teleopStop(ctx context.Context) {
	cmdCtx, cancel := context.WithTimeout(ctx, teleopCmdTimeout)
	defer cancel()
	if _, err := j.motionSvc.DoCommand(cmdCtx, map[string]interface{}{"teleop_stop": ""}); err != nil {
		j.logger.CWarnw(cmdCtx, "teleop_stop failed", "err", err)
	}
}

// DoCommand exposes current velocities so operators can sanity-check the input path.
func (j *jog) DoCommand(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
	j.velMu.RLock()
	defer j.velMu.RUnlock()
	return map[string]interface{}{
		"velocities": map[string]float64{
			"x":  j.velocities[0],
			"y":  j.velocities[1],
			"z":  j.velocities[2],
			"rx": j.velocities[3],
			"ry": j.velocities[4],
			"rz": j.velocities[5],
		},
	}, nil
}

func (j *jog) Close(ctx context.Context) error {
	j.cancelFunc()
	j.wg.Wait()
	j.teleopStop(ctx)
	return nil
}

func firstNonNil(p *float64, def float64) float64 {
	if p == nil {
		return def
	}
	return *p
}

func tickInterval(hz float64) time.Duration {
	if hz <= 0 {
		hz = defaultTickHz
	}
	d := time.Duration(math.Round(float64(time.Second) / hz))
	if d < time.Millisecond {
		d = time.Millisecond
	}
	return d
}
