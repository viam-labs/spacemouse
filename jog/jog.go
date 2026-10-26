// Package jog jogs an arm's end effector from an input_controller's six absolute axes.
package jog

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

const referenceFrame = "world"

// Model is the registered Viam model.
var Model = resource.NewModel("viam", "spacemouse", "jog")

const (
	defaultDeadband              = 0.05
	defaultTranslationMMPerSec   = 100.0
	defaultRotationDegPerSec     = 45.0
	defaultTickHz                = 50.0
	defaultMaxJointVelDegsPerSec = 30.0
	defaultMotionName            = "motion"

	posDeadbandMM  = 1.0
	rotDeadbandDeg = 1.0
	commandTimeout = 500 * time.Millisecond
	errorCooldown  = 1 * time.Second

	// Safety: cap how far the virtual goal may run ahead of the arm. If the arm
	// cannot keep up (speed cap configured above arm capability), goal is snapped
	// back toward actual so release-to-stop stays within these bounds.
	maxLagMM  = 50.0
	maxLagDeg = 20.0

	parkDrainTimeout = 500 * time.Millisecond
	parkDrainPoll    = 5 * time.Millisecond
)

// Config wires an arm to an input_controller.
type Config struct {
	ArmName    string `json:"arm"`
	InputName  string `json:"input_controller"`
	MotionName string `json:"motion,omitempty"`

	Deadband              *float64 `json:"deadband,omitempty"`
	TranslationMMPerSec   *float64 `json:"translation_mm_per_sec,omitempty"`
	RotationDegPerSec     *float64 `json:"rotation_deg_per_sec,omitempty"`
	TickHz                *float64 `json:"tick_hz,omitempty"`
	MaxJointVelDegsPerSec *float64 `json:"max_joint_vel_degs_per_sec,omitempty"`
}

// Validate returns the deps the service requires.
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
	arm       arm.Arm
	armName   string
	motionSvc motion.Service
	kinematic referenceframe.Model

	deadband        float64
	transPerSec     float64
	rotPerSec       float64
	tickInterval    time.Duration
	maxJointVelRads float64

	velMu      sync.RWMutex
	velocities [6]float64

	// movePending gates a single in-flight goal send; ticks drop otherwise.
	movePending atomic.Bool

	moveMu       sync.Mutex
	moving       bool
	lastSentPose spatialmath.Pose
	errorUntil   time.Time
	// virtualGoal advances at translation_mm_per_sec regardless of arm position.
	// Nil when idle; initialized from arm.EndPosition on the next active tick.
	virtualGoal spatialmath.Pose

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

	kinematic, err := armRes.Kinematics(ctx)
	if err != nil {
		return nil, fmt.Errorf("fetch arm kinematics: %w", err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	j := &jog{
		Named:           conf.ResourceName().AsNamed(),
		logger:          logger,
		arm:             armRes,
		armName:         cfg.ArmName,
		motionSvc:       motionSvc,
		kinematic:       kinematic,
		deadband:        firstNonNil(cfg.Deadband, defaultDeadband),
		transPerSec:     firstNonNil(cfg.TranslationMMPerSec, defaultTranslationMMPerSec),
		rotPerSec:       firstNonNil(cfg.RotationDegPerSec, defaultRotationDegPerSec),
		tickInterval:    tickInterval(firstNonNil(cfg.TickHz, defaultTickHz)),
		maxJointVelRads: utils.DegToRad(firstNonNil(cfg.MaxJointVelDegsPerSec, defaultMaxJointVelDegsPerSec)),
		cancelCtx:       cancelCtx,
		cancelFunc:      cancelFunc,
	}

	if err := j.subscribe(ctx, inRes); err != nil {
		cancelFunc()
		return nil, fmt.Errorf("subscribe to input_controller: %w", err)
	}

	if err := j.openPipeline(ctx); err != nil {
		cancelFunc()
		return nil, fmt.Errorf("open pipeline: %w", err)
	}

	j.wg.Add(1)
	go j.drainLoop()

	logger.CInfow(ctx, "jog service started",
		"arm", cfg.ArmName,
		"input_controller", cfg.InputName,
		"motion", motionName,
		"tick_interval_ms", j.tickInterval.Milliseconds(),
		"translation_mm_per_sec", j.transPerSec,
		"rotation_deg_per_sec", j.rotPerSec,
	)
	return j, nil
}

func poseInFrameJSON(pose spatialmath.Pose) string {
	pt := pose.Point()
	ov := pose.Orientation().OrientationVectorDegrees()
	return fmt.Sprintf(
		`{"reference_frame":"%s","pose":{"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}}`,
		referenceFrame, pt.X, pt.Y, pt.Z, ov.OX, ov.OY, ov.OZ, ov.Theta,
	)
}

func (j *jog) openPipeline(ctx context.Context) error {
	currentPose, err := j.arm.EndPosition(ctx, nil)
	if err != nil {
		return fmt.Errorf("read current pose: %w", err)
	}
	startReq := fmt.Sprintf(
		`{"name":"builtin","component_name":"%s","destination":%s}`,
		j.armName, poseInFrameJSON(currentPose),
	)
	cmdCtx, cancel := context.WithTimeout(ctx, commandTimeout)
	defer cancel()
	_, err = j.motionSvc.DoCommand(cmdCtx, map[string]interface{}{
		"teleop_start": startReq,
	})
	return err
}

func (j *jog) sendGoal(ctx context.Context, pose spatialmath.Pose) error {
	cmdCtx, cancel := context.WithTimeout(ctx, commandTimeout)
	defer cancel()
	_, err := j.motionSvc.DoCommand(cmdCtx, map[string]interface{}{
		"teleop_move":    poseInFrameJSON(pose),
		"component_name": j.armName,
	})
	return err
}

func (j *jog) closePipeline(ctx context.Context) {
	cmdCtx, cancel := context.WithTimeout(ctx, commandTimeout)
	defer cancel()
	if _, err := j.motionSvc.DoCommand(cmdCtx, map[string]interface{}{"teleop_stop": ""}); err != nil {
		j.logger.CWarnw(cmdCtx, "close pipeline failed", "err", err)
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
	j.closePipeline(ctx)
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
