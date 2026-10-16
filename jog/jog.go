// Package jog jogs an arm's end effector from an input_controller's six absolute axes.
package jog

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

// Model is the registered Viam model for the jog service.
var Model = resource.NewModel("viam", "spacemouse", "jog")

const (
	defaultDeadband            = 0.05
	defaultTranslationMMPerSec = 100.0
	defaultRotationDegPerSec   = 45.0
	defaultTickHz              = 50.0
)

// Config wires an arm to an input_controller.
type Config struct {
	ArmName   string `json:"arm"`
	InputName string `json:"input_controller"`

	Deadband            *float64 `json:"deadband,omitempty"`
	TranslationMMPerSec *float64 `json:"translation_mm_per_sec,omitempty"`
	RotationDegPerSec   *float64 `json:"rotation_deg_per_sec,omitempty"`
	TickHz              *float64 `json:"tick_hz,omitempty"`
}

// Validate requires both resource names and returns them as implicit deps.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.ArmName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if c.InputName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "input_controller")
	}
	return []string{
		arm.Named(c.ArmName).String(),
		input.Named(c.InputName).String(),
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

	logger logging.Logger
	armRes arm.Arm

	deadband     float64
	transPerSec  float64
	rotPerSec    float64
	tickInterval time.Duration

	velMu      sync.RWMutex
	velocities [6]float64

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

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	j := &jog{
		Named:        conf.ResourceName().AsNamed(),
		logger:       logger,
		armRes:       armRes,
		deadband:     firstNonNil(cfg.Deadband, defaultDeadband),
		transPerSec:  firstNonNil(cfg.TranslationMMPerSec, defaultTranslationMMPerSec),
		rotPerSec:    firstNonNil(cfg.RotationDegPerSec, defaultRotationDegPerSec),
		tickInterval: tickInterval(firstNonNil(cfg.TickHz, defaultTickHz)),
		cancelCtx:    cancelCtx,
		cancelFunc:   cancelFunc,
	}

	if err := j.subscribe(ctx, inRes); err != nil {
		cancelFunc()
		return nil, fmt.Errorf("subscribe to input_controller: %w", err)
	}

	j.wg.Add(1)
	go j.drainLoop()

	logger.CInfow(ctx, "jog service started",
		"arm", cfg.ArmName,
		"input_controller", cfg.InputName,
		"tick_interval_ms", j.tickInterval.Milliseconds(),
		"translation_mm_per_sec", j.transPerSec,
		"rotation_deg_per_sec", j.rotPerSec,
		"deadband", j.deadband,
	)
	return j, nil
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

func (j *jog) Close(_ context.Context) error {
	j.cancelFunc()
	j.wg.Wait()
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
