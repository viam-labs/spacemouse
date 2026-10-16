package jog

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

func fiftyHz(t *testing.T, deadband float64) *jog {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	return &jog{
		logger:       logging.NewTestLogger(t),
		deadband:     deadband,
		transPerSec:  100.0,
		rotPerSec:    45.0,
		tickInterval: 20 * time.Millisecond,
		cancelCtx:    ctx,
		cancelFunc:   cancel,
	}
}

func TestPoseDelta_BelowDeadbandIsZero(t *testing.T) {
	j := fiftyHz(t, 0.1)
	d := j.poseDelta([6]float64{0.05, -0.09, 0.0, 0.1, -0.05, 0.08}, 0.02)
	test.That(t, d.allZero(), test.ShouldBeTrue)
}

func TestPoseDelta_ScalesByRateAndDt(t *testing.T) {
	j := fiftyHz(t, 0.0)
	dt := 0.02

	d := j.poseDelta([6]float64{1.0, 0, 0, 0, 0, 0}, dt)
	test.That(t, d.X, test.ShouldAlmostEqual, 100.0*dt) // 2mm per tick at full stick
	test.That(t, d.Y, test.ShouldEqual, 0.0)

	d = j.poseDelta([6]float64{0, 0, 0, 0.5, 0, 0}, dt)
	test.That(t, d.RX, test.ShouldAlmostEqual, 45.0*0.5*dt) // 0.45deg per tick at half
}

func TestAnyAboveDeadband(t *testing.T) {
	j := fiftyHz(t, 0.1)
	test.That(t, j.anyAboveDeadband([6]float64{}), test.ShouldBeFalse)
	test.That(t, j.anyAboveDeadband([6]float64{0.05, 0.05, 0.05, 0.05, 0.05, 0.05}), test.ShouldBeFalse)
	test.That(t, j.anyAboveDeadband([6]float64{0, 0, 0, 0, 0, 0.2}), test.ShouldBeTrue)
	test.That(t, j.anyAboveDeadband([6]float64{-0.2, 0, 0, 0, 0, 0}), test.ShouldBeTrue)
}

func TestApplyDelta_TranslationOnly(t *testing.T) {
	start := spatialmath.NewPose(
		r3.Vector{X: 100, Y: 200, Z: 300},
		&spatialmath.R4AA{Theta: 0, RX: 1},
	)
	out, err := applyDelta(start, poseDeltaStep{X: 5, Y: -3, Z: 2})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out.Point().X, test.ShouldAlmostEqual, 105.0)
	test.That(t, out.Point().Y, test.ShouldAlmostEqual, 197.0)
	test.That(t, out.Point().Z, test.ShouldAlmostEqual, 302.0)
	// orientation unchanged
	oriBefore := start.Orientation().AxisAngles()
	oriAfter := out.Orientation().AxisAngles()
	test.That(t, oriAfter.Theta, test.ShouldAlmostEqual, oriBefore.Theta)
}

func TestApplyDelta_RotationChangesOrientation(t *testing.T) {
	start := spatialmath.NewPose(
		r3.Vector{X: 100, Y: 200, Z: 300},
		&spatialmath.R4AA{Theta: 0, RX: 1},
	)
	out, err := applyDelta(start, poseDeltaStep{RZ: 10})
	test.That(t, err, test.ShouldBeNil)
	// position unchanged
	test.That(t, out.Point().X, test.ShouldAlmostEqual, 100.0)
	// orientation theta should now be ~10 degrees in radians
	ori := out.Orientation().AxisAngles()
	test.That(t, ori.Theta, test.ShouldAlmostEqual, 10.0*3.141592653589793/180.0, 1e-9)
}

func TestTick_BelowDeadbandSkipsArmCalls(t *testing.T) {
	j := fiftyHz(t, 0.1)
	a := &inject.Arm{}
	var endPosCalls, moveCalls int
	a.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		endPosCalls++
		return nil, nil
	}
	a.MoveToPositionFunc = func(context.Context, spatialmath.Pose, map[string]interface{}) error {
		moveCalls++
		return nil
	}
	j.armRes = a

	j.velocities = [6]float64{0.05, 0.05, 0.05, 0.05, 0.05, 0.05}
	j.tick(0.02)

	test.That(t, endPosCalls, test.ShouldEqual, 0)
	test.That(t, moveCalls, test.ShouldEqual, 0)
}

func TestTick_AboveDeadbandMovesArm(t *testing.T) {
	j := fiftyHz(t, 0.05)
	a := &inject.Arm{}
	start := spatialmath.NewPose(r3.Vector{X: 100, Y: 200, Z: 300}, &spatialmath.R4AA{Theta: 0, RX: 1})
	var moved spatialmath.Pose
	a.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return start, nil
	}
	a.MoveToPositionFunc = func(_ context.Context, to spatialmath.Pose, _ map[string]interface{}) error {
		moved = to
		return nil
	}
	j.armRes = a

	j.velocities = [6]float64{1.0, 0, 0, 0, 0, 0}
	j.tick(0.02)

	test.That(t, moved, test.ShouldNotBeNil)
	test.That(t, moved.Point().X, test.ShouldAlmostEqual, 102.0) // +100mm/s * 0.02
}

func TestTick_EndPositionErrorSkipsMove(t *testing.T) {
	j := fiftyHz(t, 0.05)
	a := &inject.Arm{}
	var moveCalls int
	a.EndPositionFunc = func(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
		return nil, errors.New("boom")
	}
	a.MoveToPositionFunc = func(context.Context, spatialmath.Pose, map[string]interface{}) error {
		moveCalls++
		return nil
	}
	j.armRes = a

	j.velocities = [6]float64{1.0, 0, 0, 0, 0, 0}
	j.tick(0.02)

	test.That(t, moveCalls, test.ShouldEqual, 0)
}

func TestSubscribe_CallbackUpdatesVelocity(t *testing.T) {
	j := fiftyHz(t, 0.0)
	fake := &fakeInput{callbacks: map[axisKey]input.ControlFunction{}}
	err := j.subscribe(context.Background(), fake)
	test.That(t, err, test.ShouldBeNil)

	fake.fire(input.AbsoluteX, input.PositionChangeAbs, 0.42)
	fake.fire(input.AbsoluteRZ, input.PositionChangeAbs, -0.7)

	v := j.snapshot()
	test.That(t, v[0], test.ShouldAlmostEqual, 0.42)
	test.That(t, v[5], test.ShouldAlmostEqual, -0.7)
}

func TestSubscribe_DisconnectZeroes(t *testing.T) {
	j := fiftyHz(t, 0.0)
	fake := &fakeInput{callbacks: map[axisKey]input.ControlFunction{}}
	err := j.subscribe(context.Background(), fake)
	test.That(t, err, test.ShouldBeNil)

	fake.fire(input.AbsoluteX, input.PositionChangeAbs, 0.9)
	test.That(t, j.snapshot()[0], test.ShouldAlmostEqual, 0.9)

	fake.fire(input.AbsoluteX, input.Disconnect, 0.0)
	test.That(t, j.snapshot(), test.ShouldEqual, [6]float64{})
}

func TestTickInterval(t *testing.T) {
	test.That(t, tickInterval(50), test.ShouldEqual, 20*time.Millisecond)
	test.That(t, tickInterval(0), test.ShouldEqual, 20*time.Millisecond)     // defaults
	test.That(t, tickInterval(10_000), test.ShouldEqual, 1*time.Millisecond) // floor at 1ms
	test.That(t, tickInterval(100), test.ShouldEqual, 10*time.Millisecond)
}

func TestFirstNonNil(t *testing.T) {
	test.That(t, firstNonNil(nil, 42.0), test.ShouldEqual, 42.0)
	v := 7.0
	test.That(t, firstNonNil(&v, 42.0), test.ShouldEqual, 7.0)
}

func TestConfigValidate(t *testing.T) {
	_, _, err := (&Config{}).Validate("/foo")
	test.That(t, err, test.ShouldNotBeNil)

	_, _, err = (&Config{ArmName: "a"}).Validate("/foo")
	test.That(t, err, test.ShouldNotBeNil)

	deps, _, err := (&Config{ArmName: "my-arm", InputName: "my-sm"}).Validate("/foo")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{
		arm.Named("my-arm").String(),
		input.Named("my-sm").String(),
	})
}

// fakeInput is a minimal input.Controller stand-in that lets tests fire events.
type axisKey struct {
	control input.Control
	event   input.EventType
}

type fakeInput struct {
	input.Controller
	mu        sync.Mutex
	callbacks map[axisKey]input.ControlFunction
}

func (f *fakeInput) RegisterControlCallback(
	_ context.Context,
	control input.Control,
	triggers []input.EventType,
	cb input.ControlFunction,
	_ map[string]interface{},
) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	for _, trig := range triggers {
		f.callbacks[axisKey{control, trig}] = cb
	}
	return nil
}

func (f *fakeInput) fire(control input.Control, event input.EventType, value float64) {
	f.mu.Lock()
	cb := f.callbacks[axisKey{control, event}]
	f.mu.Unlock()
	if cb != nil {
		cb(context.Background(), input.Event{
			Control: control, Event: event, Value: value,
		})
	}
}

// pin unused imports so the file compiles cleanly without "declared but not used" noise.
var _ = input.AbsoluteX
