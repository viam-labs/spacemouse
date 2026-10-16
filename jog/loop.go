package jog

import (
	"context"
	"math"
	"time"

	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

var axisControls = [6]input.Control{
	input.AbsoluteX,
	input.AbsoluteY,
	input.AbsoluteZ,
	input.AbsoluteRX,
	input.AbsoluteRY,
	input.AbsoluteRZ,
}

func (j *jog) subscribe(ctx context.Context, ctrl input.Controller) error {
	for i, control := range axisControls {
		idx := i
		cb := func(_ context.Context, ev input.Event) {
			j.velMu.Lock()
			j.velocities[idx] = ev.Value
			j.velMu.Unlock()
		}
		err := ctrl.RegisterControlCallback(ctx, control,
			[]input.EventType{input.PositionChangeAbs}, cb, nil)
		if err != nil {
			return err
		}
	}

	// Zero velocities on disconnect so the arm stops if the controller drops.
	disc := func(_ context.Context, _ input.Event) {
		j.velMu.Lock()
		j.velocities = [6]float64{}
		j.velMu.Unlock()
	}
	for _, control := range axisControls {
		if err := ctrl.RegisterControlCallback(ctx, control,
			[]input.EventType{input.Disconnect}, disc, nil); err != nil {
			return err
		}
	}
	return nil
}

func (j *jog) drainLoop() {
	defer j.wg.Done()
	ticker := time.NewTicker(j.tickInterval)
	defer ticker.Stop()

	dt := j.tickInterval.Seconds()
	for {
		select {
		case <-j.cancelCtx.Done():
			return
		case <-ticker.C:
			j.tick(dt)
		}
	}
}

func (j *jog) tick(dt float64) {
	vel := j.snapshot()
	if !j.anyAboveDeadband(vel) {
		return
	}

	delta := j.poseDelta(vel, dt)
	if delta.allZero() {
		return
	}

	currentPose, err := j.armRes.EndPosition(j.cancelCtx, nil)
	if err != nil {
		j.logger.CWarnw(j.cancelCtx, "arm EndPosition failed", "err", err)
		return
	}

	newPose, err := applyDelta(currentPose, delta)
	if err != nil {
		j.logger.CWarnw(j.cancelCtx, "pose composition failed", "err", err)
		return
	}

	if err := j.armRes.MoveToPosition(j.cancelCtx, newPose, nil); err != nil {
		j.logger.CWarnw(j.cancelCtx, "arm MoveToPosition failed", "err", err)
	}
}

func (j *jog) snapshot() [6]float64 {
	j.velMu.RLock()
	defer j.velMu.RUnlock()
	return j.velocities
}

func (j *jog) anyAboveDeadband(vel [6]float64) bool {
	for _, v := range vel {
		if math.Abs(v) > j.deadband {
			return true
		}
	}
	return false
}

type poseDeltaStep struct {
	X, Y, Z    float64
	RX, RY, RZ float64
}

func (p poseDeltaStep) allZero() bool {
	return p.X == 0 && p.Y == 0 && p.Z == 0 &&
		p.RX == 0 && p.RY == 0 && p.RZ == 0
}

func (j *jog) poseDelta(vel [6]float64, dt float64) poseDeltaStep {
	trans := func(v float64) float64 {
		if math.Abs(v) <= j.deadband {
			return 0
		}
		return v * j.transPerSec * dt
	}
	rot := func(v float64) float64 {
		if math.Abs(v) <= j.deadband {
			return 0
		}
		return v * j.rotPerSec * dt
	}
	return poseDeltaStep{
		X: trans(vel[0]), Y: trans(vel[1]), Z: trans(vel[2]),
		RX: rot(vel[3]), RY: rot(vel[4]), RZ: rot(vel[5]),
	}
}

// Fixed RX→RY→RZ composition order keeps results deterministic across ticks.
func applyDelta(current spatialmath.Pose, d poseDeltaStep) (spatialmath.Pose, error) {
	pt := current.Point()
	pt.X += d.X
	pt.Y += d.Y
	pt.Z += d.Z

	pose := spatialmath.NewPose(pt, current.Orientation())
	for _, step := range [...]struct {
		axis string
		deg  float64
	}{
		{"rx", d.RX}, {"ry", d.RY}, {"rz", d.RZ},
	} {
		if step.deg == 0 {
			continue
		}
		var err error
		pose, err = rotatePose(pose, step.axis, step.deg)
		if err != nil {
			return nil, err
		}
	}
	return pose, nil
}

// Ported from viam:beanjamin:dial-control-motion.
func rotatePose(pose spatialmath.Pose, axis string, deg float64) (spatialmath.Pose, error) {
	theta := deg * math.Pi / 180.0
	half := theta / 2.0
	cosHalf, sinHalf := math.Cos(half), math.Sin(half)

	var rotQ quat.Number
	switch axis {
	case "rx":
		rotQ = quat.Number{Real: cosHalf, Imag: sinHalf}
	case "ry":
		rotQ = quat.Number{Real: cosHalf, Jmag: sinHalf}
	case "rz":
		rotQ = quat.Number{Real: cosHalf, Kmag: sinHalf}
	default:
		return nil, errUnknownAxis(axis)
	}

	currentQ := pose.Orientation().Quaternion()
	newQ := quat.Mul(quat.Number{
		Real: currentQ.Real,
		Imag: currentQ.Imag,
		Jmag: currentQ.Jmag,
		Kmag: currentQ.Kmag,
	}, rotQ)

	if mag := quat.Abs(newQ); mag > 1e-10 {
		newQ = quat.Scale(1.0/mag, newQ)
	}

	w := math.Min(1.0, math.Max(-1.0, newQ.Real))
	theta2 := 2.0 * math.Acos(w)
	sinT := math.Sin(theta2 / 2.0)

	var ori spatialmath.Orientation
	if sinT < 1e-10 {
		ori = &spatialmath.R4AA{Theta: 0, RX: 1}
	} else {
		ori = &spatialmath.R4AA{
			Theta: theta2,
			RX:    newQ.Imag / sinT,
			RY:    newQ.Jmag / sinT,
			RZ:    newQ.Kmag / sinT,
		}
	}
	return spatialmath.NewPose(pose.Point(), ori), nil
}

type errUnknownAxis string

func (e errUnknownAxis) Error() string { return "unknown rotation axis " + string(e) }
