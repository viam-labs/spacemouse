package jog

import (
	"math"

	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

type poseDeltaStep struct {
	X, Y, Z    float64
	RX, RY, RZ float64
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

// poseDiffExceeds returns true when either the translational distance exceeds
// posMM or the rotation angle between orientations exceeds rotDeg.
func poseDiffExceeds(a, b spatialmath.Pose, posMM, rotDeg float64) bool {
	pa := a.Point()
	pb := b.Point()
	dx := pa.X - pb.X
	dy := pa.Y - pb.Y
	dz := pa.Z - pb.Z
	if math.Sqrt(dx*dx+dy*dy+dz*dz) > posMM {
		return true
	}
	qa := a.Orientation().Quaternion()
	qb := b.Orientation().Quaternion()
	dot := qa.Real*qb.Real + qa.Imag*qb.Imag + qa.Jmag*qb.Jmag + qa.Kmag*qb.Kmag
	if dot < 0 {
		dot = -dot
	}
	if dot > 1 {
		dot = 1
	}
	angleRad := 2 * math.Acos(dot)
	return angleRad*180/math.Pi > rotDeg
}

type errUnknownAxis string

func (e errUnknownAxis) Error() string { return "unknown rotation axis " + string(e) }
