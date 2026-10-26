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

// Duplicated in viam:beanjamin:dial-control-motion — candidate for rdk/spatialmath upstream.
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

func poseDiffExceeds(a, b spatialmath.Pose, posMM, rotDeg float64) bool {
	if a.Point().Sub(b.Point()).Norm() > posMM {
		return true
	}
	angleDeg := math.Abs(spatialmath.OrientationBetween(a.Orientation(), b.Orientation()).AxisAngles().Theta) * 180 / math.Pi
	return angleDeg > rotDeg
}

type errUnknownAxis string

func (e errUnknownAxis) Error() string { return "unknown rotation axis " + string(e) }
