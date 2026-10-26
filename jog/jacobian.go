package jog

import (
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
)

const (
	// jacobianPerturbation is the joint-angle step (rad) used for the numerical
	// Jacobian. Small enough for linear approximation, large enough to avoid
	// float noise.
	jacobianPerturbation = 1e-5

	// dlsLambda damps the pseudo-inverse near singularities. Higher = smoother
	// behavior near singularity (less twitchy scaling), at the cost of some
	// accuracy in the joint-velocity prediction. 0.1 is a well-tested middle
	// ground for 6-DoF arms.
	dlsLambda = 0.1
)

// jacobianAt computes a 6×DoF numerical Jacobian at `inputs`. Rows are the
// Cartesian velocity [dx, dy, dz, drx, dry, drz] in mm and rad per unit joint
// angle change.
func jacobianAt(model referenceframe.Model, inputs []referenceframe.Input) (*mat.Dense, error) {
	basePose, err := model.Transform(inputs)
	if err != nil {
		return nil, fmt.Errorf("jacobian baseline FK: %w", err)
	}

	dof := len(inputs)
	J := mat.NewDense(6, dof, nil)

	perturbed := make([]referenceframe.Input, dof)
	for i := 0; i < dof; i++ {
		copy(perturbed, inputs)
		perturbed[i] += jacobianPerturbation

		pose, err := model.Transform(perturbed)
		if err != nil {
			return nil, fmt.Errorf("jacobian FK on joint %d: %w", i, err)
		}

		transDelta := pose.Point().Sub(basePose.Point())
		rotDelta := axisAngleVec(spatialmath.OrientationBetween(basePose.Orientation(), pose.Orientation()))

		inv := 1.0 / jacobianPerturbation
		J.Set(0, i, transDelta.X*inv)
		J.Set(1, i, transDelta.Y*inv)
		J.Set(2, i, transDelta.Z*inv)
		J.Set(3, i, rotDelta.X*inv)
		J.Set(4, i, rotDelta.Y*inv)
		J.Set(5, i, rotDelta.Z*inv)
	}
	return J, nil
}

// axisAngleVec returns the axis-angle representation as a 3-vector (axis × θ).
func axisAngleVec(o spatialmath.Orientation) r3.Vector {
	aa := o.AxisAngles()
	return r3.Vector{X: aa.RX * aa.Theta, Y: aa.RY * aa.Theta, Z: aa.RZ * aa.Theta}
}

// identity6 returns a 6×6 identity matrix. Used by Jacobian unit tests.
func identity6() *mat.Dense {
	I := mat.NewDense(6, 6, nil)
	for i := 0; i < 6; i++ {
		I.Set(i, i, 1)
	}
	return I
}

// solveDLS solves J · dθ = ΔX for dθ using damped least-squares. Handles
// singularities gracefully: as J becomes ill-conditioned, dθ smoothly shrinks
// instead of exploding.
//
//	dθ = Jᵀ (J Jᵀ + λ²I)⁻¹ ΔX
func solveDLS(J *mat.Dense, tcpDelta []float64, lambda float64) ([]float64, error) {
	rows, cols := J.Dims()
	if rows != 6 {
		return nil, fmt.Errorf("solveDLS: expected 6-row Jacobian, got %d", rows)
	}
	if len(tcpDelta) != 6 {
		return nil, fmt.Errorf("solveDLS: expected 6-element TCP delta, got %d", len(tcpDelta))
	}

	JJT := mat.NewDense(6, 6, nil)
	JJT.Mul(J, J.T())

	lambda2 := lambda * lambda
	for i := 0; i < 6; i++ {
		JJT.Set(i, i, JJT.At(i, i)+lambda2)
	}

	deltaVec := mat.NewVecDense(6, tcpDelta)
	y := mat.NewVecDense(6, nil)
	if err := y.SolveVec(JJT, deltaVec); err != nil {
		return nil, fmt.Errorf("solveDLS: solve failed: %w", err)
	}

	dtheta := mat.NewVecDense(cols, nil)
	dtheta.MulVec(J.T(), y)

	result := make([]float64, cols)
	for i := 0; i < cols; i++ {
		result[i] = dtheta.AtVec(i)
	}
	return result, nil
}

// jointVelScale returns a scale ∈ [0, 1] that caps the max joint delta in
// jointDelta at `maxJointVelRads × dtSec`. Returns 1.0 when all joints are
// already within the budget.
func jointVelScale(jointDelta []float64, maxJointVelRads, dtSec float64) float64 {
	budget := maxJointVelRads * dtSec
	if budget <= 0 {
		return 1.0
	}
	maxAbs := 0.0
	for _, d := range jointDelta {
		if a := math.Abs(d); a > maxAbs {
			maxAbs = a
		}
	}
	if maxAbs <= budget {
		return 1.0
	}
	return budget / maxAbs
}
