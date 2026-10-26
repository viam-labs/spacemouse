package jog

import (
	"math"
	"testing"

	"go.viam.com/test"
)

func TestJointVelScale(t *testing.T) {
	// Budget: 30°/s × 20ms = 0.6° per tick.
	maxVelRads := 30 * math.Pi / 180
	dtSec := 0.02

	t.Run("under budget returns 1", func(t *testing.T) {
		dtheta := []float64{0.001, 0.002, -0.003}
		test.That(t, jointVelScale(dtheta, maxVelRads, dtSec), test.ShouldEqual, 1.0)
	})

	t.Run("at budget returns 1", func(t *testing.T) {
		budget := maxVelRads * dtSec
		dtheta := []float64{0, 0, budget}
		test.That(t, jointVelScale(dtheta, maxVelRads, dtSec), test.ShouldAlmostEqual, 1.0)
	})

	t.Run("over budget scales down", func(t *testing.T) {
		budget := maxVelRads * dtSec
		dtheta := []float64{0, 0, 2 * budget}
		test.That(t, jointVelScale(dtheta, maxVelRads, dtSec), test.ShouldAlmostEqual, 0.5)
	})

	t.Run("worst-joint drives the scale", func(t *testing.T) {
		budget := maxVelRads * dtSec
		dtheta := []float64{0.5 * budget, 10 * budget, 0.1 * budget}
		test.That(t, jointVelScale(dtheta, maxVelRads, dtSec), test.ShouldAlmostEqual, 0.1)
	})

	t.Run("absolute values matter, not sign", func(t *testing.T) {
		budget := maxVelRads * dtSec
		dtheta := []float64{0, -4 * budget, 0}
		test.That(t, jointVelScale(dtheta, maxVelRads, dtSec), test.ShouldAlmostEqual, 0.25)
	})

	t.Run("zero budget returns 1 (disabled)", func(t *testing.T) {
		dtheta := []float64{100, 100, 100}
		test.That(t, jointVelScale(dtheta, 0, dtSec), test.ShouldEqual, 1.0)
	})
}

func TestSolveDLS_IdentityJacobian(t *testing.T) {
	// With J = I (6×6), dθ = ΔX / (1 + λ²).
	// For small λ, dθ ≈ ΔX.
	J := identity6()
	tcpDelta := []float64{1, 2, 3, 0.1, 0.2, 0.3}
	dtheta, err := solveDLS(J, tcpDelta, 0.001)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(dtheta), test.ShouldEqual, 6)
	for i, want := range tcpDelta {
		test.That(t, dtheta[i], test.ShouldAlmostEqual, want, 1e-3)
	}
}

func TestSolveDLS_SingularJacobianBounded(t *testing.T) {
	// Rank-5 Jacobian: one column of zeros simulates a singular direction.
	// DLS should produce a bounded (not NaN/Inf) dθ.
	J := identity6()
	J.Set(0, 0, 0)
	J.Set(1, 0, 0)
	J.Set(2, 0, 0)
	J.Set(3, 0, 0)
	J.Set(4, 0, 0)
	J.Set(5, 0, 0)

	tcpDelta := []float64{10, 0, 0, 0, 0, 0}
	dtheta, err := solveDLS(J, tcpDelta, 0.01)
	test.That(t, err, test.ShouldBeNil)
	for _, d := range dtheta {
		test.That(t, math.IsNaN(d), test.ShouldBeFalse)
		test.That(t, math.IsInf(d, 0), test.ShouldBeFalse)
	}
}
