package jog

import (
	"context"
	"math"
	"strings"
	"time"

	"go.viam.com/rdk/components/input"
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

	for {
		select {
		case <-j.cancelCtx.Done():
			return
		case <-ticker.C:
			j.tick()
		}
	}
}

func (j *jog) tick() {
	vel := j.currentVelocities()
	above := j.anyAxisAboveDeadband(vel)

	if !above {
		// Clear virtualGoal on every idle tick so a stale goal from a dropped
		// send or failed park can never carry over.
		j.moveMu.Lock()
		wasMoving := j.moving
		j.moving = false
		j.virtualGoal = nil
		j.moveMu.Unlock()
		if !wasMoving {
			return
		}
		j.parkArm()
		return
	}

	j.moveMu.Lock()
	cooldownUntil := j.errorUntil
	j.moveMu.Unlock()
	if time.Now().Before(cooldownUntil) {
		return
	}

	if !j.movePending.CompareAndSwap(false, true) {
		return
	}

	currentPose, err := j.arm.EndPosition(j.cancelCtx, nil)
	if err != nil {
		j.movePending.Store(false)
		j.logger.CWarnw(j.cancelCtx, "arm EndPosition failed", "err", err)
		return
	}

	j.moveMu.Lock()
	base := j.virtualGoal
	j.moveMu.Unlock()
	if base == nil {
		base = currentPose
	}

	dtSec := j.tickInterval.Seconds()
	delta := poseDeltaStep{
		X:  vel[0] * j.transPerSec * dtSec,
		Y:  vel[1] * j.transPerSec * dtSec,
		Z:  vel[2] * j.transPerSec * dtSec,
		RX: vel[3] * j.rotPerSec * dtSec,
		RY: vel[4] * j.rotPerSec * dtSec,
		RZ: vel[5] * j.rotPerSec * dtSec,
	}
	goal, err := applyDelta(base, delta)
	if err != nil {
		j.movePending.Store(false)
		j.logger.CWarnw(j.cancelCtx, "pose composition failed", "err", err)
		return
	}

	// Clamp how far the virtual goal may run ahead of the arm. If arm can't
	// keep up with the configured speed, goal snaps back toward actual.
	if poseDiffExceeds(currentPose, goal, maxLagMM, maxLagDeg) {
		goal = currentPose
	}

	j.moveMu.Lock()
	j.virtualGoal = goal
	lastSent := j.lastSentPose
	j.moveMu.Unlock()
	if lastSent != nil && !poseDiffExceeds(lastSent, goal, posDeadbandMM, rotDeadbandDeg) {
		j.movePending.Store(false)
		return
	}

	j.moveMu.Lock()
	j.moving = true
	j.moveMu.Unlock()

	go func() {
		defer j.movePending.Store(false)
		if err := j.sendGoal(j.cancelCtx, goal); err != nil {
			// Motion service drops the teleop pipeline on any reconfigure (RDK quirk,
			// see services/motion/builtin/builtin.go). Re-open and let the next tick retry.
			if strings.Contains(err.Error(), "not running") {
				if openErr := j.openPipeline(j.cancelCtx); openErr != nil {
					j.logger.CWarnw(j.cancelCtx, "pipeline re-open failed", "err", openErr)
					j.moveMu.Lock()
					j.errorUntil = time.Now().Add(errorCooldown)
					j.moveMu.Unlock()
				}
				return
			}
			j.logger.CWarnw(j.cancelCtx, "send goal failed", "err", err)
			j.moveMu.Lock()
			j.errorUntil = time.Now().Add(errorCooldown)
			j.moveMu.Unlock()
			return
		}
		j.moveMu.Lock()
		j.lastSentPose = goal
		j.moveMu.Unlock()
	}()
}

func (j *jog) parkArm() {
	// Park is safety-critical: must always fire. Wait for any in-flight send to
	// drain so we don't race it (and have the arm chase a stale goal set by the
	// in-flight send after we park).
	deadline := time.Now().Add(parkDrainTimeout)
	for !j.movePending.CompareAndSwap(false, true) {
		if time.Now().After(deadline) {
			j.logger.CWarn(j.cancelCtx, "park: in-flight send did not drain; sending anyway")
			break
		}
		time.Sleep(parkDrainPoll)
	}
	defer j.movePending.Store(false)

	defer func() {
		j.moveMu.Lock()
		j.virtualGoal = nil
		j.moveMu.Unlock()
	}()

	pose, err := j.arm.EndPosition(j.cancelCtx, nil)
	if err != nil {
		j.logger.CWarnw(j.cancelCtx, "arm EndPosition failed on park", "err", err)
		return
	}
	if err := j.sendGoal(j.cancelCtx, pose); err != nil {
		j.logger.CWarnw(j.cancelCtx, "park arm failed", "err", err)
		return
	}
	j.moveMu.Lock()
	j.lastSentPose = pose
	j.moveMu.Unlock()
}

func (j *jog) currentVelocities() [6]float64 {
	j.velMu.RLock()
	defer j.velMu.RUnlock()
	return j.velocities
}

func (j *jog) anyAxisAboveDeadband(vel [6]float64) bool {
	for _, v := range vel {
		if math.Abs(v) > j.deadband {
			return true
		}
	}
	return false
}
