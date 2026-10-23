package jog

import (
	"context"
	"math"
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

// tick computes a goal pose from current input velocity and sends it via
// teleop_move. Gates: one in-flight call, error cooldown, pose deadband.
func (j *jog) tick() {
	vel := j.snapshot()
	above := j.anyAboveDeadband(vel)

	if !above {
		// On the moving→idle edge, pin the arm's current pose so the pipeline
		// decelerates to a stop. Then stay quiet until the user pushes again.
		j.moveMu.Lock()
		wasMoving := j.moving
		j.moving = false
		j.moveMu.Unlock()
		if !wasMoving {
			return
		}
		j.sendPinPose()
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

	currentPose, err := j.armRes.EndPosition(j.cancelCtx, nil)
	if err != nil {
		j.movePending.Store(false)
		j.logger.CWarnw(j.cancelCtx, "arm EndPosition failed", "err", err)
		return
	}

	lookaheadSec := j.lookahead.Seconds()
	delta := poseDeltaStep{
		X:  vel[0] * j.transPerSec * lookaheadSec,
		Y:  vel[1] * j.transPerSec * lookaheadSec,
		Z:  vel[2] * j.transPerSec * lookaheadSec,
		RX: vel[3] * j.rotPerSec * lookaheadSec,
		RY: vel[4] * j.rotPerSec * lookaheadSec,
		RZ: vel[5] * j.rotPerSec * lookaheadSec,
	}
	goal, err := applyDelta(currentPose, delta)
	if err != nil {
		j.movePending.Store(false)
		j.logger.CWarnw(j.cancelCtx, "pose composition failed", "err", err)
		return
	}

	// Skip the send if the goal hasn't moved past the pose deadband.
	j.moveMu.Lock()
	lastSent := j.lastSentPose
	j.moveMu.Unlock()
	if lastSent != nil && !poseDiffExceeds(lastSent, goal, j.posDeadbandMM, j.rotDeadbandDeg) {
		j.movePending.Store(false)
		return
	}

	j.moveMu.Lock()
	j.moving = true
	j.moveMu.Unlock()

	go func() {
		defer j.movePending.Store(false)
		if err := j.teleopMove(j.cancelCtx, goal); err != nil {
			j.logger.CWarnw(j.cancelCtx, "teleop_move failed", "err", err)
			j.moveMu.Lock()
			j.errorUntil = time.Now().Add(j.errorCooldown)
			j.moveMu.Unlock()
			return
		}
		j.moveMu.Lock()
		j.lastSentPose = goal
		j.moveMu.Unlock()
	}()
}

func (j *jog) sendPinPose() {
	if !j.movePending.CompareAndSwap(false, true) {
		return
	}
	defer j.movePending.Store(false)
	pose, err := j.armRes.EndPosition(j.cancelCtx, nil)
	if err != nil {
		j.logger.CWarnw(j.cancelCtx, "arm EndPosition failed on pin", "err", err)
		return
	}
	if err := j.teleopMove(j.cancelCtx, pose); err != nil {
		j.logger.CWarnw(j.cancelCtx, "teleop_move pin failed", "err", err)
		return
	}
	j.moveMu.Lock()
	j.lastSentPose = pose
	j.moveMu.Unlock()
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
