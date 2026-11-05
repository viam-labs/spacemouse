package jog

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/input"
)

type jogMode int32

const (
	modeSlow jogMode = iota
	modeFast
)

func (m jogMode) String() string {
	if m == modeFast {
		return "fast"
	}
	return "slow"
}

const defaultModeToggleButton = "ButtonSelect"

// Validated at config time so a typo fails on startup rather than silently never firing.
var validModeToggleButtons = map[string]input.Control{
	"ButtonSelect": input.ButtonSelect,
	"ButtonMenu":   input.ButtonMenu,
	"ButtonSouth":  input.ButtonSouth,
	"ButtonEast":   input.ButtonEast,
	"ButtonWest":   input.ButtonWest,
	"ButtonNorth":  input.ButtonNorth,
	"ButtonStart":  input.ButtonStart,
}

func parseModeToggleButton(name string) (input.Control, error) {
	if name == "" {
		return validModeToggleButtons[defaultModeToggleButton], nil
	}
	c, ok := validModeToggleButtons[name]
	if !ok {
		return "", fmt.Errorf("unknown mode_toggle_button %q", name)
	}
	return c, nil
}

func (j *jog) currentMode() jogMode {
	return jogMode(j.mode.Load())
}

func (j *jog) setMode(m jogMode) {
	j.mode.Store(int32(m))
}

func (j *jog) toggleMode(ctx context.Context, _ input.Event) {
	j.modeMu.Lock()
	defer j.modeMu.Unlock()

	next := modeFast
	if j.currentMode() == modeFast {
		next = modeSlow
	}
	j.setMode(next)

	j.moveMu.Lock()
	j.virtualGoal = nil
	j.lastSentPose = nil
	j.moving = false
	j.moveMu.Unlock()

	// Close pipeline on entering slow so stale planner state can't leak over.
	if next == modeSlow && j.pipelineOpen.Load() {
		j.closePipeline(ctx)
	}

	j.logger.CInfow(ctx, "jog mode toggled", "mode", next.String())
}
