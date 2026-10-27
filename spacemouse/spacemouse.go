// Package spacemouse implements a Viam input_controller for 3Dconnexion SpaceMouse devices.
package spacemouse

import (
	"context"
	"fmt"
	"sync"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/utils"
)

// Model is the registered Viam model for this input_controller.
var Model = resource.NewModel("viam", "spacemouse", "controller")

// Config picks a specific SpaceMouse by USB vendor/product ID, with optional
// serial or OS device path for disambiguation when multiple identical devices
// are plugged in. DevicePath takes precedence over Serial when both are set.
type Config struct {
	VendorID   string `json:"vendor_id"`
	ProductID  string `json:"product_id"`
	Serial     string `json:"serial,omitempty"`
	DevicePath string `json:"device_path,omitempty"`
}

// Validate requires vendor_id and product_id and checks they parse as hex.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.VendorID == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "vendor_id")
	}
	if c.ProductID == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "product_id")
	}
	if _, err := parseHex16(c.VendorID); err != nil {
		return nil, nil, resource.NewConfigValidationError(path, err)
	}
	if _, err := parseHex16(c.ProductID); err != nil {
		return nil, nil, resource.NewConfigValidationError(path, err)
	}
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(input.API, Model, resource.Registration[input.Controller, *Config]{
		Constructor: newSpaceMouse,
	})
}

type spaceMouse struct {
	resource.Named
	resource.AlwaysRebuild

	logger  logging.Logger
	cfg     *Config
	reader  *Reader
	workers *utils.StoppableWorkers

	mu         sync.RWMutex
	controls   []input.Control
	lastEvents map[input.Control]input.Event
	callbacks  map[input.Control]map[input.EventType]input.ControlFunction
}

func newSpaceMouse(
	ctx context.Context,
	_ resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (input.Controller, error) {
	cfg, ok := conf.ConvertedAttributes.(*Config)
	if !ok {
		return nil, errors.Errorf("expected *Config, got %T", conf.ConvertedAttributes)
	}

	vid, pid, err := parseIDs(cfg.VendorID, cfg.ProductID)
	if err != nil {
		return nil, err
	}
	dev, err := pickDevice(vid, pid)
	if err != nil {
		return nil, err
	}

	sm := &spaceMouse{
		Named:      conf.ResourceName().AsNamed(),
		logger:     logger,
		cfg:        cfg,
		controls:   dev.Controls(),
		lastEvents: map[input.Control]input.Event{},
		callbacks:  map[input.Control]map[input.EventType]input.ControlFunction{},
	}

	sm.reader = NewReader(dev, cfg.Serial, cfg.DevicePath, logger, sm.dispatch)
	sm.workers = utils.NewBackgroundStoppableWorkers(sm.reader.Run)

	logger.CInfow(ctx, "spacemouse initialized",
		"device", dev.String(),
		"vendor_id", fmt.Sprintf("%04x", dev.VendorID),
		"product_id", fmt.Sprintf("%04x", dev.ProductID),
	)
	return sm, nil
}

func pickDevice(vid, pid uint16) (Device, error) {
	if d := LookupDevice(vid, pid); d != nil {
		return *d, nil
	}
	return Device{}, errors.Errorf("vendor_id=%04x product_id=%04x not in supported device table", vid, pid)
}

func parseIDs(vendor, product string) (uint16, uint16, error) {
	vid, err := parseHex16(vendor)
	if err != nil {
		return 0, 0, errors.Wrap(err, "bad vendor_id")
	}
	pid, err := parseHex16(product)
	if err != nil {
		return 0, 0, errors.Wrap(err, "bad product_id")
	}
	return vid, pid, nil
}

func parseHex16(s string) (uint16, error) {
	if s == "" {
		return 0, nil
	}
	if len(s) > 2 && (s[0:2] == "0x" || s[0:2] == "0X") {
		s = s[2:]
	}
	var n uint32
	_, err := fmt.Sscanf(s, "%x", &n)
	if err != nil {
		return 0, err
	}
	if n > 0xFFFF {
		return 0, errors.Errorf("value %q exceeds 16 bits", s)
	}
	return uint16(n), nil
}

func (sm *spaceMouse) dispatch(ev input.Event) {
	sm.mu.Lock()
	sm.lastEvents[ev.Control] = ev
	cbs := sm.callbacks[ev.Control]
	cb, hasCB := cbs[ev.Event]
	cbAll, hasAll := cbs[input.AllEvents]
	sm.mu.Unlock()

	if hasCB && cb != nil {
		sm.workers.Add(func(ctx context.Context) { cb(ctx, ev) })
	}
	if hasAll && cbAll != nil {
		sm.workers.Add(func(ctx context.Context) { cbAll(ctx, ev) })
	}
}

func (sm *spaceMouse) Controls(_ context.Context, _ map[string]interface{}) ([]input.Control, error) {
	sm.mu.RLock()
	defer sm.mu.RUnlock()
	out := make([]input.Control, len(sm.controls))
	copy(out, sm.controls)
	return out, nil
}

func (sm *spaceMouse) Events(_ context.Context, _ map[string]interface{}) (map[input.Control]input.Event, error) {
	sm.mu.RLock()
	defer sm.mu.RUnlock()
	out := make(map[input.Control]input.Event, len(sm.lastEvents))
	for k, v := range sm.lastEvents {
		out[k] = v
	}
	return out, nil
}

func (sm *spaceMouse) RegisterControlCallback(
	_ context.Context,
	control input.Control,
	triggers []input.EventType,
	ctrlFunc input.ControlFunction,
	_ map[string]interface{},
) error {
	sm.mu.Lock()
	defer sm.mu.Unlock()
	if sm.callbacks[control] == nil {
		sm.callbacks[control] = map[input.EventType]input.ControlFunction{}
	}
	for _, trigger := range triggers {
		if trigger == input.ButtonChange {
			sm.callbacks[control][input.ButtonPress] = ctrlFunc
			sm.callbacks[control][input.ButtonRelease] = ctrlFunc
		} else {
			sm.callbacks[control][trigger] = ctrlFunc
		}
	}
	return nil
}

func (sm *spaceMouse) Close(_ context.Context) error {
	if sm.workers != nil {
		sm.workers.Stop()
	}
	return nil
}
