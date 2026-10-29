package spacemouse

import (
	"context"
	"errors"
	"sync"
	"time"

	"github.com/sstallion/go-hid"
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
)

// Larger than sensor jitter but below any realistic deadband a consumer
// would apply, so we don't re-emit when the user holds a steady deflection.
const axisEpsilon = 1.0 / 350.0

// Reader runs the HID I/O loop for a single SpaceMouse device.
type Reader struct {
	dev     Device
	serial  string
	path    string
	logger  logging.Logger
	onEvent func(input.Event)

	mu          sync.Mutex
	hidDev      hidHandle
	connected   bool
	lastAxes    [6]float64
	lastButtons map[input.Control]bool
}

type hidHandle interface {
	ReadWithTimeout(buf []byte, timeout time.Duration) (int, error)
	Close() error
}

var hidOpen = func(vid, pid uint16, serial, path string) (hidHandle, error) {
	if err := hid.Init(); err != nil {
		return nil, err
	}
	if path != "" {
		return hid.OpenPath(path)
	}
	if serial == "" {
		return hid.OpenFirst(vid, pid)
	}
	return hid.Open(vid, pid, serial)
}

// NewReader constructs a Reader for the given device entry. If path is
// non-empty it takes precedence over vendor/product/serial matching.
func NewReader(d Device, serial, path string, logger logging.Logger, onEvent func(input.Event)) *Reader {
	// Seed to false so the first report doesn't fire ButtonRelease for a button that was never pressed.
	initialButtons := make(map[input.Control]bool, len(d.Buttons))
	for _, b := range d.Buttons {
		initialButtons[b.Control] = false
	}
	return &Reader{
		dev:         d,
		serial:      serial,
		path:        path,
		logger:      logger,
		onEvent:     onEvent,
		lastButtons: initialButtons,
	}
}

// Device returns the device entry this Reader was built for.
func (r *Reader) Device() Device { return r.dev }

// Connected reports whether the HID handle is currently open.
func (r *Reader) Connected() bool {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.connected
}

// Snapshot returns the last-seen axis values and button states.
func (r *Reader) Snapshot() ([6]float64, map[input.Control]bool) {
	r.mu.Lock()
	defer r.mu.Unlock()
	axes := r.lastAxes
	buttons := make(map[input.Control]bool, len(r.lastButtons))
	for k, v := range r.lastButtons {
		buttons[k] = v
	}
	return axes, buttons
}

// Run drives the open → read → close → retry cycle until ctx is cancelled.
func (r *Reader) Run(ctx context.Context) {
	backoff := 250 * time.Millisecond
	for {
		if ctx.Err() != nil {
			r.emitDisconnect()
			return
		}
		h, err := hidOpen(r.dev.VendorID, r.dev.ProductID, r.serial, r.path)
		if err != nil {
			r.logger.CDebugw(ctx, "hid open failed, will retry", "device", r.dev.Name, "err", err)
			if !sleep(ctx, backoff) {
				return
			}
			continue
		}
		r.setHandle(h, true)
		r.emitConnect()
		r.readLoop(ctx, h)
		r.closeHandle()
		r.emitDisconnect()
	}
}

func (r *Reader) setHandle(h hidHandle, connected bool) {
	r.mu.Lock()
	r.hidDev = h
	r.connected = connected
	r.mu.Unlock()
}

func (r *Reader) closeHandle() {
	r.mu.Lock()
	h := r.hidDev
	r.hidDev = nil
	r.connected = false
	r.mu.Unlock()
	if h != nil {
		_ = h.Close()
	}
}

func (r *Reader) readLoop(ctx context.Context, h hidHandle) {
	buf := make([]byte, 64)
	for {
		if ctx.Err() != nil {
			return
		}
		n, err := h.ReadWithTimeout(buf, 500*time.Millisecond)
		if err != nil {
			if errors.Is(err, context.Canceled) {
				return
			}
			r.logger.CDebugw(ctx, "hid read error, treating as disconnect", "device", r.dev.Name, "err", err)
			return
		}
		if n <= 0 {
			continue
		}
		r.handleReport(buf[:n])
	}
}

func (r *Reader) handleReport(report []byte) {
	now := time.Now()

	for axis := AxisX; axis <= AxisRZ; axis++ {
		if r.dev.Axes[axis].ReportID != report[0] {
			continue
		}
		v, ok := r.dev.ParseAxis(report, axis)
		if !ok {
			continue
		}
		r.mu.Lock()
		prev := r.lastAxes[axis]
		if abs(v-prev) < axisEpsilon {
			r.mu.Unlock()
			continue
		}
		r.lastAxes[axis] = v
		r.mu.Unlock()
		r.onEvent(input.Event{
			Time:    now,
			Event:   input.PositionChangeAbs,
			Control: axis.Control(),
			Value:   v,
		})
	}

	buttons := r.dev.ParseButtons(report)
	if len(buttons) == 0 {
		return
	}
	for ctrl, pressed := range buttons {
		r.mu.Lock()
		prev, had := r.lastButtons[ctrl]
		if had && prev == pressed {
			r.mu.Unlock()
			continue
		}
		r.lastButtons[ctrl] = pressed
		r.mu.Unlock()
		et := input.ButtonRelease
		val := 0.0
		if pressed {
			et = input.ButtonPress
			val = 1.0
		}
		r.onEvent(input.Event{
			Time:    now,
			Event:   et,
			Control: ctrl,
			Value:   val,
		})
	}
}

func (r *Reader) emitConnect() {
	now := time.Now()
	r.mu.Lock()
	r.lastAxes = [6]float64{}
	r.mu.Unlock()
	for _, c := range r.dev.Controls() {
		r.onEvent(input.Event{
			Time:    now,
			Event:   input.Connect,
			Control: c,
			Value:   0,
		})
	}
}

// Zero axes and release held buttons before emitting Disconnect, so a
// live consumer never sees a stale deflection after the device drops.
func (r *Reader) emitDisconnect() {
	now := time.Now()
	r.mu.Lock()
	for i := range r.lastAxes {
		if r.lastAxes[i] != 0 {
			r.lastAxes[i] = 0
			r.onEvent(input.Event{
				Time:    now,
				Event:   input.PositionChangeAbs,
				Control: AxisIndex(i).Control(),
				Value:   0,
			})
		}
	}
	for c, pressed := range r.lastButtons {
		if pressed {
			r.lastButtons[c] = false
			r.onEvent(input.Event{
				Time:    now,
				Event:   input.ButtonRelease,
				Control: c,
				Value:   0,
			})
		}
	}
	r.mu.Unlock()
	for _, c := range r.dev.Controls() {
		r.onEvent(input.Event{
			Time:    now,
			Event:   input.Disconnect,
			Control: c,
			Value:   0,
		})
	}
}

func sleep(ctx context.Context, d time.Duration) bool {
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return false
	case <-t.C:
		return true
	}
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

// A SpaceMouse / 3Dconnexion wireless receiver typically exposes multiple HID
// interfaces (keyboard, mouse, multi-axis controller, etc.). Only the
// Multi-axis Controller interface (UsagePage 0x01, Usage 0x08) produces axis
// reports; the others would silently fail to drive teleop, and including them
// clutters discovery with 5x as many entries as there are physical devices.
const (
	usagePageGenericDesktop = 0x01
	usageMultiAxisCtrl      = 0x08
)

// EnumerateConnected returns every supported SpaceMouse currently plugged in,
// filtered to the Multi-axis Controller HID interface.
func EnumerateConnected() ([]DeviceInstance, error) {
	if err := hid.Init(); err != nil {
		return nil, err
	}
	var found []DeviceInstance
	err := hid.Enumerate(hid.VendorIDAny, hid.ProductIDAny, func(info *hid.DeviceInfo) error {
		if info.UsagePage != usagePageGenericDesktop || info.Usage != usageMultiAxisCtrl {
			return nil
		}
		if d := LookupDevice(info.VendorID, info.ProductID); d != nil {
			found = append(found, DeviceInstance{
				Device:       *d,
				SerialNumber: info.SerialNbr,
				Path:         info.Path,
			})
		}
		return nil
	})
	if err != nil {
		return nil, err
	}
	return dedupe(found), nil
}

// DeviceInstance is a connected device plus its HID path and serial number.
type DeviceInstance struct {
	Device       Device
	SerialNumber string
	Path         string
}

// Some OSes enumerate a single device multiple times (one entry per HID usage page).
// When SerialNumber is empty (e.g. 3Dconnexion Universal Receivers), each distinct
// Path represents a different physical device and must not be deduped away.
func dedupe(in []DeviceInstance) []DeviceInstance {
	seen := map[string]bool{}
	out := in[:0]
	for _, inst := range in {
		key := inst.Device.Name + "|" + inst.SerialNumber
		if inst.SerialNumber == "" {
			key += "|" + inst.Path
		}
		if seen[key] {
			continue
		}
		seen[key] = true
		out = append(out, inst)
	}
	return out
}
