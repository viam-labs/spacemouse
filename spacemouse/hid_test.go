package spacemouse

import (
	"encoding/binary"
	"sync"
	"testing"

	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

type eventCollector struct {
	mu     sync.Mutex
	events []input.Event
}

func (ec *eventCollector) record(e input.Event) {
	ec.mu.Lock()
	defer ec.mu.Unlock()
	ec.events = append(ec.events, e)
}

func (ec *eventCollector) snapshot() []input.Event {
	ec.mu.Lock()
	defer ec.mu.Unlock()
	out := make([]input.Event, len(ec.events))
	copy(out, ec.events)
	return out
}

func TestHandleReport_AxisChangeEmitsEvent(t *testing.T) {
	d := *LookupDevice(0x256f, 0xc63a)
	ec := &eventCollector{}
	r := NewReader(d, "", "", logging.NewTestLogger(t), ec.record)

	report := make([]byte, 13)
	report[0] = 1
	binary.LittleEndian.PutUint16(report[1:], uint16(175)) // X at ~0.5

	r.handleReport(report)

	events := ec.snapshot()
	test.That(t, len(events), test.ShouldEqual, 1)
	test.That(t, events[0].Control, test.ShouldEqual, input.AbsoluteX)
	test.That(t, events[0].Event, test.ShouldEqual, input.PositionChangeAbs)
	test.That(t, events[0].Value, test.ShouldAlmostEqual, 0.5, 0.01)
}

func TestHandleReport_SameValueDoesNotRepeat(t *testing.T) {
	d := *LookupDevice(0x256f, 0xc63a)
	ec := &eventCollector{}
	r := NewReader(d, "", "", logging.NewTestLogger(t), ec.record)

	report := make([]byte, 13)
	report[0] = 1
	binary.LittleEndian.PutUint16(report[1:], uint16(175))

	r.handleReport(report)
	r.handleReport(report)
	r.handleReport(report)

	events := ec.snapshot()
	test.That(t, len(events), test.ShouldEqual, 1)
}

func TestHandleReport_ButtonPressReleaseTransitions(t *testing.T) {
	d := *LookupDevice(0x256f, 0xc63a)
	ec := &eventCollector{}
	r := NewReader(d, "", "", logging.NewTestLogger(t), ec.record)

	// Press left.
	r.handleReport([]byte{3, 0b01, 0, 0})
	// Keep pressed — no new event.
	r.handleReport([]byte{3, 0b01, 0, 0})
	// Also press right — one new event.
	r.handleReport([]byte{3, 0b11, 0, 0})
	// Release left — one new event (left released).
	r.handleReport([]byte{3, 0b10, 0, 0})

	events := ec.snapshot()
	test.That(t, len(events), test.ShouldEqual, 3)
	test.That(t, events[0].Event, test.ShouldEqual, input.ButtonPress)
	test.That(t, events[1].Event, test.ShouldEqual, input.ButtonPress)
	test.That(t, events[2].Event, test.ShouldEqual, input.ButtonRelease)
}

func TestSnapshot_TracksLatest(t *testing.T) {
	d := *LookupDevice(0x256f, 0xc63a)
	r := NewReader(d, "", "", logging.NewTestLogger(t), func(input.Event) {})

	report := make([]byte, 13)
	report[0] = 1
	var xRaw int16 = 350
	var yRaw int16 = -350
	binary.LittleEndian.PutUint16(report[1:], uint16(xRaw))
	binary.LittleEndian.PutUint16(report[3:], uint16(yRaw)) // Y Viam value = -(-350)/350 = +1
	r.handleReport(report)

	axes, _ := r.Snapshot()
	test.That(t, axes[AxisX], test.ShouldAlmostEqual, 1.0)
	test.That(t, axes[AxisY], test.ShouldAlmostEqual, 1.0)
}

func TestParseHex16(t *testing.T) {
	v, err := parseHex16("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, v, test.ShouldEqual, uint16(0))

	v, err = parseHex16("256f")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, v, test.ShouldEqual, uint16(0x256f))

	v, err = parseHex16("0x256F")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, v, test.ShouldEqual, uint16(0x256f))

	_, err = parseHex16("notahex")
	test.That(t, err, test.ShouldNotBeNil)

	_, err = parseHex16("1ffff")
	test.That(t, err, test.ShouldNotBeNil)
}
