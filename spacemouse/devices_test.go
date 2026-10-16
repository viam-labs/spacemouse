package spacemouse

import (
	"encoding/binary"
	"testing"

	"go.viam.com/rdk/components/input"
	"go.viam.com/test"
)

// wirelessBTReport1 builds a synthetic report ID 1 for the Wireless BT Edition
// (and other all-in-one-report devices). Values are raw signed int16s per axis.
func wirelessBTReport1(x, y, z, pitch, roll, yaw int16) []byte {
	b := make([]byte, 13)
	b[0] = 1
	binary.LittleEndian.PutUint16(b[1:], uint16(x))
	binary.LittleEndian.PutUint16(b[3:], uint16(y))
	binary.LittleEndian.PutUint16(b[5:], uint16(z))
	binary.LittleEndian.PutUint16(b[7:], uint16(pitch))
	binary.LittleEndian.PutUint16(b[9:], uint16(roll))
	binary.LittleEndian.PutUint16(b[11:], uint16(yaw))
	return b
}

func TestLookupWirelessBT(t *testing.T) {
	d := LookupDevice(0x256f, 0xc63a)
	test.That(t, d, test.ShouldNotBeNil)
	test.That(t, d.Name, test.ShouldContainSubstring, "Wireless BT")
	test.That(t, d.Tested, test.ShouldBeTrue)
	test.That(t, d.AxisScale, test.ShouldEqual, 350.0)
}

func TestLookupUnknown(t *testing.T) {
	test.That(t, LookupDevice(0x0000, 0x0000), test.ShouldBeNil)
	test.That(t, LookupDevice(0xdead, 0xbeef), test.ShouldBeNil)
}

func TestParseAxisWirelessBT(t *testing.T) {
	d := *LookupDevice(0x256f, 0xc63a)

	t.Run("full positive deflection on X", func(t *testing.T) {
		r := wirelessBTReport1(350, 0, 0, 0, 0, 0)
		v, ok := d.ParseAxis(r, AxisX)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, v, test.ShouldAlmostEqual, 1.0)
	})

	t.Run("full raw Y positive maps to -1 on AbsoluteY per sign flip", func(t *testing.T) {
		r := wirelessBTReport1(0, 350, 0, 0, 0, 0)
		v, ok := d.ParseAxis(r, AxisY)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, v, test.ShouldAlmostEqual, -1.0)
	})

	t.Run("values beyond scale clamp to [-1, 1]", func(t *testing.T) {
		var hi int16 = 2000
		var lo int16 = -2000
		r := wirelessBTReport1(hi, 0, 0, 0, 0, 0)
		v, ok := d.ParseAxis(r, AxisX)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, v, test.ShouldAlmostEqual, 1.0)

		r = wirelessBTReport1(lo, 0, 0, 0, 0, 0)
		v, ok = d.ParseAxis(r, AxisX)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, v, test.ShouldAlmostEqual, -1.0)
	})

	t.Run("wrong report ID returns ok=false", func(t *testing.T) {
		r := wirelessBTReport1(350, 0, 0, 0, 0, 0)
		r[0] = 3
		_, ok := d.ParseAxis(r, AxisX)
		test.That(t, ok, test.ShouldBeFalse)
	})

	t.Run("short buffer returns ok=false", func(t *testing.T) {
		r := []byte{1}
		_, ok := d.ParseAxis(r, AxisX)
		test.That(t, ok, test.ShouldBeFalse)
	})

	t.Run("all signs round-trip to expected polarity", func(t *testing.T) {
		r := wirelessBTReport1(100, 100, 100, 100, 100, 100)
		for axis, wantPositive := range map[AxisIndex]bool{
			AxisX:  true,
			AxisY:  false,
			AxisZ:  false,
			AxisRX: false,
			AxisRY: false,
			AxisRZ: true,
		} {
			v, ok := d.ParseAxis(r, axis)
			test.That(t, ok, test.ShouldBeTrue)
			if wantPositive {
				test.That(t, v, test.ShouldBeGreaterThan, 0)
			} else {
				test.That(t, v, test.ShouldBeLessThan, 0)
			}
		}
	})
}

func TestParseButtonsWirelessBT(t *testing.T) {
	d := *LookupDevice(0x256f, 0xc63a)

	t.Run("no buttons pressed", func(t *testing.T) {
		r := []byte{3, 0, 0, 0}
		pressed := d.ParseButtons(r)
		test.That(t, pressed[input.ButtonSelect], test.ShouldBeFalse)
		test.That(t, pressed[input.ButtonMenu], test.ShouldBeFalse)
	})

	t.Run("left (bit 0) pressed", func(t *testing.T) {
		r := []byte{3, 0b00000001, 0, 0}
		pressed := d.ParseButtons(r)
		test.That(t, pressed[input.ButtonSelect], test.ShouldBeTrue)
		test.That(t, pressed[input.ButtonMenu], test.ShouldBeFalse)
	})

	t.Run("right (bit 1) pressed", func(t *testing.T) {
		r := []byte{3, 0b00000010, 0, 0}
		pressed := d.ParseButtons(r)
		test.That(t, pressed[input.ButtonSelect], test.ShouldBeFalse)
		test.That(t, pressed[input.ButtonMenu], test.ShouldBeTrue)
	})

	t.Run("both pressed", func(t *testing.T) {
		r := []byte{3, 0b00000011, 0, 0}
		pressed := d.ParseButtons(r)
		test.That(t, pressed[input.ButtonSelect], test.ShouldBeTrue)
		test.That(t, pressed[input.ButtonMenu], test.ShouldBeTrue)
	})

	t.Run("report ID mismatch yields no button entries", func(t *testing.T) {
		r := []byte{1, 0b11111111, 0, 0}
		pressed := d.ParseButtons(r)
		test.That(t, len(pressed), test.ShouldEqual, 0)
	})
}

func TestControls(t *testing.T) {
	d := *LookupDevice(0x256f, 0xc63a)
	controls := d.Controls()
	test.That(t, len(controls), test.ShouldEqual, 8) // 6 axes + 2 buttons

	// Spot-check: all 6 axes present.
	seen := map[input.Control]bool{}
	for _, c := range controls {
		seen[c] = true
	}
	for _, axis := range []input.Control{
		input.AbsoluteX, input.AbsoluteY, input.AbsoluteZ,
		input.AbsoluteRX, input.AbsoluteRY, input.AbsoluteRZ,
	} {
		test.That(t, seen[axis], test.ShouldBeTrue)
	}
	test.That(t, seen[input.ButtonSelect], test.ShouldBeTrue)
	test.That(t, seen[input.ButtonMenu], test.ShouldBeTrue)
}

func TestReportIDs(t *testing.T) {
	t.Run("all-in-one has report 1 and 3", func(t *testing.T) {
		d := *LookupDevice(0x256f, 0xc63a)
		ids := d.ReportIDs()
		seen := map[uint8]bool{}
		for _, id := range ids {
			seen[id] = true
		}
		test.That(t, seen[1], test.ShouldBeTrue)
		test.That(t, seen[3], test.ShouldBeTrue)
	})

	t.Run("split-report has 1, 2, 3", func(t *testing.T) {
		d := *LookupDevice(0x046d, 0xc626)
		seen := map[uint8]bool{}
		for _, id := range d.ReportIDs() {
			seen[id] = true
		}
		test.That(t, seen[1], test.ShouldBeTrue)
		test.That(t, seen[2], test.ShouldBeTrue)
		test.That(t, seen[3], test.ShouldBeTrue)
	})
}

func TestSupportedDevicesReturnsCopy(t *testing.T) {
	devs := SupportedDevices()
	test.That(t, len(devs), test.ShouldBeGreaterThan, 1)

	// Mutating the returned slice must not affect the table.
	devs[0].Name = "mutated"
	fresh := SupportedDevices()
	test.That(t, fresh[0].Name, test.ShouldNotEqual, "mutated")
}

func TestEveryTableEntryIsValid(t *testing.T) {
	for _, d := range deviceTable {
		t.Run(d.Name, func(t *testing.T) {
			test.That(t, d.AxisScale, test.ShouldBeGreaterThan, 0)
			test.That(t, d.VendorID, test.ShouldNotEqual, 0)
			test.That(t, d.ProductID, test.ShouldNotEqual, 0)
			for axis := AxisX; axis <= AxisRZ; axis++ {
				m := d.Axes[axis]
				test.That(t, m.ReportID, test.ShouldNotEqual, 0)
				test.That(t, m.Offset, test.ShouldBeGreaterThanOrEqualTo, 1)
				test.That(t, m.Sign*m.Sign, test.ShouldEqual, 1) // |sign| == 1
			}
			for _, b := range d.Buttons {
				test.That(t, b.ReportID, test.ShouldNotEqual, 0)
				test.That(t, b.Byte, test.ShouldBeGreaterThanOrEqualTo, 1)
				test.That(t, b.Bit, test.ShouldBeLessThan, uint(8))
			}
		})
	}
}
