package spacemouse

import (
	"encoding/binary"
	"strings"

	"go.viam.com/rdk/components/input"
)

// AxisIndex enumerates the six SpaceMouse axes.
type AxisIndex int

// Axis identifiers, in the device-table order X, Y, Z, RX, RY, RZ.
const (
	AxisX AxisIndex = iota
	AxisY
	AxisZ
	AxisRX
	AxisRY
	AxisRZ
)

// Control returns the Viam input.Control that corresponds to this axis.
func (a AxisIndex) Control() input.Control {
	return [...]input.Control{
		input.AbsoluteX,
		input.AbsoluteY,
		input.AbsoluteZ,
		input.AbsoluteRX,
		input.AbsoluteRY,
		input.AbsoluteRZ,
	}[a]
}

// AxisMapping locates a signed 16-bit LE axis sample within a HID report.
type AxisMapping struct {
	ReportID uint8
	Offset   int
	Sign     int
}

// ButtonMapping locates a button bit within a HID report and maps it to a Viam control.
type ButtonMapping struct {
	ReportID uint8
	Byte     int
	Bit      uint
	Control  input.Control
}

// Device is one entry in the HID device table.
type Device struct {
	Name      string
	VendorID  uint16
	ProductID uint16
	AxisScale float64
	Axes      [6]AxisMapping
	Buttons   []ButtonMapping
	Tested    bool
	Notes     string
}

// Ported from PySpaceMouse devices.toml and cross-checked with spacenavd src/dev.c.
// Only the Wireless BT Edition is empirically verified. Spacenavd and PySpaceMouse
// disagree with the printed axis labels on some devices — verify push/pull vs
// lift/press on real hardware before trusting an untested entry.
var deviceTable = []Device{
	{
		Name:      "SpaceMouse Wireless BT Edition",
		VendorID:  0x256f,
		ProductID: 0xc63a,
		AxisScale: 350.0,
		Axes:      allInOneReport1(),
		Buttons: []ButtonMapping{
			{ReportID: 3, Byte: 1, Bit: 0, Control: input.ButtonSelect},
			{ReportID: 3, Byte: 1, Bit: 1, Control: input.ButtonMenu},
		},
		Tested: true,
	},
	{
		Name:      "SpaceMouse Wireless Universal Receiver",
		VendorID:  0x256f,
		ProductID: 0xc652,
		AxisScale: 350.0,
		Axes:      allInOneReport1(),
		Buttons:   proButtons(),
	},
	{
		Name:      "SpaceMouse Wireless (2.4GHz)",
		VendorID:  0x256f,
		ProductID: 0xc62e,
		AxisScale: 350.0,
		Axes:      allInOneReport1(),
		Buttons: []ButtonMapping{
			{ReportID: 3, Byte: 1, Bit: 0, Control: input.ButtonSelect},
			{ReportID: 3, Byte: 1, Bit: 1, Control: input.ButtonMenu},
		},
	},
	{
		Name:      "SpaceMouse Pro Wireless",
		VendorID:  0x256f,
		ProductID: 0xc632,
		AxisScale: 350.0,
		Axes:      allInOneReport1(),
		Buttons:   proButtons(),
	},
	{
		Name:      "SpaceMouse Pro Wireless (Bluetooth)",
		VendorID:  0x256f,
		ProductID: 0xc638,
		AxisScale: 350.0,
		Axes:      allInOneReport1(),
		Buttons:   proButtons(),
	},
	{
		Name:      "SpaceMouse Enterprise",
		VendorID:  0x256f,
		ProductID: 0xc633,
		AxisScale: 350.0,
		Axes:      allInOneReport1(),
		Buttons:   proButtons(),
	},
	{
		Name:      "SpaceMouse Compact",
		VendorID:  0x256f,
		ProductID: 0xc635,
		AxisScale: 350.0,
		Axes:      splitReports(),
		Buttons: []ButtonMapping{
			{ReportID: 3, Byte: 1, Bit: 0, Control: input.ButtonSelect},
			{ReportID: 3, Byte: 1, Bit: 1, Control: input.ButtonMenu},
		},
	},
	{
		Name:      "SpaceMouse USB",
		VendorID:  0x256f,
		ProductID: 0xc641,
		AxisScale: 350.0,
		Axes:      splitReports(),
	},
	{
		Name:      "SpaceNavigator",
		VendorID:  0x046d,
		ProductID: 0xc626,
		AxisScale: 350.0,
		Axes:      splitReports(),
		Buttons: []ButtonMapping{
			{ReportID: 3, Byte: 1, Bit: 0, Control: input.ButtonSelect},
			{ReportID: 3, Byte: 1, Bit: 1, Control: input.ButtonMenu},
		},
	},
	{
		Name:      "SpaceExplorer",
		VendorID:  0x046d,
		ProductID: 0xc627,
		AxisScale: 350.0,
		Axes:      splitReports(),
		Buttons:   proButtons(),
	},
	{
		Name:      "SpacePilot",
		VendorID:  0x046d,
		ProductID: 0xc625,
		AxisScale: 350.0,
		Axes:      splitReports(),
	},
	{
		Name:      "SpaceMouse Pro",
		VendorID:  0x046d,
		ProductID: 0xc62b,
		AxisScale: 350.0,
		Axes:      splitReports(),
		Buttons:   proButtons(),
	},
}

func allInOneReport1() [6]AxisMapping {
	return [6]AxisMapping{
		{ReportID: 1, Offset: 1, Sign: +1},
		{ReportID: 1, Offset: 3, Sign: -1},
		{ReportID: 1, Offset: 5, Sign: -1},
		{ReportID: 1, Offset: 7, Sign: -1},
		{ReportID: 1, Offset: 9, Sign: -1},
		{ReportID: 1, Offset: 11, Sign: +1},
	}
}

func splitReports() [6]AxisMapping {
	return [6]AxisMapping{
		{ReportID: 1, Offset: 1, Sign: +1},
		{ReportID: 1, Offset: 3, Sign: -1},
		{ReportID: 1, Offset: 5, Sign: -1},
		{ReportID: 2, Offset: 1, Sign: -1},
		{ReportID: 2, Offset: 3, Sign: -1},
		{ReportID: 2, Offset: 5, Sign: +1},
	}
}

func proButtons() []ButtonMapping {
	return []ButtonMapping{
		{ReportID: 3, Byte: 2, Bit: 4, Control: input.ButtonSouth},
		{ReportID: 3, Byte: 2, Bit: 5, Control: input.ButtonEast},
		{ReportID: 3, Byte: 2, Bit: 6, Control: input.ButtonWest},
		{ReportID: 3, Byte: 2, Bit: 7, Control: input.ButtonNorth},
		{ReportID: 3, Byte: 1, Bit: 0, Control: input.ButtonMenu},
		{ReportID: 3, Byte: 1, Bit: 1, Control: input.ButtonStart},
	}
}

// Controls returns the Viam input.Controls this device exposes (6 axes + buttons).
func (d Device) Controls() []input.Control {
	controls := []input.Control{
		input.AbsoluteX, input.AbsoluteY, input.AbsoluteZ,
		input.AbsoluteRX, input.AbsoluteRY, input.AbsoluteRZ,
	}
	for _, b := range d.Buttons {
		controls = append(controls, b.Control)
	}
	return controls
}

// LookupDevice returns the device matching the given VID/PID, or nil.
func LookupDevice(vendorID, productID uint16) *Device {
	for i := range deviceTable {
		if deviceTable[i].VendorID == vendorID && deviceTable[i].ProductID == productID {
			return &deviceTable[i]
		}
	}
	return nil
}

// SupportedDevices returns a copy of the device table.
func SupportedDevices() []Device {
	out := make([]Device, len(deviceTable))
	copy(out, deviceTable)
	return out
}

// ParseAxis reads one axis sample from a HID report, normalized to [-1, 1].
func (d Device) ParseAxis(report []byte, axis AxisIndex) (float64, bool) {
	m := d.Axes[axis]
	if len(report) == 0 || report[0] != m.ReportID {
		return 0, false
	}
	if m.Offset+2 > len(report) {
		return 0, false
	}
	raw := int16(binary.LittleEndian.Uint16(report[m.Offset:]))
	v := float64(raw) * float64(m.Sign) / d.AxisScale
	if v > 1 {
		v = 1
	} else if v < -1 {
		v = -1
	}
	return v, true
}

// ParseButtons reads button bits from a HID report into a control→pressed map.
func (d Device) ParseButtons(report []byte) map[input.Control]bool {
	out := map[input.Control]bool{}
	if len(report) == 0 {
		return out
	}
	for _, b := range d.Buttons {
		if report[0] != b.ReportID {
			continue
		}
		if b.Byte >= len(report) {
			continue
		}
		pressed := (report[b.Byte]>>b.Bit)&1 == 1
		out[b.Control] = pressed
	}
	return out
}

// ReportIDs returns the distinct report IDs this device uses for axes and buttons.
func (d Device) ReportIDs() []uint8 {
	seen := map[uint8]bool{}
	for _, a := range d.Axes {
		seen[a.ReportID] = true
	}
	for _, b := range d.Buttons {
		seen[b.ReportID] = true
	}
	out := make([]uint8, 0, len(seen))
	for id := range seen {
		out = append(out, id)
	}
	return out
}

func (d Device) String() string {
	var b strings.Builder
	b.WriteString(d.Name)
	if !d.Tested {
		b.WriteString(" (untested)")
	}
	return b.String()
}
