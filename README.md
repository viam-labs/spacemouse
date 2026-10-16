# spacemouse

Viam module exposing 3Dconnexion SpaceMouse devices as an `input_controller`,
with a sibling discovery service that enumerates connected devices.

## Models

### `viam:spacemouse:controller` (input_controller)

Config:

```json
{
  "name": "my-spacemouse",
  "api": "rdk:component:input_controller",
  "model": "viam:spacemouse:controller",
  "attributes": {
    "vendor_id": "256f",
    "product_id": "c63a"
  }
}
```

| field        | required | notes                                                       |
|--------------|----------|-------------------------------------------------------------|
| `vendor_id`  | yes      | USB vendor ID in hex (`"256f"` or `"0x256f"`).              |
| `product_id` | yes      | USB product ID in hex.                                      |
| `serial`     | no       | HID serial number — needed to disambiguate identical twins. |

Run the discovery service once to find the IDs for the device plugged in.
Axis values are normalized to `[-1, 1]`.

### `viam:spacemouse:discovery` (discovery service)

Returns one suggested `input_controller` config per connected supported
device. Useful during setup to see what the host currently sees without
having to run `lsusb`.

### `viam:spacemouse:jog` (generic service)

Jogs an arm's end effector from any `input_controller` exposing six absolute
axes. Release the controller to stop.

```json
{
  "name": "my-jog",
  "api": "rdk:service:generic",
  "model": "viam:spacemouse:jog",
  "attributes": {
    "arm": "my-arm",
    "input_controller": "my-spacemouse"
  }
}
```

| field                     | required | default | notes                                   |
|---------------------------|----------|---------|-----------------------------------------|
| `arm`                     | yes      | —       | Arm resource name.                      |
| `input_controller`        | yes      | —       | input_controller resource name.         |
| `deadband`                | no       | `0.05`  | Below this axis magnitude, no motion.   |
| `translation_mm_per_sec`  | no       | `100`   | Translation speed at full axis.         |
| `rotation_deg_per_sec`    | no       | `45`    | Rotation speed at full axis.            |
| `tick_hz`                 | no       | `50`    | Drain loop rate.                        |

Operates in the arm's base frame. No safety gates — the operator is the deadman.

## Host setup

Requires `hidapi` (Linux) or `hidapi` from Homebrew (macOS):

```sh
# Linux
sudo apt install libhidapi-hidraw0 libhidapi-libusb0

# macOS
brew install hidapi
```

Linux also needs a udev rule so a non-root `viam-server` can open the HID
device. Save to `/etc/udev/rules.d/99-spacemouse.rules` and reload:

```
KERNEL=="hidraw*", ATTRS{idVendor}=="256f", MODE="0660", GROUP="plugdev"
KERNEL=="hidraw*", ATTRS{idVendor}=="046d", MODE="0660", GROUP="plugdev"
```

## Build

```sh
make setup          # go mod tidy
make build          # bin/spacemouse
make test           # go test ./...
make module.tar.gz  # packaged module for upload
```

## Adding a new device

Add one entry to `deviceTable` in `spacemouse/devices.go`. Cross-reference
PySpaceMouse's `devices.toml` and `spacenavd`'s `src/dev.c` for VID/PID,
report layout, and scale. If the core read loop needs to change to support
a model, redesign the table struct — don't fork the reader.
