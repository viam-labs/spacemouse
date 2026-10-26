# Arm teleop with the SpaceMouse

`viam:spacemouse:jog` turns a 6-axis `input_controller` into fluid end-effector
motion via the motion service's teleop pipeline.

End-effector speed is bounded directly by `translation_mm_per_sec` and
`rotation_deg_per_sec` — the service maintains a virtual goal pose that
advances at that rate, independent of the arm's physical maximum.

## Prerequisites

- An arm.
- A 6-axis `input_controller` (SpaceMouse, gamepad, etc.).
- A `rdk:service:motion/rdk:builtin:builtin` with these attributes:

```json
{ "teleop_smooth_alpha": 0.5, "teleop_interpolate_override": false }
```

## Config

```json
{
  "name": "jog",
  "api": "rdk:service:generic",
  "model": "viam:spacemouse:jog",
  "attributes": {
    "arm": "my-arm",
    "input_controller": "my-spacemouse",
    "translation_mm_per_sec": 100,
    "rotation_deg_per_sec": 45
  }
}
```

| field                     | default    | notes                                                                    |
|---------------------------|------------|--------------------------------------------------------------------------|
| `arm` *(required)*        | —          | Arm resource name.                                                       |
| `input_controller` *(req)*| —          | Input controller resource name.                                          |
| `motion`                  | `"motion"` | Name of the motion service.                                              |
| `deadband`                | `0.05`     | Minimum axis magnitude to register.                                      |
| `translation_mm_per_sec`  | `100`      | TCP speed at full stick, in mm/s. Direct cap.                            |
| `rotation_deg_per_sec`    | `45`       | TCP rotation speed at full stick, in deg/s. Direct cap.                  |
| `tick_hz`                 | `50`       | Drain loop rate.                                                         |

## Speed guide

| use case              | `translation_mm_per_sec` | `rotation_deg_per_sec` |
|-----------------------|--------------------------|------------------------|
| careful alignment     | `25`–`50`                | `10`–`20`              |
| general teleop        | `100`                    | `45`                   |
| rapid repositioning   | `200`–`300`              | `90`                   |

Both knobs are hard caps on the virtual goal's advance rate — the actual arm
tracks the goal, so TCP speed matches what you configure (bounded by the
arm's own physical capability if you set the knob higher than the arm can
move).

## Motion service attributes

The motion-service attributes shown above are the smoothness/interpolation
knobs the teleop pipeline consumes. The defaults in a stock motion service
(`teleop_smooth_alpha: 0.5`, no interpolation override) already match what
this module expects — override them only if you want to deviate.
