# Arm teleop with the SpaceMouse

`viam:spacemouse:jog` turns a 6-axis `input_controller` into fluid end-effector
motion via the motion service's teleop pipeline.

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
    "translation_mm_per_sec": 50,
    "rotation_deg_per_sec": 20
  }
}
```

| field                     | default    | notes                                                                    |
|---------------------------|------------|--------------------------------------------------------------------------|
| `arm` *(required)*        | —          | Arm resource name.                                                       |
| `input_controller` *(req)*| —          | Input controller resource name.                                          |
| `motion`                  | `"motion"` | Name of the motion service.                                              |
| `deadband`                | `0.05`     | Minimum axis magnitude to register.                                      |
| `translation_mm_per_sec`  | `100`      | EE speed at full stick.                                                  |
| `rotation_deg_per_sec`    | `45`       | EE rotation speed at full stick.                                         |
| `tick_hz`                 | `50`       | Drain loop rate.                                                         |
| `lookahead_ms`            | `300`      | How far ahead the goal pose is placed each tick.                         |

## Tuning

Start at `translation_mm_per_sec: 10` / `rotation_deg_per_sec: 5` and double
until the arm moves at a comfortable rate.

The motion-service attributes shown above are the smoothness/interpolation
knobs the teleop pipeline consumes. The defaults in a stock motion service
(`teleop_smooth_alpha: 0.5`, no interpolation override) already match what
this module expects — override them only if you want to deviate.
