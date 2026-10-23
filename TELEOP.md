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
| `pos_deadband_mm`         | `1.0`      | Minimum pose change to re-send; skips sub-threshold updates.             |
| `rot_deadband_deg`        | `1.0`      | Same, rotation.                                                          |
| `error_cooldown_ms`       | `1000`     | How long to skip sends after a `teleop_move` failure.                    |

## Tuning

Start at `translation_mm_per_sec: 10` / `rotation_deg_per_sec: 5` and double
until the arm moves at a comfortable rate. If motion is jittery, confirm the
motion service has the two attributes above and check the arm's own
`speed_degs_per_sec` isn't set too high for smooth continuous motion.

## Troubleshooting

- **No motion** — check module logs for `teleop_move failed`; the `err`
  attribute names the cause.
- **`pipeline is not running` repeatedly** — the motion service is
  reconfiguring on every tick. Check its config for misconfigured attributes.
- **`need a plan_file_path if you sent LogSlowPlanThresholdMS`** — remove
  `log_slow_plan_threshold_ms` from the motion service or add
  `plan_file_path` next to it.
