# Arm teleop with the SpaceMouse

`viam:spacemouse:jog` turns a 6-axis `input_controller` into end-effector motion.

## Modes

Two modes, flipped by pressing the configured SpaceMouse button
(`mode_toggle_button`, default `ButtonSelect`). The service starts in **slow**.

- **slow** (default): each tick calls `arm.MoveToPosition` with a small delta
  (`slow_step_mm × axis_value`, same for rotation). Every move goes through the
  standard motion planner — joint limits and collisions are respected, and
  there's no teleop pipeline state to desync. Steppy but robust; use for
  careful positioning.
- **fast**: streams goal poses into the motion service's teleop pipeline for
  continuous motion. See [Fast mode](#fast-mode) below for the virtual-goal
  and Jacobian-scaling details.

The configured gripper button and mode-toggle button must not be the same.

## Prerequisites

- An arm.
- A 6-axis `input_controller` (SpaceMouse, gamepad, etc.).
- A `rdk:service:motion/rdk:builtin:builtin`. If you plan to use fast mode, it
  needs these attributes:

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
    "translation_mm_per_sec": 150,
    "rotation_deg_per_sec": 30
  }
}
```

| field                     | default    | notes                                                                    |
|---------------------------|------------|--------------------------------------------------------------------------|
| `arm` *(required)*        | —          | Arm resource name.                                                       |
| `input_controller` *(req)*| —          | Input controller resource name.                                          |
| `motion`                  | `"motion"` | Name of the motion service.                                              |
| `gripper`                 | —          | Optional gripper resource name. Toggled by `gripper_button`.             |
| `gripper_button`          | `"ButtonMenu"` | Button Control that toggles the gripper on `ButtonPress`.            |
| `mode_toggle_button`      | `"ButtonSelect"` | Button Control that toggles slow ↔ fast on `ButtonPress`.          |
| `deadband`                | `0.05`     | Minimum axis magnitude to register.                                      |
| `slow_step_mm`            | `1.0`      | Slow-mode TCP translation per tick at full stick, in mm.                 |
| `slow_step_deg`           | `1.0`      | Slow-mode TCP rotation per tick at full stick, in deg.                   |
| `translation_mm_per_sec`  | `150`      | Fast-mode TCP speed at full stick, in mm/s. Direct cap.                  |
| `rotation_deg_per_sec`    | `30`       | Fast-mode TCP rotation speed at full stick, in deg/s. Direct cap.        |
| `max_joint_vel_degs_per_sec` | `90`    | Fast-mode per-joint velocity cap enforced via Jacobian scaling.          |
| `tick_hz`                 | `50`       | Drain loop rate.                                                         |

## Fast mode

End-effector speed is bounded directly by `translation_mm_per_sec` and
`rotation_deg_per_sec` — the service maintains a virtual goal pose that
advances at that rate, independent of the arm's physical maximum.

Joint velocity is independently bounded via Jacobian scaling: each tick the
service computes the Jacobian at the arm's current configuration, projects
the TCP delta into joint space, and scales the TCP delta down if any joint
would exceed `max_joint_vel_degs_per_sec`. This prevents singularity whips
(near a singularity, tiny TCP motion requires huge joint motion). Damped
least-squares makes the behavior smooth right up to the singularity instead
of blowing up.

### Speed guide (fast mode)

| use case              | `translation_mm_per_sec` | `rotation_deg_per_sec` |
|-----------------------|--------------------------|------------------------|
| careful alignment     | `25`–`75`                | `10`–`20`              |
| general teleop        | `150`                    | `30`                   |
| rapid repositioning   | `250`–`400`              | `60`                   |

Both knobs are hard caps on the virtual goal's advance rate — the actual arm
tracks the goal, so TCP speed matches what you configure (bounded by the
arm's own physical capability if you set the knob higher than the arm can
move).

### Motion service attributes (fast mode)

The motion-service attributes shown above are the smoothness/interpolation
knobs the teleop pipeline consumes. The defaults in a stock motion service
(`teleop_smooth_alpha: 0.5`, no interpolation override) already match what
this module expects — override them only if you want to deviate.
