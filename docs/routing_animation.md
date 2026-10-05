# Serial and hybrid routing animations

The common route model works for cables crossing from a fixed outer frame onto an
articulated robot, and for cables lying entirely on moving robot bodies. There is no
special base/platform geometry path. The original two-link example remains available;
`serial_routed_mechanism()` adds a dedicated three-link routing example.

## Demonstration models

| Model | Generalized coordinates | Cables | Motion |
| --- | --- | --- | --- |
| Spatial | 6 floating coordinates | 8 | Floating platform translation and rotation |
| Serial | 3 revolute joints | 4 | Three-link arm attached to a fixed mount |
| Hybrid | 6 floating + 3 revolute | 12 | Floating platform and its mounted three-link arm |

The hybrid retains all eight platform cables and adds the same four arm routes as
the serial example. The arm and its cable guides move with the platform.

| Arm cable | Declared route |
| --- | --- |
| `outer_to_tip` | outer feed → link 1 upper guide → link 2 upper guide → link 3 end |
| `outer_to_link2` | outer return → link 1 lower guide → link 2 end |
| `on_robot_routed` | start on link 1 → link 1 lower guide → link 2 lower guide → finish on link 3 |
| `on_robot_direct` | link 1 upper guide → finish on link 3 |

An orange route has no fixed-world attachment. Under a common rigid rotation of the
whole arm about the shoulder, its world points move but its total length stays fixed.
Elbow/wrist articulation changes its relative geometry and length. In the hybrid,
all six platform coordinates similarly leave onboard route lengths unchanged.
Outer-frame routes generally respond to both articulation and shared platform motion.
Routes wholly on one body remain constant under any robot configuration.

These are straight segments through body-fixed eyelets or declared guide points,
with equal tension in every segment. Pulley tangency, wrapping, contact, sag and
slack mechanics are not part of this routing approximation.

## Run and reproduce

After installation, from the repository root:

```sh
python -m examples.animate_robots --duration 5 --fps 20 --format gif
python -m examples.animate_robots --duration 5 --fps 20 --format both --ffmpeg /path/to/ffmpeg
python -m examples.animate_robots --robot hybrid --output-dir examples/output/hybrid
```

GIF uses Matplotlib's Pillow writer. MP4 uses the optional FFmpeg writer with H.264
and yuv420p. The default output directory is `examples/output/animations`. Each
selected robot gets a model JSON description and an individual animation; selecting
all robots also produces `three_robots.gif` and/or `three_robots.mp4`.

`motion_traces.json` stores timestamps, q, qd, cable names, route frames, classification
and cable lengths for every frame. It explicitly records `dynamics_solved: false`.
GIF timing is quantized by the format to 10 ms increments; the default 20 fps has
exact 50 ms frames. Export rejects rates that cannot preserve the requested frame
duration (for example 30 or 120 fps); use a representable rate or MP4. Timeline
validation uses explicit absolute tolerances. Exports replace the destination only
after successful encoding, preserving existing files on failure. H.264/yuv420p MP4
frames are padded to even pixel dimensions when needed.

## Kinematic simulation boundary

`sinusoidal_trajectory` samples one smooth periodic cycle:

```text
q(t) = q0 + A * [sin(2*pi*t/duration + phase) - sin(phase)]
qd(t) = A * (2*pi/duration) * cos(2*pi*t/duration + phase)
```

Each animation has 100 frames for five seconds at 20 fps, sampling t=0 through 4.95 s.
The endpoint at 5 s is omitted so a loop does not duplicate its first frame.
`AnimationPanel` holds the robot, trajectory, selected display frames and local
body edge geometry. `animate_robot` and `animate_robots` update complete cable
polylines and rigid-body graphics while keeping the camera limits fixed over motion.

The motion is prescribed and assumes ideal variable-length cable actuation. It is
not a tension-driven forward-dynamics rollout. No equilibrium feasibility at every
frame, collision avoidance, actuator limits, contact or slack behavior is implied.
Colors identify attachment topology rather than computed tension.

## Verification

The original routing expansion passed **87 tests in 5.64 seconds** on Python 3.12.13.
This is historical validation, not external milestone acceptance. Current correction
results belong in the pending `.review/` handoff until acceptance. Those original tests
cover both route classes, guide motion, summed segment lengths, shared-motion
invariance, articulation response, same-body routing, Jacobian directional checks,
virtual work, intermediate-guide body wrench contributions and JSON round trips.
Animation tests verify trajectory derivatives, nondegenerate routes, changing cable
lengths in all three topologies, rendered world-space polylines and GIF frame timing.
