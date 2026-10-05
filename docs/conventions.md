# Mathematical and physical conventions

All distances are metres, masses kilograms, time seconds, cable tensions newtons,
angles radians and Cartesian moments newton-metres. Generalized forces use newtons
for translational coordinates and newton-metres for angular coordinates. Mixed-coordinate
norms and singular-value metrics are unit-dependent; they are reference diagnostics.

## Frames and rigid transforms

World is a right-handed inertial frame. Default gravity is world `[0, 0, -9.81]`.
`T_A_B` maps coordinates expressed in B into A:
`p_A = R_A_B @ p_B + translation_A_B`. Composition is `T_A_C = T_A_B @ T_B_C`.
Rotations are proper 3-by-3 matrices. The homogeneous last row is `[0, 0, 0, 1]`.

A frame is rigidly attached to one body by `T_body_frame`. Each body has a same-named
identity frame. Root body transforms are identity in world. Multiple fixed roots are
allowed; offset world anchors can be declared through frames or fixed joints.

For a parent/child joint:

```text
T_world_child = T_world_parent @ T_parent_joint @ motion(q_joint) @ T_joint_child
```

The axis of a revolute/prismatic joint is expressed in the joint frame and normalized.
Positive revolute motion follows the right-hand rule. Positive prismatic motion
translates along that axis. `T_joint_child` is a constant post-motion offset.

## Generalized coordinates

Joint blocks follow insertion order. Fixed joints have no coordinates, revolute and
prismatic joints have one, and floating joints have six:
`[x, y, z, rx, ry, rz]`. Floating translation is in the parent joint frame; rotation is
the exponential map of a rotation vector, `R = Exp([rx, ry, rz])`. No quaternions are used.

Rotation-vector rates are **not** angular velocity at nonzero rotation. Consequently
the last three floating generalized-force entries are covectors conjugate to rotation
coordinates and are not generally Cartesian moments. Use `frame_twist_jacobian` and
`cable_wrench_matrix` when comparing to physical wrenches. The principal SO(3) logarithm
is used for pose residuals; behavior near the pi branch boundary remains a local-solver
limitation.

`RobotState.qd` uses the same order and means generalized-coordinate rates. A frame
twist is `[vx, vy, vz, wx, wy, wz]` in world axes, with linear velocity at the frame
origin. A wrench is `[Fx, Fy, Fz, Mx, My, Mz]` in world axes, about the selected origin.

## Cables and signs

A declared route is an ordered sequence of frame-local points. Segment direction
is from route point j to j+1. Length is the sum of Euclidean segment lengths. This
route specifies fixed material attachment/via points on bodies; it does not model
wrapping, pulley tangency, friction, moving contact or cable slack.

Positive tension pulls the first endpoint of each segment toward its second, and
pulls the second endpoint toward its first. The same tension acts in every segment
of a route. At an intermediate point, the forces from both adjacent segments add.
Route reversal preserves length and generalized-force mapping.

The cable Jacobian is `J_l = dl/dq`, shape `(number_of_cables, dof)`, with
`l_dot = J_l @ qd`. The generalized-force matrix is explicitly:

```text
B = -J_l.T
tau_cable = B @ tensions
tau_cable @ qd = -tensions @ l_dot
```

Gravity is an **applied** generalized force:
`tau_gravity = sum(J_com.T @ (mass * gravity_world))`. Centers of mass are expressed
in each body frame and need not be at its origin.

Static equilibrium is `B @ tensions + tau_gravity + external_load = 0`.
`solve_tension_allocation` includes gravity by default. `external_load` is an additional
generalized force; use `include_gravity=False` if it already includes gravity.
The low-level `allocate_tensions` instead accepts a **required** generalized force,
and solves `B @ tensions = target` within unilateral bounds.

## Numerical conventions

The reference Jacobian uses a centered step of `1e-6` in each coordinate, configurable
through `RobotParameters`. This mixes metres and radians by coordinate type. Tests
compare to independent analytic or five-point results with explicit tolerances.
Zero-length segments are accepted for length reporting but rejected for direction,
Jacobian and wrench computations. Local solvers use damped Gauss-Newton with
backtracking. Pose rotation weight has units metres per radian relative to translation.

Allocation is feasible when residual norm is no more than
`tolerance * (1 + norm(target))`; default `tolerance=1e-8`. Infeasible means the small
active-set search or converged box least-squares optimum cannot satisfy that tolerance.
An iteration-limited large problem is numerically unresolved. Numerical linear-algebra
failure is distinguished separately. Do not interpret an unresolved result as proof
that no physical equilibrium exists.
