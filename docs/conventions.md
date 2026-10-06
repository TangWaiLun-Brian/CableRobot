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
Same-named body frames cannot be repurposed: add a separately named offset frame.
Transform invariants use absolute tolerances (`rtol=0`), not relative relaxation.

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
This is the foundation bounded static-equilibrium allocation. Its reference numerical
implementation does not expose a configurable `t_ref` or minimize a user-specified
`||t - t_ref||_2^2` objective; its existing internal solution-selection criteria
are not a reference-tension control interface. These APIs are retained unchanged.

Milestone 2 adds `solve_equilibrium_tensions`, with the same gravity/load signs,
and low-level `allocate_reference_tensions` with the same REQUIRED-force target.
They minimize `0.5 * ||W(t-t_ref)||^2` under FULL generalized equilibrium and bounds.
References are finite nonnegative scalar/per-cable tensions (N), in insertion order;
default zero. W is diagonal with strictly positive finite scalar/per-cable weights,
default identity. Inputs supply diagonal W, not W squared. Weights are relative,
dimensionless penalties. References may be outside bounds without changing bounds.
Do not confuse this objective with cable strain energy or stiffness shaping.

## Numerical conventions

Default cable/point derivatives are analytic for all currently supported tree/forest
joints and straight routes through body-fixed points. Floating angular derivatives
use the SO(3) left Jacobian of Exp(rotvec); generalized rates/signs do not change.
Each segment contributes `unit_direction.T @ (J_next-J_previous)` to dl/dq.
See `performance_study.md` for the derivation and validated scope.

The retained finite-difference reference uses a centered step of `1e-6` in each coordinate, configurable
through `RobotParameters`. This mixes metres and radians by coordinate type. Tests
compare to independent analytic or five-point results with explicit tolerances.
Select `method='finite_difference'` explicitly; supplying a low-level `step` also
selects the reference. An explicit analytic method with a step is rejected.
`solve_configuration_from_lengths(..., jacobian_method='finite_difference')` and
`gravity_generalized_force(..., jacobian_method='finite_difference')` retain reference
derivatives; default FK warm-start/residual tolerances and gravity semantics are unchanged.
Zero-length segments are accepted for length reporting but rejected for direction,
Jacobian and wrench computations. Local solvers use damped Gauss-Newton with
backtracking. Pose rotation weight has units metres per radian relative to translation.

Allocation is feasible when residual norm is no more than
`tolerance * (1 + norm(target))`; default `tolerance=1e-8`. Infeasibility requires a
separating hyperplane: for a unit residual direction `w`,
`w @ target - support(B [lower, upper], w)` must exceed the accepted residual
threshold and a floating-point guard. The support chooses upper bounds for positive
`B.T @ w` coefficients and lower bounds otherwise. An unbounded positive support
contribution or inconclusive separation cannot certify infeasibility. A stalled or
iteration-limited solve without either a feasible witness or separation is
numerically unresolved, including small problems when conditioning prevents proof.
Numerical linear-algebra failure is distinguished separately. Do not interpret an
unresolved result as proof that no physical equilibrium exists.

The Milestone 2 dual solver additionally verifies the numerical reference-objective
condition `abs(lambda @ residual) <= tolerance * (1 + objective_value)`, with
`t = clip(t_ref + B.T @ lambda / weights**2, lower, upper)` satisfying projected
stationarity. This pairing is a numerical dual-gap diagnostic, not a proof of exact
optimality with inexact equilibrium. Final residuals always use the original full B.
An independently feasible foundation fallback is unresolved for the new objective.
Only new results with status `feasible` expose a `tension_command`; other tensions
are diagnostic candidates. Activity masks use `tolerance * (1 + abs(t_i))` N;
reserves are `t-lower` and `upper-t` in N, +infinity for an unbounded upper reserve.
See `tension_allocation.md` for algorithm, limitations and control boundaries.
