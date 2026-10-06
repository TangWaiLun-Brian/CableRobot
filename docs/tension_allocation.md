# Pose-dependent tension allocation and gravity compensation

Milestone 2 implementation plan, recorded before source changes on 2026-10-06.
Status: **IMPLEMENTED — AWAITING REVIEW**. The accepted `milestone-1` boundary is unchanged.

## Decision: an additive reference-tension API

Previous design: `allocate_tensions` finds bounded equilibrium, with exhaustive
small-system least squares and a projected large-system reference solver. Its
selection rule is not a configurable pretension objective.

New design: retain that implementation and its result/backend contracts unchanged.
Add numerical `allocate_reference_tensions(B, target, lower, upper, *,
reference_tension=0.0, weights=None, tolerance=1e-8, max_iterations=200)` and
robot-facing `solve_equilibrium_tensions(robot, state, external_load=None, *,
reference_tension=0.0, weights=None, include_gravity=True, tolerance=1e-8,
max_iterations=200)`. Scalar reference/weights broadcast to cable insertion order;
vectors have exactly one entry per cable. References are finite nonnegative tensions
and may lie outside the box. Weights are finite strictly positive diagonal entries
of W, NOT entries of W squared. Default W is identity; default reference is zero,
selecting the minimum tension norm consistent with the prescribed bounds.

Reason: introduce the milestone's secondary objective without breaking the accepted
numerical baseline, changing geometry, or confusing its old meaning of "reference".
The new solver is explicitly NumPy-only; no change to `AnalysisBackend` or
`TensionProblem`, and no implicit MATLAB selection/fallback. A future backend method
can be added after cross-backend tests, rather than making existing adapters implement
a new protocol prematurely. Numerical inputs/outputs contain no robot or hardware.

Alternatives: extending old result/problem fields would blur compatibility; replacing
the small exhaustive solver would lose a regression oracle. SciPy QP/optimization
or an external QP dependency would introduce installation/backend requirements.
Enumerating all box faces is exponential for generic many-cable robots. Use a
dependency-free dual solver, with explicit unresolved status, instead. This solver
choice and numerical certification limits are open external-review questions.

## Formulation and conventions

Reuse the existing complete generalized mapping and applied gravity:

```text
J_l = dl/dq                 B(q) = -J_l.T
applied = tau_gravity(q) + external_load
target = -applied
minimize 0.5 * ||W (t - reference)||^2
subject to B(q) t = target, lower <= t <= upper
residual = B(q) t - target = B(q) t + applied
```

Never select just translation rows for the solve. Floating rotation-coordinate
forces are covectors, not Cartesian moments at nonzero rotation; see
`conventions.md`. Gravity uses offset body COMs. `include_gravity=False` is available
when an explicitly supplied generalized load already includes it. Upper bounds may
be +infinity. Positive weights make the objective strictly convex, so a feasible
problem has a unique minimizing tension, even if multipliers are nonunique.

## Numerical method and status contract

Transform `y = W(t-reference)`, `A = B / weights`, and
`d = target - B reference`. The box transforms accordingly. For an equality dual
variable lambda, minimization of the Lagrangian gives
`y(lambda) = clip(A.T lambda, lower_y, upper_y)`. Thus the box and projected
stationarity are satisfied by construction. Solve `A y(lambda) - d = 0` using
semismooth dual Newton steps (SVD on free columns; no explicit inverses), with a
descent-gradient fallback and a bracketed monotone directional derivative search.
Dependent equilibrium rows are not discarded from the final residual.

Accept only finite, bounded candidates whose ORIGINAL full residual satisfies
`tolerance * (1 + ||target||)` and whose absolute dual pairing
`|lambda.T residual|` satisfies `tolerance * (1 + objective)` in transformed units.
The latter is a numerical dual-gap diagnostic, not an exact-arithmetic proof:
approximate feasibility and finite-difference geometry limit any optimality claim.
Recompute the final original residual; never label a clipped least-squares witness
as an optimized equilibrium merely because an iteration stalled.

If iteration/line-search progress is inconclusive, retain diagnostics and use the
unchanged foundation allocator for a bounded witness or conservative separating
certificate. A feasible fallback does not certify the secondary optimum: report
`numerically_unresolved`. Certified infeasibility is `infeasible`; arithmetic or
linear-algebra failure is `numerical_failure`; verified numerical equilibrium and
reference-objective conditions give `feasible`. Bad API inputs raise ValueError.

Return a separate result with tensions, full force/target/residual, objective,
reference/weights, lower/upper activity masks and reserves, iterations, backend
identity, multiplier/optimality diagnostics and message. Only a feasible result
provides `tension_command`; candidates for other statuses are diagnostic, not safe
hardware commands. Bound activity uses a documented tension-space tolerance and
does not substitute for a bound or residual check.

## Validation required before review

Analytic redundant/weighted equilibria, active/fixed/infinite bounds, rank-deficient
and empty systems, infeasible boxes, scale and invalid inputs, numerical failure,
iteration exhaustion, independent small-face objective checks, offset-COM gravity,
physical spatial wrench balance at rotated poses, arbitrary serial/hybrid routes,
and a nearby-pose sweep. Keep all Milestone 1 tests unchanged.

Reuse `spatial_cdpr()` in a reproducible example: equal constant 20 N baseline,
optimized tensions, all six residual coordinates, deliberately limited upper bounds,
and a small continuous translational path. Save cable-labelled tension plots, full
residual/status/reference-distance/reserve/neighbor-difference diagnostics and
single-pose geometry/force data. Inspect active-set changes and jumps. Do not add
previous-tension regularization without evidence of unnecessary switching.

## Physical and control boundaries

Rigid bodies, known pose, straight massless unilateral cables, equal tension along
each declared route, quasi-static equilibrium. This removes the MODEL static bias;
it cannot establish neutral operator feel or upward/downward effort without hardware
measurements. Friction, hysteresis, tension tracking and force reserve can still
matter. Mixed-coordinate residual norms are unit-dependent reference diagnostics.

Future concept only: measured/estimated q -> existing B(q) and gravity -> allocated
t -> validated tension command -> q_next. Real-time deadlines, sensor/motor drivers,
tracking, safety interlocks, full dynamics, stiffness/manipulability/force ellipsoids,
global workspace optimization, cable sag/mass/elasticity/slack and friction remain
deferred. No closed hardware loop or experimental results are implemented here.

## Example use and observed local behavior

```python
from cablerobot import solve_equilibrium_tensions
from cablerobot.examples import spatial_cdpr

robot, state = spatial_cdpr()
result = solve_equilibrium_tensions(robot, state, reference_tension=20.0)
if result.feasible:
    command = result.tension_command  # model output, not safety authorization
    reserves = (result.lower_margin, result.upper_margin)
```

For per-cable selection pass vectors to `reference_tension`/`weights`. The low-level
API accepts a required force; `external_load` on the robot wrapper is APPLIED force.
The inherited `residual`/`residual_norm` fields contain the complete equilibrium
residual; `equilibrium_feasible` distinguishes a feasible fallback candidate from
verified reference-objective status. `duality_gap`/`dual_multiplier` may be None for
fallback or failed solves. `objective_value` is the objective of the returned
candidate when finite, NOT an optimum claim for infeasible/unresolved status.

Executed `python -W error -m examples.equilibrium_allocation --output-dir
examples/output/milestone_2` with Agg on 2026-10-06. Origin equal-20-N full residual
norm: 19.63274885799; optimized: 1.480972598e-14. All six force equations, including
offset-COM gravity moments, participate. The rotated pose is also feasible. Limiting
all upper bounds to 2 N certifies infeasibility; its returned bounded candidate is
not a command or necessarily a minimum-residual solution.

The 81-sample path translates `[0.08s, 0.024s, -0.016s]` m for s in [-1,1], with
orientation fixed. All samples are feasible. Equal baseline residual norms range
19.32969727–20.73411225; maximum optimized residual is 3.897851374e-14. Maximum
neighbor cable change is 0.04695938 N for a translation step 0.00212603 m. Minimum
lower/upper cable reserve is 11.48669485 / 73.24088603 N. No active-set changes or
large sampled jumps occurred: no previous-tension regularizer is warranted by this
path. This is a local sampled observation, not a global smoothness or real-time claim.

`equilibrium_report.json` contains q, lengths, route directions, B, gravity, both
residuals/tension vectors, bounds/status/objective/dual diagnostics, activity/reserves
and neighbor differences. `equilibrium_sweep.png` labels all eight cables and plots
the requested diagnostics. Outputs are ignored and not committed.

The routed serial/hybrid demonstration configurations are infeasible under gravity
alone with their existing cables/bounds. Generic tests use known feasible applied
loads constructed from bounded tension witnesses, retain both outer-frame/on-robot
routes, and check ALL joint/platform coordinates. They do not misrepresent these
particular geometries as gravity-supporting hardware designs.

Review questions: suitability of the dependency-free dual solver and its budget/
numerical gap criterion for future use; sufficient diagnostics before control
integration; whether a future implemented backend merits a separate QP protocol.
Finite-difference and mixed-coordinate conditioning remain limitations. Tolerance
and input scaling may lead to unresolved results; no hard real-time guarantee.

## Verification at the review boundary

2026-10-06, Windows; Python 3.12.13, NumPy 2.1.0, Matplotlib 3.9.2, pytest 7.4.4.
Accepted baseline: 132 tests passed in 9.87 s. New cases: 61; current total: 193.

Repository-root commands (using `C:\Users\thisi\anaconda3\python.exe`):

```powershell
python -m pytest -q -W error tests/test_reference_allocation.py tests/test_equilibrium_example.py tests/test_statics.py tests/test_review_regressions.py tests/test_solvers_backends_io.py --basetemp=.pytest_tmp_m2_targeted_final
python -W error -m examples.equilibrium_allocation --output-dir examples/output/milestone_2
python -W error -m examples.spatial_cdpr --output examples/output/milestone_2/spatial_reference.png
python -W error -m examples.serial_robot --output examples/output/milestone_2/serial_reference.png
python -W error -m examples.animate_robots --duration 0.3 --fps 10 --format gif --output-dir examples/output/milestone_2/animations
python -m pytest -q -W error --basetemp=.pytest_tmp_m2_full_final
```

Targeted: **144 passed in 7.08 s**. All examples exit 0 (Agg); each GIF verified as
3 frames / 300 ms with infinite loop metadata. Full source: **193 passed in 12.04 s**.
No failed/skipped tests or warnings reported; warnings were errors. The sweep plot
was visually inspected. An additional 100 seeded four-cable problems all matched
the independent face oracle; maximum objective difference 6.59e-12.

Offline `pip wheel . --no-deps --no-build-isolation` and wheel installation in the
dedicated task-output QA environment succeeded. From OUTSIDE the repository,
`python -m pytest C:\Users\thisi\OneDrive\Desktop\CableRobot\tests
--import-mode=importlib -q -W error --basetemp=<task>/work/milestone_2/pytest_installed`
passed **193 tests in 12.15 s**. Package import was verified in that environment's
`Lib/site-packages`, not the source checkout. Tests load the canonical example
script without adding the checkout to sys.path; examples are not runtime wheel modules.
No system dependencies changed or downloads were needed.

MATLAB Engine detection returned false. Existing missing/stub behavior is regression
tested; no MATLAB numerical solve or hosted CI/Python 3.11/Linux run was performed.
CI now includes the new example smoke run. No MP4 rerender was needed for this
allocation-only change; prior animations remain prescribed, not tension dynamics.
Git whitespace checks pass, and the accepted core/math/backends and old tests are
unchanged. The temporary review package records exact commands and audited Git SHA.
External acceptance, a permanent Milestone 2 completion record/tag, and any later
milestone remain pending.
