# Milestone 1 — Generic foundation and routed animations

## Status

Completed — accepted on 2026-10-06 after the external review's minor documentation
amendment. No Milestone 2 implementation has begun.

## Git references

- Base snapshot: `7683e77961284f62b9d4f2d19c45bd0b6a6a7715`.
- Reviewed implementation/correctness fixes: `d389c8e2847a6fdb62aff0138702c8d447f169b3`.
- Amendment starting reference: `59cf377` (portable review rules).
- Final accepted commit: `milestone-1^{commit}`.
- Acceptance tag: `milestone-1`.

The acceptance tag identifies the commit containing this record and the final
documentation amendment. Resolve its exact SHA with
`git rev-parse 'milestone-1^{commit}'`; an in-commit literal self-hash is not used.
Do not move the acceptance tag. Nothing was pushed or published.

## Objective

Establish a standalone, generic Python framework in which a cable connects an
ordered route of arbitrary body-attached frames in a multibody system. Validate
the geometric, generalized-force and bounded static-equilibrium layer before
adding control-oriented tension allocation or general dynamic simulation.

## Implemented capabilities

- Tree/forest models with fixed, revolute, prismatic and floating joints; named
  body frames, offset frames, arbitrary attachments and multi-segment cable routes.
- Common body/frame transforms, route lengths, centralized numerical cable and
  point Jacobians, selected-frame twists and selected-body cable wrenches.
- Generalized gravity at offset COMs, static residuals and bounded equilibrium
  tension allocation; local length/configuration and frame position/pose solvers.
- Bounded force-set vertices, rank/basic manipulability, pointwise workspace
  analysis and versioned JSON round trips.
- Validated externally supplied DynamicsProvider mass/bias interfaces and static
  gravity special cases; built-in general multibody dynamics remain deferred.
- NumPy numerical backend; lazy MATLAB detection with explicit interface-only
  errors, not an implemented MATLAB numerical solver.
- Static plotting and prescribed spatial, three-link serial and hybrid animation.
  Serial/hybrid examples demonstrate both outer-frame-to-robot routing and cables
  lying entirely on moving robot bodies, using common geometry.
- Regression tests, contribution/CI instructions, Git history, ignored temporary
  review/output files and risk-focused portable-review rules.

## Architecture

Python owns topology, model semantics, validation and serialization. Geometry is
shared by kinematics, statics, analysis and visualization. Numerical allocators
consume arrays, not a second robot model. A directed tree/forest provides body
motion; cable routes may cross arbitrary bodies and are not limited to a classical
single base/platform CDPR. Fixed world roots and floating joints remain explicit.

The foundation retains the exhaustive active-set path for at most eight cables
and projected least squares for larger problems. It is a **bounded equilibrium /
reference numerical allocator**: "reference" means a numerical validation baseline,
not a reference tension vector. Small-problem internal comparisons prioritize
residual norm, then tension norm; they do not constitute a configurable global
`min ||t - t_ref||_2^2` interface or a guaranteed preferred distribution.

The public allocator and TensionProblem expose required generalized force, unilateral
bounds and tolerance, not `t_ref`. Single-configuration gravity-inclusive equilibrium
is implemented; configurable reference/pretension objectives and a pose-dependent
gravity-compensating control/allocation workflow remain future, separate work.

## Mathematical conventions

SI units; right-handed world with default gravity `[0,0,-9.81]`; `T_A_B` maps B into
A; joint insertion-order generalized coordinates; floating coordinates
`[x,y,z,rx,ry,rz]` use exponential-map rotation vectors. Generalized rates are not
angular velocities in the floating rotation block. World twist/wrench ordering is
linear then angular, about an explicitly selected frame origin.

`J_l = dl/dq`, `l_dot = J_l @ qd`, `B = -J_l.T`, and virtual work is
`tau_cable @ qd = -tensions @ l_dot`. Positive tension pulls adjacent route endpoints
together; equal segment tensions add forces at intermediate guides. Gravity is an
applied force summed from COM Jacobians. Equilibrium is
`B @ t + tau_gravity + external_load = 0`.

Feasibility uses `tolerance * (1 + norm(target))`, default `1e-8`. Infeasibility
requires a conservative separating support-function certificate; stalled solves
without a witness/certificate are numerically unresolved. Numerical failure is
separate. Transform/symmetry invariants retain their absolute tolerances with
`rtol=0`. No convention or numerical tolerance changed in the closure amendment.

## API changes and compatibility

The prior hardening revalidates finite state/load/tension and solver input,
topology/reference integrity, coordinate blocks and canonical body-frame ownership;
defines empty cable-Jacobian rank as zero; and rejects unsupported MATLAB selection.
Animation exports preserve existing destinations on encoder failure, enforce
representable GIF timing, synchronize timelines and pad MP4 to even dimensions.
Invalid inputs formerly accepted may now raise explicit errors.

The accepted minor amendment changes documentation only: no source/test API,
allocator algorithm, objective, exhaustive path or tolerance was modified. No
artificial code changes or regression tests were added for this wording correction.

## Tests

Historical progression: initial foundation 67 tests; routed expansion 87; correctness
review added 45 cases for 132. Before/after this minor amendment: 132 / 132, new tests 0.

Final closure environment: Windows, Python 3.12.13, NumPy 2.1.0, Matplotlib 3.9.2,
pytest 7.4.4. MATLAB Engine was unavailable. Missing-engine and installed-stub
behavior is tested with mocks, not MATLAB numerical execution. Hosted CI and its
Python 3.11/Linux matrix were not executed locally.

Targeted command, repository root:

```powershell
& 'C:\Users\thisi\anaconda3\python.exe' -m pytest -q -W error tests/test_statics.py tests/test_review_regressions.py tests/test_solvers_backends_io.py --basetemp=.pytest_tmp_foundation_targeted_2026-10-06
```

Result: **83 passed in 2.52 s**, zero failed/skipped/warnings reported.

Full command, after the relevant examples:

```powershell
& 'C:\Users\thisi\anaconda3\python.exe' -m pytest -q -W error --basetemp=.pytest_tmp_foundation_full_2026-10-06
```

Result: **132 passed in 7.52 s**, zero failed/skipped/warnings reported.

Runtime API inspection confirmed neither allocator signature nor TensionProblem
fields expose `t_ref`. No dedicated documentation-test builder is configured;
Git whitespace checks and documentation/API consistency inspection were used.
Source and tests are byte/content unchanged by the amendment (`git diff --exit-code
59cf377 -- cablerobot tests`); no test tolerances were weakened.

Earlier, on 2026-10-05, the reviewed source suite passed 132 tests in 8.53 s and
an offline-built installed wheel passed 132 tests in 8.29 s outside the checkout.
Those installed-wheel results are historical, not fresh closure-run results.

## Examples and validation

Executed on 2026-10-06 from the repository root with `MPLBACKEND=Agg`:

```powershell
& 'C:\Users\thisi\anaconda3\python.exe' -m examples.spatial_cdpr --output examples/output/foundation_closure_2026-10-06/cdpr.png
& 'C:\Users\thisi\anaconda3\python.exe' -m examples.serial_robot --output examples/output/foundation_closure_2026-10-06/serial.png
& 'C:\Users\thisi\anaconda3\python.exe' -m examples.animate_robots --duration 5 --fps 20 --format gif --output-dir examples/output/foundation_closure_2026-10-06/animations
```

All completed with exit 0. Spatial Jacobian rank 6, gravity-equilibrium residual
`9.488510541505395e-15`, overload correctly infeasible; serial reference transforms,
lengths, Jacobian and generalized forces retained.

Spatial, serial, hybrid and combined GIFs each have 100 frames and 5 s at 20 fps,
50 ms/frame and infinite loop metadata. All 300 robot states were checked against
serialized models/traces: lengths match, segments are nondegenerate, all cable
lengths vary and serial/hybrid retain both routing classes. Motion is prescribed
kinematics, not a dynamic, collision or tension-feasibility demonstration.

Historical MP4 verification on 2026-10-05: all four H.264 files had 100 frames/5 s;
an odd-dimension 30 fps case padded 401x301 to 402x302, 3 frames/0.1 s. MP4 was not
rerendered for this documentation-only amendment. Generated outputs are ignored.

## Review findings and disposition

The external feedback file `cablerobot_foundation_review_feedback.md` reports
**ACCEPT WITH MINOR CORRECTION**, without a blocking mathematical/architectural
defect. Its clarification is **accepted**: current allocation means bounded static
equilibrium/reference numerical implementation, not configurable reference-tension
tracking. README, architecture, backend, convention and validation descriptions
were clarified; already-correct limitations and module docstrings were retained.

Previously corrected findings retained in the accepted implementation:

- False infeasibility from stalled projected iterates, including a scaled feasible
  nine-cable witness: require separation, otherwise report unresolved.
- Relative-tolerance leakage in transform/inertia/mass symmetry checks: preserve
  absolute tolerances and explicitly disable relative relaxation.
- State/topology and physical/API input validation, canonical body-frame semantics,
  empty rank and explicit optional-backend errors.
- Encoder failure safety, precise GIF timing, timeline checks and odd-pixel MP4.
- Git/review/contribution/CI discipline and historical-versus-accepted status.

No feedback recommendation was rejected or used to introduce a new feature.
Configurable `t_ref` tracking, continuity/control/stiffness/manipulability optimization,
real-time allocation and hardware remain explicitly deferred, not implemented.

## Deviations from the original plan

Centralized finite-difference Jacobians replace specialized analytic/automatic
derivatives in this foundation. NumPy is the working backend; MATLAB remains an
adapter boundary. General multibody dynamics and integration are not implemented.
Routing/kinematic animation and correctness hardening were completed within the
current scope before acceptance. The documentation amendment does not redesign it.

## Known limitations

Poor conditioning may leave allocation unresolved; the reference solver provides
no configurable reference-tension or globally preferred-distribution objective.
Mixed translation/rotation residual and singular-value metrics are unit-dependent.
Local IK requires a suitable guess and has rotation-log branch limits. Models
remain mutable and do not provide arbitrary concurrent-mutation safety. Workspace
analysis is pointwise. Routes are straight, massless segments through declared
body-fixed guides, without realistic pulley/contact/slack/friction mechanics.
Static labels may overlap at coincident origins. Animation colors indicate topology.

## Deferred work and next-milestone considerations

Future Milestone 2 may deliberately introduce pose-dependent gravity-compensating
allocation/control with a configurable reference/pretension objective, including
`min ||t - t_ref||_2^2` subject to equilibrium and unilateral bounds. That work is
not authorized or implemented by this amendment. Preserve this small-system
allocator as a validation baseline rather than overwriting its exhaustive path.

Also deferred: built-in full multibody M/h and dynamic integration, stiffness and
advanced workspace/optimization analyses, closed loops, flexible links, motor and
hardware communication, and cable mass/sag/elastic/slack/pulley/friction/contact physics.

## Closure

The accepted documentation/code boundary is tagged `milestone-1`. The temporary
`.review/` handoff is removed after validation and recording this Git boundary;
`.review/` remains ignored, and this permanent record is retained. The untracked
user-provided ZIP was moved outside the repository with explicit approval and
SHA256 verification, not deleted, committed or repackaged as the accepted state.
Final checks verify the clean working tree and acceptance tag. No next milestone
has started; further work requires a separate instruction.
