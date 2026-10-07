# Best-effort study 1 — Bounded allocation and FK warm start

## Status

Completed — external **ACCEPT** received from the user on 2026-10-07, with **no
required code amendment**. Acceptance closes this study, not hardware, dynamics,
real-time certification or a subsequent milestone. Closure is documentation-only.

## Git references

- Accepted base: `469e62e04672e8af35e28a3a3301683151a8bea7`
  (`milestone-2` / `performance-study-1`).
- Initial implementation/FK measurement: `92b51c066b87747ac0fc5742111e89b12c7fe9df`.
- Refined numerical implementation/tests: `924f7160478dd2c01e26c1cc63f67f8eaf42142f`.
- Reviewed complete handoff: `8d0660555e69ef80095cf68eae8a1b0e4658bb11`.
- Final accepted commit: `best-effort-study-1^{commit}`.
- Acceptance tag: `best-effort-study-1`.

Resolve the final SHA with `git rev-parse 'best-effort-study-1^{commit}'`; no
literal in-commit self-hash. The annotated tag identifies the documentation-only
closure containing this permanent record. Earlier acceptance tags remain
unchanged. No tag is moved, pushed or published.

## Objective and implemented capabilities

Provide a bounded minimum-residual static tension proposal when exact equilibrium
is impossible, while retaining accepted exact/reference behavior. Separately
measure continuity-based FK start policies without replacing production FK.

- Array `allocate_best_effort_tensions` and robot-facing
  `solve_bounded_equilibrium_tensions`, with mandatory explicit residual scaling.
- Separate `BoundedAllocationStatus`/`BoundedTensionResult`: exact FEASIBLE versus
  verified BEST_EFFORT, unresolved, failure and exact-only infeasible semantics.
- Full original generalized residual, achieved/required forces, weighted/raw norms,
  objectives/reference distance, active bounds/margins, scaling and diagnostics.
- Explicit spatial characteristic-length helper with wrench-to-generalized map;
  no six-DOF assumption in the core, and no covectors mislabeled as moments.
- Strictly primary residual minimization, then reference selection at the same
  achieved-force image, numerically verified at separately declared tolerances.
- Six-virtual-second, 61-pose feasibility-boundary example with bounded commands,
  residual directions, activity masks and plots; prescribed statics, not dynamics.
- Developer-only previous-state/constant-velocity FK comparison, branch/history
  guards, previous-pose retry and complete iteration/latency accounting.

## Architecture and mathematical conventions

Python owns the generic body/joint/frame/cable-route model. The new robot API
assembles canonical B, COM gravity and bounds; no alternate geometry or persistent
cache is introduced. Existing exact allocators, statuses, backend contracts,
coordinates, FK core, derivatives and all previous tests are unchanged.

With `B=-J_l.T`, target is `-(gravity+other_applied_load)` and residual is
`r=B*t-target`. Primary problem: `min 0.5*||S*r||²` over original bounds, with
explicit finite invertible S. A positive scalar/diagonal vector or explicit
`ResidualScaling` map is accepted. Spatial scaling uses characteristic length
Lc and actual wrench-to-generalized map G:
`S=diag(1,1,1,1/Lc,1/Lc,1/Lc)*inverse(G)`. Generalized angular covectors at nonzero
rotvec are not Cartesian moments. All original residual rows/units are retained.

Convex projection gives a unique achieved force image even if tensions are not
unique. Reuse accepted reference allocation to minimize `0.5*||W*(t-t_ref)||²`
at that SAME image. No arbitrary residual/reference penalty. Verify projected KKT,
guarded convex box dual gap, secondary optimum and scaled force-image preservation.
BEST_EFFORT also needs conservative original-space separation beyond the unchanged
exact tolerance `tolerance*(1+norm(target))`. Inconclusive optimization/separation
is unresolved, not infeasible; arithmetic failure is distinct. Numerical checks
are not exact-arithmetic proofs. Only FEASIBLE/BEST_EFFORT expose copied proposed
commands, neither of which grants hardware authority.

An inexpensive bounded feasible witness can avoid primary screening, but is never
itself a command. A certified impossible target bypasses its exact QP and reuses
the projected primary candidate. Otherwise the unchanged original-target reference
QP selects the exact solution. Exact-only mode skips screening. Existing projected
LS and small-system exhaustive fallback are reused, not reimplemented.

See [decisions recorded before code/refinement](../best_effort_allocation.md),
[numerical/physical conventions](../conventions.md) and
[results / all 15 review answers](../best_effort_results.md).

## API compatibility and FK disposition

APIs/results/exports are additive; old exact callers keep their original semantics.
Primary and secondary numerical tolerance is distinct from physical residual
magnitude and exact equilibrium acceptance. No dependency or backend change.

**Previous successful state remains the production FK warm-start default.**
Constant-velocity `2*q_previous-q_older` remains a **developer experiment only**,
requiring two consecutive successful estimates at equal intervals. Conservative
near-pi/rotation-jump/nonfinite guards revert the whole start to previous-state.
Failed prediction retries previous-state and counts both attempts. No production
tracking API, global branch recovery, changed rotvec convention or weaker FK
convergence tolerance. Detailed comparison: [FK results](../fk_prediction_results.md).

## Tests and independent verification

Previous accepted suite: 266 cases, all retained unchanged. This study adds 94,
for 360 total. Independent analytic problems, augmented primal KKT active-face
oracle for 25 seeded four-cable problems and 100 random bounded competitors per
case check both objectives. Coverage includes infeasible upper/excess lower bounds,
unavailable forces, redundancy/rank, explicit mixed-unit/dense scaling, rotated
physical wrench conversion, routed serial/hybrid external/on-robot cases, empty/
fixed/infinite boxes, force scales, near infeasibility, conditioning, failures,
command-copy safety, exact bitwise compatibility and candidate/screen reuse.

Historical handoff: targeted 296 passed / 15.24 s; full source 360 passed / 18.66 s;
isolated offline installed wheel 360 passed / 18.40 s, imported from site-packages
outside repository source. The venv inherited existing dependencies, not a clean
independent dependency resolution; no global install changed. Wheel checks are
historical and were not rerun at closure because code/package configuration is
unchanged. Development corrections and exploratory coarse FK failures remain
explicitly disclosed in the result reports; no accepted tolerance was weakened.

Fresh acceptance closure, 2026-10-07: **296 targeted passed in 15.50 s**, then
relevant examples/smokes, then **360 full-suite passed in 18.47 s**. Warnings as
errors; zero failed/skipped/warnings reported. NumPy-only numerical execution;
MATLAB Engine availability false, optional missing/stub behavior covered by tests.
No hosted CI, alternate OS/Python, hardware or new benchmark-wide timing claim.

Environment: Windows 11, Python 3.12.13, NumPy 2.1.0, Matplotlib 3.9.2, pytest 7.4.4;
executable `C:\Users\thisi\anaconda3\python.exe`. Repository-root commands:

```powershell
$env:OPENBLAS_NUM_THREADS='1'; $env:MKL_NUM_THREADS='1'; $env:OMP_NUM_THREADS='1'; $env:MPLBACKEND='Agg'
python -m pytest -q -W error tests/test_best_effort_allocation.py tests/test_best_effort_study.py tests/test_reference_allocation.py tests/test_review_regressions.py tests/test_statics.py tests/test_analytic_kinematics.py tests/test_performance_benchmark.py --basetemp=.pytest_tmp_best_effort_acceptance_targeted_2026-10-07
python -W error -m examples.equilibrium_allocation --samples 81 --output-dir examples/output/best_effort_acceptance_2026-10-07/exact
python -W error -m examples.best_effort_allocation --samples 61 --output-dir examples/output/best_effort_acceptance_2026-10-07/best_effort
python -W error -m benchmarks.fk_prediction --frames 64 --repeats 1 --require-convergence --output-dir examples/output/best_effort_acceptance_2026-10-07/fk
python -W error -m benchmarks.control_pipeline --label best_effort_acceptance --frames 20 --repeats 1 --profile-frames 3 --output-dir examples/output/best_effort_acceptance_2026-10-07/pipeline
python -m pytest -q -W error --basetemp=.pytest_tmp_best_effort_acceptance_full_2026-10-07
```

## Examples and measured evidence

Fresh correctness reruns all exit 0; ignored outputs preserve earlier primary
measurements. Exact example: 81/81 equilibria, max full residual 4.400360274628181e-13;
deliberate 2 N overload stays INFEASIBLE. Best-effort path: 38 FEASIBLE, 23
BEST_EFFORT, all tensions within fixed 1–30 N. Maximum force-equivalent residual
10.887934903968173 N, scaled secondary image change 3.691730455790016e-13.
Guarded 64-pose FK smoke: all four paths/two policies converge, near-branch
extrapolation disabled. Accepted exact-only pipeline: both 20-frame workloads
have 20/20 Mode A feasible, FK converged and Mode B feasible. Closure smokes are
correctness checks, not replacement primary performance measurements.

Historical refined best-effort allocation mean/p95/max: **23.153 / 40.384 /
41.667 ms**, 13/23 infeasible frames above 20 ms. Two screened feasible frames
also exceeded 20 ms. About 27.3x faster than initial exhaustive fallback, yet
**best-effort allocation is not yet 50 Hz validated**. Earlier accepted exact-only
pipeline headroom does NOT validate the additive fallback. No worst-case execution,
hardware acquisition/communication/scheduling or real-time guarantee.

Historical FK: 12,000 noiseless local-tracking frames, zero final/retry failures;
mean latency reductions about 41.1% translation, 40.4% small rotation, 23.7%
moderate rotation. Guards omit prediction near pi. Exploratory eight-sample
moderate-rotation cycle had two failures under EACH policy; large jumps/noisy,
irregular-sample/global tracking robustness is not established. Evidence supports
only the guarded developer reproducer, not changing the production default.

## Review findings, deviations and limitations

External verdict: **ACCEPT**, **no required code amendment**. No additional
recommendation or scope was inferred. Closure changes documentation/records and
creates a local acceptance tag, not numerical source, tests, CI or configuration.

Documented deviation: initial exact-then-exhaustive path was too slow. Reused
projected LS plus sufficient witness/original-space separation implements the
specification's permitted efficient combined feasibility/approximation path,
without relaxing exact tolerance. Unresolved small cases may still use expensive
exhaustive fallback. The primary routine retains its 10,000-iteration limit;
max_iterations applies to the reused reference QP, not primary LS.

**Nonzero residual means cables alone cannot hold the modeled static pose.** Other
support must provide `-residual`; absent that support, motion/acceleration may
occur. Operator/contact/friction/support forces and that motion are NOT modeled.
A smooth-looking proposed tension path is not proof of smooth physical motion,
neutral operator feel, stability or hardware safety. Conditioning, infinite
support or numerical budgets may remain unresolved; physical scaling is a user
choice. BEST_EFFORT is never relabeled FEASIBLE to hide residual.

## Deferred work and next-milestone boundary

Hardware/CAN/motor control, drivers/acquisition/filtering/safety, dynamics/inertia,
friction/contact/support models, stiffness/manipulability/workspace algorithms,
sag/elasticity/pulleys, MATLAB migration, validated 50 Hz fallback and production
noisy/global/irregular-sample FK tracking remain deferred. No subsequent milestone
or planning implementation is started. Stop at this accepted boundary until the
user selects and authorizes another scope.

## Closure and retained review evidence

Permanent records are retained. Temporary `.review/` is removed only after final
validation, the acceptance commit/tag and archive verification. All 19 temporary
files (11 selected full canonical key files plus index/reports) are byte-for-byte
preserved in the exported 59,200-byte review archive outside the repository:
`C:\Users\thisi\Documents\Codex\2026-09-30\files-mentioned-by-the-user-cable\outputs\best_effort_study\best_effort_review_8d06605.zip`.
SHA256: `8076dfe659de8b2dc97e1e48c29dd58f17f4ff58d1491e379a8989ba49ae63c7`.
It is a selected audit subset, not a complete repository or independently runnable
project. The original reviewed handoff/archive is not rewritten at acceptance.
Working tree clean at closure; prior tags preserved, no push/publication or global
environment installation. No further milestone begun.
