# Performance study 1 — FK and equilibrium pipeline

## Status

Completed — external **ACCEPT** received from the user on 2026-10-07, with no
required code amendment. Milestone 2 was accepted at the same time. This is a
computational study, not Milestone 3, hardware acceptance or hard-real-time certification.

## Git references

- Starting implemented Milestone 2: `d5b733d204bf382db68f44415469ed7f01ff962d`.
- Baseline benchmark: `5877baa10ea64350de512e1f2c1349827a0cd6c9`.
- Optimized numerical implementation: `b92d3c1db02053a4ec5290301c9579659de32d03`.
- Reviewed complete study: `6e280d1e968d52079fadd7022e47b03b2c75b906`.
- Final accepted commit: `performance-study-1^{commit}`.
- Acceptance tag: `performance-study-1`.

This tag and `milestone-2` identify the same combined documentation-only closure
commit containing both permanent records. Resolve its SHA with
`git rev-parse 'performance-study-1^{commit}'`; no literal in-commit self-hash.
The measured baseline/optimized revisions remain immutable and distinct. No
acceptance tag is moved, and nothing is pushed/published.

## Objective and implemented capabilities

Measure the actual FK/equilibrium pipeline, then accelerate the measured dominant
path without weakening mathematics/tolerances or migrating to MATLAB.

- Reproducible developer benchmark with separate component timings and cProfile
  evidence, raw frame/status/error/objective data and environment/Git provenance.
- Separate pose-known allocation (Mode A), measured-length -> FK -> tension compute,
  and synthetic-sensor-inclusive virtual Mode B timing.
- User-reference geometry/trajectory adapted without executing or modifying the
  Downloads script; a second translated/rotated offset-COM workload.
- Generic analytic cable/point/COM derivatives for all current supported joint trees
  and straight body-fixed routes, with retained finite-difference reference paths.
- Call-local transform/frame reuse and one-q residual memoization inside local FK.
- Independent random/rotation-chart/offset-forest tests and a CI smoke command,
  without host-dependent timing assertions.

## Architecture, mathematics and API compatibility

Python retains model semantics. Geometry reuse is call-local, not a persistent
mutable-model cache. Generic endpoint derivatives propagate parent motion through
fixed/revolute/prismatic/floating joints and post-motion offsets. Floating angular
derivatives use the SO(3) left Jacobian of the existing Exp(rotvec) chart, with stable
small-angle coefficients, not coordinate rates treated as angular velocity.
For a point, `J_point=J_origin-skew(p_point-p_origin) J_angular`; for every routed
segment, sum `u.T (J_next-J_previous)`. No CDPR-only force-map shortcut introduced.

Coordinate/frame/unit/sign conventions remain: `J_l=dl/dq`, `B=-J_l.T`, COM applied
gravity and complete equilibrium. Both allocation algorithms and backend sources
are unchanged. FK preserves public callbacks, Gauss-Newton damping/backtracking,
previous-success warm starts, convergence tolerances and statuses. Only repeated
residual computation at the last q is memoized. No fused private FK or persistent
geometry cache was needed after measured headroom was achieved.

Cable/point/COM derivatives now default to analytic values. Additive method selectors
retain explicit finite differences; a supplied low-level step still selects the
numeric reference. Explicit analytic plus step raises rather than ignoring the step.
Last-bit parity with former numerical approximations is not promised. The derivation,
alternatives and compatibility decision were documented before implementation in
[performance_study.md](../performance_study.md); physical APIs/semantics are unchanged.

Support: current fixed/revolute/prismatic/floating trees/forests and arbitrary
body-fixed attachment/frame/guide routes, including serial/hybrid, reversed,
external-frame and robot-only paths. No closed loops or pulley/contact tangency.
Zero-length segment Jacobians still fail; rotation-chart rank loss at 2pi remains.

## Measured results and 50 Hz boundary

Primary measurements on 2026-10-06: same Windows/Ryzen 7 3700X, Python 3.12.13,
NumPy 2.1.0 and identical 500-frame paths/settings; BLAS/OpenMP threads explicitly 1.
No affinity/priority/pacing/hardware assumed. Unprofiled headline timings exclude
profiling/JSON/plotting; measured-length compute excludes synthetic lengths, not
actual acquisition latency. First FK guess equals true first pose; local warm tracking.

Practical measured-length -> FK -> tension: mean **139.5164 -> 5.8601 ms**,
median 138.5829 -> 5.5540 ms, p95 **148.7495 -> 7.4983 ms**, maximum
**163.5344 -> 9.5400 ms**, 500-cycle compute total 69.7582 -> 2.9301 s (~23.81x).
Reciprocal-mean numerical throughput 7.17 -> 170.64 Hz, not paced control frequency.
Pose-known allocation mean 43.7093 -> 2.4478 ms. Rotated workload compute mean
141.8858 -> 5.7632 ms, p95 6.5902 ms, maximum 8.2221 ms after optimization.

Before, Jacobian/repeated geometry consumed ~85% cumulative profile time; FK ~66%,
robot allocation ~32%, pure QP <1%. After, FK ~53%, combined analytic geometry ~42%,
pure QP ~17% of a much shorter run. These shares overlap and are NOT additive;
profiling overhead affects cheap functions. Independently timed pure QP remains
~1.1 ms. FK-only length geometry evaluations 900 -> 146, Jacobian calls 58 -> 58;
full-path Newton iterations 998 -> 998 per workload. No convergence weakening.

All Mode A/FK/Mode B frames succeed before/after; zero status mismatches, max tension
differences <1.5e-7 N, and every pose has rank 6 at threshold 1e-8. All 1,000 optimized
compute frames are <20 ms. Practical p95 leaves ~12.50 ms and observed maximum
10.46 ms for other work. This supports numerical 50 Hz headroom on this machine,
NOT worst-case execution, hardware latency, production safety or hard-real-time behavior.
Full component/count/error data and commands remain in
[performance_results.md](../performance_results.md); closure does not rewrite them.

## Tests and historical verification

Before study: 193 cases. Added 6 benchmark and 67 analytic cases (73 total), leaving
all previous cases/tolerances unchanged. Independent five-point/central differences
cover 200 random generic configurations plus zero/pi/2pi rotvec and offset forests;
virtual work, COM gravity, FK singular/failure/recovery, mutable-model freshness,
reference-objective/bounds/status parity are tested.

Historical 2026-10-06: targeted 239 passed in 13.29 s; full source 266 passed in
16.83 s; installed wheel 266 passed in 16.45 s outside the checkout, warnings as
errors. The wheel venv inherited existing dependencies; pip check reported unrelated
datashader 0.19.1 -> missing numba. Imports/full tests passed; no unrelated dependency
repair or global environment modification. Wheel checks were not rerun at closure.
An external script still needs the updated package in its own Python interpreter.

## Acceptance closure validation

Fresh 2026-10-07, repository root, Anaconda Python 3.12.13, NumPy 2.1.0,
Matplotlib 3.9.2, Windows; Agg for examples. Actual commands:

```powershell
& 'C:\Users\thisi\anaconda3\python.exe' -m pytest -q -W error tests/test_reference_allocation.py tests/test_equilibrium_example.py tests/test_statics.py tests/test_review_regressions.py tests/test_solvers_backends_io.py tests/test_analytic_kinematics.py tests/test_performance_benchmark.py --basetemp=.pytest_tmp_acceptance_targeted_2026-10-07
& 'C:\Users\thisi\anaconda3\python.exe' -W error -m examples.equilibrium_allocation --output-dir examples/output/acceptance_2026-10-07/equilibrium
& 'C:\Users\thisi\anaconda3\python.exe' -W error -m examples.spatial_cdpr --output examples/output/acceptance_2026-10-07/spatial.png
& 'C:\Users\thisi\anaconda3\python.exe' -W error -m examples.serial_robot --output examples/output/acceptance_2026-10-07/serial.png
& 'C:\Users\thisi\anaconda3\python.exe' -W error -m examples.animate_robots --duration 0.3 --fps 10 --format gif --output-dir examples/output/acceptance_2026-10-07/animations
$env:OPENBLAS_NUM_THREADS='1'; $env:MKL_NUM_THREADS='1'; $env:OMP_NUM_THREADS='1'
& 'C:\Users\thisi\anaconda3\python.exe' -W error -m benchmarks.control_pipeline --label acceptance_smoke --frames 20 --repeats 1 --profile-frames 3 --output-dir examples/output/acceptance_2026-10-07/performance
& 'C:\Users\thisi\anaconda3\python.exe' -m pytest -q -W error --basetemp=.pytest_tmp_acceptance_full_2026-10-07
```

Targeted **217 passed in 14.08 s**; full **266 passed in 15.63 s** after examples;
zero failed/skipped/warnings reported, warnings as errors. All examples exit 0.
Sweep 81/81 feasible, max residual `4.400360274628181e-13`. Four GIFs independently
checked with Pillow: 3 frames, total 300 ms, infinite loop. Each benchmark workload
has 20/20 Mode A feasible, FK converged and Mode B feasible. This is a correctness
smoke, not a new before/after timing comparison. No artificial new test or tolerance
change made for acceptance.

MATLAB Engine detection false; missing/stub behavior tested, no numerical execution.
No hosted CI matrix, MP4 rerender, hardware/teleoperation or loaded/noisy/cold/global
FK test. Runtime source, tests, benchmarks and CI remain identical to the reviewed
6e280d1 revision; only documentation changes at closure. Whitespace checks pass.

## Review findings and disposition

External result: **ACCEPT**, no required code amendment. Generic derivative/rotvec
conventions, reference-method compatibility and benchmark attribution were accepted
within the documented scope. No broader hardware/real-time certification inferred.
No next milestone initiated and no acceptance-driven implementation changes made.

## Deviations, limitations and deferred work

The study was explicitly authorized while Milestone 2 review was pending. Its nested
temporary packet preserved the older audit; both are now closed together. A fused
FK cache and allocator micro-optimization were not needed after measured improvement.
The sample script was reference only, untouched and not executed wholesale.

Remain deferred: noisy/real sensor lengths, cold/global FK, long/loaded-OS tails,
communication/filtering/safety latency, persistent/fused caches, JIT/backend/language
comparisons and actual MATLAB implementation. No evidence warrants a MATLAB first
remedy. No dynamics, friction, stiffness/manipulability, global workspace, sag,
elasticity, contact or drivers were added. Select future scope explicitly.

## Closure and retained review evidence

Permanent records retained; temporary `.review/` removed after validation and tags.
The small exported review is preserved outside the repository at task output
`outputs/performance_study/performance_review_6e280d1.zip`, SHA256
`A5C9C38C67142DF74F4C0FC45A12CD53B2E3AF050D545906BD3D3BD9F3EF7A41`.
It contains 13 verified canonical copies plus summary/profiling evidence, not the
whole repository or a runnable standalone project. Raw benchmark results and the
manifest remain in task outputs. Archive checksums are verified before deleting
temporary duplicates. Working tree clean; no environment installation or push.
