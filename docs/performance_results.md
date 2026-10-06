# Measured performance results

Study scope: **IMPLEMENTED — AWAITING REVIEW**, not hardware acceptance or a new
physical-model milestone. Measurements on 2026-10-06, same AMD Ryzen 7 3700X
(8 cores/16 logical), Windows x64, Python 3.12.13, NumPy 2.1.0, Matplotlib 3.9.2.
Python: C:\Users\thisi\anaconda3\python.exe. OPENBLAS_NUM_THREADS,
MKL_NUM_THREADS and OMP_NUM_THREADS were each explicitly 1 in both runs.
No CPU affinity, real-time priority, scheduling, communication or hardware assumed.

## Reproducibility and timing boundary

Baseline revision: 5877baa10ea64350de512e1f2c1349827a0cd6c9 (instrumentation added;
all cablerobot source identical to Milestone 2 review d5b733d).
Optimized numerical revision: b92d3c1db02053a4ec5290301c9579659de32d03.
Current later documentation/CI commits do not change those measured numerical files.

Repository root, each after setting the three thread variables to 1:

```powershell
& 'C:\Users\thisi\anaconda3\python.exe' -W error -m benchmarks.control_pipeline --label baseline --frames 500 --repeats 30 --profile-frames 30 --output-dir examples/output/performance/baseline
& 'C:\Users\thisi\anaconda3\python.exe' -W error -m benchmarks.control_pipeline --label optimized --frames 500 --repeats 30 --profile-frames 30 --output-dir examples/output/performance/optimized
```

Each run has two 500-frame (10 virtual seconds at 50 Hz) trajectories; separate
untimed warmup, component snapshots, unprofiled perf_counter_ns timings, and
30-frame cProfile pipeline/FK passes. Primary timings exclude profile/JSON/logging/
plotting overhead. Raw per-frame JSON and profiler text/pstats are ignored outputs.
Same settings/environment and every true-pose sample were compared, not merely two
similar trajectories. Profiler count compatibility was extended for the new coupled
analytic geometry; timing loops/workloads/settings/statistics remain unchanged.

Mode A: known q -> equilibrium tensions (no unnecessary FK). Mode B compute:
measured lengths -> warm-start FK -> equilibrium tensions. Virtual Mode B also
includes true q -> synthetic lengths. First guess is the first true pose, as in
caspr_example.py; this is local warm tracking, NOT startup/global FK. Hardware encoder
acquisition is excluded, not modeled as free. No pacing, noise or delay is simulated.

### practical_sample

| Latency (ms) | Before | After |
| --- | ---: | ---: |
| Mode A mean (pose-known allocation) | 43.7093 | 2.4478 |
| Mode B mean (lengths -> FK -> tension) | 139.5164 | 5.8601 |
| Mode B median | 138.5829 | 5.5540 |
| Mode B p95 | 148.7495 | 7.4983 |
| Mode B maximum | 163.5344 | 9.5400 |
| Mode B total (s, sum of 500 cycles) | 69.7582 | 2.9301 |
| Mode B reciprocal-mean throughput (Hz) | 7.1676 | 170.6448 |
| Mode B cycles >20 ms | 500 | 0 |
| Virtual Mode B mean (synthetic lengths included) | 142.3426 | 6.2633 |
| Virtual Mode B p95 | 151.5141 | 7.9905 |
| Virtual Mode B maximum | 166.7981 | 9.9956 |

Mean compute speedup 23.81x. Mode A/B and FK each
succeed on 500/500 frames before and after; the first FK guess is known truth.

| Component mean (ms) | Before | After |
| --- | ---: | ---: |
| cable_lengths | 2.8098 | 0.3774 |
| cable_jacobian | 40.5498 | 0.8812 |
| gravity | 2.4123 | 0.3496 |
| numerical_qp_only | 1.0802 | 1.0903 |
| robot_equilibrium_allocation | 44.5788 | 2.4419 |
| Warm-start FK (actual Mode B stage) | 95.3564 | 3.2541 |
| Allocation at estimated pose (actual Mode B stage) | 44.1600 | 2.6060 |

Snapshot components average 30 repeats at each of static, small-translation and
translated+rotated cases (90 samples/component). They are independently timed,
not addends of the separate complete-pipeline measurement.

Before/after frame matching: 0 status mismatches; max tension
change A 6.7729e-8 N / B 8.9843e-8 N;
max FK state change 5.0680e-13 mixed-coordinate units;
max B objective change 2.3505e-7. Maximum recovered-pose
error after 5.3421e-9; maximum FK length residual
7.7264e-10 m (threshold 1e-9); maximum full equilibrium residual
1.1906e-13 (same scaled 1e-8 criterion).
Total FK iterations before/after 998/998.

Profiled cumulative shares (nested, NOT additive): before FK
66.28%, Jacobian
85.22%, allocation API
31.66%, pure QP
0.68%; after FK
53.13%, analytic Jacobian/combined geometry
42.00%, allocation API
39.01%, pure QP
16.64%.

### repository_rotated

| Latency (ms) | Before | After |
| --- | ---: | ---: |
| Mode A mean (pose-known allocation) | 45.5159 | 2.5109 |
| Mode B mean (lengths -> FK -> tension) | 141.8858 | 5.7632 |
| Mode B median | 140.8003 | 5.6412 |
| Mode B p95 | 152.5883 | 6.5902 |
| Mode B maximum | 194.6976 | 8.2221 |
| Mode B total (s, sum of 500 cycles) | 70.9429 | 2.8816 |
| Mode B reciprocal-mean throughput (Hz) | 7.0479 | 173.5160 |
| Mode B cycles >20 ms | 500 | 0 |
| Virtual Mode B mean (synthetic lengths included) | 144.8480 | 6.1755 |
| Virtual Mode B p95 | 155.5078 | 7.0627 |
| Virtual Mode B maximum | 197.5511 | 8.7515 |

Mean compute speedup 24.62x. Mode A/B and FK each
succeed on 500/500 frames before and after; the first FK guess is known truth.

| Component mean (ms) | Before | After |
| --- | ---: | ---: |
| cable_lengths | 2.8304 | 0.3847 |
| cable_jacobian | 40.8251 | 0.8495 |
| gravity | 2.4308 | 0.3455 |
| numerical_qp_only | 1.1327 | 1.1607 |
| robot_equilibrium_allocation | 44.9155 | 2.4971 |
| Warm-start FK (actual Mode B stage) | 97.0217 | 3.2238 |
| Allocation at estimated pose (actual Mode B stage) | 44.8641 | 2.5394 |

Snapshot components average 30 repeats at each of static, small-translation and
translated+rotated cases (90 samples/component). They are independently timed,
not addends of the separate complete-pipeline measurement.

Before/after frame matching: 0 status mismatches; max tension
change A 1.4886e-7 N / B 1.4052e-7 N;
max FK state change 1.0220e-14 mixed-coordinate units;
max B objective change 1.8091e-6. Maximum recovered-pose
error after 1.6474e-12; maximum FK length residual
1.2308e-13 m (threshold 1e-9); maximum full equilibrium residual
9.2029e-14 (same scaled 1e-8 criterion).
Total FK iterations before/after 998/998.

Profiled cumulative shares (nested, NOT additive): before FK
66.37%, Jacobian
85.20%, allocation API
31.51%, pure QP
0.68%; after FK
52.81%, analytic Jacobian/combined geometry
41.61%, allocation API
39.13%, pure QP
16.68%.

## Evidence and changes

Before: finite differences and repeated validated transforms dominate. Practical
30-frame pipeline profile calls body_transforms 22,918 times; Jacobian cumulative
share 85.22%. Both standalone numerical tension optimizers are retained unchanged;
pure QP costs about 1.1 ms in independent snapshot timings and under 1% of the slow
profiled baseline. No MATLAB migration is supported by this evidence.

After: one call-local transform/derivative context per cable evaluation, generic
analytic endpoint/routed segment/COM derivatives including the rotvec SO(3) left
Jacobian, and one-q residual memoization inside FK. Public FK geometry callbacks,
previous-success warm starts and the Gauss-Newton algorithm/tolerances remain.
No fused per-FK Jacobian cache or persistent model cache was necessary. Validation
is NOT removed; default public calls still validate the mutable model/state.

30-frame FK-only counts, both workloads: length geometry evaluations 900 -> 146
(including 58 analytic combined-length/Jacobian evaluations); Jacobian evaluations
58 -> 58. After: 88 direct length API evaluations + 58 analytic combined evaluations.
This is fewer geometry evaluations, not fewer Newton steps or weakened tolerances.
The original numerical Jacobian remains explicitly selectable.

After profiling, FK remains the largest coarse stage (~53% cumulative); analytic
geometry and validation remain substantial (~42% combined-geometry cumulative),
and pure QP is now ~17% of a MUCH shorter profiled run. These nested shares are
not exclusive fractions and profiler overhead disproportionately affects cheap
functions. Independent pure-QP timing is essentially unchanged; further solver
micro-optimization is intentionally deferred with adequate current numerical margin.

Every pose in both 500-frame paths has analytic Jacobian rank 6 at tolerance 1e-8.
Minimum singular values: practical 0.03580705, repository 0.07318819; maximum mixed-unit
condition ratios 47.5331 / 23.1993. These are observability diagnostics, not force
ellipsoids or isotropy claims. Analytic derivatives are independently checked over
200 random generic configurations plus rotvec near zero/pi/2pi and offset forests.

## 20 ms target, correctness and limitations

The practical measured-length chain's p95 leaves about 12.50 ms and observed maximum
leaves 10.46 ms for OTHER work within a 20 ms budget. All 1,000 optimized compute
cycles across the two workloads were under 20 ms. Even virtual-sensor-inclusive
maximum was under 10 ms in the primary practical run. This supports the intended
50 Hz NUMERICAL budget on this machine for these noiseless warm-started paths.
It is not a paced 50 Hz test, worst-case execution bound, full hardware latency,
production safety decision or hard-real-time guarantee. Longer/noisy/ill-conditioned/
loaded-OS tests and measured I/O/filtering/safety costs remain necessary.

Both allocators, weights/objective, B=-J_l.T, coordinates/units/frames, bounds,
backend contract and statuses are unchanged. Default derivatives now analytic;
explicit method/step selects finite differences. New API selectors are documented
before code, rather than silently changing the meaning of a supplied difference step.
Original 193 tests retained unchanged. Added 6 benchmark and 67 analytic/correctness
cases: targeted 239 passed/13.29 s; full 266 passed/16.83 s, warnings as errors,
zero failed/skipped/warnings reported. Relevant spatial/serial/equilibrium examples
exit 0; all 81 regression sweep poses feasible, maximum residual 4.40036e-13.

Pending review questions: derivative propagation/rotvec convention and unsupported
contact boundary; default analytic compatibility/reference selectors; benchmark
latency boundaries/profiler accounting; whether noisy/loaded-machine validation
should be required before any hardware-control decision. MATLAB remains optional
and unimplemented, not justified as a first performance fix by the measured results.
No physical milestone, dynamics, friction, sag/elasticity, stiffness/manipulability,
sensors/drivers or hardware control is added. No acceptance tag is created.

