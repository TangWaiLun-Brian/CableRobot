# FK continuity / prediction — developer study

2026-10-07. **IMPLEMENTED — AWAITING REVIEW**, part of the best-effort study.
Production FK, Jacobians, convergence tolerance and previous-success warm-start
default are unchanged. `benchmarks/fk_prediction.py` is a developer script, not
included as a new production tracking interface in the wheel.

## Confirmed baseline and experiment

The accepted `benchmarks/control_pipeline.py` already initializes each next FK
call from the last SUCCESSFUL estimate, not a fixed reset. Failure preserves that
pose. Accepted benchmark tests are unchanged, and new history/call tests explicitly
confirm this behavior and reset velocity history on failure.

Compare previous pose with `2*q_previous-q_older`, requiring two consecutive
successful estimates at equal sample intervals. Predictions never use upcoming
truth. A failed prediction retries the previous pose; both attempts count in
latency/iteration totals. Nonfinite prediction, any floating rotation norm near
pi (within 0.1 rad, including larger branches), or a rotation step >0.1 rad causes
whole-state previous-pose fallback. This is a conservative LOCAL-chart guard,
not a new rotation representation or manifold tracker. Tests cover small rotation,
moderate tracking, near pi/2pi, a +3.13→-3.13 representation jump, failure history,
and retry costs.

## Reproducible measurement

```sh
python -W error -m benchmarks.fk_prediction --frames 500 --repeats 3 --output-dir examples/output/best_effort_study/fk_prediction
```

Measured at clean source `92b51c066b87747ac0fc5742111e89b12c7fe9df`, Python
3.12.13 (Anaconda), NumPy 2.1.0, Windows 11 build 26200, AMD64 Family 23 Model
113. OPENBLAS/MKL/OMP_NUM_THREADS all 1. Final code `924f716` has identical
`run_tracking`, predictor, workload and measurement functions; only CLI
`--require-convergence` smoke enforcement was added after this measurement.

Same accepted spatial robot model, identical precomputed lengths for both policies,
500 poses × three repeats per policy × four paths = 12,000 FK frames. Policies
alternate order between repeats, with an untimed warmup. Time includes predictor,
FK and any retry; excludes generating measured lengths, tension allocation, pacing,
plotting and I/O. Tolerance remains 1e-9. First initial guess is the first true pose,
so these are noiseless local-tracking tests, not global startup/sensor validation.
The practical path reproduces the supplied practice sample's translations but
uses the SAME accepted spatial model as the other paths to isolate start policy.

Statistics below pool all 1,500 frames per path/policy, not averages of p95s or only
the first repeat. No hardware/50 Hz/end-to-end claim follows from FK-only timing.

| Path / policy | Mean / p95 iterations | Mean / p95 / max latency [ms] |
|---|---|---|
| Practical translation / previous | 1.996 / 2 | 3.181 / 3.757 / 6.020 |
| Practical translation / velocity | 1.000 / 1 | 1.873 / 2.385 / 3.195 |
| Small rotation / previous | 1.996 / 2 | 3.231 / 3.911 / 4.828 |
| Small rotation / velocity | 1.000 / 1 | 1.925 / 2.491 / 3.519 |
| Moderate rotation / previous | 2.718 / 3 | 4.081 / 4.745 / 7.355 |
| Moderate rotation / velocity | 1.932 / 2 | 3.113 / 3.505 / 5.302 |
| Near-pi branch / previous | 1.996 / 2 | 3.179 / 3.461 / 5.474 |
| Near-pi branch / guarded velocity | 1.996 / 2 | 3.262 / 3.762 / 6.046 |

Every policy/path: zero final convergence failures and zero predictor-retry
failures at 500-sample resolution. Velocity was used on 1,494/1,500 frames of each
first three paths (first two per repeat need history), and ZERO near-branch frames.

| Path | Max length residual, previous / velocity [m] | Max coordinate error, previous / velocity |
|---|---|---|
| Practical translation | 2.1401e-10 / 4.3736e-10 | 1.0743e-9 / 5.7483e-9 |
| Small rotation | 1.2308e-13 / 9.0799e-12 | 1.6475e-12 / 1.1786e-10 |
| Moderate rotation | 9.8086e-10 / 9.9833e-10 | 8.8779e-8 / 1.0167e-8 |
| Near-pi branch | 1.7839e-13 / 1.7839e-13 | 2.7752e-12 / 2.7752e-12 |

Coordinate error is the raw mixed metres/radians diagnostic against synthetic
truth, not an invariant pose metric. Length residual controls the unchanged FK
convergence criterion. Raw JSON retains every iteration count, guessed/estimated
q, latency, residual and failure; SHA256
`00180e2e15468f1897027f2e135815852da222b219add91eb729fe47d514cf36` at
`examples/output/best_effort_study/fk_prediction/fk_prediction_results.json`.

## Decision and limitations

Mean latency fell about 41.1% (translation), 40.4% (small rotation), 23.7%
(moderate rotation). That justifies retaining a reproducible, guarded developer
experiment. It does NOT establish robustness for noise, irregular timing, failed
sensors, distant initial guesses or all robot topologies. No production predictor
or automatic opt-in tracking API is introduced. Default remains previous-success.
Near-branch guard correctly disables extrapolation; roughly 2.6% mean overhead
there offers no benefit, so no unguarded predictor is retained.

An exploratory eight-sample full-cycle run had two moderate-rotation failures
under EACH policy (maximum length residual 6.842e-4 m). All other coarse paths
converged. Large between-sample jumps invalidate any general tracking claim; these
failures are retained, not removed from the 500-sample experiment. A 64-sample
correctness smoke then passed all four paths and both policies. CI uses that
resolution with `--require-convergence` so diagnostic failures cannot silently
pass its smoke check. No timing threshold is used, and hosted CI was not run here.

The study's full source and isolated-wheel suites each passed 360 tests with
warnings-as-errors; see [allocation results](best_effort_results.md). Production
tracking design, timing/noise robustness and global branch recovery remain deferred.
