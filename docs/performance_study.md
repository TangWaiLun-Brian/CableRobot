# FK + equilibrium pipeline performance study

Scope authorized on 2026-10-06: computational study of the implemented pipeline,
not a new physical-model milestone or acceptance of Milestone 2. Starting revision
`d5b733d204bf382db68f44415469ed7f01ff962d` is implemented, awaiting review. Its
existing `.review/` and exported review ZIP will be preserved. This study's separate
handoff will be `.review/performance_study/` to avoid overwriting pending review.

## Plan recorded before optimization

First add a reproducible benchmark with high-resolution, unprofiled frame timings
and separate cProfile evidence. Compare the same interpreter, machine, workloads,
thread settings, tolerances and warm-start policy before/after. Record all statuses,
pose/length error, equilibrium residual, objective, timing tails and evaluation counts.
No timing thresholds in correctness tests, and no weakening solver tolerances.

Workload A knows q and only calls equilibrium allocation. Workload B generates
synthetic lengths, solves local FK using the last successful state, then allocates.
Report both virtual-sensor-inclusive latency and measured-length->FK->tension latency;
the latter excludes synthetic length generation, NOT actual encoder overhead.
The first FK guess is the first true state, matching the provided simulation sample;
this does not benchmark global recovery or cold hardware startup.

Use the sample's 0.2 kg platform, unequal [0.15,0.10,0.08] m attachment half-extents,
1 m anchor half-extents, 1–100 N bounds, reference 10 N, 0.3 m circular translation
with z from -0.3 to 0.3 at 50 Hz/10 s (500 frames). The original Downloads script is
reference data only: do not execute its plotting/main block, modify it or add hardware.
Also reuse spatial_cdpr() with offset COM and a small continuous translated/mildly
rotated 500-frame trajectory. Component snapshots include static, small translation
and translated+rotated states; check feasibility/rank rather than hiding singularity.

Measure lengths, Jacobian, gravity, pure numerical reference QP (precomputed arrays),
robot-facing allocation, FK and complete pipelines. Profiled cumulative FK/Jacobian/
allocation times overlap hierarchically; do not add them as disjoint fractions.
Count FK length/Jacobian function evaluations using a separate profile pass, keeping
instrumentation out of headline timings. Effective Hz means reciprocal mean software
computation latency, not a paced or hard-real-time control-rate measurement.

Only after evidence identifies a dominant cost, document its mathematical/API change
before implementation. If finite differences/repeated geometry dominate, retain a
numerical reference and introduce generic analytic derivatives for supported joints
and straight body-fixed routed segments, validated at translated/rotated/random
spatial, serial, hybrid and offset-joint configurations. Preserve J_l=dl/dq,
B=-J_l.T, COM gravity, complete equilibrium and both tension solver algorithms.
Prefer call-local transform reuse over persistent caches on mutable models. Avoid
changing languages, backends, optimization objectives or physical models.

Target: 20 ms/frame (50 Hz) with practical margin for communication/filtering/safety/
logging/OS jitter. Report mean/median/p95/max, overruns and limitations honestly;
ordinary Python tests do not establish hard-real-time behavior. MATLAB stays a stub,
not a per-frame migration. Final targeted/full tests and actual benchmark evidence
must precede an IMPLEMENTED — AWAITING REVIEW handoff, with selected canonical files.
