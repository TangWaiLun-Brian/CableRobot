# Milestone 2 — Pose-Dependent Tension Allocation and Gravity Compensation

## Status

Completed — external **ACCEPT** received from the user on 2026-10-07, with no
required code amendment. The computational performance study was accepted at the
same time; neither acceptance authorizes hardware use or a new milestone.

## Git references

- Accepted base: `25cec8dd60739dbfeb9ecfecd455f6d1fda8d810` (`milestone-1`).
- Original reviewed Milestone 2 implementation: `d5b733d204bf382db68f44415469ed7f01ff962d`.
- Separately reviewed performance follow-up: `6e280d1e968d52079fadd7022e47b03b2c75b906`.
- Final accepted commit: `milestone-2^{commit}`.
- Acceptance tag: `milestone-2`.

Both `milestone-2` and `performance-study-1` identify the combined closure commit
containing the two permanent records. The original implementation and performance
review references remain distinct historical boundaries. Resolve the final SHA
with `git rev-parse 'milestone-2^{commit}'`; an in-commit self-hash is not used.
The foundation tag is unchanged. No tag was moved, pushed or published.

## Objective and implemented capabilities

Add pose-dependent gravity-compensating static tension allocation with a configurable
reference/pretension objective, without replacing the accepted foundation allocator.

- Numerical `allocate_reference_tensions` and robot-facing `solve_equilibrium_tensions`.
- Scalar/per-cable references, positive diagonal weights, unilateral finite/infinite
  bounds, full generalized equilibrium, gravity and optional applied external load.
- `EquilibriumTensionResult` with objective, full residual, activity/reserves,
  iterations, NumPy identity, multiplier/optimality diagnostics and explicit statuses.
- A proposed `tension_command` only for verified feasible allocation; other returned
  candidates are diagnostic, not commands or guaranteed minimum-residual solutions.
- Reproducible equal-pretension versus allocated-tension example and local sweep.
- Generic routed serial/hybrid tests, including external-frame and robot-only routes.

## Architecture and mathematical conventions

The additive NumPy QP consumes arrays. Its robot wrapper reuses Python-owned model
semantics, common routes, `B=-J_l.T` and COM gravity. Neither the foundation
`allocate_tensions` algorithm nor its backend/result/problem contracts was replaced.
No new dependency or duplicated CDPR-only geometry was introduced.

The objective is `0.5 ||W(t-reference)||^2`, subject to
`B t = -(tau_gravity + external_load)` and declared bounds. Every equilibrium
coordinate is retained. Weights are strictly positive diagonal entries of W, not
W squared; references are finite nonnegative values and may lie outside the box.
Floating rotational generalized forces remain covectors, not Cartesian moments.
No force signs, coordinates, units, topology or physical cable assumptions changed.

The dependency-free semismooth dual solver uses free-column SVD and safeguarded
directional searches, with no explicit inverse. Final original-space equilibrium
uses `tolerance*(1+||target||)`; the numerical dual pairing uses
`|lambda.T residual| <= tolerance*(1+objective)`. These are numerical diagnostics,
not an exact-arithmetic proof. A foundation-solver fallback may supply a bounded
witness or separating certificate, but cannot certify the secondary optimum.
Stalls/iteration limits remain `numerically_unresolved`, certified separation gives
`infeasible`, and numerical failures stay distinct. Bad inputs raise explicit errors.

The subsequent accepted study changes derivative evaluation, not either allocation
algorithm; see [its record](performance_study_01_pipeline.md). The original decision,
alternatives and API details remain in [tension_allocation.md](../tension_allocation.md).

## API changes and compatibility

New APIs/results/exports are additive. Existing `AnalysisBackend`, `TensionProblem`
and foundation result semantics remain intact. The new solver is explicitly
NumPy-only, not an implemented MATLAB QP adapter. `include_gravity=False` is
available when a supplied applied load already contains gravity. No previous-tension
regularizer, continuity guarantee, dynamic integration or safety authorization added.

## Tests

Original review, 2026-10-06: previous 132 cases, 61 new cases, total 193; targeted
144 passed in 7.08 s, full source 193 passed in 12.04 s, installed wheel 193 passed
in 12.15 s outside the checkout. Original tests/tolerances were retained. An
additional 100 seeded small-face oracle probes matched objective values, worst
difference 6.59e-12; these were probes, not extra pytest cases.

Fresh integrated acceptance closure, 2026-10-07: **217 targeted passed in 14.08 s**,
**266 full-suite passed in 15.63 s** after examples, warnings as errors; zero
failed/skipped/warnings reported. The additional 73 cases belong to the accepted
performance study. Closure adds no source/test changes or new test cases.

Windows, Python 3.12.13, NumPy 2.1.0, Matplotlib 3.9.2; executable
`C:\Users\thisi\anaconda3\python.exe`. Full closure command from repository root:

```powershell
& 'C:\Users\thisi\anaconda3\python.exe' -m pytest -q -W error --basetemp=.pytest_tmp_acceptance_full_2026-10-07
```

Exact shared targeted/example/smoke commands are in the
[performance closure record](performance_study_01_pipeline.md#acceptance-closure-validation).
MATLAB Engine detection is false; optional missing/stub behavior is tested with
mocks, not MATLAB numerical execution. Hosted CI and other OS/Python versions
were not run locally. Historical installed-wheel checks were not rerun at closure.

## Examples and validation

Fresh examples use Agg and Python `-W error`; all exit 0. Outputs are ignored under
`examples/output/acceptance_2026-10-07/`.

- Equilibrium allocation: all 81 sweep poses feasible, max full residual
  `4.400360274628181e-13`; equal 20 N origin residual `19.632748857966885` versus
  allocated `4.400360274628181e-13`. All generalized force/moment rows participate.
  The 2 N upper-bound case remains infeasible. No active-set changes in this path.
- Spatial foundation example: Jacobian rank 6, bounded equilibrium residual
  `5.751836627085642e-15`, overload infeasible.
- Serial example: transforms/lengths/Jacobians/generalized forces retain reference
  behavior; no false claim that this demonstration supports gravity alone.
- Spatial/serial/hybrid/combined GIFs: independently checked as 3 frames, 300 ms,
  infinite loop, at 10 fps. Prescribed kinematics, not tension-driven dynamics.
- Both 20-frame pipeline smoke workloads pass; this is not new primary timing data.

Historical residuals at the original finite-difference review boundary remain in
the formulation document; later analytic rounding does not rewrite that evidence.

## Review findings and disposition

External result: **ACCEPT**, no required code amendment. No recommendation was
invented, rejected or used to add features. Only acceptance documentation/records
and local tags change at closure; implementation, tests and tolerances are unchanged.

## Deviations, limitations and deferred work

The dual NumPy solver is separate from the old backend contract; cross-backend QP
integration is deferred. The explicitly authorized performance study was completed
while this review was pending, preserved its review packet, and was separately
accepted. No next physical-model milestone was started.

Poor conditioning/scaling may still produce unresolved results. Mixed-coordinate
residual norms are unit-dependent. Model allocation cannot establish neutral
operator feel, tracking quality or safety. Existing serial/hybrid demonstration
geometries can be infeasible under gravity alone; witness-load tests do not imply
gravity-supporting hardware. Local sampled continuity is not a global guarantee.

Deferred: hardware/sensors/drivers/closed-loop tracking, friction/hysteresis,
full dynamics, regularized continuity, stiffness/manipulability/force-ellipsoid
optimization, global workspace, cable elasticity/sag/slack/contact/pulley mechanics
and MATLAB numerical allocation. Choose any next scope explicitly; do not implement
these merely because they are listed here.

## Closure and retained review evidence

The temporary `.review/` folder is removed after records/tags and validation;
permanent records remain. The exported original review ZIP is preserved outside the
repository at the task output `outputs/milestone_2/milestone_2_review_d5b733d.zip`,
SHA256 `24D9E2879E6EC85A01FFE04C51F06C570EC18BC240E40EE6C6924633BD4BF97F`.
Its checksum is verified before temporary-copy removal. The accepted tree is clean.
No new milestone implementation, environment installation, push or publication.
