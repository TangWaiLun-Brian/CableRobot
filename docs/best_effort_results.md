# Best-effort bounded allocation — study results

2026-10-07. **IMPLEMENTED — AWAITING REVIEW**. No acceptance tag or completed
milestone record. Accepted baseline: `469e62e04672e8af35e28a3a3301683151a8bea7`.
Initial implementation: `92b51c066b87747ac0fc5742111e89b12c7fe9df`.
Refined numerical implementation/tests: `924f7160478dd2c01e26c1cc63f67f8eaf42142f`.
The review handoff records its final documentation/CI commit separately.

## Implemented contract

Separate `allocate_best_effort_tensions` and `solve_bounded_equilibrium_tensions`
APIs; existing exact APIs, statuses, solvers, geometry and backends are unchanged.
For required force `target=-(gravity+external_load)`, solve
`min 0.5*||S*(B*t-target)||^2` over the original tension box. S is explicitly supplied
and nonsingular. Generic serial/hybrid systems retain ALL generalized rows.

Residual minimization is primary. Its force image is unique even if tensions are
not: the projection onto a convex achievable-force set is unique, and S is
invertible. Reuse the accepted reference QP to minimize
`0.5*||W*(t-t_ref)||^2` at that SAME force image, not an arbitrary penalty mixture.
Independently verify projected KKT, a guarded convex box dual gap, reference-QP
optimality and scaled force-image preservation. Numerical tolerances and limits
are explicit in [conventions](conventions.md) and the
[preimplementation decisions/refinement](best_effort_allocation.md).

The robot API returns the unchanged accepted reference solution whenever verified
exact equilibrium is available. A sufficient bounded witness bypasses primary
screening but never supplies a command. A verified original-space separation may
skip a known-impossible exact QP and reuse its primary candidate. This efficient
combined path preserves exact priority, rather than unnecessarily running both
expensive problems. `best_effort=False` uses the accepted exact robot wrapper only.

FEASIBLE uses the original `1e-8*(1+norm(target))` equilibrium threshold.
BEST_EFFORT requires verified primary/secondary optimization AND conservative
original-space separation beyond that threshold; it is NEVER FEASIBLE. Unverified
optimization/separation remains NUMERICALLY_UNRESOLVED, arithmetic failure is
NUMERICAL_FAILURE, malformed inputs raise ValueError. Exact-only requests may be
INFEASIBLE. Only FEASIBLE/BEST_EFFORT have a copied `tension_command`.

Results include full unscaled residual, achieved/required generalized forces,
scaled/raw norms, residual-scaling metadata, reference distance/objective, active
bounds/margins, tolerances and solver/certificate diagnostics. Physical force/moment
fields exist only with an explicit meaningful wrench-to-generalized map.

## Boundary-crossing evidence

Model: accepted spatial eight-cable platform, mass 2 kg, offset COM, unchanged
gravity. Fixed 1–30 N bounds and 20 N reference. Prescribed static poses:
`q=[1.2*sin(pi*time/6)^2,0,0,0,0,0]`, 61 samples, six virtual seconds.
Characteristic length 0.2 m; S scales actual moments by 1/0.2, with the selected
frame twist transpose supplied as the wrench-to-generalized map.

- 38 FEASIBLE, 23 BEST_EFFORT; no missing/unverified commands.
- Transitions: FEASIBLE → BEST_EFFORT → FEASIBLE; all eight tensions remain 1–30 N.
- Maximum weighted residual: 10.887934904 N force-equivalent.
- Maximum guarded primary gap: 1.0872e-8; compare the declared
  `optimality_tolerance*(1+objective)`, not an unscaled fixed gap threshold.
- Maximum scaled stage-2 force-image change: 3.6918e-13.
- Maximum neighbor tension change: 3.023133 N at 0.1 s sampling; this is not
  proof of global command continuity or smooth physical motion.
- Against the initial exhaustive implementation: maximum tension difference
  3.6104e-9 N and weighted-residual difference 5.33e-15; same statuses/transitions.

At the midpoint x=1.2 m, lower bounds are active on cables 1/3 and upper bounds
on 6/8. Tensions [N]:
`[1,7.482010402,1,1.492917970,22.209577257,30,25.133695033,30]`.
Uncompensated world force [N]: `[-4.202742888,-0.506267593,-7.406572975]`;
moment about the platform origin [N m]: `[0.082502867,-1.344877724,0.123754301]`.
Other support must supply their NEGATIVE, or the robot may move/accelerate.
This study does not model contact/operator support or simulate that motion.

The example produces individual tensions, status, scaled residual, force/moment
components and active-bound plots, plus strict raw JSON. Run:

```sh
python -W error -m examples.best_effort_allocation --samples 61 --output-dir examples/output/best_effort_study/trajectory_measured
```

## Measured fallback cost — NOT 50 Hz validation

One same-machine, single-thread run per revision, 61 identical poses. Python
3.12.13, NumPy 2.1.0, Windows 11; OPENBLAS/MKL/OMP_NUM_THREADS all 1. Timed robot
allocation only; construction of the scaling helper, reporting and plotting are
outside timing. The final measured run followed completion of both test processes.
No pacing, hardware, scheduling guarantee or new measured-length→command pipeline.

| Branch | Initial mean / p95 / max [ms] | Refined mean / p95 / max [ms] | Refined >20 ms |
|---|---|---|---|
| FEASIBLE (38) | 3.335 / 4.772 / 5.894 | 4.199 / 7.566 / 34.789 | 2 |
| BEST_EFFORT (23) | 631.881 / 784.416 / 822.916 | 23.153 / 40.384 / 41.667 | 13 |

The new fallback is about 27.3 times faster here, but still lacks reliable 20 ms
margin. Near-boundary feasible frames may incur primary screening. The accepted
exact-only pipeline remains unchanged; its previous performance acceptance does
not extend to this new fallback. Poor conditioning/infinite support can remain
unresolved. The projected routine retains its existing 10,000-iteration budget;
`max_iterations` controls the reused reference QP, not that primary routine.
Inconclusive small-system candidates may still incur exhaustive fallback cost.

Ignored raw evidence (not included as binary/generated review copies):

- `examples/output/best_effort_study/trajectory_baseline_controlled/best_effort_report.json`,
  SHA256 `ec714fde1c26a0a67f9ad56ec600dc4312729cf6a0524274d0d8272250a624dd`.
- `examples/output/best_effort_study/trajectory_measured/best_effort_report.json`,
  SHA256 `4142c6d3f3d40c38630c5da6599569c916d6e3abda65e2c161a85f5704d64eb1`.
- The PNG beside the refined JSON was visually inspected for labels, units,
  status transitions and bounds. Other preliminary runs are retained but are not
  the timing table's evidence.

## Validation and compatibility

Accepted baseline: 266 passed (15.35 s). All original test files/tolerances remain
byte-for-byte unchanged. Final targeted: **296 passed, 15.24 s**; final full source:
**360 passed, 18.66 s**. Isolated offline wheel, imported from its venv site-packages
without repository source: **360 passed, 18.40 s**. All use warnings-as-errors;
zero failures/skips/warnings in final runs. No global installation changed.

New tests independently enumerate primal-face augmented KKT systems for 25 seeded
four-cable problems and both objectives, plus 100 random bounded competitors per
case. Analytic overload/excess minimum/unavailable direction/active-bound/redundant
cases, dense/scalar scaling, rotated Cartesian wrench mapping, serial/hybrid
outer-to-tip and on-robot routes, empty/fixed/infinite boxes, force scales
1e-4/1/1e4, near infeasibility, conditioning, failure/budget/input semantics and
no-command protections are covered. Large pretension cannot buy a worse residual.
New tests verify exact bitwise tension/residual/objective/dual compatibility,
primary candidate reuse, impossible-QP bypass and exact-only screening exclusion.

The accepted 81-pose exact example still has 81 feasible samples, maximum residual
4.4004e-13, and the deliberate 2 N overload remains INFEASIBLE. New 9- and 61-pose
examples completed. FK studies/results are [reported separately](fk_prediction_results.md).
MATLAB remains only an optional unavailable adapter boundary, not an executed
backend. Hosted CI was updated but not executed here; its new commands were run
locally, with no timing threshold.

Development failures were investigated: an initial strict float-bound equality
KKT check rejected a roundoff-sized interior offset in a hybrid solution (1 failed,
75 passed); projected stationarity plus a regression corrected this without weaker
tolerances. Six initial study-script failures came from treating the joint list as
a dictionary; corrected traversal and successful-history semantics were tested.
One targeted command named a nonexistent oracle file and ran no tests; the actual
seven-file targeted command above then passed. No accepted test was removed or
relaxed. Coarse FK sampling failures are disclosed in the FK report.

## Answers to the final review questions

1. Infeasible optimization: bounded explicitly scaled linear least squares,
   followed by reference minimization at its unique achieved-force image.
2. Force/moment scaling: explicit S; spatial helper uses characteristic length
   and the actual wrench-to-generalized map, not a six-coordinate guess.
3. Priority: lexicographic, verified numerically; no residual/reference tradeoff.
4. Status: original equilibrium criterion for FEASIBLE; verified nonzero optimum
   and original-space separation for BEST_EFFORT; unverified solves are unresolved.
5. Residual: full unscaled generalized vector always available; actual spatial
   force/moment additionally available with explicit mapping.
6. Independent optimum validation: analytic cases and independent primal KKT
   active-face oracle/random competitors, not comparison only against itself.
7. Active bounds: enforced and reported; tested at upper/lower/fixed boundaries,
   with continuous projected stationarity under roundoff offsets.
8. Exact behavior: old APIs/code/defaults/tests unchanged; robot wrapper calls
   the same original-target reference QP whenever equilibrium may exist.
9. Infeasible path: all 23 infeasible samples receive bounded verified proposals
   and uncompensated-force direction/magnitude; no physical-motion guarantee.
10. FK warm start: already last successful pose; explicit call/history tests and
    accepted benchmark source confirm this, including failure preservation.
11. Predictor benefit: measured about 41%/40%/24% lower mean latency on practical
    translation/small/moderate local rotation paths respectively; see FK report.
12. Omission: production predictor correctly omitted; a guarded developer-only
    reproducer is retained. Near-branch extrapolation is disabled.
13. Previous tests: all 266 retained unchanged; current suite 360 cases.
14. Contracts: additive result/status/scaling/APIs only; explicit new numerical
    verification contract. No changed coordinates, force signs, exact tolerances,
    Jacobians, backend contract or production FK warm-start default.
15. Deferred: hardware/CAN/motors, real-time fallback guarantees, global/noisy FK
    tracking API, dynamics/inertia/friction/contact/support models, stiffness,
    workspace algorithms, sag/elasticity/pulleys and MATLAB migration.

Review focus: conservative scaling/optimality/separation checks, lexicographic
force-image preservation, exact-path compatibility and the deliberate limitations
of this model-only proposal. Do not start the next milestone before review.
