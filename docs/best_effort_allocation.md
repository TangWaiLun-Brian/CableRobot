# Best-effort bounded allocation and FK warm-start study

Starting accepted revision: `469e62e04672e8af35e28a3a3301683151a8bea7`
(`milestone-2` / `performance-study-1`). Authorized on 2026-10-07.
Status: COMPLETED — ACCEPTED on 2026-10-07, with no required code amendment.
Acceptance tag: `best-effort-study-1`; see the
[permanent record](milestones/best_effort_study_01_allocation_fk.md).
The preimplementation decisions below remain historical; closure changes no code.
Best-effort allocation is not yet 50 Hz validated. Nonzero residual requires
external support or motion. Production FK remains previous-success; constant
velocity prediction remains a developer experiment. No further milestone started.
See `best_effort_results.md` and `fk_prediction_results.md` for measured evidence.

## Architecture decision recorded before implementation

Previous: accepted exact/reference allocation returns a command only for verified
bounded equilibrium; infeasible diagnostic candidates need not minimize residual.
New: separate `allocate_best_effort_tensions` numerical API and exact-first
`solve_bounded_equilibrium_tensions` robot API. Neither accepted allocator, enum,
result, geometry, analytic Jacobian, FK core nor backend contract is rewritten.
Reason: opt-in approximate load compensation must not silently change old infeasible
results or make BEST_EFFORT appear FEASIBLE. Existing callers keep exact semantics.

The new result/status types are separate. FEASIBLE means full original-space
equilibrium meets the SAME `tolerance*(1+norm(target))` criterion. BEST_EFFORT means
the primary bounded residual optimum and secondary selection are numerically
verified, but that exact criterion is not met. Inconclusive optimization is
NUMERICALLY_UNRESOLVED; arithmetic/linear-algebra failures are NUMERICAL_FAILURE.
Exact-only use may retain INFEASIBLE. Invalid inputs raise ValueError, consistent
with accepted APIs. Only FEASIBLE/BEST_EFFORT expose copied proposed commands;
neither is hardware safety authorization.

## Scaling and physical interpretation

Residual is `r=B t-target=B t+gravity+other_applied_load`, with unchanged `B=-J_l.T`.
Primary objective is `0.5*norm(S r)^2`. An explicit residual scaling is REQUIRED;
there is no implicit raw mixed-unit default. Positive finite diagonal weights or
an explicit finite nonsingular square `ResidualScaling` map are supported. Scaling
is not inferred from coordinate count. All coordinates and original residual units
remain available. Extremely ill-conditioned scaling maps are rejected, not treated
as a reason to drop equilibrium rows.

A separate `spatial_wrench_scaling(Lc, generalized_force_from_wrench=...)` helper
uses `diag(1,1,1,1/Lc,1/Lc,1/Lc)` on Cartesian force/moment residuals. If input is
generalized floating force, the caller supplies its nonsingular wrench-to-generalized
map (e.g. selected-frame twist Jacobian transpose). Identity is only for already
Cartesian wrench input; nonzero rotvec covectors MUST NOT be mislabeled as moments.
Physical force/moment fields are provided only when such an explicit mapping exists.
Generic serial/hybrid results remain full generalized vectors, not fictitious wrenches.

A nonzero residual means cables cannot hold the modeled load at this pose. Other
support/operator/contact forces would need to supply `-r`, or the mechanism may
move/accelerate. This study does not simulate that motion or model those forces.

## Hierarchical optimization and verification

Stage 1 reuses the foundation bounded least-squares implementation on `A=S B`,
`b=S target`. Its old feasibility label is NOT a residual-optimality certificate.
Independently check box KKT signs/projected stationarity and the convex box dual
gap, including floating-point guards; inconclusive checks produce unresolved status.
For a nonzero residual outside the original exact tolerance, additionally require
the accepted conservative support-function separation in ORIGINAL force space,
using the scaled-objective normal `S.T S r`. Otherwise the result remains unresolved:
a small approximate optimality gap alone cannot prove that a nearly feasible or
poorly conditioned problem lacks equilibrium. Report this certificate separately.
Infinite upper support with a negative gradient cannot furnish a finite gap: retain
conservative unresolved behavior rather than invent a finite bound or ignore a sign.
No arbitrary residual/reference penalty lambda and no weaker exact tolerance.

The projection of b onto the convex achievable set `A[lower,upper]` is unique even
when tensions are nonunique. Since S is invertible, all residual minimizers have
the SAME achieved generalized force `B t1`. Stage 2 reuses the accepted reference
QP with equality `B t=B t1`, selecting minimum `0.5*norm(Wt(t-ref))^2` without
changing the primary force image. Recheck final primary optimality and force-image
preservation within a separately reported optimality tolerance. Secondary failure
does not become a verified BEST_EFFORT result; diagnostics remain available.

The INITIAL exact-first design called `solve_equilibrium_tensions` first; the
measured refinement below supersedes its unconditional call order.
On success, return that same tension solution/diagnostics with scaling metadata;
do NOT run both problems. With `best_effort=False`, return exact-only status and
no approximate command. Otherwise fall back after infeasible/unresolved exact
allocation, not by relabeling an arbitrary old candidate. Numerical failure stays
distinct. Duplicate fallback geometry queries use canonical public methods only;
no alternate robot-model implementation or geometry cache is introduced.

Alternatives: adding flags to the accepted function would change its result contract;
a penalty sum could sacrifice equilibrium for pretension; introducing another BVLS
or external QP dependency would duplicate the available validated numerical path.
Reuse plus explicit new optimum checks is selected. The initial foundation
exhaustive small-system / projected larger-system path was subsequently refined
as documented below; no 50 Hz fallback claim is assumed. Poor conditioning,
infinite support or exhausted secondary budget may
remain unresolved. Document actual timings, not generic global guarantees.

## FK predictor experiment (not a production solver replacement)

The accepted benchmark already passes the previous successful estimate to FK and
does not reset to a fixed pose each frame. Confirm this with call/history tests.
Compare previous-state with constant-velocity `qk+(qk-qkm1)` initial guesses using
the SAME FK solver/tolerance and equal sampling. First guess matches known truth,
as in the prior study; this is local tracking, not global startup recovery.

Keep predictor logic confined to a reproducible developer benchmark unless evidence
justifies a separate later production tracking API. It requires two consecutive
successful estimates; failure resets velocity history but retains the last success.
For floating rotation blocks, use previous-state fallback near pi/2pi chart limits,
large representation jumps or extrapolated rotations outside the guarded local chart.
Do not extrapolate blindly across rotvec branches, peek at next truth, change q,
weaken convergence, rewrite derivatives or alter the production warm-start default.

Measure iterations mean/p95, latency mean/p95/max, failures and final residual for
practical translations, small rotations, moderate rotations and near-branch paths.
Retain only the study/reproducer if no robust production-wide benefit is established.

## Validation and review plan

Keep all 266 accepted cases/tolerances. Add analytic insufficient upper/excess lower/
unavailable-force/active-bound/redundant examples; explicit mixed force/moment scaling;
independent exhaustive small-face objective oracle; two-stage weighted reference
selection, empty/fixed/infinite boxes, rank deficiency, near-feasible/scale/conditioning,
failure/status/command-copy/input checks, generic serial/hybrid and rotated physical
wrench mapping. Compare exact-first solutions with accepted exact APIs.

Demonstrate a fixed-load/finite-bound pose path FEASIBLE -> BEST_EFFORT -> FEASIBLE,
report individual tensions, statuses, weighted residual, all original residual
components and active bounds. No claim of smooth physical motion from commands.
Run targeted tests, examples, full regression suite, trajectory and FK comparison;
record actual evidence and prepare a small canonical-file `.review/` package with
Git/SHA256 provenance. No acceptance tag or completed milestone record before review.

Excluded: hardware/drivers/motors/CAN, friction, dynamics/inertia compensation,
stiffness/manipulability/workspace optimization, sag/elasticity/contact/pulleys,
MATLAB migration or unrelated performance refactoring.

## Measured primary-path refinement, recorded before code changes

Initial validated implementation `92b51c0` produced all 61 trajectory commands
(38 FEASIBLE, 23 BEST_EFFORT), but infeasible frames averaged 589.27 ms (maximum
715.34 ms). Exact frames averaged 2.71 ms. Reused exhaustive box search, including
the old exact solver's fallback, is inappropriate as the DEFAULT new control path.
This is evidence about the NEW fallback, not a reason to rewrite accepted exact code.

Revised design: reuse the existing projected least-squares routine first and certify
its candidate with the same guarded gap/KKT/separation checks. Use the unchanged
exhaustive small-system baseline only if that primary candidate remains inconclusive.
The new robot wrapper forms arrays solely from canonical B/gravity/bounds, then
screens the primary residual. If original-space separation is proven, skip the
known-impossible exact QP and reuse this already computed primary candidate for
hierarchical reference selection. Otherwise call the accepted exact allocator on
the ORIGINAL target; feasible tension/objective behavior remains identical. If exact
is unresolved, fall back with the same primary candidate and conservative checks.
Exact-only mode skips the screen. This implements the permitted efficient combined
feasibility/approximation formulation, not a relaxation or relabeling of equilibrium.

The precomputed candidate is an internal call-local value, never a public override
or persistent cache. Recheck it in the full best-effort path. Numerical failure,
inconclusive separation and exhausted secondary certification stay distinct. Both
old allocator files and their defaults are untouched. Measure the revised trajectory
and retain the initial trace as historical evidence; no worst-case/50 Hz guarantee.

To avoid adding projected iterations to ordinary feasible frames, first test a
cheap sufficient witness: a least-squares correction of the clipped reference.
Only a bounded witness meeting the unchanged ORIGINAL equilibrium tolerance
bypasses primary screening; the unchanged reference QP still selects the command.
Failure to find this witness says nothing about infeasibility. This is not a
second reference optimizer, and its candidate is never returned as a command.
Canonical geometry/load arrays are formed once and passed to the accepted
`allocate_reference_tensions`; exact-only mode retains the accepted robot wrapper.
