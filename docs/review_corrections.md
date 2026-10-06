# Foundation review corrections

Status: **COMPLETED — ACCEPTED FOUNDATION**, after the external review's minor
documentation amendment. This document retains the decisions recorded before the
earlier hardening; final acceptance and validation are in the
[permanent milestone record](milestones/milestone_01_foundation.md).

Starting reference: `7683e77961284f62b9d4f2d19c45bd0b6a6a7715` (the preserved
pre-review repository). No earlier commits or accepted milestone records existed.

## Decisions recorded before implementation

### Allocation status must be supported by evidence

Previously, small problems used enumerated least squares and large problems could
report `INFEASIBLE` solely because projected iterates changed little. A feasible,
badly scaled nine-cable example disproves that latter criterion. Keep the existing
solvers and bounded residual-based feasibility test, but require a separating
hyperplane certificate before reporting infeasibility at any problem size.

For a unit direction `w`, the support of `B [lower, upper]` is the sum of
`(B.T @ w)[j]` times the corresponding upper bound for positive coefficients and
lower bound otherwise. If `w @ target` exceeds this support by more than the
accepted residual tolerance plus a floating-point guard, infeasibility is
certified. An unbounded positive contribution or inconclusive separation is not
a certificate: return `NUMERICALLY_UNRESOLVED`. Numerical failures remain distinct.
The support calculation conservatively accounts for coefficient cancellation and
summation roundoff. Nonfinite force/residual arithmetic cannot report feasibility.
This deliberately favors an honest unresolved result over false infeasibility.
It does not add a new optimization objective, dependency, or force convention.

### Enforce existing invariants at public boundaries

NumPy's default relative tolerance was unintentionally weakening transform and
symmetry checks. Use the existing absolute tolerances with `rtol=0`. Robot states
and topology containers remain mutable, but public evaluations must reject
nonfinite states, broken references, duplicate identities/parents, altered joint
coordinate blocks, and altered canonical body frames. Same-named body frames
must stay identity on their own body; offset frames use separate names. Invalid
direct collection edits must fail rather than silently reinterpret coordinates.
Valid existing parameter and cable-bound edits remain supported. This is not an
immutable model redesign or a change to generalized-coordinate ordering.

Reject negative/nonfinite physical tensions, nonfinite external loads, invalid
solver options and targets. Define empty cable-Jacobian rank as zero. Add
regression tests for each boundary and retain virtual-work, routed geometry,
gravity and serialization tests.

### Optional backends must not claim unavailable execution

Explicit MATLAB selection must report the adapter's unimplemented status even
for a request without an allocation target. Keep detection lazy; importing and
using NumPy must not require MATLAB. MATLAB remains interface-only, not a tested
numerical backend. No independent MATLAB robot model or backend protocol rewrite
is introduced.

### Animation duration and file integrity are part of the export contract

Retain prescribed kinematics and both routing classes. Tighten timeline checks
with explicit tolerances. GIF stores integer multiples of 10 ms; reject rates
that cannot preserve the requested per-frame duration instead of silently
shortening playback. MP4 retains arbitrary positive rates supported by the
trajectory. Export to an owned temporary sibling and replace the destination
only after success; failed encoders must preserve an existing artifact. Pad MP4
frames to even dimensions for the existing H.264/yuv420p encoding choice.

## Workflow and compatibility

Add review ignores, contribution instructions, line-ending policy and an index
for milestone records. Preserve the user's `AGENTS.md` and full workflow. Do not
create a completed milestone record or acceptance tag before external review.
The final handoff must record actual commands, results, Git references and review
questions. Stricter rejection of previously invalid input and unrepresentable
GIF frame rates is intentional; valid default examples keep their APIs.

## Questions raised for external review

The external review accepted the foundation without a blocking architectural or
mathematical defect. These questions remain future design considerations, not
unresolved acceptance blockers.

- Is conservative residual-direction separation sufficient for this reference
  allocator, or should a later milestone add a better-conditioned certified
  numerical backend?
- Should mixed translational/rotational residuals gain explicit scaling? The
  current documented Euclidean norm and physical units remain unchanged here.
- Should model/state mutation eventually use explicit update APIs or immutable
  snapshots? This correction only validates current contracts.
- Built-in multibody dynamics, pulley wraps/friction/contact, continuous workspace
  certification and hardware remain deferred; these are not review fixes.

## Accepted minor amendment — 2026-10-06

Clarified "reference allocator" as a bounded-equilibrium/reference numerical
implementation, not user-configurable reference-tension tracking. Source/API
inspection confirmed neither allocator nor TensionProblem exposes `t_ref`.
Documentation alone was amended; allocator paths, numerical criteria, tests and
tolerances were preserved. Configurable `||t - t_ref||_2^2` allocation and associated
control work remain deferred. No Milestone 2 functionality was introduced.
