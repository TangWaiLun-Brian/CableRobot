# Milestone records

- [Milestone 1 — Generic foundation and routed animations](milestone_01_foundation.md):
  **Completed**, accepted after the external documentation amendment; Git tag
  `milestone-1` identifies the accepted boundary.

- [Milestone 2 — Pose-Dependent Tension Allocation and Gravity Compensation](milestone_02_tension_allocation.md):
  **Completed**, accepted on 2026-10-07 without required code amendment; tag
  `milestone-2` identifies the accepted integrated boundary.
- [Performance study 1 — FK and equilibrium pipeline](performance_study_01_pipeline.md):
  **Completed**, accepted on 2026-10-07 without required code amendment; tag
  `performance-study-1` identifies the same closure commit. This is a separate
  computational study, not Milestone 3 or hardware acceptance.
- [Best-effort study 1 — Bounded allocation and FK warm start](best_effort_study_01_allocation_fk.md):
  **Completed**, accepted on 2026-10-07 without required code amendment; tag
  `best-effort-study-1`. Best-effort allocation is **not yet 50 Hz validated**.
  Nonzero residual requires external support or motion; previous-success FK
  remains production default and velocity prediction is a developer experiment.

Temporary `.review/` handoffs are removed at acceptance and remain gitignored.
Exported small review archives remain outside the repository. Permanent records
retain capabilities, decisions, validation, limitations and deferred work. Historical
Milestone 1 documentation/tag is unchanged.

The accepted study's detailed design/results remain outside this record directory.
Its exported review archive is preserved outside the repository. No further
milestone has begun; stop at this accepted boundary until a new scope is requested.

Future milestones follow `../development_workflow.md`; do not treat an
implemented or tested state as acceptance before its review is resolved.
