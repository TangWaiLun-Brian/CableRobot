# CableRobot — Milestone, Review, and Documentation Workflow

This document defines the standing development workflow for this repository.

It supplements the technical requirements of the project and does **not** replace the repository specification, mathematical conventions, architecture documentation, tests, or milestone-specific implementation instructions.

The goals are to:

- maintain a clean engineering history;
- make milestone reviews efficient;
- avoid repeatedly packaging the entire repository;
- preserve important architectural and mathematical decisions;
- distinguish temporary review material from permanent project documentation;
- ensure every milestone begins and ends from a known, tested state.

Follow these rules throughout development unless explicitly instructed otherwise.

---

# 1. Sources of truth

Maintain a clear distinction between the following.

## 1.1 Source code and tests

The actual implementation and automated tests are the authoritative description of current software behavior.

Do not allow milestone documentation to substitute for tests or implementation.

## 1.2 Git history

Git is the authoritative detailed record of code changes.

Use commits to preserve implementation-level history.

Do not manually reproduce every code change inside milestone documents.

## 1.3 Permanent milestone records

Maintain:

```text
docs/milestones/
```

Each completed milestone must have one permanent milestone document.

These documents explain:

- what the milestone intended to accomplish;
- what was actually implemented;
- important architectural decisions;
- mathematical or physical conventions introduced or changed;
- significant deviations from the original plan;
- test and example results;
- limitations and deferred functionality;
- important review findings;
- the final accepted state of the milestone.

These records are an engineering history, not a commit log.

## 1.4 Temporary review material

Use:

```text
.review/
```

for temporary material prepared specifically for external technical review.

Add `.review/` to `.gitignore`.

Never treat `.review/` as permanent project documentation.

Delete or regenerate it after a milestone has been accepted.

---

# 2. Repository cleanliness

Do not store temporary review artifacts in normal source directories.

Do not commit:

```text
.review/
__pycache__/
.pytest_cache/
build/
dist/
temporary logs
generated review bundles
temporary figures
temporary benchmark output
```

unless a particular generated artifact has been explicitly designated as permanent project material.

Do not duplicate source files unnecessarily.

The repository itself remains the authoritative codebase.

---

# 3. Development within a milestone

During implementation:

1. follow the current milestone specification;
2. preserve the generic cable-robot architecture;
3. keep mathematical conventions explicit;
4. add or update tests together with implementation changes;
5. avoid unrelated refactoring unless it clearly improves correctness or architecture;
6. document significant architectural decisions when they are made;
7. keep optional or future functionality isolated rather than partially implementing it without validation;
8. do not silently change established conventions.

If implementation experience suggests that an existing architectural decision should change, explicitly document:

- the previous design;
- the problem discovered;
- the proposed new design;
- why the new design is preferable;
- compatibility consequences;
- tests required to validate the change.

Do not make major architectural changes merely for aesthetic reasons.

---

# 4. Mathematical and physical conventions

CableRobot is a modeling and analysis framework, so conventions are part of the software contract.

Whenever relevant, verify consistency with:

```text
docs/conventions.md
```

This includes, where applicable:

- world-frame definition;
- transform direction;
- rotation representation;
- generalized-coordinate ordering;
- generalized-velocity ordering;
- spatial-vector convention;
- cable direction;
- positive cable tension;
- cable-length Jacobian sign;
- generalized cable-force sign;
- gravity direction;
- wrench ordering;
- units;
- quaternion ordering if quaternions are used.

Do not leave new sign conventions implicit.

If a milestone introduces a new mathematical convention, update `docs/conventions.md` and mention the change in the milestone record.

Tests should verify important identities and invariants whenever practical.

---

# 5. Testing requirements during development

Tests should validate physics and mathematics, not merely implementation details.

For relevant functionality, prefer tests such as:

- analytically known solutions;
- invariance tests;
- finite-difference Jacobian checks;
- virtual-work consistency;
- force/moment equilibrium;
- dimension and shape checks;
- feasible and infeasible cases;
- boundary cases;
- topology-independent behavior;
- graceful handling of optional dependencies.

Use explicit numerical tolerances.

Avoid weakening tolerances merely to make a failing test pass without understanding the numerical reason.

Before declaring a milestone complete:

1. run targeted tests for new functionality;
2. run all relevant examples;
3. run the complete test suite;
4. record test count;
5. record runtime when available;
6. record failed/skipped tests, if any;
7. investigate unexpected warnings;
8. state clearly whether optional backends were available during testing.

A milestone is not complete simply because new tests pass.

Regression tests from previous milestones must continue to pass unless an intentional breaking change has been approved and documented.

---

# 6. Preparing a milestone review

When implementation for a milestone appears complete, **do not immediately begin the next milestone**.

First prepare a review package in:

```text
.review/
```

Use a structure similar to:

```text
.review/
├── review_summary.md
├── repository_tree.txt
├── changed_files.txt
├── architecture_changes.md
├── test_results.txt
└── key_files/
```

The exact structure may be adjusted if there is a clear reason, but keep the package small and readable.

---

# 7. `review_summary.md`

This is the primary handoff document.

Include:

## Milestone

- milestone name;
- objective;
- starting Git reference if available;
- current Git commit.

## Implemented

Summarize completed functionality.

Focus on capabilities and behavior rather than individual code edits.

## Not implemented

Explicitly list functionality that remains deferred.

Do not imply that interfaces or placeholders constitute complete implementations.

## Important design decisions

Explain significant choices made during the milestone.

Examples:

- ownership relationships between `CableRobot`, `Body`, `Frame`, `Joint`, and `Cable`;
- configuration representation;
- Jacobian convention;
- tension-allocation formulation;
- backend boundaries;
- solver abstraction;
- serialization decisions.

## Mathematical decisions

Report any important equations, conventions, assumptions, or numerical methods introduced.

## Deviations from plan

For each meaningful deviation from the milestone specification, state:

- what changed;
- why;
- consequences.

Do not hide deviations.

## Known limitations

Report unresolved limitations, numerical weaknesses, performance issues, incomplete interfaces, or assumptions.

## Tests

Report:

- full test count;
- passed;
- failed;
- skipped;
- runtime;
- important targeted tests.

## Examples

List examples executed and whether they completed successfully.

## Review questions

If there are architectural decisions that deserve external review, explicitly list them.

Do not pretend uncertain decisions are settled.

---

# 8. `repository_tree.txt`

Provide a concise current repository tree.

Include important source, test, documentation, and example files.

Exclude noise such as:

```text
.git/
.venv/
__pycache__/
.pytest_cache/
generated output
large datasets
temporary files
```

Do not dump irrelevant dependency directories.

---

# 9. `changed_files.txt`

When a previous milestone Git reference exists, derive this from Git rather than manually guessing.

Prefer information equivalent to:

```bash
git diff --name-status <previous-milestone>..HEAD
git diff --stat <previous-milestone>..HEAD
```

Report files as:

```text
A  path/to/new_file.py
M  path/to/modified_file.py
D  path/to/removed_file.py
R  old/path.py -> new/path.py
```

Briefly explain particularly important changes where useful.

Do not copy the entire Git diff unless explicitly requested.

---

# 10. `architecture_changes.md`

Only create substantial content here when meaningful architectural changes occurred.

For each major change, document:

```text
Decision:
Previous design:
New design:
Reason:
Alternatives considered:
Consequences:
Compatibility impact:
Tests validating the decision:
```

Do not fill this file with trivial implementation changes.

If there were no significant architecture changes, state that clearly.

---

# 11. `test_results.txt`

Include the commands that were actually run and summarize their results.

For example:

```text
Full test suite
---------------
Command:
pytest

Result:
117 passed
Runtime:
24.8 s
```

Also record important example or validation commands.

Do not fabricate commands or results.

If something was not run, say so.

---

# 12. `key_files/`

Do not copy the whole repository.

Choose the smallest set of canonical source, test, and documentation files
necessary to audit the milestone's highest-risk mathematical and architectural
changes. Tie selection to the decisions and questions in `architecture_changes.md`
and `review_summary.md`, rather than copying every changed file.

When an external reviewer **cannot access the repository**, copy that selected
set into `.review/key_files/`. Local path references alone are not an adequate
handoff. Include only the supporting context necessary to understand the risks;
do not recursively copy all dependencies or turn the subset into a repository
mirror.

Typical candidates include:

- central model abstractions;
- state/configuration representation;
- important Jacobian implementation;
- force/tension mapping;
- solver interfaces;
- backend protocol;
- convention documentation;
- important new tests.

For example, a force-allocation and validation review might include:

```text
key_files/
├── README.md
├── allocation_tension.py
├── equilibrium.py
├── jacobians.py
├── test_statics.py
├── test_review_regressions.py
└── conventions.md
```

This is illustrative, not a required checklist for every milestone. A different
risk profile requires a different selection.

Preserve each copied file byte-for-byte from the canonical file at the recorded
review Git reference. Do not rewrite, summarize, or modify source/test copies.
When flattening paths, use unambiguous filenames such as `allocation_tension.py`
for `cablerobot/allocation/tension.py`; never overwrite two files with the same
basename.

Include a small `key_files/README.md` index recording, for each copy:

- review-package filename;
- canonical repository-relative path;
- audited Git reference;
- SHA256 of the copied bytes;
- reason for inclusion and associated mathematical or architectural risk.

Verify the copies match the recorded canonical revision. State explicitly if the
selected subset is not independently runnable; do not claim it reproduces the
full test suite without the remaining repository and dependencies.

When the reviewer **can access the recorded repository revision**, a path-based
index may suffice and unnecessary copies should be avoided. Neither access mode
permits copying the entire repository. Keep copied review files temporary and
gitignored, just like the rest of `.review/`.

---

# 13. External review stage

Once `.review/` has been prepared:

**stop before starting the next milestone.**

The milestone should be considered:

```text
IMPLEMENTED — AWAITING REVIEW
```

not fully closed.

External review may identify:

- architectural issues;
- mathematical inconsistencies;
- missing tests;
- API problems;
- numerical weaknesses;
- documentation gaps;
- requirements that were accidentally missed.

Do not begin major work on the next milestone until the review findings have been considered.

---

# 14. Applying review feedback

When review feedback is returned:

1. classify each recommendation;
2. determine whether it should be accepted, modified, deferred, or rejected;
3. explain decisions where nontrivial;
4. implement accepted corrections;
5. add regression tests where appropriate;
6. rerun targeted tests;
7. rerun the complete test suite;
8. rerun relevant examples.

Do not blindly implement every reviewer suggestion.

Engineering correctness and project consistency take priority.

If review feedback conflicts with the established project specification, explicitly identify the conflict before changing architecture.

---

# 15. Permanent milestone record

After review corrections are complete and all acceptance criteria are satisfied, create:

```text
docs/milestones/<milestone_name>.md
```

For example:

```text
docs/milestones/milestone_01_foundation.md
docs/milestones/milestone_02_dynamics.md
```

Use a consistent format.

---

# 16. Milestone document template

Each permanent milestone document should contain:

```markdown
# Milestone N — Name

## Status

Completed

## Git references

Base:
Final commit:
Tag:

## Objective

What this milestone was intended to establish.

## Implemented capabilities

What was actually completed.

## Architecture

Important architectural decisions introduced or modified.

## Mathematical conventions

Important equations, sign conventions, state representations,
coordinate conventions, or numerical methods introduced or changed.

## API changes

Important public interfaces added, removed, or changed.

## Tests

Previous test count:
Current test count:
New tests:
Full-suite result:
Runtime:

## Examples and validation

Examples or validation cases executed and their outcomes.

## Review findings

Important issues identified during review and how they were resolved.

## Deviations from original plan

Any meaningful departure from the milestone specification and why.

## Known limitations

What remains incomplete, approximate, experimentally validated only,
or otherwise limited.

## Deferred work

Capabilities intentionally postponed to later milestones.

## Compatibility notes

Any breaking or potentially breaking changes.

## Next milestone considerations

Questions or issues that should inform selection of the next milestone.
```

Keep this concise enough to remain useful.

It is not necessary to describe every changed function.

---

# 17. Relationship between Git and milestone records

Use the following principle:

```text
Git tells us WHAT changed.
Milestone records tell us WHY the system changed.
Tests tell us WHETHER the expected behavior works.
Documentation tells us HOW the system is intended to be used.
```

Do not make any one of these substitutes for the others.

Whenever practical, tag accepted milestones.

For example:

```text
milestone-1
milestone-2
milestone-3
```

or another consistent project convention.

Record the corresponding Git tag in the milestone document.

---

# 18. Closing the milestone

A milestone can be marked complete only after:

- required functionality is implemented;
- relevant documentation is updated;
- targeted tests pass;
- complete regression tests pass;
- required examples execute;
- review has occurred;
- accepted review corrections are applied;
- the permanent milestone document is written;
- the final Git reference is recorded.

Then:

1. finalize the milestone record;
2. create the milestone tag if the project uses tags;
3. remove `.review/`;
4. ensure the working tree is clean;
5. begin planning the next milestone.

Do **not** delete:

```text
docs/milestones/
```

The permanent milestone history must remain.

---

# 19. Starting the next milestone

Before implementing the next milestone:

1. read the previous milestone record;
2. inspect unresolved limitations and deferred items;
3. inspect current conventions;
4. verify the current test suite is healthy;
5. determine whether the proposed next milestone is still appropriate;
6. avoid automatically implementing deferred features merely because they appear in earlier planning documents.

The next milestone should be selected based on the current architecture and project needs, not solely on the original roadmap.

---

# 20. Storage discipline

As the repository grows, avoid repeatedly creating full repository copies.

For external review:

- prefer summaries and changed-file lists;
- provide selected canonical key-file copies when the reviewer cannot access
  the repository, with provenance as specified in section 12; otherwise a
  path-based index may suffice;
- use Git to identify changes;
- avoid including datasets, environments, generated simulations, videos, plots, binary outputs, caches, or Git history unless specifically required.

Temporary review material must remain disposable.

Permanent milestone documentation should remain lightweight text.

---

# 21. General reporting rules

When reporting progress:

Be precise about the difference between:

- implemented;
- partially implemented;
- interface only;
- tested;
- experimentally exercised;
- deferred.

Never report an interface stub as a completed capability.

Never state that all tests pass unless the complete intended test suite was actually run.

Never state that an example works unless it was actually executed.

Never silently omit test failures or warnings.

Report numerical or architectural uncertainty explicitly.

---

# 22. CableRobot-specific architectural discipline

Continue respecting the repository's foundational goal:

> A cable connects arbitrary frames attached to bodies in a general multibody system.

Do not gradually introduce assumptions that restrict the framework to a classical single-platform CDPR unless such specialization exists only in an appropriately isolated application layer.

Python remains the owner of robot-model semantics.

Optional numerical backends must not become independent robot-model implementations.

Keep geometry, kinematics, statics, dynamics, analysis, allocation, simulation, visualization, hardware, and backend responsibilities separated according to their conceptual roles.

Do not introduce hardware control into a milestone merely because the architecture eventually intends to support hardware.

Preserve extension points without prematurely implementing unnecessary complexity.

---

# 23. Rule for large architectural decisions

Before making a change that substantially affects any of the following:

- generalized-coordinate representation;
- configuration manifold representation;
- body/joint graph;
- frame semantics;
- cable routing model;
- Jacobian definition;
- generalized-force mapping;
- dynamics representation;
- backend architecture;
- public API;
- serialization format;

do not treat the decision as a routine refactor.

Instead:

1. identify the architectural issue;
2. explain the proposed change;
3. evaluate consequences;
4. preserve or add tests for mathematical invariants;
5. document the final decision in the milestone record.

When uncertainty is significant, flag the issue for review before propagating the design throughout the codebase.

---

# 24. Primary principle

Optimize for long-term correctness, clarity, mathematical consistency, and extensibility—not for completing milestones as quickly as possible.

A milestone is successful when it leaves behind a cleaner and better-validated foundation for the next one.

Do not move forward merely because the implementation runs.

Move forward when the current layer is sufficiently understood, tested, documented, and reviewed.
