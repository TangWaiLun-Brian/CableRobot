# CableRobot Development Rules

## Project principles

- This is a generic cable-robotics framework.
- Do not assume a classical base-to-platform CDPR topology.
- Python owns robot-model semantics.
- Preserve explicit mathematical and physical conventions.
- Do not silently introduce breaking architectural changes.

## Development

- Add/update tests together with implementation.
- Test mathematical and physical invariants where practical.
- Do not weaken tolerances just to make tests pass.
- Avoid unrelated refactoring.
- Clearly distinguish implemented, partial, interface-only, and deferred functionality.

## Architecture changes

Before making major changes to:
- generalized coordinates/state representation;
- body/joint/frame topology;
- cable routing;
- Jacobians or force mappings;
- backend architecture;
- public API;

document the reason and consequences.

Flag uncertain major architectural decisions for review.

## Milestone workflow

At the end of each milestone:

1. Complete implementation.
2. Run targeted tests.
3. Run relevant examples.
4. Run the full test suite.
5. Prepare `.review/`.
6. Stop and mark the milestone as awaiting review.
7. Do not start the next milestone until review feedback is resolved.

## Review package

Create:

.review/
├── review_summary.md
├── repository_tree.txt
├── changed_files.txt
├── architecture_changes.md
├── test_results.txt
└── key_files/

Keep `.review/` temporary and gitignored.

Do not copy the entire repository into `.review/`.

## After review

- Evaluate review recommendations rather than blindly applying them.
- Implement accepted corrections.
- Rerun targeted and full tests.
- Update documentation.
- Create/update:
  docs/milestones/<milestone>.md
- Record the relevant Git commit/tag.
- Delete `.review/`.
- Only then begin the next milestone.

## Permanent milestone records

Each milestone record should summarize:

- objective;
- implemented capabilities;
- major architectural decisions;
- mathematical conventions changed or introduced;
- public API changes;
- tests and runtime;
- examples executed;
- review findings and corrections;
- deviations from the original plan;
- known limitations;
- deferred work;
- next-milestone considerations.

Git records exactly what changed.
Milestone documents record why the system changed.

## Repository cleanliness

Do not commit:
- `.review/`
- caches;
- temporary logs;
- generated review bundles;
- unnecessary generated outputs.

## Full workflow

See:

docs/development_workflow.md