# Contributing

Read `AGENTS.md`, `docs/development_workflow.md`, `docs/conventions.md` and
`docs/architecture.md` before changing the current milestone. The first two
documents define the required procedure and review gate.

## Local development

Use Python 3.11 or newer in an isolated environment:

```sh
python -m venv .venv
# Windows: .venv\Scripts\activate
# macOS/Linux: source .venv/bin/activate
python -m pip install -e ".[dev]"
python -m pytest -q -W error
```

Add regression tests with corrections; retain explicit mathematical tolerances.
Document changes to conventions, state/topology semantics, numerical status or
public contracts before implementation. Avoid speculative features and unrelated
refactoring. Run affected tests before the full suite.

Exercise relevant examples in a headless environment (`MPLBACKEND=Agg`):

```sh
python -m examples.spatial_cdpr --output examples/output/cdpr.png
python -m examples.serial_robot --output examples/output/serial.png
python -m examples.animate_robots --duration 5 --fps 20 --format gif
python -m pip wheel . --no-deps --wheel-dir dist
git diff --check
```

For optional MP4 validation, select a working FFmpeg executable with `--ffmpeg`
and use `--format both`. Record its version and validate actual frame count,
duration and routing geometry. GIF rates must represent an exact positive
multiple of 10 ms per frame. MATLAB is still an explicit interface-only stub;
engine detection is not evidence of numerical validation.

## End-of-milestone handoff

`.github/workflows/ci.yml` configures Python 3.11/3.12 on Windows and Linux for
tests with warnings as errors, headless examples, a short GIF smoke test and wheel
building. Actions are pinned to verified upstream commits with read-only repository
permissions. A local successful run does not mean hosted CI has executed; this
repository currently has no configured remote. MP4 and MATLAB are not CI prerequisites.

Commit coherent source/test/documentation changes. Generate the small ignored
`.review/` package prescribed by the workflow, including base/current Git refs,
Git-derived changed paths, architecture rationale, actual tests/examples,
limitations and questions. If the reviewer cannot access the repository, copy
only the canonical source/test/documentation files needed to audit the highest-risk
changes into `.review/key_files/`, with an index of canonical paths, Git references,
SHA256 hashes and inclusion reasons. Preserve the canonical bytes and avoid filename
collisions. A path-based index may suffice for reviewers with repository access.
Never copy the whole repository or imply the subset is independently runnable.
Do not commit caches, review logs, environments, copied review files or generated media.

Stop at **IMPLEMENTED — AWAITING REVIEW**. After external feedback is evaluated
and accepted corrections pass tests/examples, record the accepted milestone in
`docs/milestones/`, record its Git reference/tag and remove `.review/`. Only then
start the next milestone. Historical validation reports do not imply acceptance.
