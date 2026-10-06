# NumPy and optional MATLAB backends

Python owns topology, models, validation, serialization, public result dataclasses
and visualization. `AnalysisBackend` is a structural protocol with a name and
`solve_tension_problem(TensionProblem)` method. The working `NumPyBackend` delegates
to the bounded-equilibrium/reference numerical allocation implementation. Here
"reference" means a validation baseline, not a reference tension vector.
`TensionProblem.target` is a required generalized force; neither this structure
nor the public allocator exposes `t_ref` or a configurable reference-tension
tracking objective.

`MatlabBackend` is an intentional optional stub. Availability is checked lazily with
`importlib` and no MATLAB engine is imported at package import time. An absent engine
raises `BackendUnavailableError` with an installation explanation. A detected engine
still raises an explicit adapter-not-implemented error in this milestone. No MATLAB
process is started and no MATLAB installation is required by baseline tests.
`get_backend("matlab")` rejects explicit selection, including geometry-only analysis,
until a solver is implemented. `MatlabBackend.available()` only detects engine
installation; it does not mean this repository has a usable MATLAB solver.

## Numerical boundary contract

| Field | Shape | Meaning |
| --- | --- | --- |
| `force_matrix` | `(n, m)` | B mapping cable tension to generalized force |
| `target` | `(n,)` | Required generalized force, excluding allocator geometry |
| `lower`, `upper` | `(m,)` | Cable tension bounds, newtons |
| `tolerance` | scalar | Scaled feasibility residual threshold |
| result `tensions` | `(m,)` | Python float64 cable tensions |
| result force/residual vectors | `(n,)` | Achieved force, target and achieved-minus-target |

The adapter must create MATLAB doubles with the same numerical row/column layout:
B is n-by-m and vector inputs are column vectors n-by-1 or m-by-1. NumPy memory
contiguity does not define semantic shape; flattening requires explicit order. Convert
MATLAB result columns to one-dimensional NumPy float64 arrays and Python dataclasses.
Never pass robot objects, MATLAB arrays or engine handles through the core model API.

Python indices are zero-based and MATLAB indices one-based. Convert indices only
at the adapter boundary, if an algorithm needs them. Coordinate/cable ordering must
be exactly the Python insertion order. Units and signs are unchanged from
`conventions.md`; B is always `-J_l.T`. The MATLAB solver must distinguish infeasible,
unresolved and engine/algorithm errors. Catch engine exceptions in the adapter and
translate them into Python errors/results with the original diagnostic attached.

## Reproducibility for a future implemented adapter

Record Python, NumPy, MATLAB and engine versions, selected solver/version, tolerances,
iteration limits, model schema and numerical problem arrays. For stochastic methods,
record seeds. An engine import is not evidence of a working license or successful
solver call. Keep optional MATLAB setup and license errors explicit; never silently
fall back to NumPy when the user selected MATLAB. Adding an engine-backed solver
requires cross-backend numerical tests before enabling it.
