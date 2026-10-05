from .base import AnalysisBackend, BackendUnavailableError, TensionProblem
from .matlab_backend import MatlabBackend
from .numpy_backend import NumPyBackend


def get_backend(name: str) -> AnalysisBackend:
    if name == "numpy":
        return NumPyBackend()
    if name == "matlab":
        backend = MatlabBackend()
        backend.require_solver()
        return backend
    raise ValueError(f"unknown backend {name!r}")


__all__ = ["AnalysisBackend", "BackendUnavailableError", "MatlabBackend", "NumPyBackend", "TensionProblem", "get_backend"]
