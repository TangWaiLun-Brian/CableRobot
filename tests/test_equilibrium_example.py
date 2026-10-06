import importlib.util
import json
from pathlib import Path

import numpy as np


# Examples are source scripts, intentionally not wheel runtime modules. Load the
# canonical script without adding the checkout to sys.path so installed-wheel
# testing can still exercise the installed cablerobot package from outside it.
_spec = importlib.util.spec_from_file_location(
    "equilibrium_example", Path(__file__).resolve().parents[1] / "examples/equilibrium_allocation.py")
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
run_validation = _module.run_validation


def test_example_exports_full_equilibrium_and_bound_diagnostics(tmp_path):
    report = run_validation(tmp_path, samples=7)
    stored = json.loads((tmp_path / "equilibrium_report.json").read_text(encoding="utf-8"))
    assert stored == report
    assert (tmp_path / "equilibrium_sweep.png").stat().st_size > 10_000
    assert report["sweep_summary"]["feasible_samples"] == 7
    assert report["sweep_summary"]["active_set_changes"] == 0
    assert report["single_poses"]["upper_bound_2N"]["status"] == "infeasible"
    for record in report["sweep"]:
        matrix, gravity = np.array(record["B"]), np.array(record["gravity_generalized_force"])
        expected = matrix @ record["optimized_tensions_N"] + gravity
        np.testing.assert_allclose(expected, record["optimized_residual"], atol=1e-12, rtol=0)
        assert len(record["optimized_residual"]) == 6
        assert len(record["route_segment_directions_world"]) == 8
        assert record["equal_within_bounds"] and record["optimality_verified"]
