import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location("control_benchmark", Path(__file__).resolve().parents[1] / "benchmarks/control_pipeline.py")
_benchmark = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_benchmark)


def test_statistics_report_tails_budget_and_reciprocal_mean():
    statistics = _benchmark.timing_statistics([10., 20., 30.])
    assert statistics["mean_ms"] == 20 and statistics["median_ms"] == 20
    assert statistics["p95_ms"] == 29 and statistics["max_ms"] == 30
    assert statistics["effective_hz"] == 50 and statistics["over_20ms"] == 1
    assert statistics["total_s"] == pytest.approx(.06)
    assert _benchmark.timing_statistics([]) is None


@pytest.mark.parametrize("frames,frequency", [(1,50), (True,50), (2,0), (2,np.nan)])
def test_invalid_benchmark_options(frames, frequency):
    with pytest.raises(ValueError):
        _benchmark.workload("practical_sample", frames, frequency)


def test_benchmark_modes_provenance_and_profile_counts(tmp_path):
    # Small continuous steps rather than a two-frame jump across the whole path.
    robot, qs, reference = _benchmark.workload("repository_rotated", 500)
    a = _benchmark._mode_a(robot, qs[:3], reference)
    b = _benchmark._mode_b(robot, qs[:3], reference)
    assert a["feasible"] == b["fk_converged"] == b["allocation_feasible"] == 3
    assert b["max_fk_length_residual"] <= 1e-9
    assert b["max_q_error_norm"] < 1e-6
    profile = _benchmark._profile(lambda: _benchmark._mode_b(robot, qs[:3], reference), tmp_path / "profile")
    assert profile["jacobian_evaluations"] > 0
    assert (tmp_path / "profile.pstats").exists() and (tmp_path / "profile.txt").exists()
    json.dumps({"A": a, "B": b, "profile": profile}, allow_nan=False)
    # No performance threshold assertion: timing tails depend on the host.
