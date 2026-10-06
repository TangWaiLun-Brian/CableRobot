"""Profile pose-known vs virtual-encoder/FK/equilibrium paths without plotting.

python -m benchmarks.control_pipeline --label baseline --output-dir examples/output/performance/baseline
Use the same thread settings, parameters and machine for a before/after comparison.
"""

import argparse
import contextlib
import cProfile
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import platform
import pstats
import os
import subprocess
import sys
from time import perf_counter_ns

import numpy as np

from cablerobot import (Attachment, Body, Cable, CableRobot, CableRoute, Joint,
                       RobotState, allocate_reference_tensions,
                       gravity_generalized_force, solve_configuration_from_lengths,
                       solve_equilibrium_tensions)
from cablerobot.examples import spatial_cdpr


def sample_robot():
    """Adapt only the user's reference geometry/parameters, not their executable file."""
    robot = CableRobot("Practical sample spatial CDPR")
    robot.add_body(Body("outer_frame", fixed=True))
    robot.add_body(Body("platform", mass=0.2))
    robot.add_joint(Joint("platform_pose", "outer_frame", "platform", "floating"))
    # Exact corner order/unequal extents from caspr_example.py.
    signs = np.array([[1,1,1], [1,1,-1], [1,-1,1], [1,-1,-1],
                      [-1,1,1], [-1,1,-1], [-1,-1,1], [-1,-1,-1]])
    for index, sign in enumerate(signs):
        robot.add_cable(Cable(f"cable_{index+1}", CableRoute([
            Attachment("outer_frame", sign.astype(float)),
            Attachment("platform", sign * [0.15, 0.10, 0.08]),
        ]), tension_min=1., tension_max=100.))
    robot.validate()
    return robot


def workload(name, frames, frequency=50.0):
    if isinstance(frames, bool) or not isinstance(frames, int) or frames < 2:
        raise ValueError("frames must be an integer >= 2")
    if not np.isfinite(frequency) or frequency <= 0:
        raise ValueError("frequency must be positive and finite")
    theta = 2 * np.pi * np.arange(frames) / frames
    if name == "practical_sample":
        robot, reference = sample_robot(), 10.
        qs = np.column_stack([0.3*np.cos(theta), 0.3*np.sin(theta),
                              np.linspace(-0.3, 0.3, frames), np.zeros((frames, 3))])
    elif name == "repository_rotated":
        robot, _ = spatial_cdpr()
        reference = 20.
        qs = np.column_stack([0.04*np.cos(theta), 0.03*np.sin(theta), 0.02*np.sin(theta/2),
                              0.02*np.sin(theta), 0.015*np.cos(theta), 0.01*np.sin(theta/2)])
    else:
        raise ValueError(f"unknown workload {name!r}")
    return robot, qs, reference


def timing_statistics(milliseconds):
    values = np.asarray(milliseconds, dtype=float)
    if values.size == 0:
        return None
    if values.ndim != 1 or not np.all(np.isfinite(values)) or np.any(values < 0):
        raise ValueError("timings must be finite nonnegative milliseconds")
    mean = float(np.mean(values))
    return {"frames": int(values.size), "mean_ms": mean, "median_ms": float(np.median(values)),
            "p95_ms": float(np.percentile(values, 95)), "max_ms": float(np.max(values)),
            "total_s": float(np.sum(values)/1000), "effective_hz": 1000/mean if mean > 0 else None,
            "over_20ms": int(np.count_nonzero(values > 20))}


def _timed(function):
    start = perf_counter_ns()
    value = function()
    return value, (perf_counter_ns() - start)/1e6


def _mode_a(robot, qs, reference):
    rows = []
    for q in qs:
        state = RobotState(q)
        result, elapsed = _timed(lambda: solve_equilibrium_tensions(robot, state, reference_tension=reference))
        rows.append({"frame_ms": elapsed, "q": q.tolist(), "status": result.status.value,
                     "tensions": result.tensions.tolist(), "residual_norm": result.residual_norm,
                     "objective": result.objective_value, "iterations": result.iterations})
    return {"timing": timing_statistics([r["frame_ms"] for r in rows]),
            "feasible": sum(r["status"] == "feasible" for r in rows), "frames": rows}


def _mode_b(robot, qs, reference):
    previous = RobotState(qs[0].copy())
    rows = []
    for q in qs:
        state = RobotState(q)
        start = perf_counter_ns()
        lengths = robot.cable_lengths(state)
        sensed = perf_counter_ns()
        fk = solve_configuration_from_lengths(robot, lengths, previous)
        recovered = perf_counter_ns()
        allocation = None
        if fk.converged:
            previous = fk.state  # only successful estimates replace the warm start
            allocation = solve_equilibrium_tensions(robot, fk.state, reference_tension=reference)
        finished = perf_counter_ns()
        rows.append({"virtual_sensor_ms": (sensed-start)/1e6, "fk_ms": (recovered-sensed)/1e6,
                     "allocation_ms": (finished-recovered)/1e6,
                     "compute_frame_ms": (finished-sensed)/1e6, "virtual_frame_ms": (finished-start)/1e6,
                     "fk_status": fk.status.value, "fk_iterations": fk.iterations,
                     "fk_length_residual": fk.residual_norm, "q_error_norm": float(np.linalg.norm(fk.state.q-q)),
                     "q_estimated": fk.state.q.tolist(),
                     "allocation_status": "not_run" if allocation is None else allocation.status.value,
                     "tensions": None if allocation is None else allocation.tensions.tolist(),
                     "equilibrium_residual": None if allocation is None else allocation.residual_norm,
                     "objective": None if allocation is None else allocation.objective_value})
    return {"timings": {key: timing_statistics([r[key] for r in rows]) for key in
                        ("virtual_sensor_ms", "fk_ms", "allocation_ms", "compute_frame_ms", "virtual_frame_ms")},
            "fk_converged": sum(r["fk_status"] == "converged" for r in rows),
            "allocation_feasible": sum(r["allocation_status"] == "feasible" for r in rows),
            "max_q_error_norm": max(r["q_error_norm"] for r in rows),
            "max_fk_length_residual": max(r["fk_length_residual"] for r in rows), "frames": rows}


def _components(robot, reference, repeats):
    cases = {"static": np.zeros(6), "small_translation": np.array([.01,-.01,.01,0,0,0]),
             "translated_rotated": np.array([.04,-.03,.02,.03,-.02,.04])}
    reports = {}
    for name, q in cases.items():
        state = RobotState(q)
        matrix = robot.cable_force_matrix(state)
        target = -gravity_generalized_force(robot, state)
        lower, upper = robot.tension_bounds()
        functions = {
            "cable_lengths": lambda: robot.cable_lengths(state),
            "cable_jacobian": lambda: robot.cable_jacobian(state),
            "gravity": lambda: gravity_generalized_force(robot, state),
            "numerical_qp_only": lambda: allocate_reference_tensions(matrix, target, lower, upper, reference_tension=reference),
            "robot_equilibrium_allocation": lambda: solve_equilibrium_tensions(robot, state, reference_tension=reference),
        }
        timings = {}
        for key, function in functions.items():
            function()  # untimed component warmup
            timings[key] = timing_statistics([_timed(function)[1] for _ in range(repeats)])
        allocation = functions["robot_equilibrium_allocation"]()
        reports[name] = {"q": q.tolist(), "rank": int(np.linalg.matrix_rank(matrix, tol=1e-8)),
                         "allocation_status": allocation.status.value, "timings": timings}
    return reports


def _profile(function, filename):
    profiler = cProfile.Profile()
    profiler.runcall(function)
    profiler.dump_stats(str(filename.with_suffix(".pstats")))
    stats = pstats.Stats(profiler)
    stream = io.StringIO()
    pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(25)
    filename.with_suffix(".txt").write_text(stream.getvalue(), encoding="utf-8")
    rows = [{"file": str(Path(file).name), "source_path": file, "line": line, "function": name,
             "calls": calls, "self_s": self_s, "cumulative_s": cumulative}
            for (file, line, name), (_, calls, self_s, cumulative, _) in stats.stats.items()]
    rows.sort(key=lambda row: row["cumulative_s"], reverse=True)
    def entries(file, name):
        return [r for r in rows if r["source_path"].replace("\\", "/").endswith(file) and r["function"] == name]
    fractions = {}
    for key, file, name in [("fk", "kinematics/inverse.py", "solve_configuration_from_lengths"),
                            ("jacobian", "kinematics/jacobians.py", "cable_jacobian"),
                            ("allocation_api", "allocation/reference.py", "solve_equilibrium_tensions"),
                            ("pure_qp", "allocation/reference.py", "allocate_reference_tensions")]:
        fractions[key] = sum(r["cumulative_s"] for r in entries(file, name))/stats.total_tt if stats.total_tt else 0.
    return {"total_profiled_s": stats.total_tt, "top_cumulative": rows[:25],
            "nested_cumulative_fractions_NOT_ADDITIVE": fractions,
            "length_evaluations": sum(r["calls"] for r in entries("kinematics/cables.py", "cable_lengths")),
            "jacobian_evaluations": sum(r["calls"] for r in entries("kinematics/jacobians.py", "cable_jacobian")),
            "note": "Profiling is a separate instrumented pass; not headline latency. FK and Jacobian times overlap."}


def _git(arguments):
    try:
        return subprocess.check_output(["git", *arguments], text=True, cwd=Path(__file__).resolve().parents[1]).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def run_study(output_dir, *, frames=500, frequency=50., repeats=30, profile_frames=30, label="study"):
    if isinstance(repeats, bool) or not isinstance(repeats, int) or repeats < 1:
        raise ValueError("repeats must be a positive integer")
    if isinstance(profile_frames, bool) or not isinstance(profile_frames, int) or profile_frames < 1:
        raise ValueError("profile_frames must be a positive integer")
    # Validate workload options before output mutation.
    workload("practical_sample", frames, frequency)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    configuration = io.StringIO()
    with contextlib.redirect_stdout(configuration):
        np.show_config()
    report = {"label": label, "utc": datetime.now(timezone.utc).isoformat(),
              "git_commit": _git(["rev-parse", "HEAD"]), "dirty_files": _git(["diff", "--name-only"]),
              "environment": {"python": sys.version, "executable": sys.executable, "numpy": np.__version__,
                              "platform": platform.platform(), "processor": platform.processor(),
                              "numpy_configuration": configuration.getvalue(),
                              "thread_environment": {key: os.environ.get(key) for key in
                                                     ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "OMP_NUM_THREADS")}},
              "settings": {"frames": frames, "frequency_hz": frequency, "virtual_duration_s": frames/frequency,
                           "component_repeats": repeats, "profile_frames": min(profile_frames, frames),
                           "fk_tolerance": 1e-9, "allocation_tolerance": 1e-8, "cycle_budget_ms": 20,
                           "notes": "No pacing, communication, plotting, scheduling or hardware. First FK guess equals first true pose."},
              "workloads": {}}
    for name in ("practical_sample", "repository_rotated"):
        robot, qs, reference = workload(name, frames, frequency)
        solve_equilibrium_tensions(robot, RobotState(qs[0]), reference_tension=reference)
        solve_configuration_from_lengths(robot, robot.cable_lengths(RobotState(qs[0])), RobotState(qs[0]+.001))
        components = _components(robot, reference, repeats)
        a, b = _mode_a(robot, qs, reference), _mode_b(robot, qs, reference)
        nprofile = min(profile_frames, frames)
        profile_pipeline = _profile(lambda: _mode_b(robot, qs[:nprofile], reference), output_dir / f"{name}_pipeline_profile")
        # Precompute measured lengths outside the FK-only profile.
        measured = [robot.cable_lengths(RobotState(q)) for q in qs[:nprofile]]
        def fk_profile():
            previous = RobotState(qs[0])
            for lengths in measured:
                result = solve_configuration_from_lengths(robot, lengths, previous)
                if result.converged:
                    previous = result.state
        profile_fk = _profile(fk_profile, output_dir / f"{name}_fk_profile")
        report["workloads"][name] = {"reference_tension": reference, "model": robot.name,
                                     "components": components, "mode_A_pose_known": a, "mode_B_virtual_sensing": b,
                                     "pipeline_profile": profile_pipeline, "fk_only_profile": profile_fk}
        print(name, "A:", json.dumps(a["timing"]), "B compute:", json.dumps(b["timings"]["compute_frame_ms"]), flush=True)
        print("success A/FK/B:", a["feasible"], b["fk_converged"], b["allocation_feasible"],
              "profile fractions:", profile_pipeline["nested_cumulative_fractions_NOT_ADDITIVE"], flush=True)
    (output_dir / "benchmark_results.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("examples/output/performance/study"))
    parser.add_argument("--label", default="study")
    parser.add_argument("--frames", type=int, default=500)
    parser.add_argument("--frequency", type=float, default=50.)
    parser.add_argument("--repeats", type=int, default=30)
    parser.add_argument("--profile-frames", type=int, default=30)
    args = parser.parse_args()
    run_study(args.output_dir, frames=args.frames, frequency=args.frequency, repeats=args.repeats,
              profile_frames=args.profile_frames, label=args.label)


if __name__ == "__main__":
    main()
