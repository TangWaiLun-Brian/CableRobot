"""Equal tension vs pose-dependent full equilibrium; no hardware/effort claims.

Run: python -m examples.equilibrium_allocation --output-dir examples/output/equilibrium
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from cablerobot import AllocationStatus, RobotState, gravity_generalized_force, solve_equilibrium_tensions
from cablerobot.examples import spatial_cdpr
from cablerobot.kinematics.cables import cable_route_points


def evaluate_pose(robot, state, *, reference_tension=20.0, equal_tension=20.0):
    """Audit-ready single-pose data from the existing geometry and force maps."""
    result = solve_equilibrium_tensions(robot, state, reference_tension=reference_tension)
    matrix = robot.cable_force_matrix(state)
    gravity = gravity_generalized_force(robot, state)
    lower, upper = robot.tension_bounds()
    baseline = np.full(robot.cable_count, equal_tension)
    baseline_residual = matrix @ baseline + gravity
    directions = []
    for cable in robot.cables:
        segments = np.diff(cable_route_points(robot, cable, state), axis=0)
        directions.append((segments / np.linalg.norm(segments, axis=1)[:, None]).tolist())
    return {
        "q": state.q.tolist(), "cable_names": [c.name for c in robot.cables],
        "lengths_m": robot.cable_lengths(state).tolist(),
        "route_segment_directions_world": directions, "B": matrix.tolist(),
        "gravity_generalized_force": gravity.tolist(),
        "lower_bounds_N": lower.tolist(), "upper_bounds_N": upper.tolist(),
        "reference_tension_N": result.reference_tension.tolist(), "weights": result.weights.tolist(),
        "equal_tensions_N": baseline.tolist(), "equal_within_bounds": bool(np.all((baseline >= lower) & (baseline <= upper))),
        "equal_residual": baseline_residual.tolist(), "equal_residual_norm": float(np.linalg.norm(baseline_residual)),
        "optimized_tensions_N": result.tensions.tolist(), "optimized_residual": result.residual.tolist(),
        "optimized_residual_norm": result.residual_norm, "status": result.status.value,
        "tension_role": "optimized equilibrium" if result.feasible else "diagnostic candidate; not a command",
        "command_available": result.tension_command is not None,
        "objective_value": result.objective_value, "optimality_verified": result.optimality_verified,
        "duality_gap": result.duality_gap, "iterations": result.iterations, "backend": result.backend,
        "active_lower": result.active_lower.tolist(), "active_upper": result.active_upper.tolist(),
        "lower_margin_N": result.lower_margin.tolist(), "upper_margin_N": result.upper_margin.tolist(),
        "reference_distance_N": float(np.linalg.norm(result.tensions - result.reference_tension)),
        "full_residual_acceptance_threshold": 1e-8 * (1 + float(np.linalg.norm(gravity))),
    }


def run_validation(output_dir: Path, *, samples=81):
    if isinstance(samples, bool) or not isinstance(samples, int) or samples < 3:
        raise ValueError("samples must be an integer >= 3")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    robot, initial = spatial_cdpr()
    origin = evaluate_pose(robot, initial)
    rotated = evaluate_pose(robot, RobotState([0.05, -0.04, 0.03, 0.08, -0.06, 0.05]))
    for record in (origin, rotated):
        assert record["status"] == AllocationStatus.FEASIBLE.value
        assert record["optimized_residual_norm"] <= record["full_residual_acceptance_threshold"]
        assert record["equal_residual_norm"] > 1
    limited_robot, limited_state = spatial_cdpr()
    for cable in limited_robot.cables:
        cable.tension_max = 2.0
    infeasible = evaluate_pose(limited_robot, limited_state, equal_tension=2.0)
    assert infeasible["status"] == AllocationStatus.INFEASIBLE.value
    assert infeasible["equal_within_bounds"]

    parameter = np.linspace(-1, 1, samples)
    sweep = []
    for value in parameter:
        state = RobotState(initial.q + np.array([0.08*value, 0.024*value, -0.016*value, 0, 0, 0]))
        record = evaluate_pose(robot, state)
        record["path_parameter"] = float(value)
        record["neighbor_max_tension_change_N"] = 0.0 if not sweep else float(np.max(np.abs(
            np.array(record["optimized_tensions_N"]) - np.array(sweep[-1]["optimized_tensions_N"]))))
        record["neighbor_configuration_distance"] = 0.0 if not sweep else float(np.linalg.norm(
            state.q - np.array(sweep[-1]["q"])))
        sweep.append(record)
    assert all(record["status"] == "feasible" for record in sweep)
    assert all(record["optimized_residual_norm"] <= record["full_residual_acceptance_threshold"] for record in sweep)
    summary = {
        "samples": samples,
        "feasible_samples": sum(record["status"] == "feasible" for record in sweep),
        "equal_residual_norm_min": min(record["equal_residual_norm"] for record in sweep),
        "equal_residual_norm_max": max(record["equal_residual_norm"] for record in sweep),
        "optimized_residual_norm_max": max(record["optimized_residual_norm"] for record in sweep),
        "max_neighbor_tension_change_N": max(record["neighbor_max_tension_change_N"] for record in sweep),
        "max_neighbor_configuration_distance": max(record["neighbor_configuration_distance"] for record in sweep),
        "min_lower_reserve_N": min(min(record["lower_margin_N"]) for record in sweep),
        "min_upper_reserve_N": min(min(record["upper_margin_N"]) for record in sweep),
        "active_set_changes": sum((current["active_lower"], current["active_upper"]) !=
                                  (previous["active_lower"], previous["active_upper"])
                                  for previous, current in zip(sweep, sweep[1:])),
        "interpretation": "Sampled static model only; no hardware neutrality, friction, real-time or global continuity claim.",
    }
    report = {
        "model": robot.name, "mass_kg": robot.bodies["moving_platform"].mass,
        "center_of_mass_body_m": robot.bodies["moving_platform"].center_of_mass.tolist(),
        "gravity_world_m_s2": robot.parameters.gravity.tolist(),
        "formulation": "B=-J_l.T; minimize 0.5||W(t-ref)||^2 with B t + gravity = 0 and bounds",
        "residual_norm_note": "Full mixed-coordinate generalized-force norm is unit-dependent; translation N, angular covectors N m.",
        "single_poses": {"origin": origin, "rotated": rotated, "upper_bound_2N": infeasible},
        "sweep_summary": summary, "sweep": sweep,
    }
    (output_dir / "equilibrium_report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    _plot_report(report, output_dir / "equilibrium_sweep.png")
    print("origin: equal residual", origin["equal_residual_norm"], "equilibrium residual", origin["optimized_residual_norm"])
    print("origin optimized tensions [N]:", np.round(origin["optimized_tensions_N"], 6))
    print("2 N upper-bound case:", infeasible["status"], "residual", infeasible["optimized_residual_norm"])
    print("sweep:", json.dumps(summary))
    print("output:", output_dir.resolve())
    return report


def _plot_report(report, destination):
    sweep = report["sweep"]
    parameter = [record["path_parameter"] for record in sweep]
    tensions = np.array([record["optimized_tensions_N"] for record in sweep])
    figure, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True)
    for index, name in enumerate(sweep[0]["cable_names"]):
        axes[0, 0].plot(parameter, tensions[:, index], label=name)
    axes[0, 0].axhline(20, color="black", linestyle=":", label="equal/reference 20 N")
    axes[0, 0].set(title="Individual equilibrium tensions", ylabel="Tension [N]")
    axes[0, 0].legend(fontsize=8, ncols=3)
    for key, label in [("equal_residual_norm", "Equal 20 N"), ("optimized_residual_norm", "Equilibrium"),
                       ("full_residual_acceptance_threshold", "Acceptance threshold")]:
        axes[0, 1].semilogy(parameter, np.maximum([r[key] for r in sweep], 1e-16), label=label)
    axes[0, 1].set(title="Full generalized-force residual (unit-dependent)", ylabel="Mixed-coordinate norm")
    axes[0, 1].legend(fontsize=9)
    axes[1, 0].scatter(parameter, [1 if r["status"] == "feasible" else 0 for r in sweep], s=12)
    axes[1, 0].set(title="Solver status at every sampled pose", yticks=[0, 1], yticklabels=["Not verified", "Feasible"], ylim=(-0.2, 1.2))
    axes[1, 1].plot(parameter, [r["reference_distance_N"] for r in sweep])
    axes[1, 1].set(title="Distance from reference", ylabel="||t - reference|| [N]")
    axes[2, 0].plot(parameter, [min(r["lower_margin_N"]) for r in sweep], label="Minimum lower reserve")
    axes[2, 0].plot(parameter, [min(r["upper_margin_N"]) for r in sweep], label="Minimum upper reserve")
    axes[2, 0].set(title="Reserve to tension bounds", ylabel="Minimum cable reserve [N]")
    axes[2, 0].legend(fontsize=9)
    axes[2, 1].plot(parameter[1:], [r["neighbor_max_tension_change_N"] for r in sweep[1:]])
    axes[2, 1].set(title="Neighbor command change (sampling-dependent)", ylabel="max |t_k - t_(k-1)| [N]")
    for axis in axes.flat:
        axis.set_xlabel("Path parameter s; translation [0.08s, 0.024s, -0.016s] m")
        axis.grid(alpha=0.2)
    figure.suptitle("Static gravity compensation — 6 coordinates, 8 cables; not a hardware/control trial")
    figure.savefig(destination, dpi=130)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("examples/output/equilibrium"))
    parser.add_argument("--samples", type=int, default=81)
    args = parser.parse_args()
    run_validation(args.output_dir, samples=args.samples)


if __name__ == "__main__":
    main()
