"""Validation and command construction for the isolated trajectory laboratory.

This module deliberately has no ROS imports so the most important safety
property can be unit-tested without a ROS installation: every endpoint used by
the laboratory lives below ``/mapping/trajectory_lab`` and parameter overrides
are ephemeral command-line arguments, never writes to navigation YAML files.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Iterable


LAB_NAMESPACE = "/mapping/trajectory_lab"
MAX_LAB_OBSTACLES = 20_000
MAX_PATH_POINTS = 8_000


LAB_TOPICS = {
    "goal": f"{LAB_NAMESPACE}/goal",
    "odom": f"{LAB_NAMESPACE}/odometry",
    "cloud": f"{LAB_NAMESPACE}/obstacles",
    "cmd_vel": f"{LAB_NAMESPACE}/cmd_vel",
    "cost_map": f"{LAB_NAMESPACE}/cost_map",
    "terrain_label_map": f"{LAB_NAMESPACE}/terrain_label_map",
    "global_plan": f"{LAB_NAMESPACE}/global_plan",
    "minco_path": f"{LAB_NAMESPACE}/minco_path",
    "opt_path": f"{LAB_NAMESPACE}/opt_path",
    "planning_constraints": f"{LAB_NAMESPACE}/planning_constraints",
    "planning_constraints_markers": f"{LAB_NAMESPACE}/planning_constraints_markers",
    "global_plan_marker": f"{LAB_NAMESPACE}/debug/global_plan",
    "minco_trajectory_marker": f"{LAB_NAMESPACE}/debug/minco_trajectory",
    "safe_corridor": f"{LAB_NAMESPACE}/debug/safe_corridor",
}


@dataclass(frozen=True)
class LabParameter:
    parameter: str
    default: float
    minimum: float
    maximum: float


LAB_PARAMETERS = {
    "safe_dist": LabParameter("planner.minco_optimizer.safe_dist", 0.33, 0.10, 1.00),
    "collision_dist": LabParameter("planner.minco_optimizer.collision_dist", 0.28, 0.10, 0.80),
    "max_velocity": LabParameter("planner.minco_optimizer.max_velocity", 3.0, 0.10, 6.00),
    "max_acceleration": LabParameter("planner.minco_optimizer.max_acceleration", 4.0, 0.10, 10.00),
    "penalty_weight_time": LabParameter("planner.minco_optimizer.penalty_weight_time", 100.0, 0.0, 5000.0),
    "esdf_weight": LabParameter("planner.smac_2d.esdf_weight", 1.0, 0.0, 20.0),
}


def validate_parameters(raw: Any) -> dict[str, float]:
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError("轨迹参数必须是对象")
    unknown = sorted(set(raw) - set(LAB_PARAMETERS))
    if unknown:
        raise ValueError(f"不支持的轨迹参数：{', '.join(unknown)}")
    values: dict[str, float] = {}
    for name, spec in LAB_PARAMETERS.items():
        value = raw.get(name, spec.default)
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{name} 必须是数值") from error
        if not math.isfinite(number) or not spec.minimum <= number <= spec.maximum:
            raise ValueError(f"{name} 必须在 {spec.minimum:g}–{spec.maximum:g} 之间")
        values[name] = number
    if values["safe_dist"] < values["collision_dist"]:
        raise ValueError("safe_dist 不能小于 collision_dist")
    return values


def validate_pose(raw: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) not in {2, 3}:
        raise ValueError(f"{label}必须是 [x, y] 或 [x, y, yaw]")
    try:
        values = tuple(float(item) for item in raw)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label}坐标必须是数值") from error
    if not all(math.isfinite(item) for item in values):
        raise ValueError(f"{label}坐标必须是有限数值")
    return values[0], values[1], values[2] if len(values) == 3 else 0.0


def validate_obstacles(raw: Any) -> list[tuple[float, float]]:
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ValueError("临时障碍必须是坐标数组")
    if len(raw) > MAX_LAB_OBSTACLES:
        raise ValueError(f"临时障碍不能超过 {MAX_LAB_OBSTACLES:,} 个采样点")
    result: list[tuple[float, float]] = []
    for index, point in enumerate(raw):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(f"第 {index + 1} 个障碍坐标格式无效")
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError) as error:
            raise ValueError(f"第 {index + 1} 个障碍坐标必须是数值") from error
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"第 {index + 1} 个障碍坐标必须是有限数值")
        result.append((x, y))
    return result


def _format_parameter(value: float) -> str:
    """Format a double so ROS 2 reads it back as a double, not an integer.

    ``f"{1.0:g}"`` renders as ``"1"``, and ``-p name:=1`` makes ROS 2 reject the
    override for a parameter declared as double ("is of type {integer}"), which
    aborts the isolated planner before it can plan anything.  Four of the six
    whitelisted defaults are whole numbers, so every value keeps an explicit
    decimal point or exponent.
    """
    text = f"{value:g}"
    if not any(character in text for character in ".eE"):
        text += ".0"
    return text


def _parameter_arguments(parameters: dict[str, float]) -> list[str]:
    arguments: list[str] = []
    for name, value in parameters.items():
        arguments.extend(["-p", f"{LAB_PARAMETERS[name].parameter}:={_format_parameter(value)}"])
    return arguments


def build_lab_commands(
    *,
    terrain_path: Path,
    yaml_path: Path,
    config_dir: Path,
    parameters: dict[str, float],
) -> dict[str, list[str]]:
    """Build the two real ROS child commands with fully isolated topics."""
    terrain = str(Path(terrain_path).resolve())
    yaml = str(Path(yaml_path).resolve())
    configs = Path(config_dir).resolve()
    map_command = [
        "ros2", "run", "map_server", "map_server_node", "--ros-args",
        "-r", "__node:=trajectory_lab_map_server",
        "-r", f"__ns:={LAB_NAMESPACE}",
        "-p", f"terrain_map_path:={terrain}",
        "-p", f"map_yaml_path:={yaml}",
        "-p", "frame_id:=map",
        "-r", f"cost_map:={LAB_TOPICS['cost_map']}",
        "-r", f"terrain_label_map:={LAB_TOPICS['terrain_label_map']}",
        "-r", f"/cost_map:={LAB_TOPICS['cost_map']}",
        "-r", f"/terrain_label_map:={LAB_TOPICS['terrain_label_map']}",
    ]
    executor_command = [
        "ros2", "run", "mas2027_nav_executor", "mas2027_nav_executor_node", "--ros-args",
        "--params-file", str(configs / "planner_params.yaml"),
        "--params-file", str(configs / "node_params.yaml"),
        "--params-file", str(configs / "mpc_params.yaml"),
        "-r", "__node:=trajectory_lab_nav_executor",
        "-r", f"__ns:={LAB_NAMESPACE}",
        "-p", "node.frames.odom:=map",
        "-p", f"node.topics.goal_sub:={LAB_TOPICS['goal']}",
        "-p", f"node.topics.odom_sub:={LAB_TOPICS['odom']}",
        "-p", f"node.topics.trajectory_sub:={LAB_TOPICS['opt_path']}",
        "-p", f"node.topics.cmd_vel_pub:={LAB_TOPICS['cmd_vel']}",
        "-p", f"node.topics.terrain_cost_sub:={LAB_TOPICS['cost_map']}",
        "-p", f"node.topics.terrain_label_sub:={LAB_TOPICS['terrain_label_map']}",
        "-p", f"node.topics.global_plan_pub:={LAB_TOPICS['global_plan']}",
        "-p", f"node.topics.minco_path_pub:={LAB_TOPICS['minco_path']}",
        "-p", f"node.topics.global_plan_marker_pub:={LAB_TOPICS['global_plan_marker']}",
        "-p", f"node.topics.minco_trajectory_pub:={LAB_TOPICS['minco_trajectory_marker']}",
        "-p", f"node.topics.planning_constraints_pub:={LAB_TOPICS['planning_constraints']}",
        "-p", f"node.topics.planning_constraints_marker_pub:={LAB_TOPICS['planning_constraints_markers']}",
        "-p", "planner.global_frame:=map",
        "-p", "planner.frames.map_frame:=map",
        "-p", "planner.frames.rog_frame:=map",
        "-p", "planner.rog_map.frame_id:=map",
        "-p", f"planner.odom_topic:={LAB_TOPICS['odom']}",
        "-p", f"planner.rog_map.ros_callback.odom_topic:={LAB_TOPICS['odom']}",
        "-p", f"planner.rog_map.ros_callback.cloud_topic:={LAB_TOPICS['cloud']}",
        "-p", "planner.rog_map.performance.enable:=false",
        "-p", "planner.rog_map.performance.summary_csv_enable:=false",
        "-p", "planner.rog_map.performance.detailed_csv_enable:=false",
        "-p", "planner.rog_map.visualization.enable:=false",
        "-r", f"/opt_path:={LAB_TOPICS['opt_path']}",
        "-r", f"/nav_executor/debug/safe_corridor:={LAB_TOPICS['safe_corridor']}",
    ]
    executor_command.extend(_parameter_arguments(parameters))
    return {"map_server": map_command, "nav_executor": executor_command}


def decimate_path(points: Iterable[tuple[float, float]], limit: int = MAX_PATH_POINTS) -> list[list[float]]:
    result = list(points)
    if len(result) > limit:
        step = math.ceil(len(result) / limit)
        reduced = result[::step]
        if reduced[-1] != result[-1]:
            reduced.append(result[-1])
        result = reduced
    return [[float(x), float(y)] for x, y in result]
