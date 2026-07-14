#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simulation_waypoints_to_nodes_json.py

Simulation-only helper for util_3dnav.

It converts planning_ws/scripts/odom_waypoints.txt into the Robot_Midware
nodes.json format used by midware_bridge_node.py, and also initializes a
simulation task_state.json. The output files are removed and recreated every
time this node starts, so simulation debug always uses the latest extracted
odom waypoints.
"""

import fcntl
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

import rospy


DEFAULT_WAYPOINTS_FILE = Path("/home/xxh/king_ws/planning_ws/scripts/odom_waypoints.txt")


def yaw_to_quaternion(yaw: float) -> dict:
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw * 0.5),
        "w": math.cos(yaw * 0.5),
    }


def load_waypoints(filepath: Path) -> Tuple[Dict[str, dict], List[str]]:
    if not filepath.exists():
        raise FileNotFoundError(f"waypoints file does not exist: {filepath}")

    nodes: Dict[str, dict] = {}
    path_ids: List[str] = []
    with filepath.open("r", encoding="utf-8") as f:
        for line_no, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) < 5:
                rospy.logwarn(
                    "[util_3dnav simulation] skip invalid waypoint line %d: expected 5 columns, got %d",
                    line_no,
                    len(parts),
                )
                continue

            try:
                waypoint_id = str(int(float(parts[0])))
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                heading = float(parts[4])
            except ValueError as exc:
                rospy.logwarn("[util_3dnav simulation] skip invalid waypoint line %d: %s", line_no, exc)
                continue

            nodes[waypoint_id] = {
                "id": waypoint_id,
                "pose": {
                    "position": {
                        "x": x,
                        "y": y,
                        "z": z,
                    },
                    "orientation": yaw_to_quaternion(heading),
                },
                "heading_rad": heading,
            }
            path_ids.append(waypoint_id)

    if not nodes:
        raise ValueError(f"no valid waypoints found in {filepath}")
    return nodes, path_ids


def write_json_file(filepath: Path, data: dict) -> None:
    filepath.parent.mkdir(parents=True, exist_ok=True)

    if filepath.exists():
        filepath.unlink()
        rospy.loginfo("[util_3dnav simulation] removed old json: %s", filepath)

    with filepath.open("w", encoding="utf-8") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
        f.flush()


def expand_task_route(task_route: str, path_ids: List[str]) -> List[str]:
    if len(path_ids) < 2:
        raise ValueError("simulation task requires at least two waypoint IDs")

    route = str(task_route or "full").strip().lower()
    if route in ("", "full", "all"):
        return path_ids

    parts = route.replace("_", "-").split("-")
    try:
        milestones = [int(p) for p in parts]
    except ValueError as exc:
        raise ValueError(
            f"unsupported task_route '{task_route}', expected numeric IDs separated by '-'"
        ) from exc

    if len(milestones) < 2:
        raise ValueError(
            f"unsupported task_route '{task_route}', need at least two milestone IDs"
        )

    # Build the full waypoint sequence by walking each consecutive segment.
    # e.g. "0-20-0-15-0" -> 0..20, 19..0, 1..15, 14..0
    available = set(path_ids)
    expanded: List[str] = [str(milestones[0])]
    for i in range(1, len(milestones)):
        prev, curr = milestones[i - 1], milestones[i]
        if prev < curr:
            segment = [str(x) for x in range(prev + 1, curr + 1)]
        elif prev > curr:
            segment = [str(x) for x in range(prev - 1, curr - 1, -1)]
        else:
            continue  # same milestone twice, skip
        expanded.extend(segment)

    missing = [node_id for node_id in expanded if node_id not in available]
    if missing:
        raise ValueError(
            f"task_route '{task_route}' references missing waypoint IDs: {', '.join(missing[:10])}"
        )

    return expanded


def build_task_state(map_name: str, path_ids: List[str], task_route: str) -> dict:
    if len(path_ids) < 2:
        raise ValueError("simulation task requires at least two waypoint IDs")

    task_path = expand_task_route(task_route, path_ids)

    return {
        "status": "running",
        "map_name": map_name,
        "path": task_path,
        "visited": [task_path[0]],
        "current_target": task_path[1],
        "current_index": 1,
        "message": f"simulation task initialized: {task_route}, points={len(task_path)}",
    }


def main() -> None:
    rospy.init_node("simulation_waypoints_to_nodes_json")

    waypoints_file = Path(rospy.get_param("~waypoints_file", str(DEFAULT_WAYPOINTS_FILE)))
    nodes_output_file = Path(rospy.get_param("~output_file"))
    task_state_file = Path(rospy.get_param("~task_state_file", ""))
    map_name = rospy.get_param("~map_name", "campus")
    task_route = rospy.get_param("~task_route", "0-100-0")

    nodes, path_ids = load_waypoints(waypoints_file)
    write_json_file(nodes_output_file, nodes)

    if str(task_state_file):
        task_state = build_task_state(map_name, path_ids, task_route)
        write_json_file(task_state_file, task_state)
        rospy.loginfo(
            "[util_3dnav simulation] initialized task json: %s route=%s points=%d",
            task_state_file,
            task_route,
            len(task_state["path"]),
        )

    rospy.loginfo(
        "[util_3dnav simulation] wrote %d waypoint nodes from %s to %s",
        len(nodes),
        waypoints_file,
        nodes_output_file,
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("[util_3dnav simulation] failed to generate nodes json: %s", exc)
        raise
