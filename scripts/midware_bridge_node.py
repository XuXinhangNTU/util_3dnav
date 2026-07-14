#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
midware_bridge_node.py

Bridge Robot_Midware JSON tasks to the 3D navigation planner.

Responsibilities:
  - Poll task_state.json from Robot_Midware.
  - Resolve map nodes from nodes.json.
  - Publish the active task path as nav_msgs/Path, normally to /pct_path.
  - Watch odometry and write visited/current_target/completed progress back.
"""

import copy
import fcntl
import json
import math
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, Int8


# Speed preset → (max_vel m/s, max_acc m/s²) for /speed_limit/set
_SPEED_PRESETS: Dict[int, Tuple[float, float]] = {
    0: (1.0, 2.0),   # normal
    1: (0.5, 2.0),   # slow
    2: (2.0, 2.0),  # fast
}

# Human-readable messages for each /planning/dodge_status value.
_DODGE_OBS_MESSAGES: Dict[int, str] = {
    0: "",
    1: "避障中",
    2: "无法绕过障碍，等待清除",
}

DEFAULT_DATA_DIR = Path(
    "/home/root01/workspace/ws_robot/src/Robot_Midware/ros/robot_midware/flask_service/data"
)


def read_json(filepath: Path) -> dict:
    if not filepath.exists():
        return {}
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError, ValueError):
        return {}


def write_json(filepath: Path, updates: dict) -> None:
    filepath.parent.mkdir(parents=True, exist_ok=True)
    mode = "r+" if filepath.exists() else "w"
    with open(filepath, mode, encoding="utf-8") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        try:
            f.seek(0)
            raw = f.read()
            state = json.loads(raw) if raw.strip() else {}
        except (json.JSONDecodeError, ValueError):
            state = {}

        state.update(updates)
        f.seek(0)
        f.truncate()
        f.write(json.dumps(state, ensure_ascii=False, indent=2))
        f.write("\n")


def pose_position(pose: dict) -> Tuple[float, float, float]:
    pos = pose.get("position", {})
    return float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), float(pos.get("z", 0.0))


def yaw_to_quaternion(yaw: float) -> dict:
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw * 0.5),
        "w": math.cos(yaw * 0.5),
    }


def normalized_orientation(pose: dict, fallback_yaw: float = 0.0) -> dict:
    ori = pose.get("orientation")
    if not isinstance(ori, dict):
        return yaw_to_quaternion(fallback_yaw)

    q = {
        "x": float(ori.get("x", 0.0)),
        "y": float(ori.get("y", 0.0)),
        "z": float(ori.get("z", 0.0)),
        "w": float(ori.get("w", 1.0)),
    }
    norm = math.sqrt(q["x"] ** 2 + q["y"] ** 2 + q["z"] ** 2 + q["w"] ** 2)
    if norm < 1e-6:
        return yaw_to_quaternion(fallback_yaw)
    return {k: v / norm for k, v in q.items()}


class MidwareBridge:
    def __init__(self) -> None:
        rospy.init_node("midware_bridge_node")

        data_dir = rospy.get_param("~data_dir", str(DEFAULT_DATA_DIR))
        self.data_dir = Path(data_dir)
        self.task_file = self.data_dir / "task_state.json"
        self.robot_file = self.data_dir / "robot_state.json"
        self.maps_dir = self.data_dir / "maps"

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.path_topic = rospy.get_param("~path_topic", "/pct_path")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry_gazebo")
        self.poll_rate = float(rospy.get_param("~poll_rate", 2.0))
        self.republish_period = float(rospy.get_param("~republish_period", 2.0))
        self.path_point_spacing = max(0.05, float(rospy.get_param("~path_point_spacing", 0.8)))
        self.arrival_threshold = float(rospy.get_param("~arrival_threshold", 0.6))
        self.turnaround_arrival_threshold = float(
            rospy.get_param("~turnaround_arrival_threshold", 0.5)
        )
        # Tighter XY arrival tolerance (m) for geometric corners -- an interior
        # waypoint B where the three consecutive nodes A-B-C bend by more than
        # corner_angle_threshold degrees. Lets the robot actually reach the bend
        # before cutting to the next leg. Set both from the launch file.
        self.corner_arrival_threshold = float(
            rospy.get_param("~corner_arrival_threshold", 0.5)
        )
        # Minimum turn angle (deg, measured in the XY plane) for A-B-C to count
        # as a corner. 0 = straight-through, 180 = full reversal. <=0 disables
        # corner detection.
        self.corner_angle_threshold = float(
            rospy.get_param("~corner_angle_threshold", 45.0)
        )
        self.z_arrival_threshold = float(rospy.get_param("~z_arrival_threshold", 1.0))
        # After the robot arrives at the route's FINAL destination the task is
        # marked completed and path publishing would normally stop immediately.
        # traj_server treats the pct_path as a heartbeat and halts the robot
        # once it goes stale, which can cut off the last stretch before the
        # robot physically finishes it. Keep republishing the final leg's path
        # for this long after arrival so the last segment is actually driven to
        # the end. 0 disables the hold (stop publishing at arrival, old behaviour).
        self.completion_hold_sec = max(0.0, float(rospy.get_param("~completion_hold_sec", 2.0)))
        self.z_bias = float(rospy.get_param("~z_bias", 0.0))
        self.remaining_only = bool(rospy.get_param("~publish_remaining_path_only", True))
        self.set_robot_status = bool(rospy.get_param("~set_robot_status_navigating", True))

        self.state_lock = threading.Lock()
        self.current_position: Optional[Tuple[float, float, float]] = None
        self.cached_task: Optional[dict] = None
        self.cached_nodes: Optional[dict] = None
        self.pending_task: Optional[dict] = None
        self.pending_updates: Optional[dict] = None
        self.last_task_key: Optional[Tuple] = None
        self.last_publish_time = rospy.Time(0)
        # Last pct_path actually published, re-sent verbatim during the
        # post-arrival completion hold to keep traj_server's heartbeat alive.
        self.last_path_msg: Optional[RosPath] = None
        # Wall-clock time until which the completion hold republishes the final
        # path; None when no hold is active.
        self.completion_hold_until: Optional[rospy.Time] = None
        self.active_target: str = ""
        self.was_running = False
        self.last_route_signature: Optional[Tuple[str, ...]] = None
        # Protected by state_lock; -1 = not yet received from topic.
        self._dodge_status: int = -1
        self._dodge_status_last_written: int = -1
        # Tracking for robot_state.json command fields; -1 = not yet read.
        # Only accessed in spin loop (single thread) -- no lock needed.
        self._last_nav_mode: int = -1
        self._last_speed: int = -1
        self._last_detour: float = -1.0
        # Extra slack (m) on top of DetourLimit for the derived arrival
        # thresholds (tracking error + controller finish tolerance).
        self.arrival_margin = float(rospy.get_param("~arrival_margin", 0.3))
        # True while the terminate WE sent (upper-system cancel) explains the
        # planner being in STOPPED, so it isn't misreported as a failure.
        self._sent_terminate = False
        # Latest /mission/state from the planner (-1 = never received).
        # 0 IDLE / 1 ACTIVE / 2 PAUSED_BLOCKED / 3 STOPPED.
        self._mission_state: int = -1
        # Publish /mission/heartbeat while a task is running so the planner's
        # watchdog stops the robot if this bridge / the upper system dies.
        self.enable_heartbeat = bool(rospy.get_param("~enable_heartbeat", True))

        self.path_pub = rospy.Publisher(self.path_topic, RosPath, queue_size=1, latch=True)
        self.bounded_dodge_enable_pub = rospy.Publisher(
            "/planning/bounded_dodge_enable", Bool, queue_size=1
        )
        self.speed_limit_pub = rospy.Publisher(
            "/speed_limit/set", Float64MultiArray, queue_size=1
        )
        # Task lifecycle control into the planner's MissionFSM: 1 = terminate
        # (emergency stop + latch STOPPED), 0 = RUN (clear the latch). Sent on
        # upper-system cancel and before each new task respectively -- a normal
        # completion sends NOTHING (the robot is already standing and IDLE).
        self.task_control_pub = rospy.Publisher("/mission/task_control", Int8, queue_size=1)
        self.heartbeat_pub = rospy.Publisher("/mission/heartbeat", Empty, queue_size=1)
        # Runtime max-detour corridor width (meters), from
        # robot_state.json navigation.DetourLimit.
        self.detour_limit_pub = rospy.Publisher("/detour_limit/set", Float64, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        self.dodge_status_sub = rospy.Subscriber(
            "/planning/dodge_status", Int8, self._dodge_status_callback, queue_size=1
        )
        self.mission_state_sub = rospy.Subscriber(
            "/mission/state", Int8, self._mission_state_callback, queue_size=1
        )

        # Heartbeat on an independent timer (5 Hz), not the poll loop: json
        # file IO in spin() can occasionally stall a tick past the planner's
        # heartbeat_timeout and trigger a spurious safety stop.
        if self.enable_heartbeat:
            self._heartbeat_timer = rospy.Timer(
                rospy.Duration(0.2), self._heartbeat_tick
            )

        rospy.loginfo("[util_3dnav] data_dir=%s", self.data_dir)
        rospy.loginfo("[util_3dnav] publishing task path to %s", self.path_topic)
        rospy.loginfo("[util_3dnav] watching odometry from %s", self.odom_topic)

    def _dodge_status_callback(self, msg: Int8) -> None:
        with self.state_lock:
            self._dodge_status = int(msg.data)

    def _mission_state_callback(self, msg: Int8) -> None:
        with self.state_lock:
            self._mission_state = int(msg.data)

    def _heartbeat_tick(self, _event) -> None:
        # Only while a task is live: the planner arms its watchdog on the
        # first heartbeat it sees, and clears it on goal-reached/RUN.
        if self.was_running:
            self.heartbeat_pub.publish(Empty())

    def _effective_thresholds(self) -> Tuple[float, float]:
        """(normal_arrival, corner_arrival) thresholds for this moment.

        The planner's max-detour corridor legitimately lets the optimized
        trajectory pass a waypoint offset by up to DetourLimit (worst at
        corners, which get cut from the inside) -- with a FIXED arrival
        radius smaller than that, the robot can drive the whole route
        without ever "arriving" anywhere. So whenever the upper system set
        navigation.DetourLimit, the normal and corner thresholds are derived
        from it (DetourLimit + arrival_margin). Turnaround pivots and the
        final destination deliberately keep their own tight threshold: each
        is the ENDPOINT of a published path phase, which the planner drives
        to exactly, corridor or not.
        """
        if self._last_detour > 0.0:
            derived = self._last_detour + self.arrival_margin
            return derived, derived
        return self.arrival_threshold, self.corner_arrival_threshold

    def _deep_update_robot_state(self, nested: Dict) -> None:
        """Deep-merge nested dict into robot_state.json under file lock.

        Each value in `nested` that is itself a dict is merged into the
        corresponding top-level dict in the file (e.g. {"navigation": {"ObsMode": 1}}
        updates only that key, leaving all other navigation fields intact).
        Non-dict values are set directly.
        """
        filepath = self.robot_file
        filepath.parent.mkdir(parents=True, exist_ok=True)
        mode = "r+" if filepath.exists() else "w"
        with open(filepath, mode, encoding="utf-8") as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            try:
                f.seek(0)
                raw = f.read()
                state = json.loads(raw) if raw.strip() else {}
            except (json.JSONDecodeError, ValueError):
                state = {}
            for key, value in nested.items():
                if isinstance(value, dict):
                    state.setdefault(key, {}).update(value)
                else:
                    state[key] = value
            f.seek(0)
            f.truncate()
            f.write(json.dumps(state, ensure_ascii=False, indent=2))
            f.write("\n")

    def _read_robot_state(self) -> dict:
        """Read robot_state.json under shared file lock."""
        filepath = self.robot_file
        if not filepath.exists():
            return {}
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                fcntl.flock(f, fcntl.LOCK_SH)
                raw = f.read()
            return json.loads(raw) if raw.strip() else {}
        except (json.JSONDecodeError, OSError, ValueError):
            return {}

    def _apply_robot_commands(self) -> None:
        """Read NavMode and Speed from robot_state.json and publish to planner topics if changed."""
        robot = self._read_robot_state()
        nav = robot.get("navigation", {})

        nav_mode = nav.get("NavMode", 0)
        if nav_mode != self._last_nav_mode:
            # NavMode 0 = dodge enabled (bounded dodge), 1 = stop-and-wait mode
            msg = Bool()
            msg.data = nav_mode == 1
            self.bounded_dodge_enable_pub.publish(msg)
            rospy.loginfo("[util_3dnav] NavMode=%d → bounded_dodge_enable=%s", nav_mode, msg.data)
            self._last_nav_mode = nav_mode

        speed = nav.get("Speed", 0)
        if speed != self._last_speed:
            vel, acc = _SPEED_PRESETS.get(speed, _SPEED_PRESETS[0])
            msg_spd = Float64MultiArray()
            msg_spd.data = [vel, acc]
            self.speed_limit_pub.publish(msg_spd)
            rospy.loginfo("[util_3dnav] Speed=%d → vel=%.2f acc=%.2f", speed, vel, acc)
            self._last_speed = speed

        # Optional runtime corridor width (meters). Forwarded only when the
        # upper system actually sets navigation.DetourLimit > 0; otherwise the
        # planner keeps its launch-time optimization/max_detour.
        try:
            detour = float(nav.get("DetourLimit", -1))
        except (TypeError, ValueError):
            detour = -1.0
        if detour > 0.0 and abs(detour - self._last_detour) > 1e-6:
            self.detour_limit_pub.publish(Float64(data=detour))
            rospy.loginfo("[util_3dnav] DetourLimit=%.2f → /detour_limit/set", detour)
            self._last_detour = detour

    def _write_robot_state_periodic(self, task: Optional[dict]) -> None:
        """Write navigation + perception status fields to robot_state.json each poll tick."""
        with self.state_lock:
            dodge = self._dodge_status if self._dodge_status >= 0 else 0

        visited = list(task.get("visited", [])) if task else []
        current_target = str(task.get("current_target", "") or "") if task else ""

        self._deep_update_robot_state({
            "perception": {
                "ObsState": 1 if dodge >= 1 else 0,
            },
            "navigation": {
                "ObsMode": dodge,
                "Visited": visited,
                "CurrentTarget": current_target,
            },
        })

    def _write_dodge_status_if_changed(self) -> None:
        """Write dodge status to robot_state.json when value changes.
        For status 1/2 also writes a combined message into task_state.json;
        status 0 leaves message untouched (navigation progress owns it then).

        Called from the spin loop (single thread) so _dodge_status_last_written
        needs no lock; only the read of shared state does.
        """
        with self.state_lock:
            status = self._dodge_status
            task = copy.deepcopy(self.cached_task) if self.cached_task else None

        if status < 0 or status == self._dodge_status_last_written:
            return

        self._deep_update_robot_state({
            "perception": {"ObsState": 1 if status >= 1 else 0},
            #"navigation": {"ObsMode": status},
        })

        if status in (1, 2):
            current_target = task.get("current_target", "") if task else ""
            dodge_msg = _DODGE_OBS_MESSAGES.get(status, "")
            message = f"前往航点 {current_target}，{dodge_msg}" if current_target else dodge_msg
            write_json(self.task_file, {"message": message})
            rospy.loginfo("[util_3dnav] dodge_status=%d → ObsMode=%d msg='%s'", status, status, message)
        else:
            rospy.logdebug("[util_3dnav] dodge_status=0 → ObsMode=0, message unchanged")

        self._dodge_status_last_written = status

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        with self.state_lock:
            self.current_position = (p.x, p.y, p.z)
            task = copy.deepcopy(self.cached_task)
            nodes = self.cached_nodes

        if task is None or nodes is None or task.get("status") != "running":
            return

        updated_task, updates = self._compute_arrival_update(task, nodes)
        if updates is None:
            return

        with self.state_lock:
            self.pending_task = updated_task
            self.pending_updates = updates

    def _map_dir(self, map_name: str) -> Path:
        mapped = self.maps_dir / map_name
        if mapped.exists():
            return mapped
        return self.data_dir / map_name

    def _nodes_file(self, map_name: str) -> Path:
        return self._map_dir(map_name) / "nodes.json"

    def _pcd_file(self, map_name: str) -> str:
        map_dir = self._map_dir(map_name)
        exact = map_dir / f"{map_name}.pcd"
        if exact.exists():
            return str(exact)
        candidates = sorted(map_dir.glob("*.pcd"))
        return str(candidates[0]) if candidates else str(exact)

    def _read_task(self) -> dict:
        task = read_json(self.task_file)
        return task if task else {"status": "idle"}

    def _read_nodes(self, map_name: str) -> dict:
        return read_json(self._nodes_file(map_name))

    def _update_task(self, updates: dict) -> None:
        write_json(self.task_file, updates)
        status = updates.get("status")
        if status == "completed":
            self._deep_update_robot_state({
                "status": "completed",
                "perception": {"ObsState": 0},
                "navigation": {
                    "Visited": list(updates.get("visited", [])),
                    "CurrentTarget": "",
                    "ObsMode": 0,
                },
            })
        elif status == "failed":
            write_json(self.robot_file, {"status": "task_failed"})

    def _update_robot_navigation_state(self, map_name: str, path: List[str]) -> None:
        updates = {
            "task_map": map_name,
            "task_path": path,
            "localization_map": map_name,
            "localization_pcd": self._pcd_file(map_name),
        }
        if self.set_robot_status:
            updates["status"] = "navigating"
        write_json(self.robot_file, updates)

    @staticmethod
    def _target_index(task: dict) -> int:
        path = [str(node_id) for node_id in task.get("path", [])]
        current = str(task.get("current_target", "") or "")
        visited = [str(node_id) for node_id in task.get("visited", [])]

        try:
            current_index = int(task.get("current_index", -1))
        except (TypeError, ValueError):
            current_index = -1

        if 0 <= current_index < len(path):
            return current_index

        if current and current in path:
            start_hint = min(max(len(visited), 0), max(len(path) - 1, 0))
            for idx in range(start_hint, len(path)):
                if path[idx] == current:
                    return idx
            return path.index(current)
        if visited:
            next_idx = min(len(visited), max(len(path) - 1, 0))
            return next_idx
        return 1 if len(path) > 1 else 0

    @staticmethod
    def _turnaround_indices(path: List[str]) -> List[int]:
        """Return indices where the traversal direction reverses.

        For paths with integer node IDs (as produced by expand_task_route),
        a pivot is where the sequence changes from ascending to descending or
        vice versa. This correctly handles multi-leg routes like 0-20-0-15-0
        where the seen-set approach fails after the first return leg (all
        subsequent nodes are already in seen, so later pivots are invisible).

        Falls back to the original seen-set approach for non-integer IDs
        (sparse/key-node task paths from the real backend).
        """
        if len(path) < 2:
            return []

        try:
            ids = [int(p) for p in path]
        except ValueError:
            # Non-integer IDs: use original seen-set approach.
            # Handles sparse key-node paths like "A, B, C, A" where A is
            # the revisited anchor and C is the implicit pivot.
            turns = []
            seen: set = set()
            in_revisit_section = False
            for idx, node_id in enumerate(path):
                is_revisit = node_id in seen
                is_motion = idx > 0 and node_id != path[idx - 1]
                if is_revisit and is_motion and not in_revisit_section:
                    pivot_idx = idx - 1
                    if pivot_idx > 0 and (not turns or turns[-1] != pivot_idx):
                        turns.append(pivot_idx)
                if is_motion:
                    in_revisit_section = is_revisit
                seen.add(node_id)
            return turns

        # Integer IDs: detect direction changes (ascending↔descending).
        turns = []
        prev_dir: Optional[int] = None  # +1 ascending, -1 descending
        for i in range(1, len(ids)):
            diff = ids[i] - ids[i - 1]
            if diff == 0:
                continue
            curr_dir = 1 if diff > 0 else -1
            if prev_dir is not None and curr_dir != prev_dir:
                pivot = i - 1
                if not turns or turns[-1] != pivot:
                    turns.append(pivot)
            prev_dir = curr_dir
        return turns

    def _corner_indices(self, path: List[str], nodes: dict) -> set:
        """Return indices of waypoints that form a geometric corner.

        For each interior waypoint B (index i), measure the turn angle in the
        XY plane between the incoming segment A->B and the outgoing segment
        B->C, where A=path[i-1] and C=path[i+1]. A straight-through pass is 0
        deg; a full reversal is 180 deg. Any bend of at least
        corner_angle_threshold degrees marks B as a corner requiring the
        tighter corner_arrival_threshold. Returns an empty set when detection
        is disabled (threshold <= 0) or the path has fewer than three nodes.
        """
        corners: set = set()
        if len(path) < 3 or self.corner_angle_threshold <= 0.0:
            return corners

        positions: List[Optional[Tuple[float, float, float]]] = []
        for node_id in path:
            pose = self._pose_for_node(nodes, node_id)
            positions.append(pose_position(pose) if pose is not None else None)

        threshold_rad = math.radians(self.corner_angle_threshold)
        for i in range(1, len(path) - 1):
            prev_p, curr_p, next_p = positions[i - 1], positions[i], positions[i + 1]
            if prev_p is None or curr_p is None or next_p is None:
                continue
            ax, ay = curr_p[0] - prev_p[0], curr_p[1] - prev_p[1]
            bx, by = next_p[0] - curr_p[0], next_p[1] - curr_p[1]
            na = math.hypot(ax, ay)
            nb = math.hypot(bx, by)
            if na < 1e-6 or nb < 1e-6:
                continue
            cos_turn = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
            turn_angle = math.acos(cos_turn)  # 0 = straight, pi = full reversal
            if turn_angle >= threshold_rad:
                corners.add(i)
        return corners

    @classmethod
    def _phase_info(cls, path: List[str], target_idx: int) -> Tuple[int, int, int, int, str]:
        """Return phase start/end/index/count/label for the current target index."""
        if not path:
            return 0, 0, 0, 1, "单程"

        target_idx = max(0, min(target_idx, len(path) - 1))
        turns = cls._turnaround_indices(path)
        phase_idx = sum(1 for turn_idx in turns if turn_idx < target_idx)
        phase_count = len(turns) + 1
        phase_start = 0 if phase_idx == 0 else turns[phase_idx - 1]
        phase_end = turns[phase_idx] if phase_idx < len(turns) else len(path) - 1

        if phase_count == 1:
            label = "单程"
        elif phase_count == 2:
            label = "去程" if phase_idx == 0 else "回程"
        else:
            label = f"第{phase_idx + 1}段"
        return phase_start, phase_end, phase_idx, phase_count, label

    def _log_route_structure_if_changed(self, path: List[str]) -> None:
        signature = tuple(path)
        if signature == self.last_route_signature:
            return
        self.last_route_signature = signature

        turns = self._turnaround_indices(path)
        if not turns:
            rospy.loginfo("[util_3dnav] route has no repeated-ID return; publish as one phase")
            return

        descriptions = [f"index={idx}, waypoint={path[idx]}" for idx in turns]
        rospy.loginfo(
            "[util_3dnav] detected %d repeated-ID return pivot(s): %s; path will be published in %d phases",
            len(turns),
            "; ".join(descriptions),
            len(turns) + 1,
        )

    def _pose_for_node(self, nodes: dict, node_id: str) -> Optional[dict]:
        node = nodes.get(str(node_id))
        if not isinstance(node, dict):
            return None
        pose = node.get("pose")
        if not isinstance(pose, dict):
            return None

        biased_pose = copy.deepcopy(pose)
        position = biased_pose.setdefault("position", {})
        position["z"] = float(position.get("z", 0.0)) + self.z_bias
        return biased_pose

    def _make_pose_msg(self, pose: dict, stamp: rospy.Time, orientation: Optional[dict] = None) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        x, y, z = pose_position(pose)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        q = orientation if orientation is not None else normalized_orientation(pose)
        msg.pose.orientation.x = q["x"]
        msg.pose.orientation.y = q["y"]
        msg.pose.orientation.z = q["z"]
        msg.pose.orientation.w = q["w"]
        return msg

    def _interpolate_segment(self, start_pose: dict, end_pose: dict) -> List[dict]:
        sx, sy, sz = pose_position(start_pose)
        ex, ey, ez = pose_position(end_pose)
        dist = math.sqrt((ex - sx) ** 2 + (ey - sy) ** 2 + (ez - sz) ** 2)
        steps = max(1, int(math.floor(dist / self.path_point_spacing)))
        yaw = math.atan2(ey - sy, ex - sx) if dist > 1e-6 else 0.0
        orientation = yaw_to_quaternion(yaw)

        points = []
        for i in range(steps):
            ratio = float(i) / float(steps)
            points.append(
                {
                    "position": {
                        "x": sx + (ex - sx) * ratio,
                        "y": sy + (ey - sy) * ratio,
                        "z": sz + (ez - sz) * ratio,
                    },
                    "orientation": orientation,
                }
            )
        return points

    def _build_path_msg(self, task: dict, nodes: dict) -> Optional[RosPath]:
        path_ids = [str(node_id) for node_id in task.get("path", [])]
        if len(path_ids) < 2:
            return None

        target_idx = self._target_index(task)
        phase_start, phase_end, _, _, _ = self._phase_info(path_ids, target_idx)
        start_idx = target_idx if self.remaining_only else phase_start
        start_idx = max(0, min(start_idx, len(path_ids) - 1))
        # A retraced path must be sent one phase at a time. Otherwise EGO may
        # project onto the spatially overlapping return leg and turn around
        # before the actual pivot is reached.
        selected_ids = path_ids[start_idx : phase_end + 1]
        poses = []

        if self.current_position is not None and self.remaining_only:
            x, y, z = self.current_position
            poses.append(
                {
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": yaw_to_quaternion(0.0),
                }
            )

        for node_id in selected_ids:
            pose = self._pose_for_node(nodes, node_id)
            if pose is None:
                rospy.logerr("[util_3dnav] node %s is missing in nodes.json", node_id)
                self._update_task({"status": "failed", "message": f"航点 {node_id} 不存在"})
                return None
            poses.append(copy.deepcopy(pose))

        if len(poses) < 2:
            return None

        dense_poses = []
        for i in range(len(poses) - 1):
            dense_poses.extend(self._interpolate_segment(poses[i], poses[i + 1]))
        dense_poses.append(poses[-1])

        stamp = rospy.Time.now()
        path_msg = RosPath()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = stamp
        path_msg.poses = [self._make_pose_msg(pose, stamp) for pose in dense_poses]
        return path_msg

    def _task_key(self, task: dict) -> Tuple:
        return (
            task.get("status"),
            task.get("map_name"),
            tuple(str(x) for x in task.get("path", [])),
            tuple(str(x) for x in task.get("visited", [])),
            str(task.get("current_target", "") or ""),
            int(task.get("current_index", -1)),
        )

    def _publish_path_if_needed(self, task: dict, nodes: dict) -> None:
        now = rospy.Time.now()
        task_key = self._task_key(task)
        periodic = (now - self.last_publish_time).to_sec() >= self.republish_period
        changed = task_key != self.last_task_key
        if not changed and not periodic:
            return

        path_msg = self._build_path_msg(task, nodes)
        if path_msg is None:
            return

        self.path_pub.publish(path_msg)
        self.last_task_key = task_key
        self.last_publish_time = now
        self.last_path_msg = path_msg  # cache for the post-arrival completion hold
        path_ids = [str(node_id) for node_id in task.get("path", [])]
        target_idx = self._target_index(task)
        _, phase_end, _, _, phase_label = self._phase_info(path_ids, target_idx)
        rospy.loginfo(
            "[util_3dnav] published %d poses to %s for target %s "
            "[%s index=%d/%d phase_end=%s]",
            len(path_msg.poses),
            self.path_topic,
            task.get("current_target", ""),
            phase_label,
            target_idx,
            max(0, len(path_ids) - 1),
            path_ids[phase_end],
        )

    def _arrival_delta(self, pose: dict) -> Optional[Tuple[float, float]]:
        if self.current_position is None:
            return None
        tx, ty, tz = pose_position(pose)
        x, y, z = self.current_position
        xy_dist = math.sqrt((tx - x) ** 2 + (ty - y) ** 2)
        z_dist = abs(tz - z)
        return xy_dist, z_dist

    def _compute_arrival_update(self, task: dict, nodes: dict) -> Tuple[dict, Optional[dict]]:
        path = [str(node_id) for node_id in task.get("path", [])]
        if len(path) < 2:
            return task, None

        target_idx = self._target_index(task)
        target_idx = max(0, min(target_idx, len(path) - 1))
        visited = [str(node_id) for node_id in task.get("visited", [])]
        turn_indices = set(self._turnaround_indices(path))
        corner_indices = self._corner_indices(path, nodes)

        advanced_targets = []
        advanced_indices = []
        while target_idx < len(path):
            evaluating_idx = target_idx
            target_id = path[target_idx]
            pose = self._pose_for_node(nodes, target_id)
            if pose is None:
                updated = copy.deepcopy(task)
                updated["status"] = "failed"
                updates = {"status": "failed", "message": f"航点 {target_id} 不存在"}
                return updated, updates

            self.active_target = target_id
            delta = self._arrival_delta(pose)
            if delta is None:
                break
            xy_dist, z_dist = delta
            # Normal and corner thresholds follow DetourLimit when set (see
            # _effective_thresholds); pivots and the final destination keep
            # the tight fixed threshold below.
            arrival_thr, corner_thr = self._effective_thresholds()
            # Use the tighter turnaround threshold (0.5m) not only at direction-
            # reversal pivots but also at the route's FINAL destination -- for a
            # single-leg trip ("单程") that last point isn't a pivot, so it would
            # otherwise settle for the looser arrival radius. Each of these is
            # the endpoint of a published path phase, which the planner drives
            # to exactly regardless of the corridor width.
            is_final_destination = evaluating_idx == len(path) - 1
            if evaluating_idx in turn_indices or is_final_destination:
                xy_threshold = self.turnaround_arrival_threshold
            elif evaluating_idx in corner_indices:
                # Geometric corner (A-B-C bend > corner_angle_threshold): the
                # corridor cuts corners from the inside, so this follows
                # DetourLimit too.
                xy_threshold = corner_thr
            else:
                xy_threshold = arrival_thr
            if xy_dist > xy_threshold or z_dist > self.z_arrival_threshold:
                break

            if target_id not in visited:
                visited.append(target_id)
            advanced_targets.append(target_id)
            advanced_indices.append(evaluating_idx)
            target_idx += 1

            # Finish the current phase at the pivot. The return phase must be
            # published before any overlapping return waypoint is consumed.
            if evaluating_idx in turn_indices:
                break

        if not advanced_targets:
            return task, None

        updated_task = copy.deepcopy(task)
        if target_idx >= len(path):
            updates = {
                "status": "completed",
                "visited": visited,
                "current_target": "",
                "current_index": len(path) - 1,
                "message": "导航完成",
            }
            updated_task.update(updates)
            return updated_task, updates

        next_target = path[target_idx]
        last_arrived = advanced_targets[-1]
        last_arrived_idx = advanced_indices[-1]
        _, _, _, _, next_phase_label = self._phase_info(path, target_idx)
        if last_arrived_idx in turn_indices:
            if next_phase_label == "回程":
                message = f"已到达折返点航点 {last_arrived}，正在返回航点 {next_target}"
            else:
                message = (
                    f"已到达折返点航点 {last_arrived}，"
                    f"开始{next_phase_label}，正在前往航点 {next_target}"
                )
        elif next_phase_label == "回程":
            message = f"已到达航点 {last_arrived}，正在返回航点 {next_target}"
        else:
            message = f"已到达航点 {last_arrived}，正在前往航点 {next_target}"
        updates = {
            "visited": visited,
            "current_target": next_target,
            "current_index": target_idx,
            "route_phase": next_phase_label,
            "message": message,
        }
        updated_task.update(updates)
        return updated_task, updates

    def _apply_arrival_updates(self, updates: dict) -> None:
        self._update_task(updates)
        if updates.get("status") == "completed":
            rospy.loginfo("[util_3dnav] task completed")
        elif "current_target" in updates:
            rospy.loginfo("[util_3dnav] %s", updates.get("message", "arrival updated"))

    def _ensure_current_target(self, task: dict) -> dict:
        path = [str(node_id) for node_id in task.get("path", [])]
        if len(path) < 2:
            self._update_task({"status": "failed", "message": "任务路径至少需要两个航点"})
            return task

        current = str(task.get("current_target", "") or "")
        try:
            current_index = int(task.get("current_index", -1))
        except (TypeError, ValueError):
            current_index = -1

        if 0 <= current_index < len(path) and current == path[current_index]:
            return task

        target_idx = self._target_index(task)
        target_id = path[target_idx]
        _, _, _, _, phase_label = self._phase_info(path, target_idx)
        action = "正在返回" if phase_label == "回程" else "正在前往"
        self._update_task(
            {
                "current_target": target_id,
                "current_index": target_idx,
                "route_phase": phase_label,
                "message": f"{action}航点 {target_id}",
            }
        )
        task = copy.deepcopy(task)
        task["current_target"] = target_id
        task["current_index"] = target_idx
        task["route_phase"] = phase_label
        return task

    def spin(self) -> None:
        rate = rospy.Rate(self.poll_rate)
        rospy.loginfo("[util_3dnav] bridge is ready")

        while not rospy.is_shutdown():
            task = self._read_task()
            if task.get("status") != "running":
                # Post-arrival completion hold: keep republishing the final leg's
                # pct_path (with a fresh stamp) so traj_server's heartbeat stays
                # alive and the robot actually finishes the last segment before
                # we stop. Only during the hold window; then fall through to the
                # normal stop/idle handling below.
                if self.completion_hold_until is not None:
                    if rospy.Time.now() < self.completion_hold_until and self.last_path_msg is not None:
                        stamp = rospy.Time.now()
                        self.last_path_msg.header.stamp = stamp
                        for pose_msg in self.last_path_msg.poses:
                            pose_msg.header.stamp = stamp
                        self.path_pub.publish(self.last_path_msg)
                        rate.sleep()
                        continue
                    self.completion_hold_until = None
                    rospy.loginfo("[util_3dnav] completion hold elapsed, stopping pct_path publication")
                if self.was_running:
                    status = str(task.get("status", "") or "")
                    if status not in ("completed", "failed"):
                        # The upper system withdrew a LIVE task (cancel): stop
                        # the robot now. Normal completion/failure sends
                        # nothing -- the robot is already standing and the
                        # planner is IDLE; a terminate here would just latch
                        # STOPPED for no reason.
                        self.task_control_pub.publish(Int8(data=1))
                        self._sent_terminate = True
                        rospy.loginfo(
                            "[util_3dnav] task cancelled by upper system -> /mission/task_control=1 (terminate)"
                        )
                    write_json(self.robot_file, {"status": "idle"})
                    rospy.loginfo("[util_3dnav] task stopped, robot_state.status set to idle")
                self.active_target = ""
                self.was_running = False
                with self.state_lock:
                    self.cached_task = None
                    self.cached_nodes = None
                    self.pending_task = None
                    self.pending_updates = None
                rate.sleep()
                continue

            map_name = str(task.get("map_name", "") or "")
            path = [str(node_id) for node_id in task.get("path", [])]
            if not map_name or len(path) < 2:
                self._update_task({"status": "failed", "message": "任务缺少 map_name 或 path"})
                rate.sleep()
                continue

            self._log_route_structure_if_changed(path)

            nodes = self._read_nodes(map_name)
            if not nodes:
                self._update_task(
                    {
                        "status": "failed",
                        "message": f"地图 {map_name} 的 nodes.json 不存在或无法读取",
                    }
                )
                rate.sleep()
                continue

            task = self._ensure_current_target(task)
            with self.state_lock:
                self.cached_task = copy.deepcopy(task)
                self.cached_nodes = nodes

                pending_task = self.pending_task
                pending_updates = self.pending_updates
                self.pending_task = None
                self.pending_updates = None

            if pending_updates is None:
                task, pending_updates = self._compute_arrival_update(task, nodes)
            else:
                task = pending_task

            if pending_updates is not None:
                self._apply_arrival_updates(pending_updates)
                with self.state_lock:
                    self.cached_task = copy.deepcopy(task)
                # Reached the route's final destination: arm the completion hold
                # so the pct_path heartbeat keeps going for completion_hold_sec,
                # letting traj_server finish driving the last segment.
                if pending_updates.get("status") == "completed" and self.completion_hold_sec > 0.0:
                    self.completion_hold_until = rospy.Time.now() + rospy.Duration(self.completion_hold_sec)
                    rospy.loginfo(
                        "[util_3dnav] final arrival; holding pct_path heartbeat for %.1fs "
                        "so traj_server finishes the last segment",
                        self.completion_hold_sec,
                    )

            if task.get("status") != "running":
                self.was_running = False
                rate.sleep()
                continue

            # A live running task means we're not in a post-arrival hold; drop
            # any stale hold timer so it can't leak into a later stop of THIS run.
            self.completion_hold_until = None

            if self.was_running:
                # Planner-side safety stop (heartbeat timeout / external
                # stop) while the upper system still thinks the task is
                # running: surface it as a failure instead of silently
                # stalling forever. Skipped when the STOPPED state is just
                # the terminate WE sent on a cancel.
                with self.state_lock:
                    planner_state = self._mission_state
                if planner_state == 3 and not self._sent_terminate:
                    self._update_task(
                        {"status": "failed", "message": "导航异常终止（安全停止）"}
                    )
                    rospy.logwarn("[util_3dnav] planner reported STOPPED; task marked failed")
                    rate.sleep()
                    continue
            else:
                # New task starting: clear a possible STOPPED latch in the
                # planner BEFORE the first path goes out (a STOPPED
                # MissionFSM rejects paths), and re-arm the command caches
                # so NavMode/Speed/DetourLimit are re-published even if
                # unchanged -- their first publication may have been lost if
                # the planner started after this bridge (pub/sub connect race).
                self.task_control_pub.publish(Int8(data=0))
                self._sent_terminate = False
                self._last_nav_mode = -1
                self._last_speed = -1
                self._last_detour = -1.0
                rospy.sleep(0.2)  # let the RUN unlock land before the path

            self.was_running = True
            self._apply_robot_commands()
            self._update_robot_navigation_state(map_name, path)
            self._publish_path_if_needed(task, nodes)
            self._write_dodge_status_if_changed()
            self._write_robot_state_periodic(task)

            rate.sleep()


if __name__ == "__main__":
    try:
        MidwareBridge().spin()
    except rospy.ROSInterruptException:
        pass
