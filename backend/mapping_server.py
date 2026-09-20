#!/usr/bin/env python3
"""ROS 2 point-cloud accumulator plus a dependency-free HTTP/WebSocket UI server."""

from __future__ import annotations

import argparse
from collections import deque
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
from typing import Any, Callable, Optional
from urllib.parse import parse_qs, urlsplit

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

try:
    from .mapping_core import MapExportConfig, VoxelAccumulator, export_session, sanitize_map_name
    from .terrain_core import (
        EDITOR_HEADER,
        MAX_EDITOR_CELLS,
        LAYER_OCCUPANCY,
        LAYER_TERRAIN,
        decode_editor_map,
        encode_editor_map,
        list_editable_maps,
        load_occupancy_editor_map,
        load_terrain_editor_map,
        map_paths,
        occupancy_to_terrain,
        save_occupancy_editor_map,
        write_terrain_msgpack,
    )
except ImportError:
    from mapping_core import MapExportConfig, VoxelAccumulator, export_session, sanitize_map_name
    from terrain_core import (
        EDITOR_HEADER,
        MAX_EDITOR_CELLS,
        LAYER_OCCUPANCY,
        LAYER_TERRAIN,
        decode_editor_map,
        encode_editor_map,
        list_editable_maps,
        load_occupancy_editor_map,
        load_terrain_editor_map,
        map_paths,
        occupancy_to_terrain,
        save_occupancy_editor_map,
        write_terrain_msgpack,
    )


WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def _websocket_frame(opcode: int, payload: bytes) -> bytes:
    first = 0x80 | opcode
    size = len(payload)
    if size < 126:
        return bytes((first, size)) + payload
    if size <= 0xFFFF:
        return bytes((first, 126)) + struct.pack("!H", size) + payload
    return bytes((first, 127)) + struct.pack("!Q", size) + payload


def _receive_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        part = sock.recv(size - len(chunks))
        if not part:
            raise ConnectionError("WebSocket closed")
        chunks.extend(part)
    return bytes(chunks)


def _receive_websocket_frame(sock: socket.socket) -> tuple[int, bytes]:
    first, second = _receive_exact(sock, 2)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", _receive_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", _receive_exact(sock, 8))[0]
    if length > 1_048_576:
        raise ValueError("WebSocket command is too large")
    mask = _receive_exact(sock, 4) if masked else b""
    payload = bytearray(_receive_exact(sock, length))
    if masked:
        for index in range(length):
            payload[index] ^= mask[index % 4]
    return opcode, bytes(payload)


class WebClient:
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.lock = threading.Lock()
        self.closed = False

    def send(self, opcode: int, payload: bytes) -> None:
        with self.lock:
            if self.closed:
                raise ConnectionError("client closed")
            self.sock.sendall(_websocket_frame(opcode, payload))

    def close(self) -> None:
        with self.lock:
            if self.closed:
                return
            self.closed = True
            try:
                self.sock.sendall(_websocket_frame(0x8, b""))
            except OSError:
                pass


class WebSocketHub:
    def __init__(self) -> None:
        self._clients: set[WebClient] = set()
        self._lock = threading.Lock()

    def add(self, client: WebClient) -> None:
        with self._lock:
            self._clients.add(client)

    def remove(self, client: WebClient) -> None:
        with self._lock:
            self._clients.discard(client)
        client.closed = True

    def _broadcast(self, opcode: int, payload: bytes) -> None:
        with self._lock:
            clients = list(self._clients)
        failed: list[WebClient] = []
        for client in clients:
            try:
                client.send(opcode, payload)
            except (OSError, ConnectionError):
                failed.append(client)
        for client in failed:
            self.remove(client)

    def broadcast_json(self, value: dict[str, Any]) -> None:
        self._broadcast(0x1, json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8"))

    def broadcast_cloud(self, points: np.ndarray) -> None:
        xyz = np.ascontiguousarray(points[:, :3], dtype="<f4")
        self._broadcast(0x2, struct.pack("<4sI", b"MAP1", len(xyz)) + xyz.tobytes(order="C"))

    def close_all(self) -> None:
        with self._lock:
            clients = list(self._clients)
            self._clients.clear()
        for client in clients:
            client.close()


class MappingSupervisor(Node):
    def __init__(self, settings: argparse.Namespace, hub: WebSocketHub) -> None:
        super().__init__("mapping_web_supervisor")
        self.settings = settings
        self.hub = hub
        self.output_root = Path(settings.output_dir).expanduser().resolve()
        self.output_root.joinpath("pcd").mkdir(parents=True, exist_ok=True)
        self.output_root.joinpath("map").mkdir(parents=True, exist_ok=True)
        self.accumulator = VoxelAccumulator(settings.voxel_size, settings.max_points)
        self.export_config = MapExportConfig(
            resolution=settings.map_resolution,
            z_min=settings.z_min,
            z_max=settings.z_max,
            radius=settings.radius_filter,
            min_neighbors=settings.min_neighbors,
            padding=settings.map_padding,
        )
        self._state_lock = threading.RLock()
        self._command_lock = threading.Lock()
        self._state = "IDLE"
        self._session_name = ""
        self._message = "可以开始新的建图会话"
        self._started_monotonic: Optional[float] = None
        self._elapsed_at_stop = 0.0
        self._travel_distance = 0.0
        self._last_odom_position: Optional[np.ndarray] = None
        self._last_cloud_at = 0.0
        self._last_odom_at = 0.0
        self._cloud_arrivals: deque[float] = deque(maxlen=100)
        self._frame_id = ""
        self._save_progress = 0.0
        self._save_stage = ""
        self._output: dict[str, Any] = {}
        self._last_error = ""
        self._stack_lock = threading.RLock()
        self._stack_processes: dict[str, tuple[subprocess.Popen[bytes], Any]] = {}
        self._stack_message = "建图链路尚未启动"
        self._stack_transition = ""
        self._map_file_lock = threading.RLock()

        retained_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(String, "/mapping/status", retained_qos)
        self.cloud_publisher = self.create_publisher(PointCloud2, "/mapping/accumulated_cloud", retained_qos)
        self.create_subscription(PointCloud2, settings.cloud_topic, self._on_cloud, qos_profile_sensor_data)
        if settings.odom_topic:
            self.create_subscription(Odometry, settings.odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_service(Trigger, "/mapping/start", self._start_service)
        self.create_service(Trigger, "/mapping/stop_and_save", self._stop_service)
        self.create_service(Trigger, "/mapping/reset", self._reset_service)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(1.0 / settings.web_publish_rate, self._publish_web_cloud)
        self.create_timer(1.0 / settings.ros_publish_rate, self._publish_ros_cloud)
        self.get_logger().info(
            f"Mapping web supervisor ready: {settings.cloud_topic} -> {self.output_root}"
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        now = time.monotonic()
        with self._state_lock:
            self._last_cloud_at = now
            self._cloud_arrivals.append(now)
            self._frame_id = message.header.frame_id or self._frame_id
            mapping = self._state == "MAPPING"
        if not mapping:
            return
        try:
            points = point_cloud2.read_points_numpy(
                message, field_names=["x", "y", "z"], skip_nans=True
            )
            self.accumulator.add(points)
            if self.accumulator.point_count >= self.settings.max_points:
                with self._state_lock:
                    self._message = f"已达到最大累计点数 {self.settings.max_points:,}，请结束并保存"
        except Exception as error:
            self.get_logger().error(f"Point cloud accumulation failed: {error}")
            with self._state_lock:
                self._last_error = str(error)
                self._message = f"点云处理异常：{error}"

    def _on_odom(self, message: Odometry) -> None:
        now = time.monotonic()
        position = np.array([
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        ], dtype=np.float64)
        with self._state_lock:
            self._last_odom_at = now
            if self._state != "MAPPING":
                return
            if self._last_odom_position is not None:
                delta = float(np.linalg.norm(position - self._last_odom_position))
                if 0.002 <= delta <= 3.0:
                    self._travel_distance += delta
            self._last_odom_position = position

    def _cloud_rate(self, now: float) -> float:
        with self._state_lock:
            while self._cloud_arrivals and now - self._cloud_arrivals[0] > 3.0:
                self._cloud_arrivals.popleft()
            if len(self._cloud_arrivals) < 2:
                return 0.0
            duration = self._cloud_arrivals[-1] - self._cloud_arrivals[0]
            return (len(self._cloud_arrivals) - 1) / duration if duration > 0 else 0.0

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._state_lock:
            elapsed = (
                now - self._started_monotonic
                if self._state == "MAPPING" and self._started_monotonic is not None
                else self._elapsed_at_stop
            )
            return {
                "state": self._state,
                "session_name": self._session_name,
                "point_count": self.accumulator.point_count,
                "bounds": self.accumulator.bounds,
                "elapsed_seconds": max(0.0, elapsed),
                "travel_distance": self._travel_distance,
                "cloud_online": self._last_cloud_at > 0 and now - self._last_cloud_at < 2.5,
                "odom_online": self._last_odom_at > 0 and now - self._last_odom_at < 2.5,
                "cloud_rate_hz": self._cloud_rate(now),
                "frame_id": self._frame_id,
                "voxel_size": self.settings.voxel_size,
                "storage_writable": os.access(self.output_root, os.W_OK),
                "save_progress": self._save_progress,
                "save_stage": self._save_stage,
                "message": self._message,
                "output": self._output,
                "stack": self.stack_status(),
            }

    def _graph_nodes(self) -> set[str]:
        try:
            return {name for name, _namespace in self.get_node_names_and_namespaces()}
        except Exception:
            return set()

    def _reap_stack_processes(self) -> None:
        for key, (process, log_stream) in list(self._stack_processes.items()):
            return_code = process.poll()
            if return_code is None:
                continue
            try:
                log_stream.close()
            except OSError:
                pass
            del self._stack_processes[key]
            if self._stack_transition != "STOPPING":
                self._stack_message = f"{key} 已退出（code {return_code}），请查看 .ros/managed 日志"

    def stack_status(self) -> dict[str, Any]:
        with self._stack_lock:
            self._reap_stack_processes()
            graph = self._graph_nodes()
            expected = {
                "robot_state_publisher": "robot_state_publisher",
                "mid360_driver": "mid360_driver",
                "small_point_lio": "small_point_lio",
            }
            nodes = {
                key: {
                    "online": node_name in graph,
                    "managed": key in self._stack_processes,
                }
                for key, node_name in expected.items()
            }
            online_count = sum(1 for value in nodes.values() if value["online"])
            if self._stack_transition:
                state = self._stack_transition
            elif online_count == len(nodes):
                state = "RUNNING"
            elif online_count:
                state = "PARTIAL"
            else:
                state = "STOPPED"
            message = self._stack_message
            if state == "RUNNING" and not self._stack_processes:
                message = "检测到完整建图链路（由外部终端启动）"
            elif (
                state == "RUNNING"
                and self._last_cloud_at > 0
                and time.monotonic() - self._last_cloud_at < 2.5
            ):
                message = "建图链路运行正常，点云与里程计已就绪"
            return {
                "state": state,
                "nodes": nodes,
                "managed_count": len(self._stack_processes),
                "message": message,
                "log_dir": str(Path(self.settings.process_log_dir).resolve()),
            }

    def _stack_commands(self) -> dict[str, list[str]]:
        params = str(Path(self.settings.lio_params).expanduser().resolve())
        return {
            "robot_state_publisher": [
                "ros2", "launch", "mas2027_nav_bringup", "robot_state_publisher_launch.py",
                "use_sim_time:=false",
            ],
            "mid360_driver": [
                "ros2", "run", "mid360_driver", "mid360_driver_node", "--ros-args",
                "--params-file", params,
            ],
            "small_point_lio": [
                "ros2", "run", "small_point_lio", "small_point_lio_node", "--ros-args",
                "--params-file", params, "-p", "publish_odom_tf:=true",
            ],
        }

    def start_stack(self) -> tuple[bool, str]:
        if not self.settings.stack_control:
            return False, "当前启动参数已禁用节点管理"
        params = Path(self.settings.lio_params).expanduser().resolve()
        if not params.is_file():
            return False, f"找不到 LIO 参数文件：{params}"
        with self._stack_lock:
            self._reap_stack_processes()
            if self._stack_transition == "STOPPING":
                return False, "建图链路正在停止，请稍候"
            graph = self._graph_nodes()
            commands = self._stack_commands()
            node_names = {
                "robot_state_publisher": "robot_state_publisher",
                "mid360_driver": "mid360_driver",
                "small_point_lio": "small_point_lio",
            }
            missing = [key for key, name in node_names.items() if name not in graph]
            if not missing:
                self._stack_message = "三个建图节点均已运行"
                return True, self._stack_message
            log_dir = Path(self.settings.process_log_dir).expanduser().resolve()
            log_dir.mkdir(parents=True, exist_ok=True)
            started: list[str] = []
            try:
                self._stack_transition = "STARTING"
                for key in ("robot_state_publisher", "mid360_driver", "small_point_lio"):
                    if key not in missing:
                        continue
                    log_path = log_dir / f"{key}.log"
                    log_stream = log_path.open("ab", buffering=0)
                    process = subprocess.Popen(
                        commands[key],
                        stdin=subprocess.DEVNULL,
                        stdout=log_stream,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        close_fds=True,
                    )
                    self._stack_processes[key] = (process, log_stream)
                    started.append(key)
                self._stack_message = "已启动缺失节点，正在等待点云和里程计"
            except Exception as error:
                self._stack_transition = ""
                self._stack_message = f"节点启动失败：{error}"
                return False, self._stack_message
        threading.Thread(target=self._finish_stack_start, daemon=True, name="stack-start-watch").start()
        return True, f"已启动：{', '.join(started)}"

    def _finish_stack_start(self) -> None:
        deadline = time.monotonic() + self.settings.stack_start_timeout
        while time.monotonic() < deadline:
            time.sleep(0.25)
            graph = self._graph_nodes()
            if {"robot_state_publisher", "mid360_driver", "small_point_lio"}.issubset(graph):
                with self._stack_lock:
                    self._stack_transition = ""
                    self._stack_message = "建图节点已启动，正在等待 LIO 数据稳定"
                return
            with self._stack_lock:
                self._reap_stack_processes()
        with self._stack_lock:
            self._stack_transition = ""
            self._stack_message = "节点启动超时，请查看 .ros/managed 日志和雷达网络"

    def begin_stop_stack(self) -> tuple[bool, str]:
        if not self.settings.stack_control:
            return False, "当前启动参数已禁用节点管理"
        with self._stack_lock:
            self._reap_stack_processes()
            if not self._stack_processes:
                return False, "没有由网页启动的节点；外部进程不会被停止"
            if self._state in {"MAPPING", "SAVING"}:
                return False, "建图或保存期间不能停止节点，请先结束并保存"
            if self._stack_transition == "STOPPING":
                return False, "建图链路正在停止"
            self._stack_transition = "STOPPING"
            self._stack_message = "正在停止网页管理的建图节点"
        threading.Thread(target=self.stop_managed_stack, daemon=True, name="stack-stop").start()
        return True, "正在停止网页管理的建图节点"

    def stop_managed_stack(self) -> None:
        with self._stack_lock:
            records = list(self._stack_processes.items())
        for _key, (process, _log_stream) in reversed(records):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except (ProcessLookupError, PermissionError):
                    pass
        deadline = time.monotonic() + 5.0
        for _key, (process, _log_stream) in records:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass
        with self._stack_lock:
            for _key, (process, log_stream) in list(self._stack_processes.items()):
                if process.poll() is None:
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except (ProcessLookupError, PermissionError):
                            pass
                try:
                    log_stream.close()
                except OSError:
                    pass
            self._stack_processes.clear()
            self._stack_transition = ""
            self._stack_message = "网页管理的建图节点已停止"

    def _publish_status(self) -> None:
        status = self.status()
        encoded = json.dumps(status, ensure_ascii=False, separators=(",", ":"))
        self.status_publisher.publish(String(data=encoded))
        self.hub.broadcast_json({"type": "status", "payload": status})

    def _publish_web_cloud(self) -> None:
        if self.accumulator.point_count:
            self.hub.broadcast_cloud(self.accumulator.snapshot(self.settings.web_max_points))

    def _publish_ros_cloud(self) -> None:
        if not self.accumulator.point_count:
            return
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._frame_id or "odom")
        cloud = point_cloud2.create_cloud_xyz32(
            header, self.accumulator.snapshot(self.settings.ros_max_points)
        )
        self.cloud_publisher.publish(cloud)

    def start_session(self, requested_name: str = "") -> tuple[bool, str]:
        try:
            name = sanitize_map_name(requested_name or time.strftime("map_%Y%m%d_%H%M%S"))
        except ValueError as error:
            return False, str(error)
        with self._command_lock, self._state_lock:
            if self._state == "SAVING":
                return False, "地图正在保存，请稍候"
            if self._state == "MAPPING":
                return False, "当前已经在建图"
            self.accumulator.clear()
            self._state = "MAPPING"
            self._session_name = name
            self._message = "正在累计点云"
            self._started_monotonic = time.monotonic()
            self._elapsed_at_stop = 0.0
            self._travel_distance = 0.0
            self._last_odom_position = None
            self._save_progress = 0.0
            self._save_stage = ""
            self._output = {}
            self._last_error = ""
        self.hub.broadcast_cloud(np.empty((0, 3), dtype=np.float32))
        return True, f"建图会话 {name} 已开始"

    def _available_output_name(self, name: str) -> str:
        candidates = (
            self.output_root / "pcd" / f"{name}.pcd",
            self.output_root / "map" / f"{name}.pgm",
            self.output_root / "map" / f"{name}.yaml",
        )
        if not any(path.exists() for path in candidates):
            return name
        return f"{name}_{time.strftime('%Y%m%d_%H%M%S')}"

    def begin_save(self) -> tuple[bool, str]:
        with self._command_lock, self._state_lock:
            if self._state == "SAVING":
                return False, "地图已经在保存"
            if self._state != "MAPPING":
                return False, "当前没有正在进行的建图会话"
            if self.accumulator.point_count < 3:
                return False, "累计点数不足，请确认点云输入后再结束建图"
            now = time.monotonic()
            self._elapsed_at_stop = now - (self._started_monotonic or now)
            self._state = "SAVING"
            self._save_progress = 0.05
            self._save_stage = "正在整理累计点云"
            self._message = "正在生成 PCD、PGM 和 YAML"
            name = self._available_output_name(self._session_name)
            self._session_name = name
        thread = threading.Thread(target=self._save_worker, args=(name,), daemon=True, name="map-export")
        thread.start()
        return True, f"正在保存 {name}"

    def _save_worker(self, name: str) -> None:
        try:
            points = self.accumulator.snapshot()

            def progress(value: float, stage: str) -> None:
                with self._state_lock:
                    self._save_progress = value
                    self._save_stage = stage

            result = export_session(self.output_root, name, points, self.export_config, progress)
            with self._state_lock:
                self._output = result
                self._state = "SAVED"
                self._message = f"地图 {name} 保存完成"
                self._save_progress = 1.0
                self._save_stage = "地图文件保存完成"
            self.hub.broadcast_json({"type": "result", "ok": True, "message": f"地图 {name} 保存完成"})
        except Exception as error:
            self.get_logger().error(f"Map export failed: {error}")
            with self._state_lock:
                self._state = "ERROR"
                self._message = f"保存失败：{error}"
                self._last_error = str(error)
                self._save_stage = "保存失败"
            self.hub.broadcast_json({"type": "result", "ok": False, "message": f"保存失败：{error}"})

    def reset_session(self) -> tuple[bool, str]:
        with self._command_lock, self._state_lock:
            if self._state == "SAVING":
                return False, "正在保存，不能清空"
            if self._state == "MAPPING":
                return False, "请先结束当前建图，再清空会话"
            self.accumulator.clear()
            self._state = "IDLE"
            self._session_name = ""
            self._message = "可以开始新的建图会话"
            self._elapsed_at_stop = 0.0
            self._travel_distance = 0.0
            self._last_odom_position = None
            self._save_progress = 0.0
            self._save_stage = ""
            self._output = {}
            self._last_error = ""
        self.hub.broadcast_cloud(np.empty((0, 3), dtype=np.float32))
        return True, "当前会话已清空"

    def dispatch(self, action: str, map_name: str = "") -> tuple[bool, str]:
        if action == "start_stack":
            return self.start_stack()
        if action == "stop_stack":
            return self.begin_stop_stack()
        if action == "start":
            return self.start_session(map_name)
        if action == "stop":
            return self.begin_save()
        if action == "reset":
            return self.reset_session()
        return False, f"未知操作：{action}"

    def editable_maps(self) -> list[dict[str, Any]]:
        with self._map_file_lock:
            return list_editable_maps(self.output_root)

    def load_editor_map(self, map_name: str, layer: int) -> bytes:
        name = sanitize_map_name(map_name)
        yaml_path, _pgm_path, terrain_path = map_paths(self.output_root, name)
        with self._map_file_lock:
            if layer == LAYER_OCCUPANCY:
                if not yaml_path.is_file():
                    raise FileNotFoundError(f"找不到二维地图：{name}.yaml")
                editor_map = load_occupancy_editor_map(yaml_path)
            elif layer == LAYER_TERRAIN:
                if not terrain_path.is_file():
                    raise FileNotFoundError(f"找不到 terrain 地图：{name}_terrain.msgpack")
                editor_map = load_terrain_editor_map(
                    terrain_path,
                    yaml_path=yaml_path if yaml_path.is_file() else None,
                )
            else:
                raise ValueError("未知地图编辑图层")
            return encode_editor_map(editor_map)

    def save_editor_map(self, map_name: str, payload: bytes) -> tuple[int, str]:
        name = sanitize_map_name(map_name)
        editor_map = decode_editor_map(payload)
        yaml_path, _pgm_path, terrain_path = map_paths(self.output_root, name)
        with self._map_file_lock:
            if editor_map.layer == LAYER_OCCUPANCY:
                if not yaml_path.is_file():
                    raise FileNotFoundError(f"找不到二维地图：{name}.yaml")
                output = save_occupancy_editor_map(yaml_path, editor_map)
                return editor_map.layer, f"二维地图已保存：{output.name}"
            if not terrain_path.is_file():
                raise FileNotFoundError(f"找不到 terrain 地图：{name}_terrain.msgpack")
            existing = load_terrain_editor_map(
                terrain_path,
                yaml_path=yaml_path if yaml_path.is_file() else None,
            )
            if (
                editor_map.metadata.width != existing.metadata.width
                or editor_map.metadata.height != existing.metadata.height
                or not math.isclose(
                    editor_map.metadata.resolution,
                    existing.metadata.resolution,
                    rel_tol=1e-9,
                    abs_tol=1e-12,
                )
            ):
                raise ValueError("不能通过编辑接口改变 terrain 地图尺寸或分辨率")
            write_terrain_msgpack(terrain_path, editor_map)
            return editor_map.layer, f"terrain 地图已保存：{terrain_path.name}"

    def convert_to_terrain(self, map_name: str, overwrite: bool = False) -> tuple[str, bytes]:
        name = sanitize_map_name(map_name)
        yaml_path, _pgm_path, terrain_path = map_paths(self.output_root, name)
        with self._map_file_lock:
            if not yaml_path.is_file():
                raise FileNotFoundError(f"找不到二维地图：{name}.yaml")
            if terrain_path.exists() and not overwrite:
                raise FileExistsError(f"{terrain_path.name} 已存在，未覆盖")
            editor_map = occupancy_to_terrain(yaml_path)
            write_terrain_msgpack(terrain_path, editor_map)
            return terrain_path.name, encode_editor_map(editor_map)

    def _start_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self.start_session()
        return response

    def _stop_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self.begin_save()
        return response

    def _reset_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self.reset_session()
        return response


class MappingHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], handler: Callable[..., SimpleHTTPRequestHandler], controller: MappingSupervisor, hub: WebSocketHub) -> None:
        self.controller = controller
        self.hub = hub
        super().__init__(address, handler)


class RequestHandler(SimpleHTTPRequestHandler):
    server: MappingHttpServer

    def do_GET(self) -> None:
        parsed = urlsplit(self.path)
        request_path = parsed.path
        if request_path == "/ws":
            self._handle_websocket()
            return
        if request_path == "/api/status":
            self._send_json(HTTPStatus.OK, self.server.controller.status())
            return
        if request_path == "/api/maps":
            try:
                self._send_json(
                    HTTPStatus.OK,
                    {"maps": self.server.controller.editable_maps()},
                )
            except (OSError, ValueError) as error:
                self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "message": str(error)})
            return
        if request_path in {"/api/editor/occupancy", "/api/editor/terrain"}:
            query = parse_qs(parsed.query)
            map_name = query.get("map_name", [""])[0]
            layer = LAYER_OCCUPANCY if request_path.endswith("occupancy") else LAYER_TERRAIN
            try:
                body = self.server.controller.load_editor_map(map_name, layer)
                self._send_bytes(HTTPStatus.OK, body, "application/x-mapping-editor")
            except FileNotFoundError as error:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "message": str(error)})
            except (OSError, ValueError) as error:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
            return
        if request_path == "/api/cloud":
            points = self.server.controller.accumulator.snapshot(
                self.server.controller.settings.web_max_points
            )
            xyz = np.ascontiguousarray(points[:, :3], dtype="<f4")
            body = struct.pack("<4sI", b"MAP1", len(xyz)) + xyz.tobytes(order="C")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlsplit(self.path)
        request_path = parsed.path
        if request_path == "/api/editor/convert":
            try:
                payload = self._read_json_body()
                filename, _editor_payload = self.server.controller.convert_to_terrain(
                    str(payload.get("map_name", "")), bool(payload.get("overwrite", False))
                )
                self._send_json(
                    HTTPStatus.OK,
                    {"ok": True, "message": f"已生成 {filename}", "filename": filename},
                )
            except FileExistsError as error:
                self._send_json(HTTPStatus.CONFLICT, {"ok": False, "message": str(error)})
            except FileNotFoundError as error:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "message": str(error)})
            except (OSError, ValueError, json.JSONDecodeError) as error:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
            return
        if request_path == "/api/editor/save":
            query = parse_qs(parsed.query)
            map_name = query.get("map_name", [""])[0]
            try:
                body = self._read_body(EDITOR_HEADER.size + MAX_EDITOR_CELLS * 2)
                layer, message = self.server.controller.save_editor_map(map_name, body)
                self._send_json(HTTPStatus.OK, {"ok": True, "message": message, "layer": layer})
            except FileNotFoundError as error:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "message": str(error)})
            except (OSError, ValueError) as error:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
            return

        action = request_path.removeprefix("/api/").strip("/")
        if action not in {"start", "stop", "reset", "start_stack", "stop_stack"}:
            self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "message": "接口不存在"})
            return
        try:
            payload = self._read_json_body()
            ok, message = self.server.controller.dispatch(action, str(payload.get("map_name", "")))
            self._send_json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT, {"ok": ok, "message": message})
        except (ValueError, json.JSONDecodeError) as error:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})

    def _read_body(self, maximum: int) -> bytes:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ValueError("Content-Length 无效") from error
        if length < 0 or length > maximum:
            raise ValueError(f"请求数据不能超过 {maximum:,} 字节")
        body = self.rfile.read(length)
        if len(body) != length:
            raise ValueError("请求数据不完整")
        return body

    def _read_json_body(self) -> dict[str, Any]:
        payload = json.loads(self._read_body(64 * 1024) or b"{}")
        if not isinstance(payload, dict):
            raise ValueError("JSON 请求体必须是对象")
        return payload

    def _send_bytes(self, status: HTTPStatus, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send_bytes(status, body, "application/json; charset=utf-8")

    def _handle_websocket(self) -> None:
        key = self.headers.get("Sec-WebSocket-Key", "")
        if self.headers.get("Upgrade", "").lower() != "websocket" or not key:
            self.send_error(HTTPStatus.UPGRADE_REQUIRED, "WebSocket upgrade required")
            return
        accept = base64.b64encode(hashlib.sha1((key + WEBSOCKET_GUID).encode("ascii")).digest()).decode("ascii")
        self.send_response(HTTPStatus.SWITCHING_PROTOCOLS)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        client = WebClient(self.connection)
        self.server.hub.add(client)
        try:
            client.send(0x1, json.dumps({"type": "status", "payload": self.server.controller.status()}, ensure_ascii=False).encode("utf-8"))
            while True:
                opcode, payload = _receive_websocket_frame(self.connection)
                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    client.send(0xA, payload)
                    continue
                if opcode != 0x1:
                    continue
                command = json.loads(payload.decode("utf-8"))
                if command.get("type") != "command":
                    continue
                ok, message = self.server.controller.dispatch(
                    str(command.get("action", "")), str(command.get("map_name", ""))
                )
                client.send(0x1, json.dumps({"type": "result", "ok": ok, "message": message}, ensure_ascii=False).encode("utf-8"))
        except (ConnectionError, OSError, ValueError, json.JSONDecodeError):
            pass
        finally:
            self.server.hub.remove(client)
            self.close_connection = True

    def end_headers(self) -> None:
        if self.path != "/ws":
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt: str, *args: object) -> None:
        if getattr(self.server.controller.settings, "http_log", False):
            super().log_message(fmt, *args)


def make_argument_parser() -> argparse.ArgumentParser:
    project_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Small Point-LIO web mapping console")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--web-dir", default=str(project_root / "web"))
    parser.add_argument("--output-dir", default=str(project_root / "data"))
    parser.add_argument(
        "--lio-params",
        default="/home/mas/mas_nav_2027_native/src/mas2027_nav_bringup/config/small_point_lio_params.yaml",
    )
    parser.add_argument("--process-log-dir", default=str(project_root / ".ros" / "managed"))
    parser.add_argument("--stack-start-timeout", type=float, default=12.0)
    parser.add_argument("--no-stack-control", dest="stack_control", action="store_false")
    parser.set_defaults(stack_control=True)
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--max-points", type=int, default=5_000_000)
    parser.add_argument("--web-max-points", type=int, default=220_000)
    parser.add_argument("--ros-max-points", type=int, default=500_000)
    parser.add_argument("--web-publish-rate", type=float, default=1.5)
    parser.add_argument("--ros-publish-rate", type=float, default=0.5)
    parser.add_argument("--map-resolution", type=float, default=0.05)
    parser.add_argument("--z-min", type=float, default=0.05)
    parser.add_argument("--z-max", type=float, default=1.50)
    parser.add_argument("--radius-filter", type=float, default=0.50)
    parser.add_argument("--min-neighbors", type=int, default=10)
    parser.add_argument("--map-padding", type=float, default=0.25)
    parser.add_argument("--http-log", action="store_true")
    return parser


def main() -> int:
    parser = make_argument_parser()
    settings, ros_args = parser.parse_known_args()
    if settings.port < 1 or settings.port > 65535:
        parser.error("port must be between 1 and 65535")
    if settings.web_publish_rate <= 0 or settings.ros_publish_rate <= 0:
        parser.error("publish rates must be greater than zero")
    web_dir = Path(settings.web_dir).expanduser().resolve()
    if not web_dir.joinpath("index.html").is_file():
        parser.error(f"web directory has no index.html: {web_dir}")

    rclpy.init(args=ros_args)
    hub = WebSocketHub()
    node = MappingSupervisor(settings, hub)
    handler = partial(RequestHandler, directory=str(web_dir))
    server = MappingHttpServer((settings.host, settings.port), handler, node, hub)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True, name="mapping-http")
    http_thread.start()
    node.get_logger().info(f"Open http://127.0.0.1:{settings.port}")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_managed_stack()
        server.shutdown()
        server.server_close()
        hub.close_all()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
