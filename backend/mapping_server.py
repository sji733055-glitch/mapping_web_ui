#!/usr/bin/env python3
"""ROS 2 point-cloud accumulator plus a dependency-free HTTP/WebSocket UI server."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import replace
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
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
    from .mapping_core import (
        BinDisappearanceConfig,
        MapExportConfig,
        VoxelAccumulator,
        build_cloud_frame,
        build_pcd_map_preview,
        convert_pcd_to_map,
        export_pcd_session,
        filter_dynamic_with_history_consensus,
        read_binary_pcd,
        resolve_pcd_input,
        sanitize_map_name,
        select_visibility_endpoints,
        slice_points_by_config,
        visibility_miss_keys,
        write_keyframe_history_chunk,
        write_keyframe_history_metadata,
        write_mapping_report,
    )
    from .map_frame_core import align_map_frame, initialize_frame_metadata, map_frame_status
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
    from mapping_core import (
        BinDisappearanceConfig,
        MapExportConfig,
        VoxelAccumulator,
        build_cloud_frame,
        build_pcd_map_preview,
        convert_pcd_to_map,
        export_pcd_session,
        filter_dynamic_with_history_consensus,
        read_binary_pcd,
        resolve_pcd_input,
        sanitize_map_name,
        select_visibility_endpoints,
        slice_points_by_config,
        visibility_miss_keys,
        write_keyframe_history_chunk,
        write_keyframe_history_metadata,
        write_mapping_report,
    )
    from map_frame_core import align_map_frame, initialize_frame_metadata, map_frame_status
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

# Slice parameters the offline 2-D conversion accepts from the operator console.
EXPORT_OVERRIDE_KEYS = (
    "resolution", "z_min", "z_max", "radius", "min_neighbors", "padding", "height_mode",
)
EXPORT_OVERRIDE_LABELS = {
    "resolution": "分辨率",
    "z_min": "Z 下限",
    "z_max": "Z 上限",
    "radius": "滤波半径",
    "min_neighbors": "最小邻点",
    "padding": "边缘留白",
    "height_mode": "高度基准",
}


def export_config_from_overrides(
    base: MapExportConfig,
    overrides: Optional[dict[str, Any]],
) -> MapExportConfig:
    """Layer operator-supplied slice parameters over the configured defaults.

    Every value is validated before it can reach the rasterizer, so a bad field
    from the browser is reported as one clear Chinese message instead of a
    partial conversion.
    """
    if not overrides:
        return base
    if not isinstance(overrides, dict):
        raise ValueError("二维切片参数必须是对象")
    values: dict[str, float | int | str] = {}
    for key in EXPORT_OVERRIDE_KEYS:
        raw = overrides.get(key)
        if raw is None or raw == "":
            continue
        label = EXPORT_OVERRIDE_LABELS[key]
        if key == "height_mode":
            mode = str(raw).strip().lower()
            if mode not in {"ground", "absolute"}:
                raise ValueError("高度基准必须是 ground 或 absolute")
            values[key] = mode
            continue
        try:
            number = float(raw)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{label}必须是数值") from error
        if not math.isfinite(number):
            raise ValueError(f"{label}必须是有限数值")
        if key == "min_neighbors":
            if number < 0 or number != int(number):
                raise ValueError("最小邻点数必须是非负整数")
            values[key] = int(number)
        else:
            values[key] = number
    try:
        config = replace(base, **values)
        config.validate()
    except (TypeError, ValueError) as error:
        raise ValueError(f"二维切片参数无效：{error}") from error
    return config


def polar_disappearance_config(settings: argparse.Namespace) -> BinDisappearanceConfig:
    """Build the offline ERASOR-style polar vote configuration once."""
    return BinDisappearanceConfig(
        min_range=settings.polar_min_range,
        max_range=settings.polar_max_range,
        rings=settings.polar_rings,
        sectors=settings.polar_sectors,
        cell_size=settings.polar_cell_size,
        min_scan_points=settings.polar_min_scan_points,
        scan_ratio_threshold=settings.polar_scan_ratio_threshold,
        structure_span=settings.polar_structure_span,
        height_threshold=settings.polar_height_threshold,
        confirm_tolerance=settings.polar_confirm_tolerance,
        min_votes=settings.polar_min_votes,
        max_removal_fraction=settings.polar_max_removal_fraction,
    )


def occupancy_preview_headers(metadata: dict[str, Any]) -> dict[str, str]:
    """Describe one preview grid through response headers.

    The body stays a plain PGM so the browser can decode pixels, while the
    authoritative geometry travels as numbers the UI can render verbatim.
    """
    headers = {
        "X-Map-Width": str(metadata["width"]),
        "X-Map-Height": str(metadata["height"]),
        "X-Map-Resolution": f"{float(metadata['resolution']):.8g}",
        "X-Map-Origin-X": f"{float(metadata['origin_x']):.8g}",
        "X-Map-Origin-Y": f"{float(metadata['origin_y']):.8g}",
        "X-Map-Z-Min": f"{float(metadata['z_min']):.8g}",
        "X-Map-Z-Max": f"{float(metadata['z_max']):.8g}",
        "X-Map-Height-Mode": str(metadata["height_mode"]),
        "X-Map-Slice-Points": str(metadata["slice_points"]),
        "X-Map-Filtered-Points": str(metadata["filtered_points"]),
        "X-Map-Occupied": str(metadata["occupied_cells"]),
        "X-Map-Preview-Stride": str(metadata["preview_stride"]),
        "X-Map-Preview-Occupied": str(metadata["preview_occupied_cells"]),
    }
    if metadata.get("height_mode") == "ground":
        headers.update({
            "X-Map-Ground-A": f"{float(metadata['ground_a']):.10g}",
            "X-Map-Ground-B": f"{float(metadata['ground_b']):.10g}",
            "X-Map-Ground-C": f"{float(metadata['ground_c']):.10g}",
            "X-Map-Ground-Tilt": f"{float(metadata['ground_tilt_deg']):.8g}",
            "X-Map-Ground-Cells": str(metadata["ground_inlier_cells"]),
        })
    return headers


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
        self._broadcast(0x2, build_cloud_frame(points)[0])

    def close_all(self) -> None:
        with self._lock:
            clients = list(self._clients)
            self._clients.clear()
        for client in clients:
            client.close()


class KeyframeHistoryRecorder:
    """Bounded asynchronous writer for ray-only mapping keyframes."""

    def __init__(self, directory: Path, settings: argparse.Namespace, map_name: str) -> None:
        self.directory = Path(directory)
        self.settings = settings
        self.map_name = map_name
        self.directory.mkdir(parents=True, exist_ok=False)
        self._queue: queue.Queue[Optional[tuple[int, np.ndarray, np.ndarray, int]]] = queue.Queue(
            maxsize=settings.keyframe_queue_size
        )
        self._lock = threading.RLock()
        self._accepting = True
        self._closed = False
        self._close_done = threading.Event()
        self._reserved_bytes = 0
        self._written_bytes = 0
        self._written_frames = 0
        self._dropped_frames = 0
        self._chunk_count = 0
        self._limit_reached = False
        self._last_error = ""
        self._thread = threading.Thread(
            target=self._worker,
            daemon=True,
            name="keyframe-history",
        )
        self._write_metadata(completed=False)
        self._thread.start()

    def _metadata(self, completed: bool) -> dict[str, object]:
        return {
            "format": "mapping-web-keyframes-v1",
            "map_name": self.map_name,
            "completed": completed,
            "written_frames": self._written_frames,
            "dropped_frames": self._dropped_frames,
            "chunk_count": self._chunk_count,
            "bytes_written": self._written_bytes,
            "limit_bytes": int(self.settings.keyframe_max_disk_gb * 1024 ** 3),
            "limit_reached": self._limit_reached,
            "last_error": self._last_error,
            "min_range": self.settings.dynamic_min_range,
            "max_range": self.settings.dynamic_max_range,
            "max_rays": self.settings.keyframe_max_rays,
            "angular_resolution_deg": self.settings.dynamic_angular_resolution,
        }

    def _write_metadata(self, completed: bool) -> None:
        write_keyframe_history_metadata(self.directory, self._metadata(completed))

    def submit(self, timestamp_ns: int, origin: np.ndarray, points: np.ndarray) -> bool:
        estimated_points = min(len(points), self.settings.keyframe_max_rays)
        estimated_bytes = 64 + estimated_points * 12
        with self._lock:
            maximum = int(self.settings.keyframe_max_disk_gb * 1024 ** 3)
            if not self._accepting or self._last_error:
                return False
            if self._reserved_bytes + estimated_bytes > maximum:
                self._limit_reached = True
                self._accepting = False
                return False
            task = (
                max(0, int(timestamp_ns)),
                np.ascontiguousarray(origin, dtype=np.float64),
                np.ascontiguousarray(points[:, :3], dtype=np.float32),
                estimated_bytes,
            )
            try:
                self._queue.put_nowait(task)
            except queue.Full:
                self._dropped_frames += 1
                return False
            self._reserved_bytes += estimated_bytes
            return True

    def _flush_chunk(
        self,
        records: list[tuple[int, np.ndarray, np.ndarray]],
    ) -> None:
        if not records:
            return
        path = self.directory / f"chunk_{self._chunk_count:06d}.bin"
        written = write_keyframe_history_chunk(path, records)
        with self._lock:
            self._written_bytes += written
            self._written_frames += len(records)
            self._chunk_count += 1
            metadata = self._metadata(completed=False)
        # Metadata fsync stays outside the submission lock so a ROS callback
        # never waits for disk I/O when enqueueing the next keyframe.
        write_keyframe_history_metadata(self.directory, metadata)
        records.clear()

    def _worker(self) -> None:
        records: list[tuple[int, np.ndarray, np.ndarray]] = []
        while True:
            task = self._queue.get()
            stopping = task is None
            try:
                if stopping:
                    self._flush_chunk(records)
                else:
                    timestamp_ns, origin, points, _estimated_bytes = task
                    # The stored rays are what the offline replay can ever carve
                    # along, so the comb rotates with the recorded frame index;
                    # a fixed subset would make whole directions unfilterable.
                    with self._lock:
                        phase = self._written_frames + len(records)
                    selected = select_visibility_endpoints(
                        origin,
                        points,
                        min_range=self.settings.dynamic_min_range,
                        max_range=self.settings.dynamic_max_range,
                        max_rays=self.settings.keyframe_max_rays,
                        angular_resolution=math.radians(self.settings.dynamic_angular_resolution),
                        phase=phase,
                    )
                    records.append((timestamp_ns, origin, selected))
                    if len(records) >= self.settings.keyframe_chunk_frames:
                        self._flush_chunk(records)
            except Exception as error:
                with self._lock:
                    self._last_error = str(error)
                    self._accepting = False
            finally:
                self._queue.task_done()
            if stopping:
                return

    def close(self, completed: bool = True) -> Path:
        with self._lock:
            if self._closed:
                already_closing = True
            else:
                already_closing = False
                self._accepting = False
                self._closed = True
        if already_closing:
            self._close_done.wait()
            with self._lock:
                return self.directory
        self._queue.put(None)
        self._thread.join()
        try:
            with self._lock:
                self._write_metadata(completed=completed and not bool(self._last_error))
                if completed and not self._last_error and self.directory.name.endswith(".partial"):
                    final_name = self.directory.name.removesuffix(".partial").lstrip(".")
                    final = self.directory.with_name(final_name + ".history")
                    if final.exists():
                        final = self.directory.with_name(
                            final_name + f"_{time.time_ns()}.history"
                        )
                    os.replace(self.directory, final)
                    self.directory = final
                return self.directory
        finally:
            self._close_done.set()

    def stats(self) -> dict[str, object]:
        with self._lock:
            return {
                "enabled": True,
                "written_frames": self._written_frames,
                "dropped_frames": self._dropped_frames,
                "queued_frames": self._queue.qsize(),
                "chunk_count": self._chunk_count,
                "bytes_written": self._written_bytes,
                "limit_bytes": int(self.settings.keyframe_max_disk_gb * 1024 ** 3),
                "limit_reached": self._limit_reached,
                "last_error": self._last_error,
                "path": str(self.directory),
            }


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
            height_mode=settings.height_mode,
        )
        self.export_config.validate()
        self.polar_dynamic_config = polar_disappearance_config(settings)
        self.polar_dynamic_config.validate()
        self._state_lock = threading.RLock()
        self._command_lock = threading.Lock()
        self._state = "IDLE"
        self._session_name = ""
        self._message = "可以开始新的建图会话"
        self._started_monotonic: Optional[float] = None
        self._elapsed_at_stop = 0.0
        self._travel_distance = 0.0
        self._last_odom_position: Optional[np.ndarray] = None
        self._odom_history: deque[tuple[int, np.ndarray, np.ndarray]] = deque(maxlen=256)
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
        self._session_epoch = 0
        self._cloud_generation = 0
        self._dynamic_condition = threading.Condition()
        self._dynamic_pending: Optional[tuple[int, int, np.ndarray, np.ndarray]] = None
        self._dynamic_shutdown = False
        self._dynamic_last_submitted_at = 0.0
        self._dynamic_stats: dict[str, Any] = {}
        self._reset_dynamic_stats()
        self._dynamic_thread: Optional[threading.Thread] = None
        if settings.dynamic_removal:
            self._dynamic_thread = threading.Thread(
                target=self._dynamic_worker,
                daemon=True,
                name="dynamic-removal",
            )
            self._dynamic_thread.start()
        self._history_recorder: Optional[KeyframeHistoryRecorder] = None
        self._history_last_origin: Optional[np.ndarray] = None
        self._history_last_orientation: Optional[np.ndarray] = None
        self._history_last_at = 0.0
        self._history_offline_result: dict[str, object] = {}

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

    @staticmethod
    def _stamp_nanoseconds(stamp: Time) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _reset_dynamic_stats(self) -> None:
        self._dynamic_stats = {
            "processed_frames": 0,
            "removed_points": 0,
            "candidate_voxels": 0,
            "skipped_no_odom": 0,
            "replaced_pending_frames": 0,
            "last_duration_ms": 0.0,
            "last_error": "",
        }

    def _matched_odom_pose(
        self,
        stamp_ns: int,
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        if not self._odom_history:
            return None
        if stamp_ns <= 0:
            _stamp, origin, orientation = self._odom_history[-1]
            return origin.copy(), orientation.copy()
        odom_stamp, origin, orientation = min(
            self._odom_history,
            key=lambda item: abs(item[0] - stamp_ns),
        )
        tolerance_ns = int(self.settings.dynamic_odom_tolerance * 1_000_000_000)
        if odom_stamp <= 0 or abs(odom_stamp - stamp_ns) > tolerance_ns:
            return None
        return origin.copy(), orientation.copy()

    @staticmethod
    def _quaternion_distance(first: np.ndarray, second: np.ndarray) -> float:
        dot = float(abs(np.dot(first, second)))
        return 2.0 * math.acos(max(-1.0, min(1.0, dot)))

    def _history_keyframe_due(
        self,
        origin: np.ndarray,
        orientation: np.ndarray,
        now: float,
    ) -> bool:
        if self._history_recorder is None:
            return False
        if self._history_last_origin is None or self._history_last_orientation is None:
            return True
        elapsed = now - self._history_last_at
        if elapsed < self.settings.keyframe_min_interval:
            return False
        translation = float(np.linalg.norm(origin - self._history_last_origin))
        rotation = self._quaternion_distance(orientation, self._history_last_orientation)
        return (
            translation >= self.settings.keyframe_translation
            or rotation >= math.radians(self.settings.keyframe_rotation_deg)
            or elapsed >= self.settings.keyframe_max_interval
        )

    def _submit_dynamic_frame(
        self,
        epoch: int,
        generation: int,
        origin: np.ndarray,
        points: np.ndarray,
        now: float,
    ) -> None:
        interval = 1.0 / self.settings.dynamic_removal_rate
        if now - self._dynamic_last_submitted_at < interval:
            return
        self._dynamic_last_submitted_at = now
        task = (
            epoch,
            generation,
            np.ascontiguousarray(origin, dtype=np.float64),
            np.ascontiguousarray(points[:, :3], dtype=np.float32),
        )
        with self._dynamic_condition:
            replaced = self._dynamic_pending is not None
            self._dynamic_pending = task
            self._dynamic_condition.notify()
        if replaced:
            with self._state_lock:
                self._dynamic_stats["replaced_pending_frames"] += 1

    def _dynamic_worker(self) -> None:
        while True:
            with self._dynamic_condition:
                while self._dynamic_pending is None and not self._dynamic_shutdown:
                    self._dynamic_condition.wait()
                if self._dynamic_shutdown:
                    return
                task = self._dynamic_pending
                self._dynamic_pending = None
            if task is None:
                continue
            epoch, generation, origin, points = task
            started = time.monotonic()
            # Each processed frame advances the ray comb so every angular bin
            # eventually produces free-space evidence, even while the sensor
            # stands still and the scan holds more bins than the ray budget.
            phase = int(self._dynamic_stats.get("processed_frames", 0))
            try:
                keys = visibility_miss_keys(
                    origin,
                    points,
                    self.settings.voxel_size,
                    min_range=self.settings.dynamic_min_range,
                    max_range=self.settings.dynamic_max_range,
                    max_rays=self.settings.dynamic_max_rays,
                    angular_resolution=math.radians(self.settings.dynamic_angular_resolution),
                    phase=phase,
                )
                with self._state_lock:
                    if self._state != "MAPPING" or epoch != self._session_epoch:
                        continue
                    removed = self.accumulator.apply_misses(
                        keys,
                        generation,
                        self.settings.dynamic_miss_threshold,
                    )
                    self._dynamic_stats["processed_frames"] += 1
                    self._dynamic_stats["removed_points"] += removed
                    self._dynamic_stats["candidate_voxels"] += len(keys)
                    self._dynamic_stats["last_duration_ms"] = (
                        time.monotonic() - started
                    ) * 1_000.0
                    self._dynamic_stats["last_error"] = ""
            except Exception as error:
                self.get_logger().error(f"Dynamic removal failed: {error}")
                with self._state_lock:
                    self._dynamic_stats["last_error"] = str(error)

    def close_dynamic_worker(self) -> None:
        with self._dynamic_condition:
            self._dynamic_shutdown = True
            self._dynamic_pending = None
            self._dynamic_condition.notify_all()
        if self._dynamic_thread is not None:
            self._dynamic_thread.join(timeout=2.0)

    def close_history_recorder(self) -> None:
        recorder = self._history_recorder
        if recorder is not None:
            try:
                recorder.close(completed=self._state not in {"MAPPING", "SAVING"})
            except Exception as error:
                self.get_logger().error(f"Keyframe history shutdown failed: {error}")

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
            dynamic_task: Optional[tuple[int, int, np.ndarray]] = None
            with self._state_lock:
                if self._state != "MAPPING":
                    return
                self._cloud_generation += 1
                generation = self._cloud_generation
                epoch = self._session_epoch
                self.accumulator.add(points, generation=generation)
                interval = 1.0 / self.settings.dynamic_removal_rate
                dynamic_due = (
                    self.settings.dynamic_removal
                    and now - self._dynamic_last_submitted_at >= interval
                )
                history_due = self._history_recorder is not None and (
                    self._history_last_origin is None
                    or now - self._history_last_at >= self.settings.keyframe_min_interval
                )
                if dynamic_due or history_due:
                    stamp_ns = self._stamp_nanoseconds(message.header.stamp)
                    matched_pose = self._matched_odom_pose(stamp_ns)
                    if matched_pose is None:
                        if dynamic_due:
                            self._dynamic_last_submitted_at = now
                            self._dynamic_stats["skipped_no_odom"] += 1
                    else:
                        origin, orientation = matched_pose
                        if dynamic_due:
                            dynamic_task = (epoch, generation, origin)
                        if self._history_keyframe_due(origin, orientation, now):
                            recorder = self._history_recorder
                            if recorder is not None and recorder.submit(stamp_ns, origin, points):
                                self._history_last_origin = origin.copy()
                                self._history_last_orientation = orientation.copy()
                                self._history_last_at = now
                if self.accumulator.point_count >= self.settings.max_points:
                    self._message = f"已达到最大累计点数 {self.settings.max_points:,}，请结束并保存"
            if dynamic_task is not None:
                epoch, generation, origin = dynamic_task
                self._submit_dynamic_frame(epoch, generation, origin, points, now)
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
        orientation = message.pose.pose.orientation
        quaternion = np.array([
            orientation.x, orientation.y, orientation.z, orientation.w,
        ], dtype=np.float64)
        offset = np.array([
            self.settings.dynamic_origin_offset_x,
            self.settings.dynamic_origin_offset_y,
            self.settings.dynamic_origin_offset_z,
        ], dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if norm > 1e-9 and np.isfinite(norm):
            quaternion /= norm
            vector = quaternion[:3]
            twice_cross = 2.0 * np.cross(vector, offset)
            offset = offset + quaternion[3] * twice_cross + np.cross(vector, twice_cross)
        else:
            quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        sensor_origin = position + offset
        stamp_ns = self._stamp_nanoseconds(message.header.stamp)
        with self._state_lock:
            self._last_odom_at = now
            if np.isfinite(sensor_origin).all():
                self._odom_history.append((stamp_ns, sensor_origin, quaternion.copy()))
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
            history = (
                self._history_recorder.stats()
                if self._history_recorder is not None
                else {
                    "enabled": bool(self.settings.keyframe_history),
                    "written_frames": 0,
                    "dropped_frames": 0,
                    "queued_frames": 0,
                    "chunk_count": 0,
                    "bytes_written": 0,
                    "limit_bytes": int(self.settings.keyframe_max_disk_gb * 1024 ** 3),
                    "limit_reached": False,
                    "last_error": "",
                    "path": "",
                }
            )
            history["offline_filter"] = dict(self._history_offline_result)
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
                "map_export": {
                    "resolution": self.export_config.resolution,
                    "z_min": self.export_config.z_min,
                    "z_max": self.export_config.z_max,
                    "radius": self.export_config.radius,
                    "min_neighbors": self.export_config.min_neighbors,
                    "padding": self.export_config.padding,
                    "height_mode": self.export_config.height_mode,
                },
                "dynamic_removal": {
                    "enabled": bool(self.settings.dynamic_removal),
                    "miss_threshold": self.settings.dynamic_miss_threshold,
                    "min_range": self.settings.dynamic_min_range,
                    "max_range": self.settings.dynamic_max_range,
                    "processing_rate_hz": self.settings.dynamic_removal_rate,
                    "max_rays": self.settings.dynamic_max_rays,
                    **self._dynamic_stats,
                },
                "polar_disappearance_filter": {
                    "enabled": bool(self.settings.polar_disappearance_filter),
                    "min_range": self.polar_dynamic_config.min_range,
                    "max_range": self.polar_dynamic_config.max_range,
                    "rings": self.polar_dynamic_config.rings,
                    "sectors": self.polar_dynamic_config.sectors,
                    "cell_size": self.polar_dynamic_config.cell_size,
                    "min_scan_points": self.polar_dynamic_config.min_scan_points,
                    "scan_ratio_threshold": self.polar_dynamic_config.scan_ratio_threshold,
                    "structure_span": self.polar_dynamic_config.structure_span,
                    "height_threshold": self.polar_dynamic_config.height_threshold,
                    "confirm_tolerance": self.polar_dynamic_config.confirm_tolerance,
                    "min_votes": self.polar_dynamic_config.min_votes,
                    "max_removal_fraction": self.polar_dynamic_config.max_removal_fraction,
                },
                "keyframe_history": history,
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

    def _local_lio_build_status(self) -> dict[str, Any]:
        source_root = Path(self.settings.local_lio_source).expanduser().resolve()
        executable = Path(os.path.abspath(os.path.expanduser(self.settings.local_lio_executable)))
        ready = executable.is_file() and os.access(executable, os.X_OK)
        stale = False
        if ready and source_root.is_dir():
            source_files = [
                path for path in source_root.rglob("*")
                if path.is_file() and (
                    path.suffix in {".c", ".cc", ".cpp", ".h", ".hh", ".hpp", ".in"}
                    or path.name in {"CMakeLists.txt", "package.xml"}
                )
            ]
            if source_files:
                try:
                    stale = max(path.stat().st_mtime_ns for path in source_files) > executable.stat().st_mtime_ns
                except OSError:
                    stale = True
        return {
            "source": str(source_root),
            "executable": str(executable),
            "ready": ready,
            "stale": stale,
        }

    def stack_status(self) -> dict[str, Any]:
        with self._stack_lock:
            self._reap_stack_processes()
            graph = self._graph_nodes()
            local_lio = self._local_lio_build_status()
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
            nodes["small_point_lio"]["implementation"] = (
                "project_local" if nodes["small_point_lio"]["managed"] else "external"
            )
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
            elif state in {"STOPPED", "PARTIAL"} and not local_lio["ready"]:
                message = "项目内 Small Point-LIO 尚未构建，请先构建 ros2_ws"
            elif state in {"STOPPED", "PARTIAL"} and local_lio["stale"]:
                message = "项目内 Small Point-LIO 源码已变更，请重新构建 ros2_ws"
            return {
                "state": state,
                "nodes": nodes,
                "managed_count": len(self._stack_processes),
                "message": message,
                "log_dir": str(Path(self.settings.process_log_dir).resolve()),
                "local_lio": local_lio,
            }

    def _stack_commands(self) -> dict[str, list[str]]:
        params = str(Path(self.settings.lio_params).expanduser().resolve())
        local_lio = os.path.abspath(os.path.expanduser(self.settings.local_lio_executable))
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
                local_lio, "--ros-args",
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
            if "small_point_lio" in missing:
                local_lio = self._local_lio_build_status()
                if not local_lio["ready"]:
                    return False, (
                        "项目内 Small Point-LIO 尚未构建；请先在 ros2_ws 执行 "
                        "colcon build --symlink-install --packages-select small_point_lio "
                        "--allow-overriding small_point_lio"
                    )
                if local_lio["stale"]:
                    return False, "项目内 Small Point-LIO 源码比可执行文件新，请重新执行 colcon build"
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
            recorder: Optional[KeyframeHistoryRecorder] = None
            if self.settings.keyframe_history:
                history_root = self.output_root / "session_history"
                history_root.mkdir(parents=True, exist_ok=True)
                history_dir = history_root / f".{name}_{time.time_ns()}.partial"
                try:
                    recorder = KeyframeHistoryRecorder(history_dir, self.settings, name)
                except Exception as error:
                    return False, f"无法创建关键帧历史：{error}"
            self._session_epoch += 1
            self._cloud_generation = 0
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
            self._dynamic_last_submitted_at = 0.0
            self._reset_dynamic_stats()
            self._history_recorder = recorder
            self._history_last_origin = None
            self._history_last_orientation = None
            self._history_last_at = 0.0
            self._history_offline_result = {}
        with self._dynamic_condition:
            self._dynamic_pending = None
        self.hub.broadcast_cloud(np.empty((0, 3), dtype=np.float32))
        features = []
        if self.settings.dynamic_removal:
            features.append("在线去动态")
        if self.settings.keyframe_history:
            features.append("关键帧历史")
        suffix = f"，{' + '.join(features)}已开启" if features else ""
        return True, f"建图会话 {name} 已开始{suffix}"

    def _available_output_name(self, name: str) -> str:
        def occupied(candidate: str) -> bool:
            return any(path.exists() for path in (
                self.output_root / "pcd" / f"{candidate}.pcd",
                self.output_root / "map" / f"{candidate}.pgm",
                self.output_root / "map" / f"{candidate}.yaml",
                self.output_root / "map" / f"{candidate}_terrain.msgpack",
                self.output_root / "map" / f"{candidate}_frame.json",
            ))

        if not occupied(name):
            return name
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        index = 1
        while True:
            suffix = f"_{timestamp}" if index == 1 else f"_{timestamp}_{index}"
            candidate = f"{name[:48 - len(suffix)]}{suffix}"
            if not occupied(candidate):
                return candidate
            index += 1

    def _has_aligned_frame(self, map_name: str) -> bool:
        """True when an operator already aligned this name's map coordinate frame.

        ``结束并保存`` writes an identity frame record together with the PCD, so
        that companion file must not push the natural same-name conversion onto a
        timestamped name; a record carrying a real alignment must be protected,
        because converting over it would reset the coordinate frame.
        """
        try:
            status = map_frame_status(self.output_root, map_name)
        except (OSError, ValueError):
            return True
        transform = status.get("source_to_map") or {}
        return any(abs(float(transform.get(axis, 0.0))) > 1e-9 for axis in ("x", "y", "yaw"))

    def _conversion_conflict(self, name: str) -> bool:
        """True when a 2-D map name is already taken by real artifacts or alignment."""
        artifacts = (
            self.output_root / "pcd" / f"{name}.pcd",
            self.output_root / "map" / f"{name}.pgm",
            self.output_root / "map" / f"{name}.yaml",
            self.output_root / "map" / f"{name}_terrain.msgpack",
        )
        return any(path.exists() for path in artifacts) or self._has_aligned_frame(name)

    def _available_pcd_conversion_name(self, source_name: str) -> str:
        """Keep the source name unless its derived 2-D map already exists."""
        map_candidates = (
            self.output_root / "map" / f"{source_name}.pgm",
            self.output_root / "map" / f"{source_name}.yaml",
            self.output_root / "map" / f"{source_name}_terrain.msgpack",
        )
        if not any(path.exists() for path in map_candidates) and not self._has_aligned_frame(source_name):
            return source_name
        return self._available_output_name(source_name)

    def _available_conversion_name(self, source_name: str, requested: str = "") -> str:
        """Pick a conflict-free output name for one PCD-to-map conversion.

        An operator-chosen name is honored unless one of its files already
        exists; the source name itself is never treated as a conflict because
        the conversion reuses that PCD instead of rewriting it.
        """
        if not requested or requested == source_name:
            return self._available_pcd_conversion_name(source_name)
        if not self._conversion_conflict(requested):
            return requested
        return self._available_output_name(requested)

    def _require_idle_for_offline_projection(self, action: str) -> None:
        with self._state_lock:
            if self._state in {"MAPPING", "SAVING"}:
                raise RuntimeError(f"建图或保存进行中，请结束后再{action}")

    def begin_save(self) -> tuple[bool, str]:
        """End the session by writing its 3-D PCD only.

        The 2-D occupancy map is no longer a side effect of saving: it is a
        separate operator step (see :meth:`convert_existing_pcd`), so the height
        slice can be previewed and tuned before it is written.
        """
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
            self._session_epoch += 1
            self._save_progress = 0.05
            self._save_stage = "正在整理累计点云"
            self._message = "正在写入三维 PCD；二维图请在下方按需生成"
            name = self._available_output_name(self._session_name)
            self._session_name = name
        with self._dynamic_condition:
            self._dynamic_pending = None
        thread = threading.Thread(
            target=self._save_worker,
            args=(name,),
            daemon=True,
            name="map-export",
        )
        thread.start()
        return True, f"正在保存 {name}.pcd；二维图不随保存生成，可在下方按需转换"

    def _save_worker(self, name: str) -> None:
        try:
            points = self.accumulator.snapshot()
            input_point_count = int(len(points))
            history_dir: Optional[Path] = None
            offline_result: dict[str, object] = {
                "applied": False,
                "reason": "关键帧历史未开启",
                "input_points": input_point_count,
                "output_points": input_point_count,
                "frames": 0,
                "removed_points": 0,
            }

            recorder = self._history_recorder
            if recorder is not None:
                with self._state_lock:
                    self._save_progress = 0.08
                    self._save_stage = "正在刷新关键帧历史"
                history_dir = recorder.close(completed=True)
                history_stats = recorder.stats()
                if history_stats.get("last_error"):
                    offline_result = {
                        "applied": False,
                        "reason": f"关键帧写入异常：{history_stats['last_error']}",
                        "frames": int(history_stats.get("written_frames", 0)),
                        "removed_points": 0,
                    }
                else:
                    def history_progress(value: float, stage: str) -> None:
                        with self._state_lock:
                            self._save_progress = 0.10 + 0.42 * value
                            self._save_stage = stage

                    try:
                        points, offline_result = filter_dynamic_with_history_consensus(
                            points,
                            history_dir,
                            self.settings.voxel_size,
                            free_frame_threshold=self.settings.keyframe_free_threshold,
                            free_hit_ratio=self.settings.keyframe_free_hit_ratio,
                            max_removal_fraction=self.settings.keyframe_max_removal_fraction,
                            polar_config=(
                                self.polar_dynamic_config
                                if self.settings.polar_disappearance_filter
                                else None
                            ),
                            progress=history_progress,
                        )
                    except Exception as error:
                        self.get_logger().warning(f"Offline keyframe filtering skipped: {error}")
                        offline_result = {
                            "applied": False,
                            "reason": f"离线清理异常：{error}",
                            "input_points": input_point_count,
                            "output_points": input_point_count,
                            "frames": int(history_stats.get("written_frames", 0)),
                            "removed_points": 0,
                        }

            with self._state_lock:
                self._history_offline_result = dict(offline_result)

            def progress(value: float, stage: str) -> None:
                with self._state_lock:
                    self._save_progress = 0.55 + 0.45 * value
                    self._save_stage = stage

            with self._map_file_lock:
                result = export_pcd_session(self.output_root, name, points, progress)
                frame_path = initialize_frame_metadata(
                    self.output_root,
                    name,
                    self._frame_id or "odom",
                )
                result["frame_metadata"] = str(frame_path)
                result["keyframe_filter"] = dict(offline_result)
                report_path = write_mapping_report(
                    self.output_root,
                    name,
                    {
                        "schema_version": 1,
                        "map_name": name,
                        "created_unix_ns": time.time_ns(),
                        "frame_id": self._frame_id or "odom",
                        "voxel_size": self.settings.voxel_size,
                        "input_point_count": input_point_count,
                        "output_point_count": int(len(points)),
                        "keyframe_ray_filter": {
                            "free_frame_threshold": self.settings.keyframe_free_threshold,
                            "free_hit_ratio": self.settings.keyframe_free_hit_ratio,
                            "max_removal_fraction": self.settings.keyframe_max_removal_fraction,
                        },
                        "polar_disappearance_filter": {
                            "enabled": bool(self.settings.polar_disappearance_filter),
                            **vars(self.polar_dynamic_config),
                        },
                        "dynamic_filter": offline_result,
                    },
                )
                result["mapping_report"] = str(report_path)
            if history_dir is not None:
                if self.settings.keep_keyframe_history:
                    result["keyframe_history"] = str(history_dir)
                elif history_dir.exists():
                    try:
                        shutil.rmtree(history_dir)
                    except OSError as error:
                        self.get_logger().warning(
                            f"Could not remove temporary keyframe history {history_dir}: {error}"
                        )
                        result["keyframe_history"] = str(history_dir)
            if offline_result.get("applied") and int(offline_result.get("removed_points", 0)) > 0:
                self.accumulator.replace(points)
                self.hub.broadcast_cloud(
                    self.accumulator.snapshot(self.settings.web_max_points)
                )
            with self._state_lock:
                self._output = result
                self._state = "SAVED"
                self._message = f"点云 {name}.pcd 已保存；二维图可在下方按需生成"
                self._save_progress = 1.0
                self._save_stage = "点云保存完成"
            self.hub.broadcast_json({"type": "result", "ok": True, "message": f"点云 {name}.pcd 已保存"})
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
            self._session_epoch += 1
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
            self._cloud_generation = 0
            self._dynamic_last_submitted_at = 0.0
            self._reset_dynamic_stats()
            self._history_recorder = None
            self._history_last_origin = None
            self._history_last_orientation = None
            self._history_last_at = 0.0
            self._history_offline_result = {}
        with self._dynamic_condition:
            self._dynamic_pending = None
        self.hub.broadcast_cloud(np.empty((0, 3), dtype=np.float32))
        return True, "当前会话已清空"

    def dispatch(
        self,
        action: str,
        map_name: str = "",
    ) -> tuple[bool, str]:
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
            maps = list_editable_maps(self.output_root)
            for entry in maps:
                entry.update(map_frame_status(self.output_root, entry["name"]))
            return maps

    def pcd_preview(
        self,
        name: str,
        overrides: Optional[dict[str, Any]] = None,
    ) -> tuple[bytes, int, int]:
        """Build a browser ``MAP1`` frame for one ``data/pcd`` file.

        Returns ``(body, shown_points, total_points)``.  The file is only read,
        never rewritten, and the name is validated before it becomes a path, so
        traversal names cannot escape ``data/pcd``.  Reading happens outside the
        map-file lock because parsing a large PCD must not stall the save path:
        exports replace files atomically, so a reader sees either version.

        Optional export overrides preview exactly the points a 2-D slice would
        consume, including ground-relative slicing.  They filter only the
        transmitted frame and never the stored file.
        """
        path = resolve_pcd_input(self.output_root / "pcd", name)
        points = read_binary_pcd(path)
        if overrides:
            config = export_config_from_overrides(self.export_config, overrides)
            points, _metadata = slice_points_by_config(points, config)
        return build_cloud_frame(points, self.settings.web_max_points)

    def pcd_files(self) -> list[dict[str, Any]]:
        """List safe, server-local PCD inputs without parsing their payloads."""
        pcd_dir = self.output_root / "pcd"
        with self._map_file_lock:
            result: list[dict[str, Any]] = []
            for path in pcd_dir.glob("*.pcd"):
                if not path.is_file():
                    continue
                try:
                    name = sanitize_map_name(path.stem)
                    stat = path.stat()
                except (OSError, ValueError):
                    continue
                result.append({
                    "name": name,
                    "filename": path.name,
                    "size_bytes": stat.st_size,
                    "modified_at": stat.st_mtime,
                })
            return sorted(result, key=lambda entry: str(entry["name"]).casefold())

    def convert_existing_pcd(
        self,
        map_name: str,
        overrides: Optional[dict[str, Any]] = None,
        output_name: str = "",
    ) -> dict[str, Any]:
        """Create PGM/YAML from a PCD already present under data/pcd."""
        name = sanitize_map_name(map_name)
        export_config = export_config_from_overrides(self.export_config, overrides)
        requested = str(output_name or "").strip()
        requested_name = sanitize_map_name(requested) if requested else ""
        with self._command_lock:
            self._require_idle_for_offline_projection("转换已有 PCD")
            with self._map_file_lock:
                output_name = self._available_conversion_name(name, requested_name)
                result = convert_pcd_to_map(
                    self.output_root,
                    name,
                    output_name,
                    export_config,
                )
                frame_path = initialize_frame_metadata(self.output_root, output_name, "pcd")
                result["frame_metadata"] = str(frame_path)
                return result

    def preview_pcd_map(
        self,
        map_name: str,
        overrides: Optional[dict[str, Any]] = None,
    ) -> tuple[bytes, dict[str, Any]]:
        """Slice one ``data/pcd`` file for the browser without writing a file.

        The preview and the real conversion share ``slice_points_to_occupancy``,
        so what the operator tunes is exactly what the export will contain.  The
        slicing deliberately runs outside the map-file lock: it only reads a PCD
        that is replaced atomically, and holding the lock would stall an editor
        save for the whole duration of a large cloud.
        """
        name = sanitize_map_name(map_name)
        export_config = export_config_from_overrides(self.export_config, overrides)
        self._require_idle_for_offline_projection("预览二维切片")
        return build_pcd_map_preview(self.output_root, name, export_config)

    def set_map_frame(
        self,
        map_name: str,
        origin_x: float,
        origin_y: float,
        heading_yaw: float,
    ) -> dict[str, Any]:
        with self._state_lock:
            if self._state in {"MAPPING", "SAVING"}:
                raise RuntimeError("建图或保存进行中，不能修改已保存地图的坐标系")
        with self._map_file_lock:
            return align_map_frame(
                self.output_root,
                map_name,
                origin_x,
                origin_y,
                heading_yaw,
            )

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
        if request_path == "/api/pcd-files":
            try:
                self._send_json(
                    HTTPStatus.OK,
                    {"pcd_files": self.server.controller.pcd_files()},
                )
            except OSError as error:
                self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "message": str(error)})
            return
        if request_path == "/api/pcd/preview":
            query = parse_qs(parsed.query)
            try:
                body, shown, total = self.server.controller.pcd_preview(
                    query.get("name", [""])[0],
                    {
                        key: query[key][0]
                        for key in ("z_min", "z_max", "height_mode")
                        if key in query and query[key][0] != ""
                    },
                )
            except FileNotFoundError as error:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "message": str(error)})
                return
            except (OSError, ValueError) as error:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
                return
            self._send_bytes(
                HTTPStatus.OK,
                body,
                "application/octet-stream",
                {"X-Preview-Points": str(shown), "X-Preview-Total": str(total)},
            )
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
            body = build_cloud_frame(points)[0]
            self._send_bytes(HTTPStatus.OK, body, "application/octet-stream")
            return
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlsplit(self.path)
        request_path = parsed.path
        if request_path == "/api/pcd/convert":
            try:
                payload = self._read_json_body()
                result = self.server.controller.convert_existing_pcd(
                    str(payload.get("map_name", "")),
                    self._export_overrides(payload),
                    str(payload.get("output_name", "")),
                )
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "message": (
                            f"{result['source_name']}.pcd 已生成 "
                            f"{result['output_name']}.pgm/.yaml（{result['point_count']:,} 点）"
                        ),
                        "result": result,
                    },
                )
            except RuntimeError as error:
                self._send_json(HTTPStatus.CONFLICT, {"ok": False, "message": str(error)})
            except FileNotFoundError as error:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "message": str(error)})
            except FileExistsError as error:
                self._send_json(HTTPStatus.CONFLICT, {"ok": False, "message": str(error)})
            except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
            return
        if request_path == "/api/pcd/map-preview":
            try:
                payload = self._read_json_body()
                body, metadata = self.server.controller.preview_pcd_map(
                    str(payload.get("map_name", "")),
                    self._export_overrides(payload),
                )
                self._send_bytes(
                    HTTPStatus.OK,
                    body,
                    "image/x-portable-graymap",
                    occupancy_preview_headers(metadata),
                )
            except RuntimeError as error:
                self._send_json(HTTPStatus.CONFLICT, {"ok": False, "message": str(error)})
            except FileNotFoundError as error:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "message": str(error)})
            except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "message": str(error)})
            return
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
        if request_path == "/api/editor/frame":
            try:
                payload = self._read_json_body()
                result = self.server.controller.set_map_frame(
                    str(payload.get("map_name", "")),
                    float(payload.get("origin_x")),
                    float(payload.get("origin_y")),
                    float(payload.get("heading_yaw")),
                )
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "message": (
                            f"{result['map_name']} 已统一到 map 坐标系；"
                            f"PCD、二维图{'+ terrain' if result['terrain_transformed'] else ''} 已同步变换"
                        ),
                        "frame": result,
                    },
                )
            except RuntimeError as error:
                self._send_json(HTTPStatus.CONFLICT, {"ok": False, "message": str(error)})
            except FileNotFoundError as error:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "message": str(error)})
            except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
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
            ok, message = self.server.controller.dispatch(
                action,
                str(payload.get("map_name", "")),
            )
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

    @staticmethod
    def _optional_float(value: str, label: str) -> Optional[float]:
        """Parse an optional numeric query parameter, rejecting junk early."""
        text = str(value).strip()
        if not text:
            return None
        try:
            number = float(text)
        except ValueError as error:
            raise ValueError(f"{label} 必须是数值") from error
        if not math.isfinite(number):
            raise ValueError(f"{label} 必须是有限数值")
        return number

    @staticmethod
    def _export_overrides(payload: dict[str, Any]) -> dict[str, Any]:
        """Keep only the slice parameters the offline conversion understands."""
        return {key: payload[key] for key in EXPORT_OVERRIDE_KEYS if key in payload}

    def _read_json_body(self) -> dict[str, Any]:
        payload = json.loads(self._read_body(64 * 1024) or b"{}")
        if not isinstance(payload, dict):
            raise ValueError("JSON 请求体必须是对象")
        return payload

    def _send_bytes(
        self,
        status: HTTPStatus,
        body: bytes,
        content_type: str,
        extra_headers: Optional[dict[str, str]] = None,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for key, value in (extra_headers or {}).items():
            self.send_header(key, value)
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
                    str(command.get("action", "")),
                    str(command.get("map_name", "")),
                )
                client.send(0x1, json.dumps({"type": "result", "ok": ok, "message": message}, ensure_ascii=False).encode("utf-8"))
        except (ConnectionError, OSError, ValueError, json.JSONDecodeError):
            pass
        finally:
            self.server.hub.remove(client)
            self.close_connection = True

    def end_headers(self) -> None:
        # ``BaseHTTPRequestHandler.parse_request()`` can call ``send_error``
        # before it has assigned ``self.path`` (for example for a malformed or
        # overly long request line).  Error responses still pass through this
        # override, so do not assume a successfully parsed request here.
        if getattr(self, "path", None) != "/ws":
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt: str, *args: object) -> None:
        if getattr(self.server.controller.settings, "http_log", False):
            super().log_message(fmt, *args)


def ssh_peer_address() -> Optional[str]:
    """Return the local address this SSH session's client can reach, else None.

    The UDP connect only asks the kernel which route and source address it would
    use; no packet is sent. This keeps the printed address on the network the
    operator is actually connected from instead of a proxy or docker interface.
    """
    peer = os.environ.get("SSH_CONNECTION", "").split(" ", 1)[0].strip()
    if peer.count(".") != 3:
        return None
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((peer, 9))
        address = str(probe.getsockname()[0])
    except OSError:
        return None
    finally:
        probe.close()
    return address if address and address != "127.0.0.1" else None


def console_urls(host: str, port: int) -> list[str]:
    """Console URLs worth showing the operator for this bind host.

    A wildcard bind means the browser may well run on another machine, where
    `127.0.0.1` points at that machine instead of this one, so the address of
    the SSH session's own network is listed as well.
    """
    if host not in {"", "*", "0.0.0.0", "::", "[::]"}:
        literal = f"[{host}]" if ":" in host and not host.startswith("[") else host
        return [f"http://{literal}:{port}"]
    urls = [f"http://127.0.0.1:{port}"]
    address = ssh_peer_address()
    if address:
        urls.append(f"http://{address}:{port}")
    return urls


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
    parser.add_argument(
        "--local-lio-source",
        default=str(project_root / "ros2_ws" / "src" / "small_point_lio"),
    )
    parser.add_argument(
        "--local-lio-executable",
        default=str(
            project_root / "ros2_ws" / "install" / "small_point_lio"
            / "lib" / "small_point_lio" / "small_point_lio_node"
        ),
    )
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
    parser.add_argument(
        "--dynamic-removal", dest="dynamic_removal", action="store_true",
        help="开启基于可见空闲射线的动态拖影清理（默认开启）",
    )
    parser.add_argument(
        "--no-dynamic-removal", dest="dynamic_removal", action="store_false",
        help="禁用动态拖影清理，恢复纯并集累积",
    )
    parser.set_defaults(dynamic_removal=True)
    parser.add_argument("--dynamic-miss-threshold", type=int, default=5)
    parser.add_argument("--dynamic-min-range", type=float, default=0.38)
    parser.add_argument("--dynamic-max-range", type=float, default=10.0)
    parser.add_argument("--dynamic-removal-rate", type=float, default=3.0)
    parser.add_argument("--dynamic-max-rays", type=int, default=4_000)
    parser.add_argument("--dynamic-angular-resolution", type=float, default=0.5)
    parser.add_argument("--dynamic-odom-tolerance", type=float, default=0.12)
    parser.add_argument("--dynamic-origin-offset-x", type=float, default=0.0)
    parser.add_argument("--dynamic-origin-offset-y", type=float, default=-0.15)
    parser.add_argument("--dynamic-origin-offset-z", type=float, default=0.11)
    parser.add_argument(
        "--keyframe-history", dest="keyframe_history", action="store_true",
        help="记录有界磁盘关键帧历史并在保存时离线去动态（默认开启）",
    )
    parser.add_argument(
        "--no-keyframe-history", dest="keyframe_history", action="store_false",
        help="禁用关键帧历史和保存时离线去动态",
    )
    parser.set_defaults(keyframe_history=True)
    parser.add_argument("--keyframe-translation", type=float, default=0.10)
    parser.add_argument("--keyframe-rotation-deg", type=float, default=5.0)
    parser.add_argument("--keyframe-min-interval", type=float, default=0.20)
    parser.add_argument("--keyframe-max-interval", type=float, default=0.75)
    parser.add_argument("--keyframe-max-rays", type=int, default=4_000)
    parser.add_argument("--keyframe-chunk-frames", type=int, default=64)
    parser.add_argument("--keyframe-queue-size", type=int, default=4)
    parser.add_argument("--keyframe-max-disk-gb", type=float, default=2.0)
    parser.add_argument("--keyframe-free-threshold", type=int, default=5)
    parser.add_argument("--keyframe-free-hit-ratio", type=float, default=2.0)
    parser.add_argument("--keyframe-max-removal-fraction", type=float, default=0.35)
    parser.add_argument(
        "--polar-disappearance-filter", dest="polar_disappearance_filter", action="store_true",
        help="在保存时开启 ERASOR 风格的极坐标格消失投票（默认开启）",
    )
    parser.add_argument(
        "--no-polar-disappearance-filter", dest="polar_disappearance_filter", action="store_false",
        help="关闭保存时的极坐标格消失投票，只保留关键帧射线清理",
    )
    parser.set_defaults(polar_disappearance_filter=True)
    parser.add_argument("--polar-min-range", type=float, default=0.4)
    parser.add_argument("--polar-max-range", type=float, default=10.0)
    parser.add_argument("--polar-rings", type=int, default=10)
    parser.add_argument("--polar-sectors", type=int, default=72)
    parser.add_argument("--polar-cell-size", type=float, default=0.2)
    parser.add_argument("--polar-min-scan-points", type=int, default=2)
    parser.add_argument("--polar-scan-ratio-threshold", type=float, default=0.25)
    parser.add_argument("--polar-structure-span", type=float, default=0.5)
    parser.add_argument("--polar-height-threshold", type=float, default=0.4)
    parser.add_argument("--polar-confirm-tolerance", type=float, default=0.3)
    parser.add_argument("--polar-min-votes", type=int, default=2)
    parser.add_argument("--polar-max-removal-fraction", type=float, default=0.20)
    parser.add_argument(
        "--keep-keyframe-history", action="store_true",
        help="成功导出后保留 data/session_history 中的关键帧历史",
    )
    parser.add_argument("--map-resolution", type=float, default=0.05)
    parser.add_argument("--z-min", type=float, default=0.05)
    parser.add_argument("--z-max", type=float, default=1.50)
    parser.add_argument(
        "--height-mode", choices=("ground", "absolute"), default="ground",
        help="二维投影高度基准：ground=自动拟合地面，absolute=全局 Z",
    )
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
    if not 1 <= settings.dynamic_miss_threshold <= 255:
        parser.error("dynamic-miss-threshold must be between 1 and 255")
    if settings.dynamic_min_range < 0 or settings.dynamic_max_range <= settings.dynamic_min_range:
        parser.error("dynamic range must satisfy 0 <= min < max")
    if settings.dynamic_removal_rate <= 0 or settings.dynamic_max_rays < 1:
        parser.error("dynamic removal rate and max rays must be greater than zero")
    if settings.dynamic_angular_resolution <= 0 or settings.dynamic_odom_tolerance < 0:
        parser.error("dynamic angular resolution must be positive and odom tolerance non-negative")
    if (
        not math.isfinite(settings.keyframe_translation)
        or not math.isfinite(settings.keyframe_rotation_deg)
        or settings.keyframe_translation < 0
        or settings.keyframe_rotation_deg < 0
    ):
        parser.error("keyframe translation and rotation thresholds must be non-negative")
    if (
        not math.isfinite(settings.keyframe_min_interval)
        or not math.isfinite(settings.keyframe_max_interval)
        or settings.keyframe_min_interval < 0
        or settings.keyframe_max_interval <= 0
        or settings.keyframe_max_interval < settings.keyframe_min_interval
    ):
        parser.error("keyframe intervals must satisfy 0 <= min <= max")
    if settings.keyframe_max_rays < 1 or settings.keyframe_chunk_frames < 1 or settings.keyframe_queue_size < 1:
        parser.error("keyframe ray, chunk, and queue limits must be positive")
    if not math.isfinite(settings.keyframe_max_disk_gb) or settings.keyframe_max_disk_gb <= 0:
        parser.error("keyframe max disk size must be finite and positive")
    if not 1 <= settings.keyframe_free_threshold <= 65535:
        parser.error("keyframe free threshold must be between 1 and 65535")
    if not math.isfinite(settings.keyframe_free_hit_ratio) or settings.keyframe_free_hit_ratio < 0:
        parser.error("keyframe free/hit ratio must be finite and non-negative")
    if not math.isfinite(settings.keyframe_max_removal_fraction) or not 0 <= settings.keyframe_max_removal_fraction <= 1:
        parser.error("keyframe max removal fraction must be between zero and one")
    try:
        polar_disappearance_config(settings).validate()
    except ValueError as error:
        parser.error(str(error))
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
    urls = console_urls(settings.host, settings.port)
    node.get_logger().info(f"本机打开 {urls[0]}")
    for url in urls[1:]:
        node.get_logger().info(f"同一网络的其他电脑请打开 {url}")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close_dynamic_worker()
        node.close_history_recorder()
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
