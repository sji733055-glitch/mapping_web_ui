"""Point-cloud accumulation and map export primitives.

This module intentionally has no ROS dependency so the data path can be tested
without a running ROS graph.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import os
import re
import threading
from typing import Callable, Optional

import numpy as np


MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9._-]{1,48}$")
_PACK_BITS = 21
_PACK_MASK = (1 << _PACK_BITS) - 1
_PACK_LIMIT = 1 << (_PACK_BITS - 1)


def sanitize_map_name(value: str) -> str:
    """Validate a map name before it becomes part of an output path."""
    name = value.strip()
    if not MAP_NAME_PATTERN.fullmatch(name) or name in {".", ".."}:
        raise ValueError("地图名称只能包含 1–48 个字母、数字、点、短横线或下划线")
    return name


@dataclass(frozen=True)
class MapExportConfig:
    resolution: float = 0.05
    z_min: float = 0.05
    z_max: float = 1.50
    radius: float = 0.50
    min_neighbors: int = 10
    padding: float = 0.25

    def validate(self) -> None:
        if not 0.005 <= self.resolution <= 1.0:
            raise ValueError("地图分辨率必须在 0.005–1.0 m 之间")
        if self.z_min >= self.z_max:
            raise ValueError("z_min 必须小于 z_max")
        if self.radius < 0 or self.min_neighbors < 0 or self.padding < 0:
            raise ValueError("滤波半径、邻点数和地图留白不能为负数")


class VoxelAccumulator:
    """Incrementally retain one representative point per fixed 3-D voxel."""

    def __init__(self, voxel_size: float = 0.05, max_points: int = 5_000_000) -> None:
        if not 0.005 <= voxel_size <= 1.0:
            raise ValueError("voxel_size 必须在 0.005–1.0 m 之间")
        self.voxel_size = float(voxel_size)
        self.max_points = int(max_points)
        self._lock = threading.RLock()
        self.clear()

    def clear(self) -> None:
        with getattr(self, "_lock", threading.RLock()):
            self._keys: set[int] = set()
            self._chunks: list[np.ndarray] = []
            self._point_count = 0
            self._minimum = np.array([math.inf, math.inf, math.inf], dtype=np.float64)
            self._maximum = np.array([-math.inf, -math.inf, -math.inf], dtype=np.float64)
            self._version = 0
            self._cached_version = -1
            self._cached_points = np.empty((0, 3), dtype=np.float32)

    @property
    def point_count(self) -> int:
        with self._lock:
            return self._point_count

    @property
    def bounds(self) -> dict[str, float]:
        with self._lock:
            if self._point_count == 0:
                return {}
            return {
                "min_x": float(self._minimum[0]), "min_y": float(self._minimum[1]),
                "min_z": float(self._minimum[2]), "max_x": float(self._maximum[0]),
                "max_y": float(self._maximum[1]), "max_z": float(self._maximum[2]),
            }

    def add(self, points: np.ndarray) -> int:
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("points 必须是 N×3 数组")
        xyz = np.ascontiguousarray(points[:, :3])
        if not len(xyz):
            return 0
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if not len(xyz):
            return 0

        voxel = np.floor(xyz / self.voxel_size).astype(np.int64)
        if np.any(voxel < -_PACK_LIMIT) or np.any(voxel >= _PACK_LIMIT):
            raise ValueError("点云坐标超出体素编码范围，请检查坐标系或增大 voxel_size")
        unsigned = np.bitwise_and(voxel, _PACK_MASK).astype(np.uint64)
        packed = (unsigned[:, 0] << np.uint64(42)) | (unsigned[:, 1] << np.uint64(21)) | unsigned[:, 2]
        unique_keys, unique_indices = np.unique(packed, return_index=True)

        with self._lock:
            remaining = self.max_points - self._point_count
            if remaining <= 0:
                return 0
            new_pairs = [
                (int(key), int(index))
                for key, index in zip(unique_keys, unique_indices)
                if int(key) not in self._keys
            ][:remaining]
            if not new_pairs:
                return 0
            keys, indices = zip(*new_pairs)
            kept = np.ascontiguousarray(xyz[np.fromiter(indices, dtype=np.int64)], dtype=np.float32)
            self._keys.update(keys)
            self._chunks.append(kept)
            self._point_count += len(kept)
            self._minimum = np.minimum(self._minimum, kept.min(axis=0))
            self._maximum = np.maximum(self._maximum, kept.max(axis=0))
            self._version += 1
            return len(kept)

    def snapshot(self, max_points: Optional[int] = None) -> np.ndarray:
        with self._lock:
            if self._cached_version != self._version:
                self._cached_points = (
                    np.concatenate(self._chunks, axis=0)
                    if self._chunks else np.empty((0, 3), dtype=np.float32)
                )
                self._cached_version = self._version
            points = self._cached_points
            if max_points and len(points) > max_points:
                step = int(math.ceil(len(points) / max_points))
                return np.ascontiguousarray(points[::step][:max_points])
            return points.copy()


def write_binary_pcd(path: Path, points: np.ndarray) -> None:
    """Write PointXYZ as a standard binary PCD using an atomic replacement."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.ascontiguousarray(points[:, :3], dtype="<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        f"COUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(xyz)}\nDATA binary\n"
    ).encode("ascii")
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        with temporary.open("wb") as stream:
            stream.write(header)
            stream.write(xyz.tobytes(order="C"))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _radius_filter(points: np.ndarray, radius: float, minimum: int) -> np.ndarray:
    if radius <= 0 or minimum <= 1 or len(points) < minimum:
        return points
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        return points
    tree = cKDTree(points)
    keep = np.zeros(len(points), dtype=bool)
    chunk = 100_000
    for start in range(0, len(points), chunk):
        stop = min(len(points), start + chunk)
        counts = tree.query_ball_point(points[start:stop], radius, return_length=True, workers=-1)
        keep[start:stop] = counts >= minimum
    return points[keep]


def write_occupancy_map(
    pgm_path: Path,
    yaml_path: Path,
    points: np.ndarray,
    config: MapExportConfig,
) -> dict[str, float | int]:
    """Slice PointXYZ by height and produce a Nav2-compatible PGM/YAML pair."""
    config.validate()
    points = np.asarray(points, dtype=np.float32)
    sliced = points[(points[:, 2] >= config.z_min) & (points[:, 2] <= config.z_max)]
    if len(sliced) < 3:
        raise ValueError(
            f"Z={config.z_min:.2f}–{config.z_max:.2f} m 切片后只有 {len(sliced)} 个点，无法生成二维地图"
        )
    filtered = _radius_filter(sliced, config.radius, config.min_neighbors)
    if len(filtered) < 3:
        raise ValueError("离群点滤波后没有足够点，请减小 min_neighbors 或扩大 radius")

    minimum = filtered[:, :2].min(axis=0) - config.padding
    maximum = filtered[:, :2].max(axis=0) + config.padding
    width, height = np.ceil((maximum - minimum) / config.resolution).astype(np.int64) + 1
    if width * height > 250_000_000:
        raise ValueError(f"二维地图尺寸异常（{width}×{height}），请检查点云坐标")

    pixels = np.full((int(height), int(width)), 254, dtype=np.uint8)
    cells = np.floor((filtered[:, :2] - minimum) / config.resolution).astype(np.int64)
    cells[:, 0] = np.clip(cells[:, 0], 0, width - 1)
    cells[:, 1] = np.clip(cells[:, 1], 0, height - 1)
    pixels[cells[:, 1], cells[:, 0]] = 0
    pixels = np.flipud(pixels)

    pgm_path, yaml_path = Path(pgm_path), Path(yaml_path)
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    pgm_tmp = pgm_path.with_name(f".{pgm_path.name}.tmp")
    yaml_tmp = yaml_path.with_name(f".{yaml_path.name}.tmp")
    try:
        with pgm_tmp.open("wb") as stream:
            stream.write(f"P5\n# generated by mapping_web_ui\n{width} {height}\n255\n".encode("ascii"))
            stream.write(pixels.tobytes(order="C"))
            stream.flush()
            os.fsync(stream.fileno())
        yaml_text = (
            f"image: {pgm_path.name}\n"
            "mode: trinary\n"
            f"resolution: {config.resolution:.8g}\n"
            f"origin: [{minimum[0]:.8g}, {minimum[1]:.8g}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
        with yaml_tmp.open("w", encoding="utf-8") as stream:
            stream.write(yaml_text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(pgm_tmp, pgm_path)
        os.replace(yaml_tmp, yaml_path)
    finally:
        pgm_tmp.unlink(missing_ok=True)
        yaml_tmp.unlink(missing_ok=True)

    return {
        "width": int(width), "height": int(height), "resolution": config.resolution,
        "origin_x": float(minimum[0]), "origin_y": float(minimum[1]),
        "slice_points": int(len(sliced)), "filtered_points": int(len(filtered)),
    }


def export_session(
    output_root: Path,
    map_name: str,
    points: np.ndarray,
    config: MapExportConfig,
    progress: Optional[Callable[[float, str], None]] = None,
) -> dict[str, object]:
    name = sanitize_map_name(map_name)
    output_root = Path(output_root).resolve()
    if len(points) < 3:
        raise ValueError("累计点数不足，至少需要 3 个点才能保存")
    pcd = output_root / "pcd" / f"{name}.pcd"
    pgm = output_root / "map" / f"{name}.pgm"
    yaml = output_root / "map" / f"{name}.yaml"
    if progress:
        progress(0.15, "正在写入三维 PCD")
    write_binary_pcd(pcd, points)
    if progress:
        progress(0.52, "正在切片并过滤二维地图")
    metadata = write_occupancy_map(pgm, yaml, points, config)
    if progress:
        progress(1.0, "地图文件保存完成")
    return {
        "pcd": str(pcd), "pgm": str(pgm), "yaml": str(yaml),
        "point_count": int(len(points)), "map": metadata,
    }
