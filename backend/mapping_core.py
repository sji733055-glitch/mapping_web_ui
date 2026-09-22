"""Point-cloud accumulation and map export primitives.

This module intentionally has no ROS dependency so the data path can be tested
without a running ROS graph.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
import json
import math
import os
import re
import struct
import tempfile
import threading
from typing import Callable, Optional

import numpy as np


MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9._-]{1,48}$")
# Hard limit for one occupancy grid; larger means the input coordinates are wrong.
MAX_OCCUPANCY_CELLS = 250_000_000
# A browser preview is pooled down to roughly this many pixels before transport.
PREVIEW_MAX_PIXELS = 4_000_000
_PACK_BITS = 21
_PACK_MASK = (1 << _PACK_BITS) - 1
_PACK_LIMIT = 1 << (_PACK_BITS - 1)


def _pack_voxel_coordinates(voxel: np.ndarray) -> np.ndarray:
    """Pack signed N×3 voxel coordinates into stable uint64 keys."""
    if np.any(voxel < -_PACK_LIMIT) or np.any(voxel >= _PACK_LIMIT):
        raise ValueError("点云坐标超出体素编码范围，请检查坐标系或增大 voxel_size")
    unsigned = np.bitwise_and(voxel, _PACK_MASK).astype(np.uint64)
    return (
        (unsigned[:, 0] << np.uint64(42))
        | (unsigned[:, 1] << np.uint64(21))
        | unsigned[:, 2]
    )


def voxel_coordinates(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Map XYZ points onto the accumulator's voxel grid.

    Every voxel decision goes through this one function on purpose.  The
    accumulator used to divide ``float32`` points while ``voxel_keys`` divided
    ``float64`` ones, so a point whose quotient landed on a cell boundary could
    fall into voxel *n* for one caller and *n+1* for the other.  On a real
    1.5 M-point map that produced duplicate keys in the offline dynamic filter,
    which then aborted the whole cleanup with
    ``离线去动态要求最终点云每个体素最多一点`` rather than removing anything.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("points 必须是 N×3 数组")
    xyz = xyz[:, :3]
    if not np.isfinite(xyz).all():
        raise ValueError("体素编码不接受 NaN 或无穷大坐标")
    return np.floor(xyz / float(voxel_size)).astype(np.int64)


def voxel_keys(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Return one packed key for every finite XYZ point."""
    return _pack_voxel_coordinates(voxel_coordinates(points, voxel_size))


def select_visibility_endpoints(
    origin: np.ndarray,
    endpoints: np.ndarray,
    min_range: float = 0.38,
    max_range: float = 10.0,
    max_rays: int = 4_000,
    angular_resolution: float = math.radians(0.5),
    phase: int = 0,
) -> np.ndarray:
    """Select the nearest finite return in each angular bin.

    ``max_rays`` bounds the per-frame cost of the free-space pass and of the
    recorded keyframe history.  When a scan has more angular bins than the
    budget, the bins are sampled with an evenly spaced comb that is rotated by
    ``phase``; callers pass a monotonically increasing frame counter.  A fixed
    comb (the original behaviour) meant a direction outside the comb could never
    produce free-space evidence while the sensor stood still, so a person's
    ghost in that direction survived no matter how long the session ran.  Over
    ``ceil(bins / max_rays)`` successive phases the rotating comb covers every
    angular bin.
    """
    if min_range < 0 or max_range <= min_range:
        raise ValueError("去动态射线距离范围无效")
    if max_rays < 1 or angular_resolution <= 0:
        raise ValueError("去动态射线数和角分辨率必须为正数")
    if not isinstance(phase, (int, np.integer)) or int(phase) < 0:
        raise ValueError("去动态射线相位必须是非负整数")

    ray_origin = np.asarray(origin, dtype=np.float64).reshape(-1)
    if ray_origin.size != 3 or not np.isfinite(ray_origin).all():
        raise ValueError("射线原点必须是有限的 XYZ 坐标")
    xyz = np.asarray(endpoints, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("endpoints 必须是 N×3 数组")
    xyz = xyz[:, :3]
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):
        return np.empty((0, 3), dtype=np.float32)

    relative = xyz - ray_origin
    ranges = np.linalg.norm(relative, axis=1)
    valid = (ranges >= min_range) & (ranges <= max_range)
    relative = relative[valid]
    ranges = ranges[valid]
    if not len(relative):
        return np.empty((0, 3), dtype=np.float32)

    azimuth = np.arctan2(relative[:, 1], relative[:, 0])
    elevation = np.arctan2(relative[:, 2], np.hypot(relative[:, 0], relative[:, 1]))
    azimuth_bins = np.floor((azimuth + math.pi) / angular_resolution).astype(np.int64)
    elevation_bins = np.floor((elevation + math.pi / 2.0) / angular_resolution).astype(np.int64)
    angular_bins = (azimuth_bins << np.int64(32)) | np.bitwise_and(
        elevation_bins, np.int64(0xFFFFFFFF)
    )

    # lexsort makes the first item in each angular bin the nearest return.
    order = np.lexsort((ranges, angular_bins))
    sorted_bins = angular_bins[order]
    first = np.empty(len(order), dtype=bool)
    first[0] = True
    first[1:] = sorted_bins[1:] != sorted_bins[:-1]
    selected = order[first]
    if len(selected) > max_rays:
        # An evenly spaced comb over the whole field of view, rotated per frame.
        step = len(selected) / max_rays
        take = (np.arange(max_rays, dtype=np.float64) * step).astype(np.int64)
        shift = int(phase) % len(selected)
        if shift:
            take = (take + shift) % len(selected)
        selected = selected[np.unique(take)]
    return np.ascontiguousarray(xyz[valid][selected], dtype=np.float32)


def _dda_free_voxel_keys(
    origin: np.ndarray,
    endpoints: np.ndarray,
    voxel_size: float,
) -> np.ndarray:
    """Traverse already-selected rays and return unique non-endpoint voxels."""
    if not 0.005 <= float(voxel_size) <= 1.0:
        raise ValueError("voxel_size 必须在 0.005–1.0 m 之间")
    ray_origin = np.asarray(origin, dtype=np.float64).reshape(-1)
    if ray_origin.size != 3 or not np.isfinite(ray_origin).all():
        raise ValueError("射线原点必须是有限的 XYZ 坐标")
    endpoints_selected = np.asarray(endpoints, dtype=np.float64)
    if endpoints_selected.ndim != 2 or endpoints_selected.shape[1] < 3:
        raise ValueError("endpoints 必须是 N×3 数组")
    endpoints_selected = endpoints_selected[:, :3]
    endpoints_selected = endpoints_selected[np.isfinite(endpoints_selected).all(axis=1)]
    if not len(endpoints_selected):
        return np.empty(0, dtype=np.uint64)
    selected_ranges = np.linalg.norm(endpoints_selected - ray_origin, axis=1)
    usable = selected_ranges >= float(voxel_size) * 0.5
    endpoints_selected = endpoints_selected[usable]
    selected_ranges = selected_ranges[usable]
    if not len(endpoints_selected):
        return np.empty(0, dtype=np.uint64)

    # Vectorized Amanatides--Woo traversal.  Every loop advances each live ray
    # by exactly one voxel boundary.  This avoids the half-voxel point sampling
    # previously used here, which could skip corner-touching voxels and created
    # considerably more temporary floating-point data.  Tie-breaking matches
    # the reference implementation (Z, then Y, then X for equal tMax values).
    resolution = float(voxel_size)
    directions = (endpoints_selected - ray_origin) / selected_ranges[:, None]
    voxels = np.floor(ray_origin / resolution).astype(np.int64)
    voxels = np.broadcast_to(voxels, (len(endpoints_selected), 3)).copy()
    end_voxels = np.floor(endpoints_selected / resolution).astype(np.int64)
    steps = np.sign(directions).astype(np.int64)

    infinity = np.full_like(directions, np.inf, dtype=np.float64)
    positive_boundaries = (voxels + 1).astype(np.float64) * resolution
    negative_boundaries = voxels.astype(np.float64) * resolution
    next_boundaries = np.where(steps > 0, positive_boundaries, negative_boundaries)
    nonzero = steps != 0
    t_max = infinity.copy()
    np.divide(
        next_boundaries - ray_origin[None, :],
        directions,
        out=t_max,
        where=nonzero,
    )
    t_delta = infinity.copy()
    np.divide(resolution, np.abs(directions), out=t_delta, where=nonzero)

    active = np.any(voxels != end_voxels, axis=1)
    key_batches: list[np.ndarray] = []
    while bool(active.any()):
        live = np.flatnonzero(active)
        live_t_max = t_max[live]
        x_before_y = live_t_max[:, 0] < live_t_max[:, 1]
        axes = np.where(
            x_before_y,
            np.where(live_t_max[:, 0] < live_t_max[:, 2], 0, 2),
            np.where(live_t_max[:, 1] < live_t_max[:, 2], 1, 2),
        )
        t_next = live_t_max[np.arange(len(live)), axes]
        voxels[live, axes] += steps[live, axes]
        t_max[live, axes] += t_delta[live, axes]

        # As in the HW filter, do not count the return's endpoint voxel and do
        # not advance farther than one voxel resolution before the return.
        keep = (
            (t_next <= selected_ranges[live] - resolution)
            & np.any(voxels[live] != end_voxels[live], axis=1)
        )
        if bool(keep.any()):
            key_batches.append(_pack_voxel_coordinates(voxels[live[keep]]))
        active[live] = keep
    if not key_batches:
        return np.empty(0, dtype=np.uint64)
    return np.unique(np.concatenate(key_batches))


def visibility_miss_keys(
    origin: np.ndarray,
    endpoints: np.ndarray,
    voxel_size: float,
    min_range: float = 0.38,
    max_range: float = 10.0,
    max_rays: int = 4_000,
    angular_resolution: float = math.radians(0.5),
    phase: int = 0,
) -> np.ndarray:
    """Return voxels observed as free by exact 3-D DDA LiDAR traversal.

    This is the online counterpart of HW ``offline_mapping_optimizer``'s
    Amanatides--Woo raycasting filter.  The nearest endpoint in every angular
    bin is used so a farther return can never carve through a nearer surface.
    The endpoint voxel is excluded; endpoint hits are handled separately by
    :class:`VoxelAccumulator`.  The result is unique, allowing at most one miss
    per voxel and source frame even when several rays cross the same voxel.

    ``max_rays`` and ``phase`` bound the per-frame cost while still sweeping
    every angular bin over successive frames; see
    :func:`select_visibility_endpoints`.
    """
    selected = select_visibility_endpoints(
        origin,
        endpoints,
        min_range=min_range,
        max_range=max_range,
        max_rays=max_rays,
        angular_resolution=angular_resolution,
        phase=phase,
    )
    return _dda_free_voxel_keys(origin, selected, voxel_size)


_HISTORY_MAGIC = b"MKH1"
_HISTORY_VERSION = 1
_HISTORY_CHUNK_HEADER = struct.Struct("<4sII")
_HISTORY_RECORD_HEADER = struct.Struct("<Q3dI")


def write_keyframe_history_chunk(
    path: Path,
    records: list[tuple[int, np.ndarray, np.ndarray]],
) -> int:
    """Atomically write a bounded chunk of ray keyframes."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    written = 0
    try:
        with temporary.open("wb") as stream:
            stream.write(_HISTORY_CHUNK_HEADER.pack(
                _HISTORY_MAGIC, _HISTORY_VERSION, len(records)
            ))
            for timestamp_ns, origin, endpoints in records:
                ray_origin = np.asarray(origin, dtype=np.float64).reshape(-1)
                points = np.ascontiguousarray(endpoints, dtype="<f4")
                if ray_origin.size != 3 or not np.isfinite(ray_origin).all():
                    raise ValueError("关键帧射线原点无效")
                if points.ndim != 2 or points.shape[1] < 3:
                    raise ValueError("关键帧点云必须是 N×3 数组")
                points = np.ascontiguousarray(points[:, :3], dtype="<f4")
                if not np.isfinite(points).all():
                    raise ValueError("关键帧点云包含非有限坐标")
                stream.write(_HISTORY_RECORD_HEADER.pack(
                    max(0, int(timestamp_ns)),
                    float(ray_origin[0]), float(ray_origin[1]), float(ray_origin[2]),
                    len(points),
                ))
                stream.write(points.tobytes(order="C"))
            stream.flush()
            os.fsync(stream.fileno())
            written = stream.tell()
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return written


def iter_keyframe_history(
    history_dir: Path,
):
    """Yield ``(timestamp_ns, origin, endpoints)`` from finalized chunks."""
    root = Path(history_dir)
    for path in sorted(root.glob("chunk_*.bin")):
        with path.open("rb") as stream:
            header = stream.read(_HISTORY_CHUNK_HEADER.size)
            if len(header) != _HISTORY_CHUNK_HEADER.size:
                raise ValueError(f"关键帧历史头不完整：{path.name}")
            magic, version, record_count = _HISTORY_CHUNK_HEADER.unpack(header)
            if magic != _HISTORY_MAGIC or version != _HISTORY_VERSION:
                raise ValueError(f"关键帧历史版本不支持：{path.name}")
            for _ in range(record_count):
                record = stream.read(_HISTORY_RECORD_HEADER.size)
                if len(record) != _HISTORY_RECORD_HEADER.size:
                    raise ValueError(f"关键帧记录头截断：{path.name}")
                timestamp_ns, x, y, z, point_count = _HISTORY_RECORD_HEADER.unpack(record)
                byte_count = int(point_count) * 12
                payload = stream.read(byte_count)
                if len(payload) != byte_count:
                    raise ValueError(f"关键帧点云截断：{path.name}")
                endpoints = np.frombuffer(payload, dtype="<f4").reshape(-1, 3).copy()
                yield timestamp_ns, np.array([x, y, z], dtype=np.float64), endpoints
            if stream.read(1):
                raise ValueError(f"关键帧 chunk 尾部有未识别数据：{path.name}")


def write_keyframe_history_metadata(history_dir: Path, metadata: dict[str, object]) -> None:
    """Atomically update human-readable session-history metadata."""
    target = Path(history_dir) / "metadata.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.tmp")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(metadata, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)


def _increment_existing_voxels(
    sorted_map_keys: np.ndarray,
    candidate_keys: np.ndarray,
    counts: np.ndarray,
) -> None:
    if not len(candidate_keys) or not len(sorted_map_keys):
        return
    candidates = np.unique(np.asarray(candidate_keys, dtype=np.uint64))
    positions = np.searchsorted(sorted_map_keys, candidates)
    within = positions < len(sorted_map_keys)
    positions = positions[within]
    candidates = candidates[within]
    matching = positions[sorted_map_keys[positions] == candidates]
    if not len(matching):
        return
    values = counts[matching].astype(np.uint32) + 1
    counts[matching] = np.minimum(values, np.iinfo(counts.dtype).max).astype(counts.dtype)


def filter_dynamic_with_keyframe_history(
    points: np.ndarray,
    history_dir: Path,
    voxel_size: float,
    free_frame_threshold: int = 5,
    free_hit_ratio: float = 2.0,
    max_removal_fraction: float = 0.35,
    progress: Optional[Callable[[float, str], None]] = None,
) -> tuple[np.ndarray, dict[str, object]]:
    """Filter a final cloud using all disk-backed ray keyframes.

    Evidence is counted once per voxel and keyframe.  A voxel is removed only
    when enough frames saw it as free *and* free evidence is at least
    ``free_hit_ratio`` times its endpoint-hit evidence.  This preserves the HW
    raycasting principle while being safer for unoptimized online odometry.
    """
    if not 1 <= int(free_frame_threshold) <= 65535:
        raise ValueError("离线去动态空闲帧阈值必须在 1–65535 之间")
    if not math.isfinite(free_hit_ratio) or free_hit_ratio < 0:
        raise ValueError("离线去动态 free/hit 比例无效")
    if not 0.0 <= max_removal_fraction <= 1.0:
        raise ValueError("离线去动态最大删除比例必须在 0–1 之间")
    xyz = np.ascontiguousarray(points[:, :3], dtype=np.float32)
    if not len(xyz):
        return xyz, {"applied": False, "reason": "点云为空", "frames": 0, "removed_points": 0}

    map_keys = voxel_keys(xyz, voxel_size)
    order = np.argsort(map_keys)
    sorted_keys = map_keys[order]
    if len(sorted_keys) > 1 and bool(np.any(sorted_keys[1:] == sorted_keys[:-1])):
        raise ValueError("离线去动态要求最终点云每个体素最多一点")
    free_counts = np.zeros(len(xyz), dtype=np.uint16)
    hit_counts = np.zeros(len(xyz), dtype=np.uint16)

    metadata_path = Path(history_dir) / "metadata.json"
    expected_frames = 0
    if metadata_path.is_file():
        try:
            expected_frames = int(json.loads(metadata_path.read_text(encoding="utf-8")).get("written_frames", 0))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            expected_frames = 0

    processed = 0
    for _timestamp_ns, origin, endpoints in iter_keyframe_history(history_dir):
        hit_keys = voxel_keys(endpoints, voxel_size) if len(endpoints) else np.empty(0, dtype=np.uint64)
        free_keys = _dda_free_voxel_keys(origin, endpoints, voxel_size)
        _increment_existing_voxels(sorted_keys, hit_keys, hit_counts)
        _increment_existing_voxels(sorted_keys, free_keys, free_counts)
        processed += 1
        if progress and (processed == 1 or processed % 4 == 0):
            denominator = max(expected_frames, processed)
            progress(min(0.99, processed / denominator), f"正在回放关键帧 {processed}/{denominator}")

    if processed < free_frame_threshold:
        return xyz, {
            "applied": False,
            "reason": f"关键帧仅 {processed} 帧，未达空闲帧阈值 {free_frame_threshold}",
            "frames": processed,
            "removed_points": 0,
        }

    remove_sorted = (
        (free_counts >= int(free_frame_threshold))
        & (free_counts.astype(np.float64) >= hit_counts.astype(np.float64) * free_hit_ratio)
    )
    removed = int(np.count_nonzero(remove_sorted))
    fraction = removed / len(xyz)
    if fraction > max_removal_fraction:
        return xyz, {
            "applied": False,
            "reason": f"候选删除 {fraction:.1%} 超过安全上限 {max_removal_fraction:.1%}",
            "frames": processed,
            "removed_points": 0,
            "candidate_removed_points": removed,
            "candidate_removal_fraction": fraction,
        }

    keep = np.ones(len(xyz), dtype=bool)
    keep[order[remove_sorted]] = False
    filtered = np.ascontiguousarray(xyz[keep], dtype=np.float32)
    if progress:
        progress(1.0, f"关键帧离线清理完成，删除 {removed:,} 点")
    return filtered, {
        "applied": True,
        "frames": processed,
        "removed_points": removed,
        "removal_fraction": fraction,
        "free_frame_threshold": int(free_frame_threshold),
        "free_hit_ratio": float(free_hit_ratio),
        "max_free_frames": int(free_counts.max(initial=0)),
        "max_hit_frames": int(hit_counts.max(initial=0)),
    }


def sanitize_map_name(value: str) -> str:
    """Validate a map name before it becomes part of an output path."""
    name = value.strip()
    if not MAP_NAME_PATTERN.fullmatch(name) or name in {".", ".."}:
        raise ValueError("地图名称只能包含 1–48 个字母、数字、点、短横线或下划线")
    return name


def write_mapping_report(
    output_root: Path,
    map_name: str,
    report: dict[str, object],
) -> Path:
    """Atomically write the audit record beside one saved PCD.

    The report deliberately contains only JSON-safe measurement and filtering
    metadata.  It never changes the PCD payload and gives an operator enough
    evidence to understand why a point cloud was, or was not, cleaned.
    """
    name = sanitize_map_name(map_name)
    root = Path(output_root).resolve()
    target = root / "pcd" / f"{name}.mapping-report.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.tmp")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
    return target


@dataclass(frozen=True)
class BinDisappearanceConfig:
    """Parameters of the ERASOR-style polar-bin disappearance vote.

    The idea is borrowed from ERASOR's region-wise pseudo occupancy descriptor
    (R-POD): a scan and the map are both binned into a polar grid around the
    sensor, each bin is described by the height span of its returns, and a bin
    whose map structure is much taller than what the scan still observes is a
    disappeared object.  ERASOR2 adds an instance/log-odds style accumulation on
    top; the same effect is kept here by counting confirming and contradicting
    keyframes per coarse map cell and requiring a majority, so an occluded static
    structure is never deleted by the frames that could not see it.
    """

    max_range: float = 10.0
    min_range: float = 0.4
    rings: int = 10
    sectors: int = 72
    cell_size: float = 0.2
    min_scan_points: int = 2
    scan_ratio_threshold: float = 0.25
    structure_span: float = 0.5
    height_threshold: float = 0.4
    confirm_tolerance: float = 0.3
    min_votes: int = 2
    max_removal_fraction: float = 0.35

    def validate(self) -> None:
        values = (
            self.max_range, self.min_range, self.cell_size, self.scan_ratio_threshold,
            self.structure_span, self.height_threshold, self.confirm_tolerance,
            self.max_removal_fraction,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("bin 去动态参数必须是有限数值")
        if self.min_range < 0 or self.max_range <= self.min_range:
            raise ValueError("bin 去动态距离范围无效")
        if not 0.005 <= self.cell_size <= 2.0:
            raise ValueError("bin 去动态投票网格必须在 0.005–2.0 m 之间")
        if self.rings < 1 or self.sectors < 4:
            raise ValueError("bin 去动态环数与扇区数必须为正")
        if not 0.0 <= self.scan_ratio_threshold <= 1.0:
            raise ValueError("bin 去动态高度比阈值必须在 0–1 之间")
        if self.structure_span < 0 or self.height_threshold < 0 or self.confirm_tolerance < 0:
            raise ValueError("bin 去动态高度阈值不能为负数")
        if self.min_scan_points < 1:
            raise ValueError("bin 去动态每格最少扫描点数必须为正")
        if self.min_votes < 1:
            raise ValueError("bin 去动态最少票数必须为正")
        if not 0.0 <= self.max_removal_fraction <= 1.0:
            raise ValueError("bin 去动态最大删除比例必须在 0–1 之间")


def _polar_bin_indices(
    dx: np.ndarray,
    dy: np.ndarray,
    config: BinDisappearanceConfig,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(in_range, bin_index)`` for offsets from one sensor pose."""
    radius = np.hypot(dx, dy)
    inside = (radius >= config.min_range) & (radius <= config.max_range)
    ring_size = (config.max_range - config.min_range) / config.rings
    sector_size = 2.0 * math.pi / config.sectors
    ring = np.clip(
        ((radius - config.min_range) / ring_size).astype(np.int64), 0, config.rings - 1
    )
    sector = np.clip(
        ((np.arctan2(dy, dx) + math.pi) / sector_size).astype(np.int64), 0, config.sectors - 1
    )
    return inside, ring * config.sectors + sector


def _bin_height_stats(
    bins: np.ndarray,
    heights: np.ndarray,
    total_bins: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per-bin return count and height range, computed with one sort."""
    counts = np.zeros(total_bins, dtype=np.int64)
    minimum = np.full(total_bins, np.inf, dtype=np.float64)
    maximum = np.full(total_bins, -np.inf, dtype=np.float64)
    if not len(bins):
        return counts, minimum, maximum
    order = np.argsort(bins, kind="stable")
    sorted_bins = bins[order]
    sorted_heights = heights[order]
    starts = np.flatnonzero(np.concatenate(([True], sorted_bins[1:] != sorted_bins[:-1])))
    occupied = sorted_bins[starts]
    counts[occupied] = np.diff(np.append(starts, len(sorted_bins)))
    minimum[occupied] = np.minimum.reduceat(sorted_heights, starts)
    maximum[occupied] = np.maximum.reduceat(sorted_heights, starts)
    return counts, minimum, maximum


def filter_disappeared_with_bin_votes(
    points: np.ndarray,
    history_dir: Path,
    config: Optional[BinDisappearanceConfig] = None,
    progress: Optional[Callable[[float, str], None]] = None,
) -> tuple[np.ndarray, dict[str, object]]:
    """Remove structure the recorded keyframes later saw as empty space.

    This is the coarse, region-wise companion of
    :func:`filter_dynamic_with_keyframe_history`.  The ray filter can only carve
    voxels a ray actually traverses; this pass works on whole polar bins, so a
    person's shell is removed even where no recorded ray crossed it exactly.
    Every keyframe votes per coarse map cell: ``negative`` when the bin's map
    structure is much taller than what that keyframe's scan still returned, and
    ``positive`` when the same keyframe did see structure up to that height.  A
    cell is removed only when negative votes reach ``min_votes`` **and** outnumber
    the positive ones, which is what keeps an occluded wall, pillar or floor out
    of the deletion list.
    """
    config = config or BinDisappearanceConfig()
    config.validate()
    xyz = np.ascontiguousarray(points[:, :3], dtype=np.float64)
    empty = np.ascontiguousarray(points[:, :3], dtype=np.float32)
    if not len(xyz):
        return empty, {"applied": False, "reason": "点云为空", "frames": 0, "removed_points": 0}

    cells = np.floor(xyz / config.cell_size).astype(np.int64)
    unique_cells, cell_of_point = np.unique(cells, axis=0, return_inverse=True)
    cell_of_point = cell_of_point.reshape(-1)
    cell_center = (unique_cells.astype(np.float64) + 0.5) * config.cell_size
    cell_total = len(unique_cells)
    total_bins = config.rings * config.sectors
    negative = np.zeros(cell_total, dtype=np.int64)
    positive = np.zeros(cell_total, dtype=np.int64)
    frames = 0
    rejected_bins = 0
    expected_frames = 0
    metadata_path = Path(history_dir) / "metadata.json"
    if metadata_path.is_file():
        try:
            expected_frames = int(
                json.loads(metadata_path.read_text(encoding="utf-8")).get("written_frames", 0)
            )
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            expected_frames = 0

    for _timestamp_ns, origin, endpoints in iter_keyframe_history(history_dir):
        frames += 1
        ray_xyz = np.asarray(endpoints, dtype=np.float64).reshape(-1, 3)
        if not len(ray_xyz):
            continue
        scan_inside, scan_bins = _polar_bin_indices(
            ray_xyz[:, 0] - origin[0], ray_xyz[:, 1] - origin[1], config
        )
        scan_counts, scan_min, scan_max = _bin_height_stats(
            scan_bins[scan_inside], ray_xyz[scan_inside, 2] - origin[2], total_bins
        )

        cell_inside, cell_bins = _polar_bin_indices(
            cell_center[:, 0] - origin[0], cell_center[:, 1] - origin[1], config
        )
        cell_h = cell_center[:, 2] - origin[2]
        map_counts, map_min, map_max = _bin_height_stats(
            cell_bins[cell_inside], cell_h[cell_inside], total_bins
        )

        has_scan = scan_counts >= config.min_scan_points
        scan_span = np.maximum(scan_max - scan_min, 1.0e-3)
        map_span = np.maximum(map_max - map_min, 1.0e-3)
        ratio = np.minimum(map_span / scan_span, scan_span / map_span)
        ground = scan_min
        # ERASOR v3 flags "map higher than scan" only for a real structure
        # (span > structure_span) that also stands above the bin's visible ground.
        disappeared = (
            has_scan
            & (map_counts > 0)
            & (ratio < config.scan_ratio_threshold)
            & (map_span > scan_span)
            & (map_span > config.structure_span)
            & (map_max > ground + config.height_threshold)
        )
        rejected_bins += int(np.count_nonzero(disappeared))
        confirmed_ceiling = np.where(np.isfinite(scan_max), scan_max + config.confirm_tolerance, -np.inf)

        in_range = cell_inside
        if not bool(in_range.any()):
            continue
        cell_ids = np.flatnonzero(in_range)
        bins_of_cells = cell_bins[cell_ids]
        heights = cell_h[cell_ids]
        negative_mask = disappeared[bins_of_cells] & (heights > ground[bins_of_cells] + config.height_threshold)
        positive_mask = (~disappeared[bins_of_cells]) & (heights <= confirmed_ceiling[bins_of_cells])
        if bool(negative_mask.any()):
            negative += np.bincount(cell_ids[negative_mask], minlength=cell_total)
        if bool(positive_mask.any()):
            positive += np.bincount(cell_ids[positive_mask], minlength=cell_total)
        if progress and frames % 8 == 0:
            denominator = max(expected_frames, frames)
            progress(min(0.99, frames / denominator), f"正在按极坐标格投票 {frames}/{denominator}")

    if frames < 3:
        return empty, {
            "applied": False,
            "reason": f"关键帧仅 {frames} 帧，不足以做极坐标格投票",
            "frames": frames,
            "removed_points": 0,
        }

    dynamic_cells = (negative >= config.min_votes) & (negative > positive)
    remove = dynamic_cells[cell_of_point]
    candidate = int(np.count_nonzero(remove))
    fraction = candidate / len(xyz)
    if fraction > config.max_removal_fraction:
        return empty, {
            "applied": False,
            "reason": f"极坐标格候选删除 {fraction:.1%} 超过安全上限 {config.max_removal_fraction:.1%}",
            "frames": frames,
            "removed_points": 0,
            "candidate_points": candidate,
            "removal_fraction": fraction,
        }

    filtered = np.ascontiguousarray(points[~remove][:, :3], dtype=np.float32)
    if progress:
        progress(1.0, f"极坐标格投票完成，删除 {candidate:,} 点")
    return filtered, {
        "applied": True,
        "frames": frames,
        "removed_points": candidate,
        "removal_fraction": fraction,
        "dynamic_cells": int(np.count_nonzero(dynamic_cells)),
        "candidate_cells": int(cell_total),
        "rejected_bins": rejected_bins,
        "min_votes": int(config.min_votes),
        "scan_ratio_threshold": float(config.scan_ratio_threshold),
        "max_negative_votes": int(negative.max(initial=0)),
        "max_positive_votes": int(positive.max(initial=0)),
    }


def filter_dynamic_with_history_consensus(
    points: np.ndarray,
    history_dir: Path,
    voxel_size: float,
    free_frame_threshold: int = 5,
    free_hit_ratio: float = 2.0,
    max_removal_fraction: float = 0.35,
    polar_config: Optional[BinDisappearanceConfig] = None,
    progress: Optional[Callable[[float, str], None]] = None,
) -> tuple[np.ndarray, dict[str, object]]:
    """Apply conservative ray and polar evidence to one finalized map.

    The ray pass is exact but can miss the shell of an object when no selected
    ray happens to traverse it.  The polar pass is region-wise and can catch
    that shell, but is less specific.  Running it *after* the ray pass gives
    it a cleaner map, while the shared final removal cap prevents their two
    independent decisions from deleting too much structure in one export.

    A polar result that would exceed the shared cap is discarded; a safe ray
    result is still retained.  This keeps the failure mode conservative when
    odometry, visibility, or polar parameters are unsuitable for a scene.
    """
    if not 0.0 <= float(max_removal_fraction) <= 1.0:
        raise ValueError("离线去动态总删除比例必须在 0–1 之间")
    original = np.ascontiguousarray(points[:, :3], dtype=np.float32)
    if not len(original):
        return original, {
            "applied": False,
            "reason": "点云为空",
            "input_points": 0,
            "output_points": 0,
            "removed_points": 0,
            "removal_fraction": 0.0,
            "ray": {"applied": False, "reason": "点云为空", "removed_points": 0},
            "polar": {"enabled": polar_config is not None, "applied": False, "removed_points": 0},
        }

    def ray_progress(value: float, stage: str) -> None:
        if progress:
            progress(0.60 * value, stage)

    ray_points, ray_result = filter_dynamic_with_keyframe_history(
        original,
        history_dir,
        voxel_size,
        free_frame_threshold=free_frame_threshold,
        free_hit_ratio=free_hit_ratio,
        max_removal_fraction=max_removal_fraction,
        progress=ray_progress,
    )
    polar_result: dict[str, object] = {
        "enabled": polar_config is not None,
        "applied": False,
        "reason": "极坐标格投票未开启" if polar_config is None else "尚未执行",
        "removed_points": 0,
    }
    final = ray_points
    if polar_config is not None:
        def polar_progress(value: float, stage: str) -> None:
            if progress:
                progress(0.60 + 0.40 * value, stage)

        candidate, raw_polar_result = filter_disappeared_with_bin_votes(
            ray_points, history_dir, polar_config, progress=polar_progress,
        )
        polar_result = {"enabled": True, **raw_polar_result}
        total_removed = len(original) - len(candidate)
        total_fraction = total_removed / len(original)
        if total_fraction > max_removal_fraction:
            polar_result = {
                **polar_result,
                "applied": False,
                "reason": (
                    f"射线与极坐标格合计候选删除 {total_fraction:.1%} "
                    f"超过总安全上限 {max_removal_fraction:.1%}，保留射线清理结果"
                ),
                "candidate_removed_points": int(raw_polar_result.get("removed_points", 0)),
                "candidate_total_removed_points": int(total_removed),
                "candidate_total_removal_fraction": float(total_fraction),
                "removed_points": 0,
            }
        else:
            final = candidate

    removed = len(original) - len(final)
    result: dict[str, object] = {
        "applied": bool(ray_result.get("applied")) or bool(polar_result.get("applied")),
        "input_points": int(len(original)),
        "output_points": int(len(final)),
        "removed_points": int(removed),
        "removal_fraction": float(removed / len(original)),
        "max_removal_fraction": float(max_removal_fraction),
        "ray": ray_result,
        "polar": polar_result,
    }
    if progress:
        progress(1.0, f"离线动态清理完成，合计删除 {removed:,} 点")
    return np.ascontiguousarray(final, dtype=np.float32), result


@dataclass(frozen=True)
class MapExportConfig:
    resolution: float = 0.05
    z_min: float = 0.05
    z_max: float = 1.50
    radius: float = 0.50
    min_neighbors: int = 10
    padding: float = 0.25
    height_mode: str = "absolute"
    filter_mode: str = "radius"
    filter_voxel_size: float = 0.10

    def validate(self) -> None:
        values = (
            self.resolution,
            self.z_min,
            self.z_max,
            self.radius,
            self.padding,
            self.filter_voxel_size,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("地图导出参数必须是有限数值")
        if not 0.005 <= self.resolution <= 1.0:
            raise ValueError("地图分辨率必须在 0.005–1.0 m 之间")
        if self.z_min >= self.z_max:
            raise ValueError("z_min 必须小于 z_max")
        if self.radius < 0 or self.min_neighbors < 0 or self.padding < 0:
            raise ValueError("滤波半径、邻点数和地图留白不能为负数")
        if self.height_mode not in {"ground", "absolute"}:
            raise ValueError("高度基准必须是 ground 或 absolute")
        if self.filter_mode not in {"voxel", "radius", "none"}:
            raise ValueError("离群点滤波模式必须是 voxel、radius 或 none")
        if not 0.02 <= self.filter_voxel_size <= 1.0:
            raise ValueError("结构体素尺寸必须在 0.02–1.0 m 之间")


def resolve_pcd_input(directory: Path, name: str) -> Path:
    """Resolve ``<name>.pcd`` inside ``directory`` after validating the name.

    Names go through :func:`sanitize_map_name` first, so separators and
    traversal names never reach the filesystem, and the resolved path is
    verified to stay inside ``directory`` as a second barrier.
    """
    safe_name = sanitize_map_name(name)
    root = Path(directory).resolve()
    path = root / f"{safe_name}.pcd"
    if path.resolve().parent != root:
        raise ValueError("点云名称超出 data/pcd 目录")
    if not path.is_file():
        raise FileNotFoundError(f"找不到点云文件 {safe_name}.pcd")
    return path


def slice_by_height(
    points: np.ndarray,
    z_min: Optional[float] = None,
    z_max: Optional[float] = None,
) -> np.ndarray:
    """Keep only the points inside an optional height window.

    The browser preview uses this so the 3-D view can show exactly the points a
    2-D slice would consume.  The accumulator and the exported PCD are never
    filtered this way: a save keeps every retained point.
    """
    xyz = np.asarray(points, dtype=np.float32)
    keep = np.ones(len(xyz), dtype=bool)
    if z_min is not None:
        keep &= xyz[:, 2] >= float(z_min)
    if z_max is not None:
        keep &= xyz[:, 2] <= float(z_max)
    return np.ascontiguousarray(xyz[keep])


def decimate_points(points: np.ndarray, max_points: Optional[int] = None) -> np.ndarray:
    """Uniformly stride a cloud down to at most ``max_points``.

    Browser transport stays cheap this way.  Export paths must not use it: PCD,
    PGM and YAML are always generated from every retained accumulator point.
    Only a strided view is copied, so the caller's array is never modified.
    """
    if max_points and len(points) > max_points:
        step = int(math.ceil(len(points) / max_points))
        return np.ascontiguousarray(points[::step][:max_points])
    return points


def build_cloud_frame(points: np.ndarray, max_points: Optional[int] = None) -> tuple[bytes, int, int]:
    """Pack one browser point-cloud frame.

    The wire layout is ASCII ``MAP1``, a little-endian uint32 point count and
    tightly packed little-endian float32 XYZ triples.  Returns
    ``(body, sent_points, total_points)``; ``max_points`` decimates only the
    transmitted frame and never mutates or shrinks the caller's array.
    """
    total = int(len(points))
    xyz = np.ascontiguousarray(decimate_points(points, max_points)[:, :3], dtype="<f4")
    return struct.pack("<4sI", b"MAP1", len(xyz)) + xyz.tobytes(order="C"), len(xyz), total


class VoxelAccumulator:
    """Retain one point per voxel and support visibility-based removal.

    A hit resets the voxel's miss counter.  ``apply_misses`` removes a voxel
    only after misses from the configured number of distinct source frames.
    Unobserved voxels never decay.  Deleted storage slots are reused, keeping
    the hard ``max_points`` limit valid without unbounded historical storage.
    """

    def __init__(self, voxel_size: float = 0.05, max_points: int = 5_000_000) -> None:
        if not 0.005 <= voxel_size <= 1.0:
            raise ValueError("voxel_size 必须在 0.005–1.0 m 之间")
        self.voxel_size = float(voxel_size)
        self.max_points = int(max_points)
        self._lock = threading.RLock()
        self.clear()

    def clear(self) -> None:
        with getattr(self, "_lock", threading.RLock()):
            self._key_to_location: dict[int, int] = {}
            self._point_chunks: list[np.ndarray] = []
            self._active_chunks: list[np.ndarray] = []
            self._miss_chunks: list[np.ndarray] = []
            self._miss_generation_chunks: list[np.ndarray] = []
            self._hit_generation_chunks: list[np.ndarray] = []
            self._free_locations: list[int] = []
            self._point_count = 0
            self._stored_count = 0
            self._removed_total = 0
            self._implicit_generation = 0
            self._minimum = np.array([math.inf, math.inf, math.inf], dtype=np.float64)
            self._maximum = np.array([-math.inf, -math.inf, -math.inf], dtype=np.float64)
            self._bounds_dirty = False
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
            if self._bounds_dirty:
                self._refresh_bounds_locked(self._snapshot_locked())
            return {
                "min_x": float(self._minimum[0]), "min_y": float(self._minimum[1]),
                "min_z": float(self._minimum[2]), "max_x": float(self._maximum[0]),
                "max_y": float(self._maximum[1]), "max_z": float(self._maximum[2]),
            }

    @property
    def removed_total(self) -> int:
        with self._lock:
            return self._removed_total

    def add(self, points: np.ndarray, generation: Optional[int] = None) -> int:
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("points 必须是 N×3 数组")
        xyz = np.ascontiguousarray(points[:, :3])
        if not len(xyz):
            return 0
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if not len(xyz):
            return 0

        voxel = voxel_coordinates(xyz, self.voxel_size)
        packed = _pack_voxel_coordinates(voxel)
        unique_keys, unique_indices = np.unique(packed, return_index=True)

        with self._lock:
            if generation is None:
                self._implicit_generation += 1
                generation = self._implicit_generation
            generation_value = np.uint32(int(generation) & 0xFFFFFFFF)
            remaining = self.max_points - self._point_count
            new_pairs: list[tuple[int, int]] = []
            for key_value, index_value in zip(unique_keys, unique_indices):
                key = int(key_value)
                location = self._key_to_location.get(key)
                if location is not None:
                    chunk_index, point_index = divmod(location, 1 << 32)
                    self._miss_chunks[chunk_index][point_index] = 0
                    self._miss_generation_chunks[chunk_index][point_index] = 0
                    self._hit_generation_chunks[chunk_index][point_index] = generation_value
                elif len(new_pairs) < remaining:
                    new_pairs.append((key, int(index_value)))
            if not new_pairs:
                return 0

            reused = min(len(self._free_locations), len(new_pairs))
            for pair_index in range(reused):
                key, source_index = new_pairs[pair_index]
                location = self._free_locations.pop()
                chunk_index, point_index = divmod(location, 1 << 32)
                self._point_chunks[chunk_index][point_index] = xyz[source_index]
                self._active_chunks[chunk_index][point_index] = True
                self._miss_chunks[chunk_index][point_index] = 0
                self._miss_generation_chunks[chunk_index][point_index] = 0
                self._hit_generation_chunks[chunk_index][point_index] = generation_value
                self._key_to_location[key] = location

            appended = new_pairs[reused:]
            if appended:
                keys, indices = zip(*appended)
                kept = np.ascontiguousarray(
                    xyz[np.fromiter(indices, dtype=np.int64)], dtype=np.float32
                )
                chunk_index = len(self._point_chunks)
                self._point_chunks.append(kept)
                self._active_chunks.append(np.ones(len(kept), dtype=bool))
                self._miss_chunks.append(np.zeros(len(kept), dtype=np.uint8))
                self._miss_generation_chunks.append(np.zeros(len(kept), dtype=np.uint32))
                self._hit_generation_chunks.append(
                    np.full(len(kept), generation_value, dtype=np.uint32)
                )
                for point_index, key in enumerate(keys):
                    self._key_to_location[key] = (chunk_index << 32) | point_index
                self._stored_count += len(kept)

            self._point_count += len(new_pairs)
            added_points = xyz[np.fromiter(
                (source_index for _key, source_index in new_pairs), dtype=np.int64
            )]
            self._minimum = np.minimum(self._minimum, added_points.min(axis=0))
            self._maximum = np.maximum(self._maximum, added_points.max(axis=0))
            self._version += 1
            return len(new_pairs)

    def apply_misses(
        self,
        packed_keys: np.ndarray,
        generation: int,
        miss_threshold: int = 5,
    ) -> int:
        """Apply one free-space observation per key for a source frame."""
        if not 1 <= int(miss_threshold) <= 255:
            raise ValueError("miss_threshold 必须在 1–255 之间")
        keys = np.unique(np.asarray(packed_keys, dtype=np.uint64).reshape(-1))
        if not len(keys):
            return 0
        generation_value = int(generation) & 0xFFFFFFFF
        removed = 0
        with self._lock:
            for key_value in keys:
                key = int(key_value)
                location = self._key_to_location.get(key)
                if location is None:
                    continue
                chunk_index, point_index = divmod(location, 1 << 32)
                # A hit in this or a newer frame overrides stale free evidence.
                if int(self._hit_generation_chunks[chunk_index][point_index]) >= generation_value:
                    continue
                if int(self._miss_generation_chunks[chunk_index][point_index]) == generation_value:
                    continue
                misses = int(self._miss_chunks[chunk_index][point_index]) + 1
                self._miss_generation_chunks[chunk_index][point_index] = generation_value
                if misses < miss_threshold:
                    self._miss_chunks[chunk_index][point_index] = misses
                    continue
                self._active_chunks[chunk_index][point_index] = False
                self._miss_chunks[chunk_index][point_index] = 0
                self._miss_generation_chunks[chunk_index][point_index] = 0
                del self._key_to_location[key]
                self._free_locations.append(location)
                self._point_count -= 1
                removed += 1
            if removed:
                self._removed_total += removed
                self._bounds_dirty = True
                self._version += 1
        return removed

    def _refresh_bounds_locked(self, points: np.ndarray) -> None:
        if len(points):
            self._minimum = points.min(axis=0).astype(np.float64)
            self._maximum = points.max(axis=0).astype(np.float64)
        else:
            self._minimum = np.array([math.inf, math.inf, math.inf], dtype=np.float64)
            self._maximum = np.array([-math.inf, -math.inf, -math.inf], dtype=np.float64)
        self._bounds_dirty = False

    def _snapshot_locked(self) -> np.ndarray:
        if self._cached_version != self._version:
            active_chunks = [
                points if bool(active.all()) else points[active]
                for points, active in zip(self._point_chunks, self._active_chunks)
                if bool(active.any())
            ]
            self._cached_points = (
                np.ascontiguousarray(np.concatenate(active_chunks, axis=0), dtype=np.float32)
                if active_chunks else np.empty((0, 3), dtype=np.float32)
            )
            self._cached_version = self._version
        if self._bounds_dirty:
            self._refresh_bounds_locked(self._cached_points)
        return self._cached_points

    def snapshot(self, max_points: Optional[int] = None) -> np.ndarray:
        with self._lock:
            points = self._snapshot_locked()
            if max_points and len(points) > max_points:
                return decimate_points(points, max_points)
            return points.copy()

    def replace(self, points: np.ndarray) -> None:
        """Atomically replace the accumulator with an already-filtered cloud."""
        with self._lock:
            self.clear()
            self.add(points)


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


def read_binary_pcd(path: Path) -> np.ndarray:
    """Read XYZ from an ASCII or uncompressed-binary PCD file.

    Extra fields (for example ``intensity`` or ``ring``) are accepted and
    ignored.  The returned array is always a contiguous N×3 float32 array so
    callers never need to retain the complete source record layout.
    """
    path = Path(path)
    with path.open("rb") as stream:
        header: dict[str, list[str]] = {}
        header_bytes = 0
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PCD 文件头不完整：{path.name}")
            header_bytes += len(line)
            if header_bytes > 64 * 1024:
                raise ValueError(f"PCD 文件头超过 64 KiB：{path.name}")
            try:
                text = line.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise ValueError(f"PCD 文件头不是 ASCII：{path.name}") from error
            if not text or text.startswith("#"):
                continue
            fields = text.split()
            key = fields[0].upper()
            header[key] = fields[1:]
            if key == "DATA":
                break

        fields = header.get("FIELDS") or header.get("FIELD")
        if not fields or len(set(fields)) != len(fields):
            raise ValueError("PCD 缺少有效且不重复的 FIELDS")
        for coordinate in ("x", "y", "z"):
            if coordinate not in fields:
                raise ValueError("PCD 必须包含 x、y、z 字段")
        try:
            sizes = [int(value) for value in header["SIZE"]]
            types = [value.upper() for value in header["TYPE"]]
            counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        except (KeyError, ValueError) as error:
            raise ValueError("PCD SIZE、TYPE 或 COUNT 字段无效") from error
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT 的字段数不一致")
        if any(count < 1 for count in counts):
            raise ValueError("PCD COUNT 必须为正整数")
        if any(counts[fields.index(coordinate)] != 1 for coordinate in ("x", "y", "z")):
            raise ValueError("PCD x/y/z 字段的 COUNT 必须为 1")
        try:
            if header.get("POINTS"):
                count = int(header["POINTS"][0])
            else:
                count = int(header["WIDTH"][0]) * int(header.get("HEIGHT", ["1"])[0])
        except (KeyError, IndexError, ValueError) as error:
            raise ValueError("PCD 缺少有效的 POINTS 或 WIDTH/HEIGHT 字段") from error
        if count < 0:
            raise ValueError("PCD POINTS 不能为负数")

        data_kind = header.get("DATA", [""])[0].lower()
        if data_kind == "binary_compressed":
            raise ValueError("PCD binary_compressed 暂不支持，请转为 binary 或 ascii")

        type_codes = {
            ("F", 4): "<f4", ("F", 8): "<f8",
            ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
            ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
        }
        try:
            scalar_types = [np.dtype(type_codes[(kind, size)]) for kind, size in zip(types, sizes)]
        except KeyError as error:
            raise ValueError(f"PCD 包含不支持的 TYPE/SIZE 组合：{error.args[0]}") from error

        if data_kind == "binary":
            dtype = np.dtype([
                (field, scalar, (field_count,)) if field_count > 1 else (field, scalar)
                for field, scalar, field_count in zip(fields, scalar_types, counts)
            ])
            expected = count * dtype.itemsize
            actual = path.stat().st_size - stream.tell()
            if actual != expected:
                raise ValueError(f"PCD 数据长度应为 {expected} 字节，实际为 {actual}")
            records = np.fromfile(stream, dtype=dtype, count=count)
            xyz = np.column_stack((records["x"], records["y"], records["z"]))
        elif data_kind == "ascii":
            try:
                values = np.fromstring(stream.read().decode("ascii"), sep=" ")
            except UnicodeDecodeError as error:
                raise ValueError("PCD ascii 数据区包含非 ASCII 字节") from error
            values_per_point = sum(counts)
            expected = count * values_per_point
            if len(values) != expected:
                raise ValueError(f"PCD ascii 数值数应为 {expected}，实际为 {len(values)}")
            records = values.reshape(count, values_per_point)
            offsets = np.cumsum([0, *counts[:-1]])
            xyz = records[:, [int(offsets[fields.index(axis)]) for axis in ("x", "y", "z")]]
        else:
            raise ValueError(f"PCD DATA 类型不支持：{data_kind or '缺失'}")

    xyz = np.ascontiguousarray(xyz, dtype=np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def convert_pcd_to_map(
    output_root: Path,
    source_name: str,
    output_name: str,
    config: MapExportConfig,
) -> dict[str, object]:
    """Convert an existing data/pcd file into an atomic saved map bundle.

    When ``output_name`` matches ``source_name`` the input PCD is never
    rewritten.  A different output name receives a normalized XYZ-only binary
    PCD so the PGM/YAML and PCD remain a same-name editable bundle.
    """
    source_name = sanitize_map_name(source_name)
    output_name = sanitize_map_name(output_name)
    config.validate()
    root = Path(output_root).resolve()
    source_pcd = root / "pcd" / f"{source_name}.pcd"
    if not source_pcd.is_file():
        raise FileNotFoundError(f"找不到点云：{source_name}.pcd")

    target_pcd = root / "pcd" / f"{output_name}.pcd"
    target_pgm = root / "map" / f"{output_name}.pgm"
    target_yaml = root / "map" / f"{output_name}.yaml"
    protected = [target_pgm, target_yaml]
    if output_name != source_name:
        protected.append(target_pcd)
    existing = next((path for path in protected if path.exists()), None)
    if existing is not None:
        raise FileExistsError(f"{existing.name} 已存在，未覆盖")

    points = read_binary_pcd(source_pcd)
    if len(points) < 3:
        raise ValueError("PCD 中的有效 XYZ 点不足 3 个，无法生成二维地图")

    root.joinpath("pcd").mkdir(parents=True, exist_ok=True)
    root.joinpath("map").mkdir(parents=True, exist_ok=True)
    installed: list[Path] = []
    with tempfile.TemporaryDirectory(prefix=".pcd-convert-", dir=root) as temporary:
        stage_root = Path(temporary)
        stage_pgm = stage_root / "map" / target_pgm.name
        stage_yaml = stage_root / "map" / target_yaml.name
        metadata = write_occupancy_map(stage_pgm, stage_yaml, points, config)
        stage_pcd: Optional[Path] = None
        if output_name != source_name:
            stage_pcd = stage_root / "pcd" / target_pcd.name
            write_binary_pcd(stage_pcd, points)
        try:
            if stage_pcd is not None:
                os.replace(stage_pcd, target_pcd)
                installed.append(target_pcd)
            os.replace(stage_pgm, target_pgm)
            installed.append(target_pgm)
            os.replace(stage_yaml, target_yaml)
            installed.append(target_yaml)
        except Exception:
            for path in reversed(installed):
                path.unlink(missing_ok=True)
            raise

    return {
        "source_pcd": str(source_pcd), "pcd": str(target_pcd),
        "pgm": str(target_pgm), "yaml": str(target_yaml),
        "source_name": source_name, "output_name": output_name,
        "point_count": int(len(points)), "map": metadata,
    }


def _radius_filter(points: np.ndarray, radius: float, minimum: int) -> np.ndarray:
    if radius <= 0 or minimum <= 1 or len(points) < minimum:
        return points
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        return points
    tree = cKDTree(points)
    keep = np.zeros(len(points), dtype=bool)
    # This runs inside an HTTP worker.  ``workers=-1`` made one preview occupy
    # every CPU core and could render the operator console (or an SSH desktop)
    # unresponsive on a large cloud.  A single cKDTree worker keeps the backend
    # responsive; chunking still bounds the temporary count array without
    # changing which points pass the exact radius test.
    chunk = 50_000
    for start in range(0, len(points), chunk):
        stop = min(len(points), start + chunk)
        counts = tree.query_ball_point(points[start:stop], radius, return_length=True, workers=1)
        keep[start:stop] = counts >= minimum
    return points[keep]


def _voxel_structure_filter(
    points: np.ndarray,
    voxel_size: float,
) -> tuple[np.ndarray, dict[str, int]]:
    """Remove unsupported single voxels using ROGMap-style column classes.

    A saved PCD does not contain the per-ray hit/miss history required to
    reproduce ROGMap's probabilistic ``OCCUPIED / KNOWN_FREE / UNKNOWN`` state.
    What *is* available offline is its projection-layer idea: classify occupied
    3-D voxels by their XY column and inspect the eight neighboring columns.

    A column is retained when it contains at least two occupied Z voxels
    (vertical structure such as a wall or pole), or when another occupied XY
    column touches it (horizontal support such as a curb or thin rail).  Only a
    truly isolated, single-Z-voxel column is discarded.  Classification is
    performed on unique voxels, so point density cannot make one noisy return
    look like structure merely because the same voxel contains many points.
    """
    source = np.asarray(points, dtype=np.float32)
    if not len(source):
        return source, {
            "filter_voxels": 0,
            "filter_columns": 0,
            "filter_kept_columns": 0,
            "filter_removed_columns": 0,
            "filter_removed_points": 0,
        }

    voxel_xyz = voxel_coordinates(source, voxel_size)
    unique_voxels, point_voxels = np.unique(voxel_xyz, axis=0, return_inverse=True)
    columns, voxel_columns, vertical_counts = np.unique(
        unique_voxels[:, :2], axis=0, return_inverse=True, return_counts=True
    )

    column_min = columns.min(axis=0)
    local_columns = columns - column_min
    width = int(local_columns[:, 0].max()) + 1
    height = int(local_columns[:, 1].max()) + 1
    if width > np.iinfo(np.int64).max // max(1, height):
        raise ValueError("点云坐标范围过大，无法进行结构体素分类")

    column_ids = local_columns[:, 1] * width + local_columns[:, 0]
    order = np.argsort(column_ids)
    sorted_ids = column_ids[order]
    sorted_xy = local_columns[order]
    has_neighbor_sorted = np.zeros(len(columns), dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            valid = (
                (sorted_xy[:, 0] + dx >= 0)
                & (sorted_xy[:, 0] + dx < width)
                & (sorted_xy[:, 1] + dy >= 0)
                & (sorted_xy[:, 1] + dy < height)
            )
            valid_indices = np.flatnonzero(valid)
            targets = sorted_ids[valid_indices] + dy * width + dx
            positions = np.searchsorted(sorted_ids, targets)
            found = positions < len(sorted_ids)
            found[found] &= sorted_ids[positions[found]] == targets[found]
            has_neighbor_sorted[valid_indices[found]] = True

    has_neighbor = np.zeros(len(columns), dtype=bool)
    has_neighbor[order] = has_neighbor_sorted
    keep_columns = (vertical_counts >= 2) | has_neighbor
    point_columns = voxel_columns[point_voxels]
    keep_points = keep_columns[point_columns]
    filtered = np.ascontiguousarray(source[keep_points], dtype=np.float32)
    return filtered, {
        "filter_voxels": int(len(unique_voxels)),
        "filter_columns": int(len(columns)),
        "filter_kept_columns": int(np.count_nonzero(keep_columns)),
        "filter_removed_columns": int(np.count_nonzero(~keep_columns)),
        "filter_removed_points": int(len(source) - len(filtered)),
    }


def estimate_ground_plane(points: np.ndarray) -> tuple[np.ndarray, dict[str, float | int]]:
    """Estimate the dominant traversable ground as ``z = ax + by + c``.

    The accumulated cloud can be tilted by a small LIO roll/pitch error, which
    makes a single world-Z slice cut through distant floor.  We first take a
    low-height representative from every populated 0.5 m XY cell and then fit
    those representatives with deterministic RANSAC.  Equal cell weighting is
    intentional: a nearby, very dense wall must not outweigh a broad floor.

    This is an offline export operation, never part of the ROS subscription
    callback.  Plane fitting uses at most 500,000 uniformly strided points;
    the resulting plane is still applied to every point by the caller.  This
    keeps multi-million-point sessions bounded without modifying the PCD.
    """
    source = np.asarray(points)
    if source.ndim != 2 or source.shape[1] < 3 or len(source) < 12:
        raise ValueError("点云不足，无法估计地面高度")
    maximum_fit_points = 500_000
    stride = max(1, int(math.ceil(len(source) / maximum_fit_points)))
    xyz = np.asarray(source[::stride][:maximum_fit_points, :3], dtype=np.float32)

    cell_size = 0.5
    xy_min = xyz[:, :2].min(axis=0)
    cell_xy = np.floor((xyz[:, :2] - xy_min) / cell_size).astype(np.int64)
    cell_width = int(cell_xy[:, 0].max()) + 1
    cell_ids = cell_xy[:, 1] * cell_width + cell_xy[:, 0]

    # Sort by cell first and height second.  This lets us select one low
    # quantile per cell without allocating a dense grid or looping over every
    # input point.
    order = np.lexsort((xyz[:, 2], cell_ids))
    sorted_ids = cell_ids[order]
    starts = np.r_[0, np.flatnonzero(sorted_ids[1:] != sorted_ids[:-1]) + 1]
    counts = np.diff(np.r_[starts, len(sorted_ids)])
    populated = counts >= 5
    if int(np.count_nonzero(populated)) < 12:
        raise ValueError("有效地面网格不足，请改用「全局 Z」高度基准")

    starts = starts[populated]
    counts = counts[populated]
    unique_ids = sorted_ids[starts]
    quantile_indices = starts + np.floor((counts - 1) * 0.10).astype(np.int64)
    samples = np.column_stack((
        xy_min[0] + (unique_ids % cell_width + 0.5) * cell_size,
        xy_min[1] + (unique_ids // cell_width + 0.5) * cell_size,
        xyz[order[quantile_indices], 2],
    ))

    rng = np.random.default_rng(2027)
    design = np.column_stack((samples[:, :2], np.ones(len(samples))))
    residual_limit = 0.10
    maximum_slope = math.tan(math.radians(20.0))
    best_mask: Optional[np.ndarray] = None
    best_score = -1
    best_error = math.inf
    iterations = min(512, max(128, len(samples) // 2))
    for _ in range(iterations):
        chosen = rng.choice(len(samples), 3, replace=False)
        try:
            coefficients = np.linalg.solve(design[chosen], samples[chosen, 2])
        except np.linalg.LinAlgError:
            continue
        if float(np.linalg.norm(coefficients[:2])) > maximum_slope:
            continue
        residuals = np.abs(design @ coefficients - samples[:, 2])
        mask = residuals <= residual_limit
        score = int(np.count_nonzero(mask))
        error = float(np.median(residuals[mask])) if score else math.inf
        if score > best_score or (score == best_score and error < best_error):
            best_mask, best_score, best_error = mask, score, error

    minimum_inliers = max(12, int(math.ceil(len(samples) * 0.08)))
    if best_mask is None or best_score < minimum_inliers:
        raise ValueError("未找到稳定地面，请改用「全局 Z」高度基准")

    # Refit a few times so the reported plane is based on all consensus cells,
    # not merely the three samples that proposed the winning model.
    coefficients = np.zeros(3, dtype=np.float64)
    for _ in range(3):
        coefficients = np.linalg.lstsq(design[best_mask], samples[best_mask, 2], rcond=None)[0]
        residuals = np.abs(design @ coefficients - samples[:, 2])
        refined = residuals <= residual_limit
        if np.array_equal(refined, best_mask):
            break
        if int(np.count_nonzero(refined)) < minimum_inliers:
            break
        best_mask = refined

    slope = float(np.linalg.norm(coefficients[:2]))
    if slope > maximum_slope:
        raise ValueError("估计地面倾斜超过 20°，请检查点云或改用「全局 Z」")
    return coefficients, {
        "ground_a": float(coefficients[0]),
        "ground_b": float(coefficients[1]),
        "ground_c": float(coefficients[2]),
        "ground_tilt_deg": float(math.degrees(math.atan(slope))),
        "ground_cells": int(len(samples)),
        "ground_inlier_cells": int(np.count_nonzero(best_mask)),
    }


def _local_ground_offsets(
    points: np.ndarray,
    plane: np.ndarray,
) -> tuple[np.ndarray, dict[str, float | int]]:
    """Estimate a smooth local correction to a fitted ground plane.

    A single plane corrects LIO roll/pitch error, but real floors and long LIO
    trajectories are not perfectly planar.  Their remaining few centimetres
    of bow can still cross the default 5 cm obstacle threshold and turn a
    traversable patch black.  This function grows a ground surface through
    neighboring 0.4 m cells whose low-height samples change gradually, then
    interpolates those offsets into occupied cells up to 2 m away.

    The initial seed band and the per-cell growth step are deliberately below
    a typical 10 cm curb.  A raised platform therefore cannot become ground
    merely because it is broad and dense.  Point work stays vectorized; only
    the bounded set of occupied XY cells is visited in Python.  Like the plane
    fit, surface sampling is capped at 500,000 strided points.
    """
    source = np.asarray(points)
    if source.ndim != 2 or source.shape[1] < 3 or not len(source):
        return np.zeros(len(source), dtype=np.float64), {
            "ground_local_cells": 0,
            "ground_local_anchor_cells": 0,
            "ground_local_offset_min": 0.0,
            "ground_local_offset_max": 0.0,
        }

    cell_size = 0.4
    maximum_fit_points = 500_000
    stride = max(1, int(math.ceil(len(source) / maximum_fit_points)))
    sample = np.asarray(source[::stride][:maximum_fit_points, :3], dtype=np.float32)
    xy_min = np.asarray(source[:, :2].min(axis=0), dtype=np.float64)
    xy_max = np.asarray(source[:, :2].max(axis=0), dtype=np.float64)
    span_cells = np.floor((xy_max - xy_min) / cell_size).astype(np.int64) + 1
    cell_width, cell_height = int(span_cells[0]), int(span_cells[1])
    if cell_width < 1 or cell_height < 1:
        return np.zeros(len(source), dtype=np.float64), {
            "ground_local_cells": 0,
            "ground_local_anchor_cells": 0,
            "ground_local_offset_min": 0.0,
            "ground_local_offset_max": 0.0,
        }
    if cell_width > np.iinfo(np.int64).max // cell_height:
        raise ValueError("点云坐标范围过大，无法估计局部地面")

    sample_x = np.floor((sample[:, 0].astype(np.float64) - xy_min[0]) / cell_size).astype(np.int64)
    sample_y = np.floor((sample[:, 1].astype(np.float64) - xy_min[1]) / cell_size).astype(np.int64)
    sample_ids = sample_y * cell_width + sample_x
    order = np.lexsort((sample[:, 2], sample_ids))
    sorted_ids = sample_ids[order]
    starts = np.r_[0, np.flatnonzero(sorted_ids[1:] != sorted_ids[:-1]) + 1]
    counts = np.diff(np.r_[starts, len(sorted_ids)])
    populated = counts >= 5
    if int(np.count_nonzero(populated)) < 12:
        return np.zeros(len(source), dtype=np.float64), {
            "ground_local_cells": int(len(starts)),
            "ground_local_anchor_cells": 0,
            "ground_local_offset_min": 0.0,
            "ground_local_offset_max": 0.0,
        }

    starts = starts[populated]
    counts = counts[populated]
    cell_ids = sorted_ids[starts]
    cell_x = cell_ids % cell_width
    cell_y = cell_ids // cell_width
    centers_x = xy_min[0] + (cell_x.astype(np.float64) + 0.5) * cell_size
    centers_y = xy_min[1] + (cell_y.astype(np.float64) + 0.5) * cell_size
    quantile_indices = starts + np.floor((counts - 1) * 0.10).astype(np.int64)
    low_z = sample[order[quantile_indices], 2].astype(np.float64)
    plane_z = plane[0] * centers_x + plane[1] * centers_y + plane[2]

    # Strict seeds avoid declaring a 10 cm curb to be floor.  Region growth
    # can still follow a smooth local bow/ramp much farther from the plane.
    ground = np.abs(low_z - plane_z) <= 0.06
    if int(np.count_nonzero(ground)) < 12:
        return np.zeros(len(source), dtype=np.float64), {
            "ground_local_cells": int(len(cell_ids)),
            "ground_local_anchor_cells": int(np.count_nonzero(ground)),
            "ground_local_offset_min": 0.0,
            "ground_local_offset_max": 0.0,
        }
    id_to_index = {int(cell_id): index for index, cell_id in enumerate(cell_ids)}
    queue = deque(np.flatnonzero(ground).tolist())
    neighbors = (
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    )
    while queue:
        index = queue.popleft()
        x, y = int(cell_x[index]), int(cell_y[index])
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= cell_width or ny < 0 or ny >= cell_height:
                continue
            adjacent = id_to_index.get(ny * cell_width + nx)
            if adjacent is None or ground[adjacent]:
                continue
            allowed_step = 0.055 * (math.sqrt(2.0) if dx and dy else 1.0)
            if abs(low_z[adjacent] - low_z[index]) <= allowed_step:
                ground[adjacent] = True
                queue.append(adjacent)

    ground_indices = np.flatnonzero(ground)
    if len(ground_indices) < 12:
        return np.zeros(len(source), dtype=np.float64), {
            "ground_local_cells": int(len(cell_ids)),
            "ground_local_anchor_cells": int(len(ground_indices)),
            "ground_local_offset_min": 0.0,
            "ground_local_offset_max": 0.0,
        }

    # The tenth percentile follows the lower envelope.  Move it at most 4 cm
    # toward the center of the local floor-return band so sensor thickness does
    # not leak above a 5 cm operator threshold.  The cap protects low curbs.
    anchor_z = low_z.copy()
    for index in ground_indices:
        cell_heights = sample[order[starts[index]:starts[index] + counts[index]], 2]
        lower_band = cell_heights[cell_heights <= low_z[index] + 0.10]
        if len(lower_band):
            lift = float(np.median(lower_band)) - low_z[index]
            anchor_z[index] += min(0.04, max(0.0, lift))
    anchor_offsets = anchor_z[ground] - plane_z[ground]
    anchor_ids = cell_ids[ground]
    anchor_lookup = {
        int(cell_id): float(offset)
        for cell_id, offset in zip(anchor_ids, anchor_offsets)
    }

    # Find every occupied cell once, then map its correction back to all points
    # with a vectorized inverse index.  This avoids per-point Python work.
    point_x = np.floor((source[:, 0].astype(np.float64) - xy_min[0]) / cell_size).astype(np.int64)
    point_y = np.floor((source[:, 1].astype(np.float64) - xy_min[1]) / cell_size).astype(np.int64)
    point_ids = point_y * cell_width + point_x
    occupied_ids, inverse = np.unique(point_ids, return_inverse=True)
    occupied_offsets = np.zeros(len(occupied_ids), dtype=np.float64)

    radius_cells = int(math.ceil(2.0 / cell_size))
    search_offsets = sorted(
        (
            (dx * dx + dy * dy, dx, dy)
            for dy in range(-radius_cells, radius_cells + 1)
            for dx in range(-radius_cells, radius_cells + 1)
            if dx * dx + dy * dy <= radius_cells * radius_cells
        ),
        key=lambda item: (item[0], item[2], item[1]),
    )
    for occupied_index, cell_id in enumerate(occupied_ids):
        x, y = int(cell_id % cell_width), int(cell_id // cell_width)
        weighted_sum = 0.0
        total_weight = 0.0
        found = 0
        for distance_squared, dx, dy in search_offsets:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= cell_width or ny < 0 or ny >= cell_height:
                continue
            offset = anchor_lookup.get(ny * cell_width + nx)
            if offset is None:
                continue
            weight = 1.0 / max(float(distance_squared), 0.25)
            weighted_sum += weight * offset
            total_weight += weight
            found += 1
            if found >= 4:
                break
        if total_weight:
            occupied_offsets[occupied_index] = weighted_sum / total_weight

    return occupied_offsets[inverse], {
        "ground_local_cells": int(len(cell_ids)),
        "ground_local_anchor_cells": int(len(ground_indices)),
        "ground_local_offset_min": float(anchor_offsets.min()),
        "ground_local_offset_max": float(anchor_offsets.max()),
    }


def slice_points_by_config(
    points: np.ndarray,
    config: MapExportConfig,
) -> tuple[np.ndarray, dict[str, float | int | str]]:
    """Select an obstacle-height band using the configured height reference."""
    config.validate()
    xyz = np.asarray(points, dtype=np.float32)
    metadata: dict[str, float | int | str] = {"height_mode": config.height_mode}
    if config.height_mode == "ground":
        plane, ground_metadata = estimate_ground_plane(xyz)
        local_offsets, local_metadata = _local_ground_offsets(xyz, plane)
        relative_z = xyz[:, 2].astype(np.float64) - (
            xyz[:, 0].astype(np.float64) * plane[0]
            + xyz[:, 1].astype(np.float64) * plane[1]
            + plane[2]
            + local_offsets
        )
        keep = (relative_z >= config.z_min) & (relative_z <= config.z_max)
        metadata.update(ground_metadata)
        metadata.update(local_metadata)
    else:
        keep = (xyz[:, 2] >= config.z_min) & (xyz[:, 2] <= config.z_max)
    return np.ascontiguousarray(xyz[keep]), metadata


def slice_points_to_occupancy(
    points: np.ndarray,
    config: MapExportConfig,
) -> tuple[np.ndarray, dict[str, float | int | str]]:
    """Rasterize a PointXYZ cloud into a Nav2-style occupancy grid.

    The result is bottom-row-first (PGM/YAML order, so +Y points up) with ``0``
    for an occupied cell and ``254`` for a free cell.  Nothing is written to
    disk, so the same function backs both the on-disk export and the read-only
    browser preview; the two can therefore never disagree about a slice.
    """
    points = np.asarray(points, dtype=np.float32)
    sliced, height_metadata = slice_points_by_config(points, config)
    if len(sliced) < 3:
        reference = "相对地面" if config.height_mode == "ground" else "全局 Z"
        raise ValueError(
            f"{reference}={config.z_min:.2f}–{config.z_max:.2f} m 切片后只有 "
            f"{len(sliced)} 个点，无法生成二维地图"
        )
    filter_metadata: dict[str, float | int | str] = {
        "filter_mode": config.filter_mode,
        "filter_removed_points": 0,
    }
    if config.filter_mode == "voxel":
        filtered, voxel_metadata = _voxel_structure_filter(
            sliced, config.filter_voxel_size
        )
        filter_metadata.update(voxel_metadata)
        filter_metadata["filter_voxel_size"] = config.filter_voxel_size
    elif config.filter_mode == "radius":
        filtered = _radius_filter(sliced, config.radius, config.min_neighbors)
        filter_metadata["filter_removed_points"] = int(len(sliced) - len(filtered))
    else:
        filtered = sliced
    if len(filtered) < 3:
        if config.filter_mode == "voxel":
            raise ValueError("结构体素滤波后没有足够点，请减小体素尺寸或改用半径滤波")
        raise ValueError("离群点滤波后没有足够点，请减小 min_neighbors 或扩大 radius")

    minimum = filtered[:, :2].min(axis=0) - config.padding
    maximum = filtered[:, :2].max(axis=0) + config.padding
    width, height = np.ceil((maximum - minimum) / config.resolution).astype(np.int64) + 1
    if width * height > MAX_OCCUPANCY_CELLS:
        raise ValueError(f"二维地图尺寸异常（{width}×{height}），请检查点云坐标")

    pixels = np.full((int(height), int(width)), 254, dtype=np.uint8)
    cells = np.floor((filtered[:, :2] - minimum) / config.resolution).astype(np.int64)
    cells[:, 0] = np.clip(cells[:, 0], 0, width - 1)
    cells[:, 1] = np.clip(cells[:, 1], 0, height - 1)
    pixels[cells[:, 1], cells[:, 0]] = 0
    pixels = np.flipud(pixels)

    return pixels, {
        "width": int(width), "height": int(height), "resolution": config.resolution,
        "origin_x": float(minimum[0]), "origin_y": float(minimum[1]),
        "z_min": config.z_min, "z_max": config.z_max,
        "slice_points": int(len(sliced)), "filtered_points": int(len(filtered)),
        **filter_metadata,
        **height_metadata,
    }


def encode_pgm(pixels: np.ndarray, comment: str = "generated by mapping_web_ui") -> bytes:
    """Encode one occupancy grid as a binary PGM (P5) byte string."""
    height, width = pixels.shape
    header = f"P5\n# {comment}\n{width} {height}\n255\n".encode("ascii")
    return header + np.ascontiguousarray(pixels, dtype=np.uint8).tobytes(order="C")


def pool_occupancy_pixels(
    pixels: np.ndarray,
    max_pixels: int = PREVIEW_MAX_PIXELS,
) -> tuple[np.ndarray, int]:
    """Shrink a grid for browser transport with integer block-min pooling.

    Occupied cells win inside every block, so a one-cell wall can never vanish
    from a decimated preview.  The pool factor is an integer, which keeps the
    preview proportional to the real map instead of rescaling it arbitrarily.
    """
    height, width = np.ascontiguousarray(pixels).shape
    if height <= 0 or width <= 0 or height * width <= max_pixels:
        return np.ascontiguousarray(pixels), 1
    stride = int(math.ceil(math.sqrt(height * width / max_pixels)))
    usable_height = (height // stride) * stride
    usable_width = (width // stride) * stride
    if usable_height <= 0 or usable_width <= 0:
        return np.ascontiguousarray(pixels), 1
    blocks = np.ascontiguousarray(pixels)[:usable_height, :usable_width].reshape(
        usable_height // stride, stride, usable_width // stride, stride
    )
    return np.ascontiguousarray(blocks.min(axis=(1, 3))), stride


def build_pcd_map_preview(
    output_root: Path,
    source_name: str,
    config: MapExportConfig,
    max_pixels: int = PREVIEW_MAX_PIXELS,
) -> tuple[bytes, dict[str, float | int | str]]:
    """Slice one ``data/pcd`` file and return a browser preview without writing.

    The name is validated before it becomes a path and the resolved file must
    stay inside ``data/pcd``, so traversal names cannot reach the filesystem.
    The source PCD is only read; a conversion that follows the preview repeats
    the same calculation and writes the real bundle.
    """
    source_name = sanitize_map_name(source_name)
    source_pcd = resolve_pcd_input(Path(output_root) / "pcd", source_name)
    points = read_binary_pcd(source_pcd)
    if len(points) < 3:
        raise ValueError("PCD 中的有效 XYZ 点不足 3 个，无法生成二维地图")
    pixels, metadata = slice_points_to_occupancy(points, config)
    pooled, stride = pool_occupancy_pixels(pixels, max_pixels)
    metadata.update({
        "source_name": source_name,
        "point_count": int(len(points)),
        "occupied_cells": int(np.count_nonzero(pixels == 0)),
        "preview_width": int(pooled.shape[1]),
        "preview_height": int(pooled.shape[0]),
        "preview_stride": int(stride),
        "preview_occupied_cells": int(np.count_nonzero(pooled == 0)),
    })
    return encode_pgm(pooled, f"preview of {source_name}.pcd"), metadata


def write_occupancy_map(
    pgm_path: Path,
    yaml_path: Path,
    points: np.ndarray,
    config: MapExportConfig,
) -> dict[str, float | int | str]:
    """Slice PointXYZ by height and produce a Nav2-compatible PGM/YAML pair."""
    pixels, metadata = slice_points_to_occupancy(points, config)
    pgm_path, yaml_path = Path(pgm_path), Path(yaml_path)
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    pgm_tmp = pgm_path.with_name(f".{pgm_path.name}.tmp")
    yaml_tmp = yaml_path.with_name(f".{yaml_path.name}.tmp")
    try:
        with pgm_tmp.open("wb") as stream:
            stream.write(encode_pgm(pixels))
            stream.flush()
            os.fsync(stream.fileno())
        yaml_text = (
            f"image: {pgm_path.name}\n"
            "mode: trinary\n"
            f"resolution: {config.resolution:.8g}\n"
            f"origin: [{metadata['origin_x']:.8g}, {metadata['origin_y']:.8g}, 0.0]\n"
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

    return metadata


def export_pcd_session(
    output_root: Path,
    map_name: str,
    points: np.ndarray,
    progress: Optional[Callable[[float, str], None]] = None,
) -> dict[str, object]:
    """Write a finished session's accumulated cloud as its 3-D PCD output.

    The 2-D occupancy map is deliberately not produced here.  ``结束并保存`` and
    the offline PCD -to- map conversion are separate operator steps, so the
    height slice can be tuned and previewed before anything is written.
    """
    name = sanitize_map_name(map_name)
    output_root = Path(output_root).resolve()
    if len(points) < 3:
        raise ValueError("累计点数不足，至少需要 3 个点才能保存")
    pcd = output_root / "pcd" / f"{name}.pcd"
    if progress:
        progress(0.3, "正在写入三维 PCD")
    write_binary_pcd(pcd, points)
    if progress:
        progress(1.0, "点云保存完成")
    return {"pcd": str(pcd), "point_count": int(len(points))}
