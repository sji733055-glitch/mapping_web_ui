"""ROS-independent ROGMap projection-layer diagnostics.

The live ROGMap visualization API publishes the final four-way type grid and
the occupied height span separately.  This module aligns those products with
the occupied voxel cloud, reconstructs the vertical occupancy ratio when the
published data really covers both ends of a column, and packs a compact frame
for the browser.  It deliberately does not claim a ratio when either endpoint
is missing: ``/rog_map/occupied`` can be clipped by the visualization box or
the decay active list.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct

import numpy as np


ROGMAP_MAGIC = b"ROG1"
ROGMAP_VERSION = 1
ROGMAP_HEADER = struct.Struct("<4sHHIIdddddd")
ROGMAP_RECORD = struct.Struct("<bBBHfff")
ROGMAP_RECORD_DTYPE = np.dtype([
    ("cell_type", "i1"),
    ("reason", "u1"),
    ("flags", "u1"),
    ("occupied_count", "<u2"),
    ("occupied_z_max", "<f4"),
    ("height_delta", "<f4"),
    ("occupancy_ratio", "<f4"),
], align=False)
if ROGMAP_RECORD_DTYPE.itemsize != ROGMAP_RECORD.size:
    raise RuntimeError("ROGMap record dtype does not match the wire protocol")

FLAG_HEIGHT_VALID = 1 << 0
FLAG_RATIO_VALID = 1 << 1

REASON_INSUFFICIENT = 0
REASON_EMPTY = 1
REASON_THIN_SURFACE = 2
REASON_SOLID_WALL = 3
REASON_HOLLOW_TUNNEL = 4
REASON_AMBIGUOUS = 5
REASON_RATIO_UNAVAILABLE = 6


@dataclass(frozen=True)
class ProjectionConfig:
    """The six thresholds used by ROGMap ``classifyCell()``."""

    surface_height_delta_max: float = 0.20
    wall_height_delta_min: float = 0.80
    wall_occupancy_ratio_min: float = 0.90
    tunnel_height_delta_min: float = 0.25
    tunnel_height_delta_max: float = 0.40
    tunnel_occupancy_ratio_max: float = 0.45

    def validate(self) -> None:
        values = (
            self.surface_height_delta_max,
            self.wall_height_delta_min,
            self.wall_occupancy_ratio_min,
            self.tunnel_height_delta_min,
            self.tunnel_height_delta_max,
            self.tunnel_occupancy_ratio_max,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("ROGMap 投影分类参数必须是有限数值")
        if self.surface_height_delta_max < 0.0:
            raise ValueError("surface_height_delta_max 不能为负数")
        if self.wall_height_delta_min < 0.0:
            raise ValueError("wall_height_delta_min 不能为负数")
        if not 0.0 <= self.wall_occupancy_ratio_min <= 1.0:
            raise ValueError("wall_occupancy_ratio_min 必须在 0–1 之间")
        if self.tunnel_height_delta_min < 0.0 or self.tunnel_height_delta_max < 0.0:
            raise ValueError("tunnel height delta 不能为负数")
        if not 0.0 <= self.tunnel_occupancy_ratio_max <= 1.0:
            raise ValueError("tunnel_occupancy_ratio_max 必须在 0–1 之间")
        if self.surface_height_delta_max >= self.tunnel_height_delta_min:
            raise ValueError("surface_height_delta_max 必须小于 tunnel_height_delta_min")
        if self.surface_height_delta_max >= self.wall_height_delta_min:
            raise ValueError("surface_height_delta_max 必须小于 wall_height_delta_min")
        if self.tunnel_height_delta_min > self.tunnel_height_delta_max:
            raise ValueError("tunnel_height_delta_min 不能大于 tunnel_height_delta_max")
        if self.tunnel_occupancy_ratio_max >= self.wall_occupancy_ratio_min:
            raise ValueError("tunnel_occupancy_ratio_max 必须小于 wall_occupancy_ratio_min")

    def as_dict(self) -> dict[str, float]:
        return {
            "surface_height_delta_max": self.surface_height_delta_max,
            "wall_height_delta_min": self.wall_height_delta_min,
            "wall_occupancy_ratio_min": self.wall_occupancy_ratio_min,
            "tunnel_height_delta_min": self.tunnel_height_delta_min,
            "tunnel_height_delta_max": self.tunnel_height_delta_max,
            "tunnel_occupancy_ratio_max": self.tunnel_occupancy_ratio_max,
        }


def classify_projection(
    height_delta: np.ndarray,
    ratio: np.ndarray,
    height_valid: np.ndarray,
    ratio_valid: np.ndarray,
    config: ProjectionConfig,
) -> np.ndarray:
    """Vectorized copy of ROGMap's occupied-column classification order.

    Empty and insufficiently observed columns are assigned by the caller from
    the actual type grid.  Here ``height_valid`` means the column has occupied
    height evidence.  A thick column whose ratio cannot be reconstructed gets
    an explicit diagnostic reason instead of being guessed as wall/tunnel.
    """

    config.validate()
    span = np.asarray(height_delta, dtype=np.float64)
    occupancy_ratio = np.asarray(ratio, dtype=np.float64)
    has_height = np.asarray(height_valid, dtype=bool)
    has_ratio = np.asarray(ratio_valid, dtype=bool)
    if not (span.shape == occupancy_ratio.shape == has_height.shape == has_ratio.shape):
        raise ValueError("ROGMap 分类数组尺寸不一致")

    reason = np.full(span.shape, REASON_INSUFFICIENT, dtype=np.uint8)
    reason[has_height] = REASON_RATIO_UNAVAILABLE
    thin = has_height & (span <= config.surface_height_delta_max)
    reason[thin] = REASON_THIN_SURFACE
    eligible = has_height & ~thin & has_ratio
    wall = (
        eligible
        & (span >= config.wall_height_delta_min)
        & (occupancy_ratio >= config.wall_occupancy_ratio_min)
    )
    reason[eligible] = REASON_AMBIGUOUS
    reason[wall] = REASON_SOLID_WALL
    tunnel = (
        eligible
        & ~wall
        & (span >= config.tunnel_height_delta_min)
        & (span <= config.tunnel_height_delta_max)
        & (occupancy_ratio <= config.tunnel_occupancy_ratio_max)
    )
    reason[tunnel] = REASON_HOLLOW_TUNNEL
    return reason


@dataclass(frozen=True)
class ProjectionSnapshot:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    type_stamp: float
    height_stamp: float
    occupied_stamp: float
    cell_type: np.ndarray
    reason: np.ndarray
    flags: np.ndarray
    occupied_count: np.ndarray
    occupied_z_max: np.ndarray
    height_delta: np.ndarray
    occupancy_ratio: np.ndarray

    @property
    def cell_count(self) -> int:
        return self.width * self.height


def _as_points(points: np.ndarray, columns: int) -> np.ndarray:
    values = np.asarray(points, dtype=np.float64)
    if values.size == 0:
        return np.empty((0, columns), dtype=np.float64)
    if values.ndim != 2 or values.shape[1] < columns:
        raise ValueError(f"点云数组必须至少有 {columns} 列")
    return values[:, :columns]


def _grid_indices(
    points: np.ndarray,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> tuple[np.ndarray, np.ndarray]:
    finite = np.isfinite(points[:, 0]) & np.isfinite(points[:, 1])
    x = np.full(points.shape[0], -1, dtype=np.int64)
    y = np.full(points.shape[0], -1, dtype=np.int64)
    x[finite] = np.floor((points[finite, 0] - origin_x) / resolution).astype(np.int64)
    y[finite] = np.floor((points[finite, 1] - origin_y) / resolution).astype(np.int64)
    valid = finite & (x >= 0) & (x < width) & (y >= 0) & (y < height)
    return y[valid] * width + x[valid], valid


def build_projection_snapshot(
    *,
    cell_type: np.ndarray,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    type_stamp: float,
    height_points: np.ndarray,
    height_stamp: float,
    occupied_points: np.ndarray,
    occupied_stamp: float,
    config: ProjectionConfig,
    synchronization_tolerance: float = 0.25,
) -> ProjectionSnapshot:
    """Align the three published ROGMap diagnostics into one grid snapshot."""

    config.validate()
    if width <= 0 or height <= 0 or width * height > 16_000_000:
        raise ValueError("ROGMap 投影网格尺寸无效")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("ROGMap 投影分辨率必须为正数")
    if not all(math.isfinite(value) for value in (origin_x, origin_y)):
        raise ValueError("ROGMap 投影原点必须是有限数值")
    count = width * height
    types = np.asarray(cell_type, dtype=np.int8).reshape(-1)
    if types.size != count:
        raise ValueError("layer_type 栅格数量与几何尺寸不一致")

    z_max = np.full(count, np.nan, dtype=np.float32)
    span = np.full(count, np.nan, dtype=np.float32)
    ratio = np.full(count, np.nan, dtype=np.float32)
    occupied_count = np.zeros(count, dtype=np.uint16)
    flags = np.zeros(count, dtype=np.uint8)

    height_values = _as_points(height_points, 4)
    if height_values.size:
        indices, valid_xy = _grid_indices(
            height_values, width, height, resolution, origin_x, origin_y
        )
        aligned = height_values[valid_xy]
        valid_measurement = np.isfinite(aligned[:, 2]) & np.isfinite(aligned[:, 3]) & (aligned[:, 3] >= 0.0)
        indices = indices[valid_measurement]
        aligned = aligned[valid_measurement]
        if indices.size:
            z_max[indices] = aligned[:, 2].astype(np.float32)
            span[indices] = aligned[:, 3].astype(np.float32)
            flags[indices] |= FLAG_HEIGHT_VALID

    synchronized = (
        math.isfinite(type_stamp)
        and math.isfinite(height_stamp)
        and math.isfinite(occupied_stamp)
        and abs(type_stamp - height_stamp) <= synchronization_tolerance
        and abs(type_stamp - occupied_stamp) <= synchronization_tolerance
    )
    occupied_values = _as_points(occupied_points, 3)
    if synchronized and occupied_values.size:
        indices, valid_xy = _grid_indices(
            occupied_values, width, height, resolution, origin_x, origin_y
        )
        aligned = occupied_values[valid_xy]
        finite_z = np.isfinite(aligned[:, 2])
        indices = indices[finite_z]
        aligned = aligned[finite_z]
        if indices.size:
            has_height = (flags[indices] & FLAG_HEIGHT_VALID) != 0
            expected_high = z_max[indices].astype(np.float64)
            expected_low = expected_high - span[indices].astype(np.float64)
            tolerance = resolution * 0.55 + 1.0e-6
            inside = (
                has_height
                & (aligned[:, 2] >= expected_low - tolerance)
                & (aligned[:, 2] <= expected_high + tolerance)
            )
            indices = indices[inside]
            z_values = aligned[inside, 2]
            if indices.size:
                # The published occupied cloud has one point per voxel, but
                # quantizing Z makes the reconstruction robust to duplicates.
                z_layer = np.rint(z_values / resolution).astype(np.int64)
                keys = indices.astype(np.int64) * (1 << 32) + (z_layer & 0xFFFFFFFF)
                _, unique_positions = np.unique(keys, return_index=True)
                indices = indices[unique_positions]
                z_values = z_values[unique_positions]

                layer_counts = np.bincount(indices, minlength=count)
                occupied_count[:] = np.minimum(layer_counts, np.iinfo(np.uint16).max).astype(np.uint16)
                observed_low = np.full(count, np.inf, dtype=np.float64)
                observed_high = np.full(count, -np.inf, dtype=np.float64)
                np.minimum.at(observed_low, indices, z_values)
                np.maximum.at(observed_high, indices, z_values)
                height_mask = (flags & FLAG_HEIGHT_VALID) != 0
                endpoint_covered = (
                    height_mask
                    & (occupied_count > 0)
                    & (np.abs(observed_low - (z_max - span)) <= tolerance)
                    & (np.abs(observed_high - z_max) <= tolerance)
                )
                thin = endpoint_covered & (span <= resolution * 0.5)
                ratio[thin] = 1.0
                thick = endpoint_covered & ~thin & (span > 0.0)
                ratio[thick] = np.clip(
                    ((occupied_count[thick].astype(np.float64) - 1.0) * resolution)
                    / span[thick].astype(np.float64),
                    0.0,
                    1.0,
                ).astype(np.float32)
                flags[endpoint_covered] |= FLAG_RATIO_VALID

    # ROGMap assigns ratio=1.0 before returning from the thin-surface branch;
    # that value is authoritative even when the separately published occupied
    # cloud is clipped and cannot be used to recount the column.
    thin_surface = ((flags & FLAG_HEIGHT_VALID) != 0) & (
        span <= config.surface_height_delta_max
    )
    ratio[thin_surface] = 1.0
    flags[thin_surface] |= FLAG_RATIO_VALID

    height_valid = (flags & FLAG_HEIGHT_VALID) != 0
    ratio_valid = (flags & FLAG_RATIO_VALID) != 0
    reason = classify_projection(span, ratio, height_valid, ratio_valid, config)
    without_height = ~height_valid
    reason[without_height & (types == -1)] = REASON_INSUFFICIENT
    reason[without_height & (types == 33)] = REASON_EMPTY
    return ProjectionSnapshot(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        type_stamp=type_stamp,
        height_stamp=height_stamp,
        occupied_stamp=occupied_stamp,
        cell_type=np.ascontiguousarray(types),
        reason=np.ascontiguousarray(reason),
        flags=np.ascontiguousarray(flags),
        occupied_count=np.ascontiguousarray(occupied_count),
        occupied_z_max=np.ascontiguousarray(z_max),
        height_delta=np.ascontiguousarray(span),
        occupancy_ratio=np.ascontiguousarray(ratio),
    )


def encode_projection_snapshot(snapshot: ProjectionSnapshot) -> bytes:
    """Encode a fixed-stride ``ROG1`` frame for HTTP transport."""

    count = snapshot.cell_count
    arrays = (
        snapshot.cell_type,
        snapshot.reason,
        snapshot.flags,
        snapshot.occupied_count,
        snapshot.occupied_z_max,
        snapshot.height_delta,
        snapshot.occupancy_ratio,
    )
    if any(np.asarray(values).reshape(-1).size != count for values in arrays):
        raise ValueError("ROGMap 快照通道长度不一致")
    header = ROGMAP_HEADER.pack(
        ROGMAP_MAGIC,
        ROGMAP_VERSION,
        ROGMAP_RECORD.size,
        snapshot.width,
        snapshot.height,
        snapshot.resolution,
        snapshot.origin_x,
        snapshot.origin_y,
        snapshot.type_stamp,
        snapshot.height_stamp,
        snapshot.occupied_stamp,
    )
    records = np.empty(count, dtype=ROGMAP_RECORD_DTYPE)
    records["cell_type"] = snapshot.cell_type
    records["reason"] = snapshot.reason
    records["flags"] = snapshot.flags
    records["occupied_count"] = snapshot.occupied_count
    records["occupied_z_max"] = snapshot.occupied_z_max
    records["height_delta"] = snapshot.height_delta
    records["occupancy_ratio"] = snapshot.occupancy_ratio
    return header + records.tobytes(order="C")
