"""Coordinate-frame alignment for a saved PCD/PGM/terrain map bundle.

The selected pose describes the new ``map`` origin and +X axis in the map's
current coordinates.  Alignment applies the inverse planar transform to every
artifact, then resamples the raster layers onto an axis-aligned grid.  This is
important for the HW terrain server, which supports origin_x/origin_y but no
grid-origin yaw.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import uuid
from typing import Any

import numpy as np

try:
    from .mapping_core import read_binary_pcd, sanitize_map_name, write_binary_pcd
    from .terrain_core import (
        EditorMap,
        LAYER_OCCUPANCY,
        LAYER_TERRAIN,
        MapMetadata,
        load_occupancy_editor_map,
        load_terrain_editor_map,
        map_paths,
        read_map_yaml,
        replace_map_origin,
        write_pgm,
        write_terrain_msgpack,
    )
except ImportError:
    from mapping_core import read_binary_pcd, sanitize_map_name, write_binary_pcd
    from terrain_core import (
        EditorMap,
        LAYER_OCCUPANCY,
        LAYER_TERRAIN,
        MapMetadata,
        load_occupancy_editor_map,
        load_terrain_editor_map,
        map_paths,
        read_map_yaml,
        replace_map_origin,
        write_pgm,
        write_terrain_msgpack,
    )


FRAME_ID = "map"


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _rotation(yaw: float) -> np.ndarray:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array(((cosine, -sine), (sine, cosine)), dtype=np.float64)


def _transform_dict(x: float, y: float, yaw: float) -> dict[str, float]:
    return {"x": float(x), "y": float(y), "z": 0.0, "yaw": float(_normalize_angle(yaw))}


def _compose(after: dict[str, float], before: dict[str, float]) -> dict[str, float]:
    """Return ``after ∘ before`` for planar coordinate transforms."""
    translated = _rotation(after["yaw"]) @ np.array((before["x"], before["y"]), dtype=np.float64)
    translated += np.array((after["x"], after["y"]), dtype=np.float64)
    return _transform_dict(translated[0], translated[1], after["yaw"] + before["yaw"])


def _inverse(transform: dict[str, float]) -> dict[str, float]:
    yaw = -transform["yaw"]
    translation = -(_rotation(yaw) @ np.array((transform["x"], transform["y"]), dtype=np.float64))
    return _transform_dict(translation[0], translation[1], yaw)


def frame_metadata_path(output_root: Path, map_name: str) -> Path:
    name = sanitize_map_name(map_name)
    return Path(output_root).resolve() / "map" / f"{name}_frame.json"


def _identity_metadata(source_frame: str) -> dict[str, Any]:
    source = source_frame.strip() or "odom"
    if len(source) > 128 or any(ord(character) < 32 for character in source):
        raise ValueError("点云源坐标系名称无效")
    identity = _transform_dict(0.0, 0.0, 0.0)
    return {
        "format": "mapping_web_ui/map-frame-v1",
        "frame_id": FRAME_ID,
        "source_frame": source,
        "revision": 0,
        "source_to_map": identity,
        "map_to_source": identity.copy(),
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def initialize_frame_metadata(output_root: Path, map_name: str, source_frame: str) -> Path:
    """Record the identity convention used by a newly exported map bundle."""
    path = frame_metadata_path(output_root, map_name)
    _write_json_atomic(path, _identity_metadata(source_frame))
    return path


def _load_metadata(output_root: Path, map_name: str, source_frame: str = "odom") -> dict[str, Any]:
    path = frame_metadata_path(output_root, map_name)
    if not path.is_file():
        return _identity_metadata(source_frame)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        transform = payload["source_to_map"]
        normalized = _transform_dict(
            float(transform["x"]),
            float(transform["y"]),
            float(transform["yaw"]),
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError(f"坐标系元数据损坏：{path.name}") from error
    payload["source_to_map"] = normalized
    payload["map_to_source"] = _inverse(normalized)
    payload["source_frame"] = str(payload.get("source_frame") or source_frame)
    payload["frame_id"] = FRAME_ID
    payload["revision"] = int(payload.get("revision", 0))
    return payload


def map_frame_status(output_root: Path, map_name: str) -> dict[str, Any]:
    name = sanitize_map_name(map_name)
    root = Path(output_root).resolve()
    pcd_path = root / "pcd" / f"{name}.pcd"
    metadata_path = frame_metadata_path(root, name)
    result: dict[str, Any] = {
        "has_pcd": pcd_path.is_file(),
        "has_frame_metadata": metadata_path.is_file(),
        "frame_id": FRAME_ID,
    }
    if metadata_path.is_file():
        metadata = _load_metadata(root, name)
        result.update({
            "source_frame": metadata["source_frame"],
            "frame_revision": metadata["revision"],
            "source_to_map": metadata["source_to_map"],
        })
    return result


def _target_geometry(source: MapMetadata, origin: np.ndarray, heading: float) -> MapMetadata:
    local_corners = np.array(
        (
            (0.0, 0.0),
            (source.width * source.resolution, 0.0),
            (0.0, source.height * source.resolution),
            (source.width * source.resolution, source.height * source.resolution),
        ),
        dtype=np.float64,
    )
    current_corners = local_corners @ _rotation(source.origin_yaw).T
    current_corners += np.array((source.origin_x, source.origin_y), dtype=np.float64)
    target_corners = (current_corners - origin) @ _rotation(-heading).T
    lower = target_corners.min(axis=0)
    extent = target_corners.max(axis=0) - lower
    size = np.maximum(
        1,
        np.ceil(extent / source.resolution - 1e-9).astype(np.int64),
    )
    metadata = MapMetadata(
        int(size[0]),
        int(size[1]),
        source.resolution,
        float(lower[0]),
        float(lower[1]),
        0.0,
    )
    metadata.validate()
    return metadata


def _resample_map(
    source: EditorMap,
    target: MapMetadata,
    selected_origin: np.ndarray,
    selected_heading: float,
) -> EditorMap:
    source.validate()
    fill = 254 if source.layer == LAYER_OCCUPANCY else 0
    values = np.full((target.height, target.width), fill, dtype=np.uint8)
    directions = np.zeros_like(values) if source.layer == LAYER_TERRAIN else None
    source_values = source.values.reshape(source.metadata.height, source.metadata.width)
    source_directions = (
        source.direction.reshape(source.metadata.height, source.metadata.width)
        if source.direction is not None else None
    )
    target_x = target.origin_x + (np.arange(target.width, dtype=np.float64) + 0.5) * target.resolution
    rotation_to_current = _rotation(selected_heading)
    rotation_to_source_grid = _rotation(-source.metadata.origin_yaw)
    source_translation = np.array((source.metadata.origin_x, source.metadata.origin_y), dtype=np.float64)

    for target_y in range(target.height):
        y_value = target.origin_y + (target_y + 0.5) * target.resolution
        target_points = np.column_stack((target_x, np.full(target.width, y_value)))
        current_points = target_points @ rotation_to_current.T + selected_origin
        local_points = (current_points - source_translation) @ rotation_to_source_grid.T
        source_x = np.floor(local_points[:, 0] / source.metadata.resolution).astype(np.int64)
        source_y = np.floor(local_points[:, 1] / source.metadata.resolution).astype(np.int64)
        valid = (
            (source_x >= 0) & (source_x < source.metadata.width)
            & (source_y >= 0) & (source_y < source.metadata.height)
        )
        values[target_y, valid] = source_values[source_y[valid], source_x[valid]]
        if directions is not None and source_directions is not None:
            directions[target_y, valid] = source_directions[source_y[valid], source_x[valid]]

    flat_values = np.ascontiguousarray(values.reshape(-1))
    flat_directions = None
    if directions is not None:
        directional = flat_values >= 2
        old_angle = directions.reshape(-1).astype(np.float64) / 255.0 * (2.0 * math.pi)
        rotated = np.mod(old_angle - selected_heading, 2.0 * math.pi)
        encoded = np.rint(rotated / (2.0 * math.pi) * 255.0).astype(np.uint8)
        encoded[~directional] = 0
        flat_directions = np.ascontiguousarray(encoded)
    result = EditorMap(source.layer, target, flat_values, flat_directions)
    result.validate()
    return result


def _transactional_replace(pairs: list[tuple[Path, Path]]) -> None:
    token = f"frame-{os.getpid()}-{threading.get_ident()}-{uuid.uuid4().hex}"
    records: list[tuple[Path, Path, bool]] = []
    try:
        for target, temporary in pairs:
            backup = target.with_name(f".{target.name}.{token}.bak")
            existed = target.exists()
            if existed:
                os.replace(target, backup)
            records.append((target, backup, existed))
            os.replace(temporary, target)
    except Exception:
        for target, backup, existed in reversed(records):
            if target.exists():
                target.unlink()
            if existed and backup.exists():
                os.replace(backup, target)
        raise
    finally:
        for _target, backup, _existed in records:
            backup.unlink(missing_ok=True)
        for _target, temporary in pairs:
            temporary.unlink(missing_ok=True)


def align_map_frame(
    output_root: Path,
    map_name: str,
    origin_x: float,
    origin_y: float,
    heading_yaw: float,
) -> dict[str, Any]:
    """Align a complete saved map bundle to a user-defined planar frame."""
    name = sanitize_map_name(map_name)
    values = (float(origin_x), float(origin_y), float(heading_yaw))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("map 原点和朝向必须是有限数值")
    heading = _normalize_angle(values[2])
    origin = np.array(values[:2], dtype=np.float64)
    root = Path(output_root).resolve()
    yaml_path, _default_pgm, terrain_path = map_paths(root, name)
    pcd_path = root / "pcd" / f"{name}.pcd"
    metadata_path = frame_metadata_path(root, name)
    if not yaml_path.is_file():
        raise FileNotFoundError(f"找不到二维地图：{name}.yaml")
    if not pcd_path.is_file():
        raise FileNotFoundError(f"找不到对应点云：{name}.pcd")

    image_path, _resolution, _ox, _oy, _occupied, _negate = read_map_yaml(yaml_path)
    if not image_path.is_file():
        raise FileNotFoundError(f"找不到二维图像：{image_path.name}")
    occupancy = load_occupancy_editor_map(yaml_path)
    target = _target_geometry(occupancy.metadata, origin, heading)
    transformed_occupancy = _resample_map(occupancy, target, origin, heading)
    terrain = None
    if terrain_path.is_file():
        source_terrain = load_terrain_editor_map(terrain_path, yaml_path=yaml_path)
        if (
            source_terrain.metadata.width != occupancy.metadata.width
            or source_terrain.metadata.height != occupancy.metadata.height
            or not math.isclose(source_terrain.metadata.resolution, occupancy.metadata.resolution)
        ):
            raise ValueError("terrain 与二维 PGM 的尺寸或分辨率不一致，不能成组变换")
        terrain = _resample_map(source_terrain, target, origin, heading)

    points = read_binary_pcd(pcd_path)
    transformed_points = points.copy()
    transformed_points[:, :2] = (points[:, :2].astype(np.float64) - origin) @ _rotation(-heading).T

    old_metadata = _load_metadata(root, name)
    relative_source_to_map = _transform_dict(
        *(-(_rotation(-heading) @ origin)),
        -heading,
    )
    source_to_map = _compose(relative_source_to_map, old_metadata["source_to_map"])
    metadata = {
        "format": "mapping_web_ui/map-frame-v1",
        "frame_id": FRAME_ID,
        "source_frame": old_metadata["source_frame"],
        "revision": int(old_metadata.get("revision", 0)) + 1,
        "source_to_map": source_to_map,
        "map_to_source": _inverse(source_to_map),
        "last_definition": {
            "origin_in_previous_frame": {"x": values[0], "y": values[1]},
            "x_axis_yaw_in_previous_frame": heading,
        },
        "grid": {
            "origin_x": target.origin_x,
            "origin_y": target.origin_y,
            "yaw": 0.0,
            "width": target.width,
            "height": target.height,
            "resolution": target.resolution,
        },
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }

    token = uuid.uuid4().hex
    pcd_temp = pcd_path.with_name(f".{pcd_path.name}.{token}.tmp")
    pgm_temp = image_path.with_name(f".{image_path.name}.{token}.tmp")
    yaml_temp = yaml_path.with_name(f".{yaml_path.name}.{token}.tmp")
    terrain_temp = terrain_path.with_name(f".{terrain_path.name}.{token}.tmp")
    metadata_temp = metadata_path.with_name(f".{metadata_path.name}.{token}.tmp")
    pairs: list[tuple[Path, Path]] = []
    try:
        write_binary_pcd(pcd_temp, transformed_points)
        pixels_top_down = np.flipud(transformed_occupancy.values.reshape(target.height, target.width))
        write_pgm(pgm_temp, pixels_top_down)
        yaml_text = replace_map_origin(
            yaml_path.read_text(encoding="utf-8"),
            target.origin_x,
            target.origin_y,
            0.0,
        )
        with yaml_temp.open("w", encoding="utf-8") as stream:
            stream.write(yaml_text)
            stream.flush()
            os.fsync(stream.fileno())
        with metadata_temp.open("w", encoding="utf-8") as stream:
            json.dump(metadata, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        pairs.extend(((pcd_path, pcd_temp), (image_path, pgm_temp), (yaml_path, yaml_temp)))
        if terrain is not None:
            write_terrain_msgpack(terrain_temp, terrain)
            pairs.append((terrain_path, terrain_temp))
        pairs.append((metadata_path, metadata_temp))
        _transactional_replace(pairs)
    finally:
        for temporary in (pcd_temp, pgm_temp, yaml_temp, terrain_temp, metadata_temp):
            temporary.unlink(missing_ok=True)

    return {
        "map_name": name,
        "frame_id": FRAME_ID,
        "source_frame": metadata["source_frame"],
        "revision": metadata["revision"],
        "source_to_map": source_to_map,
        "map_to_source": metadata["map_to_source"],
        "grid": metadata["grid"],
        "point_count": int(len(transformed_points)),
        "terrain_transformed": terrain is not None,
        "metadata": str(metadata_path),
    }
