"""ROS-independent occupancy and terrain-map editing primitives.

The browser uses a compact, fixed-layout payload while files on disk retain
their native PGM/YAML and MessagePack representations. Terrain files contain
only the uint8 label channel. Legacy direction data is ignored on read.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import struct
from typing import Any

import numpy as np

try:
    from .mapping_core import sanitize_map_name
except ImportError:
    from mapping_core import sanitize_map_name


EDITOR_MAGIC = b"MPE2"
EDITOR_HEADER = struct.Struct("<4sB3xIIdddd")
LEGACY_EDITOR_MAGIC = b"MPE1"
LEGACY_EDITOR_HEADER = struct.Struct("<4sB3xIIddd")
LAYER_OCCUPANCY = 0
LAYER_TERRAIN = 1
MAX_EDITOR_CELLS = 16_000_000
TERRAIN_LABELS = (0, 1, 5, 6, 7)


@dataclass(frozen=True)
class MapMetadata:
    width: int
    height: int
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0

    @property
    def cell_count(self) -> int:
        return self.width * self.height

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("地图宽高必须为正数")
        if self.cell_count > MAX_EDITOR_CELLS:
            raise ValueError(
                f"地图包含 {self.cell_count:,} 个栅格，超过网页编辑上限 {MAX_EDITOR_CELLS:,}"
            )
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("地图分辨率必须是正数")
        if not all(math.isfinite(value) for value in (self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError("地图原点必须是有限数值")


@dataclass
class EditorMap:
    layer: int
    metadata: MapMetadata
    values: np.ndarray

    def validate(self) -> None:
        self.metadata.validate()
        if self.layer not in {LAYER_OCCUPANCY, LAYER_TERRAIN}:
            raise ValueError("未知地图编辑图层")
        values = np.asarray(self.values, dtype=np.uint8)
        if values.size != self.metadata.cell_count:
            raise ValueError("地图栅格数量与宽高不一致")
        self.values = np.ascontiguousarray(values.reshape(-1), dtype=np.uint8)
        if self.layer == LAYER_OCCUPANCY:
            return
        invalid = ~np.isin(self.values, TERRAIN_LABELS)
        if np.any(invalid):
            bad = int(self.values[invalid][0])
            raise ValueError(f"terrain 标签 {bad} 无效；只支持 0、1、5、6、7")


def encode_editor_map(editor_map: EditorMap) -> bytes:
    editor_map.validate()
    meta = editor_map.metadata
    header = EDITOR_HEADER.pack(
        EDITOR_MAGIC,
        editor_map.layer,
        meta.width,
        meta.height,
        meta.resolution,
        meta.origin_x,
        meta.origin_y,
        meta.origin_yaw,
    )
    return header + editor_map.values.tobytes(order="C")


def decode_editor_map(payload: bytes) -> EditorMap:
    if len(payload) < LEGACY_EDITOR_HEADER.size:
        raise ValueError("地图编辑数据头不完整")
    magic = payload[:4]
    if magic == EDITOR_MAGIC:
        if len(payload) < EDITOR_HEADER.size:
            raise ValueError("地图编辑数据头不完整")
        magic, layer, width, height, resolution, origin_x, origin_y, origin_yaw = EDITOR_HEADER.unpack_from(payload)
        header_size = EDITOR_HEADER.size
    elif magic == LEGACY_EDITOR_MAGIC:
        magic, layer, width, height, resolution, origin_x, origin_y = LEGACY_EDITOR_HEADER.unpack_from(payload)
        origin_yaw = 0.0
        header_size = LEGACY_EDITOR_HEADER.size
    else:
        raise ValueError("地图编辑数据 magic 不正确")
    metadata = MapMetadata(width, height, resolution, origin_x, origin_y, origin_yaw)
    metadata.validate()
    if layer not in {LAYER_OCCUPANCY, LAYER_TERRAIN}:
        raise ValueError("未知地图编辑图层")
    expected = header_size + metadata.cell_count
    if len(payload) != expected:
        raise ValueError(f"地图编辑数据长度应为 {expected} 字节，实际为 {len(payload)}")
    start = header_size
    stop = start + metadata.cell_count
    values = np.frombuffer(payload[start:stop], dtype=np.uint8).copy()
    result = EditorMap(layer, metadata, values)
    result.validate()
    return result


def _yaml_scalar(text: str, key: str, required: bool = True) -> str | None:
    match = re.search(rf"^\s*{re.escape(key)}\s*:\s*(.*?)\s*(?:#.*)?$", text, re.MULTILINE)
    if not match:
        if required:
            raise ValueError(f"YAML 缺少 {key} 字段")
        return None
    value = match.group(1).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'\"', "'"}:
        value = value[1:-1]
    return value


def read_map_yaml(yaml_path: Path) -> tuple[Path, float, float, float, float, bool]:
    yaml_path = Path(yaml_path).resolve()
    text = yaml_path.read_text(encoding="utf-8")
    image_name = _yaml_scalar(text, "image")
    resolution = float(_yaml_scalar(text, "resolution"))
    origin_text = _yaml_scalar(text, "origin", required=False) or "[0, 0, 0]"
    origin_match = re.fullmatch(r"\[\s*([^,]+),\s*([^,]+),\s*([^]]+)\s*\]", origin_text)
    if not origin_match:
        raise ValueError("YAML origin 必须是 [x, y, yaw]")
    origin_x, origin_y = float(origin_match.group(1)), float(origin_match.group(2))
    occupied = float(_yaml_scalar(text, "occupied_thresh", required=False) or "0.65")
    negate = bool(int(_yaml_scalar(text, "negate", required=False) or "0"))
    if not 0.0 <= occupied <= 1.0:
        raise ValueError("occupied_thresh 必须在 0–1 之间")
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("YAML resolution 必须是正数")
    image_path = (yaml_path.parent / str(image_name)).resolve()
    try:
        image_path.relative_to(yaml_path.parent)
    except ValueError as error:
        raise ValueError("YAML image 不能指向地图目录之外") from error
    if image_path.suffix.lower() != ".pgm":
        raise ValueError("网页地图编辑器当前只支持 PGM 图像")
    return image_path, resolution, origin_x, origin_y, occupied, negate


def read_map_origin(yaml_path: Path) -> tuple[float, float, float]:
    """Return the full Nav2 image origin pose as ``x, y, yaw``."""
    text = Path(yaml_path).resolve().read_text(encoding="utf-8")
    origin_text = _yaml_scalar(text, "origin", required=False) or "[0, 0, 0]"
    match = re.fullmatch(r"\[\s*([^,]+),\s*([^,]+),\s*([^]]+)\s*\]", origin_text)
    if not match:
        raise ValueError("YAML origin 必须是 [x, y, yaw]")
    values = tuple(float(match.group(index)) for index in range(1, 4))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("YAML origin 必须是有限数值")
    return values


def _format_origin_value(value: float) -> str:
    """Format an origin component compactly while keeping float notation."""
    text = f"{value:.10g}"
    if not any(character in text for character in ".eE"):
        text += ".0"
    return text


def replace_map_origin(yaml_text: str, origin_x: float, origin_y: float, origin_yaw: float = 0.0) -> str:
    """Replace only the YAML origin line while preserving all other settings."""
    if not all(math.isfinite(value) for value in (origin_x, origin_y, origin_yaw)):
        raise ValueError("地图原点必须是有限数值")
    replacement = f"origin: [{_format_origin_value(origin_x)}, {_format_origin_value(origin_y)}, {_format_origin_value(origin_yaw)}]"
    updated, count = re.subn(
        r"^\s*origin\s*:\s*.*?(?:\s+#.*)?$",
        replacement,
        yaml_text,
        count=1,
        flags=re.MULTILINE,
    )
    if count:
        return updated
    suffix = "" if yaml_text.endswith("\n") else "\n"
    return f"{yaml_text}{suffix}{replacement}\n"


def read_pgm(path: Path) -> np.ndarray:
    data = Path(path).read_bytes()
    offset = 0

    def token() -> bytes:
        nonlocal offset
        while offset < len(data):
            if data[offset] == 35:  # '#'
                newline = data.find(b"\n", offset)
                offset = len(data) if newline < 0 else newline + 1
            elif data[offset] in b" \t\r\n\v\f":
                offset += 1
            else:
                break
        start = offset
        while offset < len(data) and data[offset] not in b" \t\r\n\v\f#":
            offset += 1
        if start == offset:
            raise ValueError("PGM 文件头不完整")
        return data[start:offset]

    magic = token()
    if magic not in {b"P2", b"P5"}:
        raise ValueError("只支持 P2/P5 PGM 地图")
    width, height, maximum = int(token()), int(token()), int(token())
    metadata = MapMetadata(width, height, 1.0)
    metadata.validate()
    if maximum <= 0 or maximum > 255:
        raise ValueError("只支持最大灰度值不超过 255 的 PGM")
    count = width * height
    if magic == b"P2":
        pixels = np.fromiter((int(token()) for _ in range(count)), dtype=np.int64, count=count)
        if np.any((pixels < 0) | (pixels > maximum)):
            raise ValueError("PGM 像素超出灰度范围")
        return np.rint(pixels * (255.0 / maximum)).astype(np.uint8).reshape(height, width)
    if offset >= len(data) or data[offset] not in b" \t\r\n\v\f":
        raise ValueError("PGM 二进制数据前缺少分隔符")
    if data[offset:offset + 2] == b"\r\n":
        offset += 2
    else:
        offset += 1
    raw = data[offset:offset + count]
    if len(raw) != count:
        raise ValueError(f"PGM 像素数据应为 {count} 字节，实际为 {len(raw)}")
    pixels = np.frombuffer(raw, dtype=np.uint8).copy().reshape(height, width)
    if maximum != 255:
        pixels = np.rint(pixels.astype(np.float64) * (255.0 / maximum)).astype(np.uint8)
    return pixels


def _atomic_write(path: Path, payload: bytes) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        with temporary.open("wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def write_pgm(path: Path, pixels_top_down: np.ndarray) -> None:
    pixels = np.asarray(pixels_top_down, dtype=np.uint8)
    if pixels.ndim != 2 or not pixels.size:
        raise ValueError("PGM 像素必须是非空二维数组")
    height, width = pixels.shape
    MapMetadata(width, height, 1.0).validate()
    header = f"P5\n# edited by mapping_web_ui\n{width} {height}\n255\n".encode("ascii")
    _atomic_write(Path(path), header + np.ascontiguousarray(pixels).tobytes(order="C"))


def load_occupancy_editor_map(yaml_path: Path) -> EditorMap:
    image_path, resolution, origin_x, origin_y, _occupied, _negate = read_map_yaml(yaml_path)
    _origin_x, _origin_y, origin_yaw = read_map_origin(yaml_path)
    pixels_top_down = read_pgm(image_path)
    height, width = pixels_top_down.shape
    # Browser/editor arrays use map coordinates: row zero is the bottom row.
    values = np.ascontiguousarray(np.flipud(pixels_top_down).reshape(-1), dtype=np.uint8)
    result = EditorMap(
        LAYER_OCCUPANCY,
        MapMetadata(width, height, resolution, origin_x, origin_y, origin_yaw),
        values,
    )
    result.validate()
    return result


def save_occupancy_editor_map(yaml_path: Path, editor_map: EditorMap) -> Path:
    editor_map.validate()
    if editor_map.layer != LAYER_OCCUPANCY:
        raise ValueError("保存 PGM 时必须提供二维占据图图层")
    image_path, resolution, origin_x, origin_y, _occupied, _negate = read_map_yaml(yaml_path)
    _origin_x, _origin_y, origin_yaw = read_map_origin(yaml_path)
    expected = MapMetadata(
        editor_map.metadata.width,
        editor_map.metadata.height,
        resolution,
        origin_x,
        origin_y,
        origin_yaw,
    )
    if (editor_map.metadata.width, editor_map.metadata.height) != (expected.width, expected.height):
        raise ValueError("不能通过编辑接口改变 PGM 地图尺寸")
    pixels = editor_map.values.reshape(expected.height, expected.width)
    write_pgm(image_path, np.flipud(pixels))
    return image_path


def occupancy_to_terrain(yaml_path: Path) -> EditorMap:
    image_path, resolution, origin_x, origin_y, occupied_thresh, negate = read_map_yaml(yaml_path)
    _origin_x, _origin_y, origin_yaw = read_map_origin(yaml_path)
    pixels = read_pgm(image_path)
    probability = pixels.astype(np.float32) / 255.0 if negate else (255.0 - pixels) / 255.0
    obstacle_top_down = probability >= occupied_thresh
    terrain = np.flipud(obstacle_top_down).astype(np.uint8).reshape(-1)
    height, width = pixels.shape
    result = EditorMap(
        LAYER_TERRAIN,
        MapMetadata(width, height, resolution, origin_x, origin_y, origin_yaw),
        terrain,
    )
    result.validate()
    return result


def _pack_string(value: str) -> bytes:
    encoded = value.encode("utf-8")
    if len(encoded) < 32:
        return bytes((0xA0 | len(encoded),)) + encoded
    if len(encoded) <= 0xFF:
        return b"\xD9" + bytes((len(encoded),)) + encoded
    return b"\xDA" + struct.pack(">H", len(encoded)) + encoded


def _pack_uint8_array(values: np.ndarray) -> bytes:
    """Encode a uint8 vector as a MessagePack ARRAY for the HW map server.

    Values through 127 use positive fixint; larger values use the uint8 marker.
    Building the element stream with NumPy avoids a Python loop for multi-
    million-cell maps while preserving the exact ARRAY layout consumed by
    ``mas_nav_2027_native``'s ``load_terrain_msgpack()``.
    """
    raw = np.ascontiguousarray(values, dtype=np.uint8).reshape(-1)
    size = len(raw)
    if size < 16:
        prefix = bytes((0x90 | size,))
    elif size <= 0xFFFF:
        prefix = b"\xDC" + struct.pack(">H", size)
    else:
        prefix = b"\xDD" + struct.pack(">I", size)

    high = raw > 0x7F
    if not bool(np.any(high)):
        return prefix + raw.tobytes(order="C")
    positions = np.arange(size, dtype=np.int64) + np.cumsum(high, dtype=np.int64)
    encoded = np.empty(size + int(np.count_nonzero(high)), dtype=np.uint8)
    encoded[positions] = raw
    encoded[positions[high] - 1] = 0xCC
    return prefix + encoded.tobytes(order="C")


def pack_terrain_msgpack(editor_map: EditorMap) -> bytes:
    editor_map.validate()
    if editor_map.layer != LAYER_TERRAIN:
        raise ValueError("只有 terrain 图层可以写入 msgpack")
    items = (
        ("width", b"\xD2" + struct.pack(">i", editor_map.metadata.width)),
        ("height", b"\xD2" + struct.pack(">i", editor_map.metadata.height)),
        ("resolution", b"\xCB" + struct.pack(">d", editor_map.metadata.resolution)),
        ("terrain", _pack_uint8_array(editor_map.values)),
    )
    return b"\x84" + b"".join(_pack_string(key) + value for key, value in items)


class _MessagePackReader:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.offset = 0

    def _take(self, size: int) -> bytes:
        stop = self.offset + size
        if size < 0 or stop > len(self.data):
            raise ValueError("msgpack 数据意外结束")
        value = self.data[self.offset:stop]
        self.offset = stop
        return value

    def _number(self, fmt: str) -> int | float:
        return struct.unpack(fmt, self._take(struct.calcsize(fmt)))[0]

    def read(self, depth: int = 0) -> Any:
        if depth > 8:
            raise ValueError("msgpack 嵌套过深")
        code = self._take(1)[0]
        if code <= 0x7F:
            return code
        if code >= 0xE0:
            return code - 256
        if 0xA0 <= code <= 0xBF:
            return self._take(code & 0x1F).decode("utf-8")
        if 0x90 <= code <= 0x9F:
            size = code & 0x0F
            result = bytearray(size)
            for index in range(size):
                value = self.read(depth + 1)
                if not isinstance(value, int) or not 0 <= value <= 255:
                    raise ValueError("terrain msgpack 数组必须只包含 uint8")
                result[index] = value
            return bytes(result)
        if 0x80 <= code <= 0x8F:
            return {self.read(depth + 1): self.read(depth + 1) for _ in range(code & 0x0F)}
        if code == 0xC4:
            return self._take(int(self._number(">B")))
        if code == 0xC5:
            return self._take(int(self._number(">H")))
        if code == 0xC6:
            return self._take(int(self._number(">I")))
        if code == 0xCA:
            return self._number(">f")
        if code == 0xCB:
            return self._number(">d")
        if code == 0xCC:
            return self._number(">B")
        if code == 0xCD:
            return self._number(">H")
        if code == 0xCE:
            return self._number(">I")
        if code == 0xCF:
            return self._number(">Q")
        if code == 0xD0:
            return self._number(">b")
        if code == 0xD1:
            return self._number(">h")
        if code == 0xD2:
            return self._number(">i")
        if code == 0xD3:
            return self._number(">q")
        if code == 0xD9:
            return self._take(int(self._number(">B"))).decode("utf-8")
        if code == 0xDA:
            return self._take(int(self._number(">H"))).decode("utf-8")
        if code == 0xDB:
            return self._take(int(self._number(">I"))).decode("utf-8")
        if code in {0xDC, 0xDD}:
            size = int(self._number(">H" if code == 0xDC else ">I"))
            if size > MAX_EDITOR_CELLS:
                raise ValueError("msgpack 数组超过网页编辑上限")
            result = bytearray(size)
            for index in range(size):
                value = self.read(depth + 1)
                if not isinstance(value, int) or not 0 <= value <= 255:
                    raise ValueError("terrain msgpack 数组必须只包含 uint8")
                result[index] = value
            return bytes(result)
        if code in {0xDE, 0xDF}:
            size = int(self._number(">H" if code == 0xDE else ">I"))
            if size > 64:
                raise ValueError("msgpack MAP 字段过多")
            return {self.read(depth + 1): self.read(depth + 1) for _ in range(size)}
        raise ValueError(f"不支持的 msgpack 类型 0x{code:02x}")


def unpack_terrain_msgpack(
    payload: bytes,
    *,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_yaw: float = 0.0,
) -> EditorMap:
    reader = _MessagePackReader(payload)
    data = reader.read()
    if reader.offset != len(payload):
        raise ValueError("msgpack 根对象之后还有多余数据")
    if not isinstance(data, dict):
        raise ValueError("msgpack 根对象必须是 MAP")
    try:
        width = int(data["width"])
        height = int(data["height"])
        resolution = float(data["resolution"])
        terrain_value = data["terrain"]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("msgpack 缺少 width/height/resolution/terrain") from error
    terrain = np.frombuffer(terrain_value, dtype=np.uint8).copy() if isinstance(terrain_value, bytes) else np.asarray(terrain_value, dtype=np.uint8)
    result = EditorMap(
        LAYER_TERRAIN,
        MapMetadata(width, height, resolution, origin_x, origin_y, origin_yaw),
        terrain,
    )
    result.validate()
    return result


def load_terrain_editor_map(path: Path, *, yaml_path: Path | None = None) -> EditorMap:
    origin_x = origin_y = origin_yaw = 0.0
    if yaml_path is not None and Path(yaml_path).is_file():
        _image, _resolution, origin_x, origin_y, _occupied, _negate = read_map_yaml(yaml_path)
        _origin_x, _origin_y, origin_yaw = read_map_origin(yaml_path)
    return unpack_terrain_msgpack(
        Path(path).read_bytes(),
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
    )


def write_terrain_msgpack(path: Path, editor_map: EditorMap) -> None:
    _atomic_write(Path(path), pack_terrain_msgpack(editor_map))


def map_paths(output_root: Path, map_name: str) -> tuple[Path, Path, Path]:
    name = sanitize_map_name(map_name)
    root = Path(output_root).resolve() / "map"
    return root / f"{name}.yaml", root / f"{name}.pgm", root / f"{name}_terrain.msgpack"


def list_editable_maps(output_root: Path) -> list[dict[str, Any]]:
    map_dir = Path(output_root).resolve() / "map"
    map_dir.mkdir(parents=True, exist_ok=True)
    names = {path.stem for path in map_dir.glob("*.yaml") if path.is_file()}
    names.update(
        path.name.removesuffix("_terrain.msgpack")
        for path in map_dir.glob("*_terrain.msgpack") if path.is_file()
    )
    result: list[dict[str, Any]] = []
    for name in sorted(names, key=str.casefold):
        try:
            safe_name = sanitize_map_name(name)
        except ValueError:
            continue
        yaml_path, default_pgm, terrain_path = map_paths(output_root, safe_name)
        pgm_path = default_pgm
        resolution = None
        width = height = None
        error = ""
        if yaml_path.is_file():
            try:
                pgm_path, resolution, _ox, _oy, _occupied, _negate = read_map_yaml(yaml_path)
                if pgm_path.is_file():
                    pixels = read_pgm(pgm_path)
                    height, width = pixels.shape
            except (OSError, ValueError) as exception:
                error = str(exception)
        result.append({
            "name": safe_name,
            "has_yaml": yaml_path.is_file(),
            "has_occupancy": yaml_path.is_file() and pgm_path.is_file(),
            "has_terrain": terrain_path.is_file(),
            "width": width,
            "height": height,
            "resolution": resolution,
            "error": error,
        })
    return result
