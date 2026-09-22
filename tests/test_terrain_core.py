from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np

from backend.terrain_core import (
    EditorMap,
    LAYER_OCCUPANCY,
    LAYER_TERRAIN,
    MapMetadata,
    decode_editor_map,
    encode_editor_map,
    load_occupancy_editor_map,
    load_terrain_editor_map,
    occupancy_to_terrain,
    pack_terrain_msgpack,
    read_pgm,
    save_occupancy_editor_map,
    unpack_terrain_msgpack,
    write_pgm,
    write_terrain_msgpack,
)


class TerrainCoreTests(unittest.TestCase):
    def _write_yaml(self, root: Path, *, negate: int = 0) -> Path:
        path = root / "arena.yaml"
        path.write_text(
            "image: arena.pgm\n"
            "mode: trinary\n"
            "resolution: 0.05\n"
            "origin: [-1.5, 2.25, 0.0]\n"
            f"negate: {negate}\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n",
            encoding="utf-8",
        )
        return path

    def test_editor_protocol_round_trip_for_both_layers(self):
        metadata = MapMetadata(3, 2, 0.05, -1.0, 2.0, 0.25)
        occupancy = EditorMap(
            LAYER_OCCUPANCY,
            metadata,
            np.array([0, 254, 205, 12, 34, 56], dtype=np.uint8),
        )
        decoded = decode_editor_map(encode_editor_map(occupancy))
        self.assertEqual(decoded.layer, LAYER_OCCUPANCY)
        self.assertAlmostEqual(decoded.metadata.origin_yaw, 0.25)
        np.testing.assert_array_equal(decoded.values, occupancy.values)

        terrain = EditorMap(
            LAYER_TERRAIN,
            metadata,
            np.array([0, 1, 2, 3, 4, 6], dtype=np.uint8),
            np.array([99, 88, 64, 128, 200, 255], dtype=np.uint8),
        )
        decoded = decode_editor_map(encode_editor_map(terrain))
        np.testing.assert_array_equal(decoded.values, terrain.values)
        # Non-directional labels must never retain stale direction values.
        np.testing.assert_array_equal(decoded.direction, [0, 0, 64, 128, 200, 255])

    def test_pgm_editor_uses_map_coordinates_and_saves_atomically(self):
        top_down = np.array([[0, 10, 20], [230, 240, 250]], dtype=np.uint8)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_pgm(root / "arena.pgm", top_down)
            yaml_path = self._write_yaml(root)
            editor = load_occupancy_editor_map(yaml_path)
            np.testing.assert_array_equal(editor.values.reshape(2, 3), np.flipud(top_down))
            editor.values[0] = 77
            output = save_occupancy_editor_map(yaml_path, editor)
            self.assertEqual(output, root / "arena.pgm")
            expected = top_down.copy()
            expected[-1, 0] = 77
            np.testing.assert_array_equal(read_pgm(output), expected)
            self.assertFalse((root / ".arena.pgm.tmp").exists())

    def test_occupancy_conversion_honors_negate_without_double_inversion(self):
        pixels = np.array([[0, 255], [255, 0]], dtype=np.uint8)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_pgm(root / "arena.pgm", pixels)
            yaml_path = self._write_yaml(root, negate=0)
            normal = occupancy_to_terrain(yaml_path)
            np.testing.assert_array_equal(normal.values.reshape(2, 2), [[0, 1], [1, 0]])

            yaml_path = self._write_yaml(root, negate=1)
            negated = occupancy_to_terrain(yaml_path)
            np.testing.assert_array_equal(negated.values.reshape(2, 2), [[1, 0], [0, 1]])

    def test_msgpack_round_trip_writes_native_array_channels(self):
        terrain = EditorMap(
            LAYER_TERRAIN,
            MapMetadata(3, 2, 0.1),
            np.array([0, 1, 2, 3, 5, 6], dtype=np.uint8),
            np.array([0, 0, 127, 128, 200, 255], dtype=np.uint8),
        )
        packed = pack_terrain_msgpack(terrain)
        terrain_field = packed.index(b"terrain") + len(b"terrain")
        direction_field = packed.index(b"direction") + len(b"direction")
        self.assertEqual(packed[terrain_field], 0x96)
        self.assertEqual(packed[direction_field], 0x96)
        self.assertIn(b"\xCC\x80", packed)
        self.assertIn(b"\xCC\xFF", packed)
        decoded = unpack_terrain_msgpack(packed)
        np.testing.assert_array_equal(decoded.values, terrain.values)
        np.testing.assert_array_equal(decoded.direction, terrain.direction)

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "arena_terrain.msgpack"
            write_terrain_msgpack(path, terrain)
            loaded = load_terrain_editor_map(path)
            np.testing.assert_array_equal(loaded.values, terrain.values)
            np.testing.assert_array_equal(loaded.direction, terrain.direction)

    def test_msgpack_uses_array16_and_still_reads_legacy_bin_channels(self):
        terrain = EditorMap(
            LAYER_TERRAIN,
            MapMetadata(300, 1, 0.05),
            np.arange(300, dtype=np.uint16).astype(np.uint8) % 7,
            np.zeros(300, dtype=np.uint8),
        )
        packed = pack_terrain_msgpack(terrain)
        terrain_field = packed.index(b"terrain") + len(b"terrain")
        direction_field = packed.index(b"direction") + len(b"direction")
        self.assertEqual(packed[terrain_field], 0xDC)
        self.assertEqual(packed[direction_field], 0xDC)

        legacy = (
            b"\x85"
            b"\xA5width\xD2\x00\x00\x00\x03"
            b"\xA6height\xD2\x00\x00\x00\x02"
            b"\xAAresolution\xCB" + struct.pack(">d", 0.1) +
            b"\xA7terrain\xC4\x06\x00\x01\x02\x03\x05\x06"
            b"\xA9direction\xC4\x06\x00\x00\x7F\x80\xC8\xFF"
        )
        decoded = unpack_terrain_msgpack(legacy)
        np.testing.assert_array_equal(decoded.values, [0, 1, 2, 3, 5, 6])
        np.testing.assert_array_equal(decoded.direction, [0, 0, 127, 128, 200, 255])


if __name__ == "__main__":
    unittest.main()
