import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from backend.map_frame_core import align_map_frame, initialize_frame_metadata, map_frame_status
from backend.mapping_core import read_binary_pcd, write_binary_pcd
from backend.terrain_core import (
    EditorMap,
    LAYER_TERRAIN,
    MapMetadata,
    load_occupancy_editor_map,
    load_terrain_editor_map,
    read_map_origin,
    write_pgm,
    write_terrain_msgpack,
)


class MapFrameCoreTests(unittest.TestCase):
    def _write_bundle(self, root: Path) -> np.ndarray:
        (root / "pcd").mkdir(parents=True)
        (root / "map").mkdir(parents=True)
        points = np.array(
            [[0.0, 0.0, 0.2], [2.0, 0.0, 0.4], [0.0, 3.0, 0.6], [1.0, 2.0, 0.8]],
            dtype=np.float32,
        )
        write_binary_pcd(root / "pcd" / "arena.pcd", points)
        pixels = np.array(
            [[0, 20], [40, 60], [80, 100]],
            dtype=np.uint8,
        )
        write_pgm(root / "map" / "arena.pgm", pixels)
        (root / "map" / "arena.yaml").write_text(
            "image: arena.pgm\n"
            "mode: trinary\n"
            "resolution: 1.0\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n",
            encoding="utf-8",
        )
        terrain = EditorMap(
            LAYER_TERRAIN,
            MapMetadata(2, 3, 1.0),
            np.full(6, 5, dtype=np.uint8),
        )
        write_terrain_msgpack(root / "map" / "arena_terrain.msgpack", terrain)
        return points

    def test_alignment_transforms_pcd_rasters_and_metadata_together(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            points = self._write_bundle(root)
            initialize_frame_metadata(root, "arena", "odom")

            result = align_map_frame(root, "arena", 0.0, 0.0, math.pi / 2)

            transformed = read_binary_pcd(root / "pcd" / "arena.pcd")
            expected = points.copy()
            expected[:, 0] = points[:, 1]
            expected[:, 1] = -points[:, 0]
            np.testing.assert_allclose(transformed, expected, atol=1e-6)

            occupancy = load_occupancy_editor_map(root / "map" / "arena.yaml")
            terrain = load_terrain_editor_map(
                root / "map" / "arena_terrain.msgpack",
                yaml_path=root / "map" / "arena.yaml",
            )
            self.assertEqual((occupancy.metadata.width, occupancy.metadata.height), (3, 2))
            self.assertEqual((terrain.metadata.width, terrain.metadata.height), (3, 2))
            self.assertEqual(read_map_origin(root / "map" / "arena.yaml"), (0.0, -2.0, 0.0))
            self.assertTrue(np.all(terrain.values == 5))

            metadata = json.loads((root / "map" / "arena_frame.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["source_frame"], "odom")
            self.assertEqual(metadata["revision"], 1)
            self.assertAlmostEqual(metadata["source_to_map"]["yaw"], -math.pi / 2)
            self.assertEqual(result["point_count"], len(points))
            self.assertTrue(result["terrain_transformed"])
            self.assertEqual(map_frame_status(root, "arena")["frame_revision"], 1)

    def test_unaligned_origin_keeps_the_grid_size_and_pixels(self):
        # (source origin, source yaw, selected origin, selected heading, new origin)
        cases = (
            # Re-selecting the current pose must be a no-op for the raster.
            ((-0.83, 1.27), 0.0, (-0.83, 1.27), 0.0, (0.0, 0.0)),
            # A grid that is itself rotated in the YAML must resample onto itself.
            ((-0.83, 1.27), 0.25, (-0.83, 1.27), 0.25, (0.0, 0.0)),
            # A sub-cell origin keeps the same extent: snapping the bounding box
            # to the resolution grid would add a fill row and column here.
            ((-0.83, 1.27), 0.0, (-0.53, 1.67), 0.0, (-0.3, -0.4)),
            ((0.0, 0.0), 0.0, (0.3, 0.4), 0.0, (-0.3, -0.4)),
        )
        for source_origin, source_yaw, selected, heading, expected in cases:
            with self.subTest(source=source_origin, yaw=source_yaw, selected=selected):
                with tempfile.TemporaryDirectory() as temporary:
                    root = Path(temporary)
                    self._write_bundle(root)
                    yaml_path = root / "map" / "arena.yaml"
                    yaml_path.write_text(
                        yaml_path.read_text(encoding="utf-8").replace(
                            "origin: [0.0, 0.0, 0.0]",
                            f"origin: [{source_origin[0]}, {source_origin[1]}, {source_yaw}]",
                        ),
                        encoding="utf-8",
                    )
                    before_pgm = (root / "map" / "arena.pgm").read_bytes()
                    before = load_occupancy_editor_map(yaml_path)

                    result = align_map_frame(root, "arena", selected[0], selected[1], heading)

                    after = load_occupancy_editor_map(yaml_path)
                    self.assertEqual(
                        (after.metadata.width, after.metadata.height),
                        (before.metadata.width, before.metadata.height),
                    )
                    self.assertEqual(
                        (result["grid"]["width"], result["grid"]["height"]),
                        (before.metadata.width, before.metadata.height),
                    )
                    # No fill pixels are introduced by a pure translation.
                    self.assertEqual((root / "map" / "arena.pgm").read_bytes(), before_pgm)
                    np.testing.assert_array_equal(after.values, before.values)
                    self.assertAlmostEqual(after.metadata.origin_x, expected[0], places=9)
                    self.assertAlmostEqual(after.metadata.origin_y, expected[1], places=9)
                    self.assertAlmostEqual(after.metadata.origin_yaw, 0.0, places=12)

    def test_redefinition_composes_source_to_map_transform(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self._write_bundle(root)
            initialize_frame_metadata(root, "arena", "lio_odom")
            align_map_frame(root, "arena", 1.0, 0.0, 0.0)
            align_map_frame(root, "arena", 0.0, 2.0, 0.0)

            metadata = json.loads((root / "map" / "arena_frame.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["revision"], 2)
            self.assertEqual(metadata["source_frame"], "lio_odom")
            self.assertAlmostEqual(metadata["source_to_map"]["x"], -1.0)
            self.assertAlmostEqual(metadata["source_to_map"]["y"], -2.0)
            np.testing.assert_allclose(
                read_binary_pcd(root / "pcd" / "arena.pcd")[0],
                [-1.0, -2.0, 0.2],
                atol=1e-6,
            )


if __name__ == "__main__":
    unittest.main()
