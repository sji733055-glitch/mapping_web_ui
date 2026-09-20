from pathlib import Path
import tempfile
import unittest

import numpy as np

from backend.mapping_core import (
    MapExportConfig,
    VoxelAccumulator,
    export_session,
    sanitize_map_name,
)


class MappingCoreTests(unittest.TestCase):
    def test_name_validation_blocks_paths(self):
        self.assertEqual(sanitize_map_name("venue-01"), "venue-01")
        for bad in ("", "../map", "map name", "/tmp/x", ".."):
            with self.assertRaises(ValueError):
                sanitize_map_name(bad)

    def test_voxel_accumulator_deduplicates_and_resets(self):
        accumulator = VoxelAccumulator(voxel_size=0.1)
        added = accumulator.add(np.array([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [0.2, 0.0, 0.0]]))
        self.assertEqual(added, 2)
        self.assertEqual(accumulator.point_count, 2)
        self.assertEqual(accumulator.snapshot().shape, (2, 3))
        accumulator.clear()
        self.assertEqual(accumulator.point_count, 0)

    def test_export_writes_pcd_and_nav2_map(self):
        xs, ys = np.meshgrid(np.linspace(-1, 1, 30), np.linspace(-0.8, 0.8, 24))
        points = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, 0.6))).astype(np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            result = export_session(
                Path(tmp), "test_map", points,
                MapExportConfig(resolution=0.05),
            )
            for key in ("pcd", "pgm", "yaml"):
                self.assertTrue(Path(result[key]).is_file())
            pcd = Path(result["pcd"]).read_bytes()
            self.assertIn(b"DATA binary\n", pcd[:300])
            yaml = Path(result["yaml"]).read_text(encoding="utf-8")
            self.assertIn("resolution: 0.05", yaml)
            self.assertIn("mode: trinary", yaml)


if __name__ == "__main__":
    unittest.main()
