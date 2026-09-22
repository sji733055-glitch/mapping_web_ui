import struct
import unittest

import numpy as np

from backend.rogmap_debug import (
    FLAG_HEIGHT_VALID,
    FLAG_RATIO_VALID,
    REASON_HOLLOW_TUNNEL,
    REASON_RATIO_UNAVAILABLE,
    REASON_SOLID_WALL,
    ROGMAP_HEADER,
    ROGMAP_MAGIC,
    ROGMAP_RECORD,
    ProjectionConfig,
    build_projection_snapshot,
    classify_projection,
    encode_projection_snapshot,
)


class ProjectionConfigTests(unittest.TestCase):
    def test_enforces_same_derived_constraints_as_rogmap(self):
        ProjectionConfig().validate()

        with self.assertRaisesRegex(ValueError, "surface_height_delta_max"):
            ProjectionConfig(
                surface_height_delta_max=0.25,
                tunnel_height_delta_min=0.25,
            ).validate()
        with self.assertRaisesRegex(ValueError, "tunnel_occupancy_ratio_max"):
            ProjectionConfig(
                wall_occupancy_ratio_min=0.45,
                tunnel_occupancy_ratio_max=0.45,
            ).validate()

    def test_vectorized_classification_preserves_wall_before_tunnel_order(self):
        config = ProjectionConfig(
            wall_height_delta_min=0.25,
            wall_occupancy_ratio_min=0.80,
            tunnel_height_delta_min=0.25,
            tunnel_height_delta_max=0.40,
            tunnel_occupancy_ratio_max=0.45,
        )
        reasons = classify_projection(
            np.array([0.30, 0.30, 0.30]),
            np.array([1.00, 0.20, np.nan]),
            np.ones(3, dtype=bool),
            np.array([True, True, False]),
            config,
        )

        self.assertEqual(reasons.tolist(), [
            REASON_SOLID_WALL,
            REASON_HOLLOW_TUNNEL,
            REASON_RATIO_UNAVAILABLE,
        ])


class ProjectionSnapshotTests(unittest.TestCase):
    def test_reconstructs_ratio_only_when_both_column_endpoints_are_visible(self):
        config = ProjectionConfig()
        height_points = np.array([
            [0.025, 0.025, 0.30, 0.30],
            [0.075, 0.025, 0.80, 0.80],
            [0.125, 0.025, 0.50, 0.30],
        ], dtype=np.float32)
        tunnel_layers = [[0.025, 0.025, z] for z in (0.00, 0.30)]
        wall_layers = [[0.075, 0.025, z] for z in np.arange(0.0, 0.801, 0.05)]
        missing_low_endpoint = [[0.125, 0.025, z] for z in (0.30, 0.50)]
        occupied = np.asarray(tunnel_layers + wall_layers + missing_low_endpoint, dtype=np.float32)

        snapshot = build_projection_snapshot(
            cell_type=np.array([66, 100, 100], dtype=np.int8),
            width=3,
            height=1,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            type_stamp=10.0,
            height_points=height_points,
            height_stamp=10.01,
            occupied_points=occupied,
            occupied_stamp=10.02,
            config=config,
        )

        self.assertTrue(snapshot.flags[0] & FLAG_HEIGHT_VALID)
        self.assertTrue(snapshot.flags[0] & FLAG_RATIO_VALID)
        self.assertEqual(snapshot.occupied_count[0], 2)
        self.assertAlmostEqual(snapshot.occupancy_ratio[0], 1.0 / 6.0, places=5)
        self.assertEqual(snapshot.reason[0], REASON_HOLLOW_TUNNEL)
        self.assertTrue(snapshot.flags[1] & FLAG_RATIO_VALID)
        self.assertAlmostEqual(snapshot.occupancy_ratio[1], 1.0, places=5)
        self.assertEqual(snapshot.reason[1], REASON_SOLID_WALL)
        self.assertFalse(snapshot.flags[2] & FLAG_RATIO_VALID)
        self.assertEqual(snapshot.reason[2], REASON_RATIO_UNAVAILABLE)

    def test_unsynchronized_cloud_never_claims_a_thick_column_ratio(self):
        snapshot = build_projection_snapshot(
            cell_type=np.array([66], dtype=np.int8),
            width=1,
            height=1,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            type_stamp=1.0,
            height_points=np.array([[0.025, 0.025, 0.30, 0.30]], dtype=np.float32),
            height_stamp=1.0,
            occupied_points=np.array([[0.025, 0.025, 0.0], [0.025, 0.025, 0.3]], dtype=np.float32),
            occupied_stamp=2.0,
            config=ProjectionConfig(),
        )

        self.assertFalse(snapshot.flags[0] & FLAG_RATIO_VALID)
        self.assertEqual(snapshot.reason[0], REASON_RATIO_UNAVAILABLE)

    def test_binary_frame_has_fixed_browser_stride(self):
        snapshot = build_projection_snapshot(
            cell_type=np.array([66], dtype=np.int8),
            width=1,
            height=1,
            resolution=0.05,
            origin_x=-1.0,
            origin_y=-2.0,
            type_stamp=3.0,
            height_points=np.array([[ -0.975, -1.975, 0.30, 0.30]], dtype=np.float32),
            height_stamp=3.0,
            occupied_points=np.array([[-0.975, -1.975, 0.0], [-0.975, -1.975, 0.3]], dtype=np.float32),
            occupied_stamp=3.0,
            config=ProjectionConfig(),
        )

        payload = encode_projection_snapshot(snapshot)
        header = ROGMAP_HEADER.unpack_from(payload)

        self.assertEqual(header[0], ROGMAP_MAGIC)
        self.assertEqual(header[2], ROGMAP_RECORD.size)
        self.assertEqual(len(payload), ROGMAP_HEADER.size + ROGMAP_RECORD.size)
        self.assertEqual(struct.unpack_from("<b", payload, ROGMAP_HEADER.size)[0], 66)


if __name__ == "__main__":
    unittest.main()
