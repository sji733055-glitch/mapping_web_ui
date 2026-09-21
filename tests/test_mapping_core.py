from pathlib import Path
import json
import math
import struct
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

from backend.mapping_core import (
    BinDisappearanceConfig,
    MapExportConfig,
    PREVIEW_MAX_PIXELS,
    VoxelAccumulator,
    _radius_filter,
    _voxel_structure_filter,
    build_cloud_frame,
    build_pcd_map_preview,
    convert_pcd_to_map,
    export_pcd_session,
    filter_dynamic_with_history_consensus,
    filter_dynamic_with_keyframe_history,
    iter_keyframe_history,
    pool_occupancy_pixels,
    read_binary_pcd,
    select_visibility_endpoints,
    resolve_pcd_input,
    sanitize_map_name,
    slice_by_height,
    slice_points_by_config,
    slice_points_to_occupancy,
    visibility_miss_keys,
    voxel_keys,
    write_binary_pcd,
    write_keyframe_history_chunk,
    write_keyframe_history_metadata,
    write_mapping_report,
    write_occupancy_map,
)


def decode_pgm(body: bytes) -> tuple[int, int, np.ndarray]:
    """Parse a binary PGM (P5) payload written by ``encode_pgm``."""
    header, _, data = body.partition(b"255\n")
    tokens = [token for line in header.split(b"\n") if line and not line.startswith(b"#") for token in line.split()]
    assert tokens[0] == b"P5", tokens[:1]
    width, height = int(tokens[1]), int(tokens[2])
    pixels = np.frombuffer(data, dtype=np.uint8)
    assert pixels.size == width * height, (pixels.size, width, height)
    return width, height, pixels.reshape(height, width)


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

    def test_voxel_accumulator_replaces_saved_cloud_atomically(self):
        accumulator = VoxelAccumulator(voxel_size=0.1)
        accumulator.add(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32))
        replacement = np.array([[2.0, 0.0, 0.0]], dtype=np.float32)
        accumulator.replace(replacement)
        self.assertEqual(accumulator.point_count, 1)
        np.testing.assert_allclose(accumulator.snapshot(), replacement)

    def test_visibility_removes_trail_after_five_distinct_misses(self):
        accumulator = VoxelAccumulator(voxel_size=0.1)
        trail = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
        wall = np.array([[2.0, 0.0, 0.0]], dtype=np.float32)
        accumulator.add(np.vstack((trail, wall)), generation=1)
        misses = visibility_miss_keys(
            np.zeros(3), wall, 0.1,
            min_range=0.2, max_range=3.0, max_rays=10,
        )
        trail_key = int(voxel_keys(trail, 0.1)[0])
        wall_key = int(voxel_keys(wall, 0.1)[0])
        self.assertIn(trail_key, {int(key) for key in misses})
        self.assertNotIn(wall_key, {int(key) for key in misses})

        occluded = visibility_miss_keys(
            np.zeros(3),
            np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32),
            0.1, min_range=0.2, max_range=3.0, max_rays=10,
        )
        behind_near_surface = int(voxel_keys(
            np.array([[1.5, 0.0, 0.0]], dtype=np.float32), 0.1
        )[0])
        self.assertNotIn(behind_near_surface, {int(key) for key in occluded})

        # Re-applying one source frame cannot manufacture extra evidence.
        for _ in range(3):
            self.assertEqual(accumulator.apply_misses(misses, 2, 5), 0)
        self.assertEqual(accumulator.point_count, 2)

        for generation in range(3, 6):
            accumulator.add(wall, generation=generation)
            self.assertEqual(accumulator.apply_misses(misses, generation, 5), 0)
        accumulator.add(wall, generation=6)
        self.assertEqual(accumulator.apply_misses(misses, 6, 5), 1)
        remaining = accumulator.snapshot()
        self.assertEqual(remaining.shape, (1, 3))
        np.testing.assert_allclose(remaining[0], wall[0])
        self.assertEqual(accumulator.removed_total, 1)

    def test_visibility_uses_hw_dda_and_excludes_return_voxel(self):
        origin = np.array([0.05, 0.05, 0.05], dtype=np.float64)
        endpoint = np.array([[0.45, 0.45, 0.05]], dtype=np.float32)
        misses = {
            int(key) for key in visibility_miss_keys(
                origin,
                endpoint,
                0.1,
                min_range=0.0,
                max_range=1.0,
                max_rays=10,
                angular_resolution=0.01,
            )
        }

        def key(x: float, y: float) -> int:
            point = np.array([[x, y, 0.05]], dtype=np.float32)
            return int(voxel_keys(point, 0.1)[0])

        # HW's Amanatides--Woo traversal advances one boundary at a time, so
        # a diagonal corner crossing visits the adjacent voxel as well.
        self.assertIn(key(0.05, 0.15), misses)
        self.assertIn(key(0.15, 0.15), misses)
        self.assertIn(key(0.15, 0.25), misses)
        self.assertIn(key(0.25, 0.25), misses)
        self.assertNotIn(key(0.45, 0.45), misses)

    def test_keyframe_history_round_trip_and_removes_final_position(self):
        origin = np.zeros(3, dtype=np.float64)
        trail = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
        wall = np.array([[2.0, 0.0, 0.0]], dtype=np.float32)
        untouched = np.array([[0.0, 2.0, 0.0]], dtype=np.float32)
        records = [
            (generation, origin, wall)
            for generation in range(1, 6)
        ]
        # The object appears only in the final frame.  Unlike the online
        # accumulator, offline replay can use the five earlier free-space
        # observations to remove this final resting position.
        records.append((6, origin, trail))

        with tempfile.TemporaryDirectory() as tmp:
            history = Path(tmp)
            size = write_keyframe_history_chunk(history / "chunk_000000.bin", records)
            self.assertGreater(size, 0)
            write_keyframe_history_metadata(history, {"written_frames": len(records)})
            loaded = list(iter_keyframe_history(history))
            self.assertEqual(len(loaded), len(records))
            np.testing.assert_allclose(loaded[-1][2], trail)

            filtered, stats = filter_dynamic_with_keyframe_history(
                np.vstack((trail, wall, untouched)),
                history,
                voxel_size=0.1,
                free_frame_threshold=5,
                free_hit_ratio=2.0,
                max_removal_fraction=0.5,
            )
            self.assertTrue(stats["applied"])
            self.assertEqual(stats["frames"], 6)
            self.assertEqual(stats["removed_points"], 1)
            self.assertEqual(len(filtered), 2)
            np.testing.assert_allclose(
                filtered[np.argsort(filtered[:, 1])],
                np.vstack((wall, untouched))[np.argsort(np.vstack((wall, untouched))[:, 1])],
            )

    def test_hits_reset_misses_and_unobserved_voxels_do_not_decay(self):
        accumulator = VoxelAccumulator(voxel_size=0.1)
        static = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
        unobserved = np.array([[0.0, 2.0, 0.0]], dtype=np.float32)
        accumulator.add(np.vstack((static, unobserved)), generation=1)
        static_key = voxel_keys(static, 0.1)
        for generation in range(2, 22):
            accumulator.apply_misses(static_key, generation, 5)
            if generation % 4 == 0:
                accumulator.add(static, generation=generation)
        self.assertEqual(accumulator.point_count, 2)
        self.assertEqual(accumulator.removed_total, 0)

    def test_deletion_updates_bounds_and_reuses_hard_limit_storage(self):
        accumulator = VoxelAccumulator(voxel_size=0.1, max_points=3)
        original = np.array([
            [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0],
        ], dtype=np.float32)
        accumulator.add(original, generation=1)
        first_key = voxel_keys(original[:1], 0.1)
        for generation in range(2, 7):
            accumulator.apply_misses(first_key, generation, 5)
        self.assertEqual(accumulator.point_count, 2)
        self.assertAlmostEqual(accumulator.bounds["min_x"], 2.0)
        self.assertEqual(
            accumulator.add(np.array([[4.0, 0.0, 0.0]], dtype=np.float32), generation=7),
            1,
        )
        self.assertEqual(accumulator.point_count, 3)
        np.testing.assert_allclose(
            np.sort(accumulator.snapshot()[:, 0]),
            np.array([2.0, 3.0, 4.0], dtype=np.float32),
        )

    def test_accumulator_add_and_snapshot_are_thread_safe(self):
        accumulator = VoxelAccumulator(voxel_size=0.05, max_points=10_000)
        failures = []

        def add_points(offset: int) -> None:
            try:
                for batch in range(20):
                    x = offset + batch + np.arange(50, dtype=np.float32) * 0.051
                    accumulator.add(np.column_stack((x, np.zeros(50), np.zeros(50))))
            except Exception as error:  # pragma: no cover - failure reporting path
                failures.append(error)

        workers = [threading.Thread(target=add_points, args=(index * 100,)) for index in range(3)]
        for worker in workers:
            worker.start()
        while any(worker.is_alive() for worker in workers):
            snapshot = accumulator.snapshot(max_points=200)
            self.assertEqual(snapshot.shape[1], 3)
        for worker in workers:
            worker.join()
        self.assertFalse(failures)
        self.assertEqual(accumulator.point_count, len(accumulator.snapshot()))

    def test_session_export_writes_only_the_pcd(self):
        xs, ys = np.meshgrid(np.linspace(-1, 1, 30), np.linspace(-0.8, 0.8, 24))
        points = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, 0.6))).astype(np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            result = export_pcd_session(Path(tmp), "test_map", points)

            self.assertTrue(Path(result["pcd"]).is_file())
            self.assertNotIn("pgm", result)
            self.assertNotIn("yaml", result)
            pcd = Path(result["pcd"]).read_bytes()
            self.assertIn(b"DATA binary\n", pcd[:300])
            # 结束保存不再生成二维图：二维 PGM/YAML 必须由显式转换步骤产生。
            self.assertFalse((root / "map" / "test_map.pgm").exists())
            self.assertFalse((root / "map" / "test_map.yaml").exists())

    def test_height_slice_filter_keeps_the_window_and_never_mutates_input(self):
        points = np.array([[0.0, 0.0, 0.1], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]], dtype=np.float32)

        self.assertEqual(slice_by_height(points, 0.2, 1.0).tolist(), [[0.0, 0.0, 0.5]])
        self.assertEqual(len(slice_by_height(points)), 3)
        self.assertEqual(len(slice_by_height(points, z_max=0.5)), 2)
        self.assertEqual(points.shape, (3, 3))

    def test_ground_relative_slice_removes_tilted_floor_and_keeps_obstacles(self):
        xs, ys = np.meshgrid(
            np.arange(-3.0, 3.01, 0.1, dtype=np.float32),
            np.arange(-2.0, 2.01, 0.1, dtype=np.float32),
        )
        floor_z = 0.08 * xs - 0.03 * ys + 0.2
        floor = np.column_stack((xs.ravel(), ys.ravel(), floor_z.ravel())).astype(np.float32)
        wall_y, wall_height = np.meshgrid(
            np.arange(-1.5, 1.51, 0.1, dtype=np.float32),
            np.arange(0.1, 1.41, 0.1, dtype=np.float32),
        )
        wall_x = np.full(wall_y.size, 2.0, dtype=np.float32)
        wall_ground = 0.08 * wall_x - 0.03 * wall_y.ravel() + 0.2
        wall = np.column_stack((wall_x, wall_y.ravel(), wall_ground + wall_height.ravel()))
        points = np.vstack((floor, wall)).astype(np.float32)

        relative, metadata = slice_points_by_config(
            points,
            MapExportConfig(z_min=0.05, z_max=1.5, height_mode="ground"),
        )
        absolute, _ = slice_points_by_config(
            points,
            MapExportConfig(z_min=0.05, z_max=1.5, height_mode="absolute"),
        )

        self.assertEqual(len(relative), len(wall))
        np.testing.assert_allclose(relative, wall, atol=1e-5)
        self.assertGreater(len(absolute), len(relative) + len(floor) // 2)
        self.assertAlmostEqual(metadata["ground_a"], 0.08, delta=0.003)
        self.assertAlmostEqual(metadata["ground_b"], -0.03, delta=0.003)
        self.assertAlmostEqual(metadata["ground_c"], 0.2, delta=0.02)
        self.assertAlmostEqual(
            metadata["ground_tilt_deg"],
            math.degrees(math.atan(math.hypot(0.08, 0.03))),
            delta=0.1,
        )

    def test_export_config_rejects_unknown_height_mode(self):
        with self.assertRaisesRegex(ValueError, "ground 或 absolute"):
            MapExportConfig(height_mode="local").validate()

    def test_export_config_validates_structure_voxel_filter(self):
        for mode in ("voxel", "radius", "none"):
            MapExportConfig(filter_mode=mode).validate()
        with self.assertRaisesRegex(ValueError, "voxel、radius 或 none"):
            MapExportConfig(filter_mode="statistical").validate()
        with self.assertRaisesRegex(ValueError, "0.02–1.0"):
            MapExportConfig(filter_voxel_size=0.01).validate()

    def test_voxel_structure_filter_removes_only_unsupported_single_columns(self):
        horizontal = np.array([
            [0.05, 0.05, 0.55],
            [0.15, 0.05, 0.55],
        ], dtype=np.float32)
        vertical = np.array([
            [2.05, 2.05, 0.55],
            [2.05, 2.05, 0.65],
        ], dtype=np.float32)
        # Repeating points inside one voxel must not manufacture structural
        # support: classification is based on unique occupied voxels.
        isolated = np.repeat(
            np.array([[5.05, 5.05, 0.55]], dtype=np.float32), 4, axis=0
        )
        points = np.vstack((horizontal, vertical, isolated))
        original = points.copy()

        filtered, metadata = _voxel_structure_filter(points, 0.10)

        np.testing.assert_array_equal(filtered, np.vstack((horizontal, vertical)))
        np.testing.assert_array_equal(points, original)
        self.assertEqual(metadata["filter_voxels"], 5)
        self.assertEqual(metadata["filter_columns"], 4)
        self.assertEqual(metadata["filter_kept_columns"], 3)
        self.assertEqual(metadata["filter_removed_columns"], 1)
        self.assertEqual(metadata["filter_removed_points"], 4)

    def test_voxel_structure_filter_metadata_reaches_occupancy_projection(self):
        points = np.array([
            [0.05, 0.05, 0.55], [0.15, 0.05, 0.55],
            [2.05, 2.05, 0.55], [2.05, 2.05, 0.65],
            [5.05, 5.05, 0.55],
        ], dtype=np.float32)

        _, metadata = slice_points_to_occupancy(
            points,
            MapExportConfig(
                z_min=0.1,
                z_max=1.0,
                filter_mode="voxel",
                filter_voxel_size=0.1,
                padding=0.0,
            ),
        )

        self.assertEqual(metadata["filter_mode"], "voxel")
        self.assertEqual(metadata["slice_points"], 5)
        self.assertEqual(metadata["filtered_points"], 4)
        self.assertEqual(metadata["filter_removed_points"], 1)
        self.assertEqual(metadata["filter_removed_columns"], 1)

    def test_radius_filter_never_occupies_all_cpu_workers(self):
        calls = []

        class FakeTree:
            def __init__(self, points):
                self.points = points

            def query_ball_point(self, points, radius, *, return_length, workers):
                calls.append((len(points), radius, return_length, workers))
                return np.full(len(points), 3, dtype=np.int64)

        points = np.zeros((120_001, 3), dtype=np.float32)
        with patch("scipy.spatial.cKDTree", FakeTree):
            filtered = _radius_filter(points, radius=0.5, minimum=3)

        self.assertEqual(len(filtered), len(points))
        self.assertEqual([call[0] for call in calls], [50_000, 50_000, 20_001])
        self.assertTrue(all(call[2] is True and call[3] == 1 for call in calls), calls)

    def test_slice_preview_matches_the_export_and_writes_nothing(self):
        xs, ys = np.meshgrid(np.linspace(-1, 1, 30), np.linspace(-0.8, 0.8, 24))
        points = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, 0.6))).astype(np.float32)
        config = MapExportConfig(
            resolution=0.1,
            radius=0.0,
            min_neighbors=0,
            filter_mode="voxel",
            filter_voxel_size=0.1,
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_binary_pcd(root / "pcd" / "venue.pcd", points)
            listing = lambda: sorted(
                str(path.relative_to(root)) for path in root.rglob("*") if path.is_file()
            )
            before = listing()

            body, metadata = build_pcd_map_preview(root, "venue", config)
            self.assertEqual(listing(), before)
            width, height, pixels = decode_pgm(body)
            exported = write_occupancy_map(
                root / "map" / "venue.pgm", root / "map" / "venue.yaml", points, config
            )
            exported_width, exported_height, exported_pixels = decode_pgm(
                (root / "map" / "venue.pgm").read_bytes()
            )

            self.assertEqual((width, height), (exported_width, exported_height))
            np.testing.assert_array_equal(pixels, exported_pixels)
            self.assertEqual(metadata["width"], exported["width"])
            self.assertEqual(metadata["filter_mode"], "voxel")
            self.assertEqual(metadata["preview_stride"], 1)
            self.assertEqual(metadata["occupied_cells"], metadata["preview_occupied_cells"])
            self.assertGreater(metadata["occupied_cells"], 0)
            self.assertEqual(listing(), sorted([*before, "map/venue.pgm", "map/venue.yaml"]))

    def test_preview_pooling_never_drops_a_thin_wall(self):
        pixels = np.full((3000, 3000), 254, dtype=np.uint8)
        pixels[::3, ::3] = 0

        pooled, stride = pool_occupancy_pixels(pixels, max_pixels=PREVIEW_MAX_PIXELS)

        self.assertGreater(stride, 1)
        self.assertLessEqual(pooled.size, PREVIEW_MAX_PIXELS)
        self.assertGreater(int(np.count_nonzero(pooled == 0)), 0)
        self.assertTrue(np.array_equal(pool_occupancy_pixels(pixels[:10, :10])[0], pixels[:10, :10]))

    def test_convert_uses_the_supplied_slice_parameters(self):
        xs, ys = np.meshgrid(np.linspace(-1, 1, 20), np.linspace(-1, 1, 20))
        points = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, 0.6))).astype(np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_binary_pcd(root / "pcd" / "venue.pcd", points)

            result = convert_pcd_to_map(
                root,
                "venue",
                "venue_slice",
                MapExportConfig(
                    resolution=0.2, z_min=0.4, z_max=0.8, radius=0.0, min_neighbors=0, padding=0.5
                ),
            )

            yaml = Path(result["yaml"]).read_text(encoding="utf-8")
            self.assertIn("resolution: 0.2", yaml)
            self.assertAlmostEqual(result["map"]["origin_x"], -1.5, places=6)
            self.assertAlmostEqual(result["map"]["origin_y"], -1.5, places=6)
            self.assertEqual(result["map"]["z_min"], 0.4)
            self.assertEqual(result["map"]["z_max"], 0.8)
            self.assertLess(result["map"]["width"], 20)

    def test_voxel_grid_is_shared_by_accumulator_and_keys(self):
        # 这两个真实点曾让累积器（float32 除法）与 voxel_keys（float64 除法）落在相邻
        # 体素：float32 下 y=-1.6500001/0.05 正好是 -33.0，float64 下是 -33.000002。
        # 于是离线去动态的“每体素最多一点”守卫在真实地图上触发，整段清理被放弃。
        points = np.array([
            [0.73304796, -1.6590824, 0.07668611],
            [0.70544297, -1.6500001, 0.08783014],
        ], dtype=np.float32)
        self.assertNotEqual(
            np.floor(np.float64(points[1, 1]) / 0.05),
            np.floor(points[1, 1] / np.float32(0.05)),
            "这两个点的浮点陷阱已经失效，请换一组边界点",
        )
        accumulator = VoxelAccumulator(0.05, 1000)

        added = accumulator.add(points)
        snapshot = accumulator.snapshot()

        self.assertEqual(added, 1)
        self.assertEqual(len(snapshot), 1)
        keys = voxel_keys(snapshot, 0.05)
        self.assertEqual(len(np.unique(keys)), len(keys))

    def test_accumulator_keys_stay_unique_for_boundary_heavy_clouds(self):
        rng = np.random.default_rng(2027)
        columns = np.floor(rng.uniform(-40, 40, size=(20_000, 3))) * 0.05
        offsets = rng.choice([-1.0e-7, 0.0, 1.0e-7], size=(20_000, 3))
        points = np.ascontiguousarray(columns + offsets, dtype=np.float32)
        accumulator = VoxelAccumulator(0.05, 200_000)

        accumulator.add(points)
        snapshot = accumulator.snapshot()
        keys = np.sort(voxel_keys(snapshot, 0.05))

        self.assertGreater(len(snapshot), 0)
        self.assertFalse(bool(np.any(keys[1:] == keys[:-1])))

    def test_offline_filter_keeps_running_on_a_boundary_heavy_cloud(self):
        rng = np.random.default_rng(11)
        origin = np.array([0.0, 0.0, 0.5])
        wall_y, wall_z = np.meshgrid(
            np.arange(-3.0, 3.0, 0.05, dtype=np.float32),
            np.arange(0.0, 1.55, 0.05, dtype=np.float32),
        )
        wall = np.column_stack((
            np.full(wall_y.size, 2.5, dtype=np.float32), wall_y.ravel(), wall_z.ravel(),
        )).astype(np.float32)
        # 两个真实边界陷阱点 + 一个可被射线穿过的拖影点。
        trap = np.array([
            [0.73304796, -1.6590824, 0.07668611],
            [0.70544297, -1.6500001, 0.08783014],
            [1.0, 1.0, 0.5],
        ], dtype=np.float32)
        accumulator = VoxelAccumulator(0.05, 200_000)
        accumulator.add(np.vstack((trap, wall)))
        snapshot = accumulator.snapshot()
        trail_key = int(voxel_keys(np.array([[1.0, 1.0, 0.5]], dtype=np.float32), 0.05)[0])
        self.assertGreater(len(snapshot), 1)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            records = [
                (index * 1_000_000, origin.copy(), wall.copy())
                for index in range(6)
            ]
            write_keyframe_history_chunk(directory / "chunk_000000.bin", records)
            write_keyframe_history_metadata(directory, {"written_frames": len(records)})

            # 回归：快照里带边界陷阱点时，这里曾抛“每个体素最多一点”并放弃全部清理。
            filtered, result = filter_dynamic_with_keyframe_history(
                snapshot, directory, 0.05, free_frame_threshold=5,
            )

        self.assertTrue(result["applied"], result)
        self.assertEqual(result["frames"], 6)
        self.assertNotIn(trail_key, {int(key) for key in voxel_keys(filtered, 0.05)})
        self.assertLess(len(filtered), len(snapshot))

    def test_polar_disappearance_votes_remove_a_confirmed_object_but_keep_ground(self):
        origin = np.zeros(3, dtype=np.float64)
        current_ground = np.array([
            [3.0, 0.0, 0.00], [3.0, 0.0, 0.10],
        ], dtype=np.float32)
        disappeared_object = np.array([
            [3.0, 0.0, 0.60], [3.0, 0.0, 1.40],
        ], dtype=np.float32)
        # A separate, persistently mapped structure must remain untouched.
        static_structure = np.array([
            [6.0, 3.0, 0.20], [6.0, 3.0, 0.80], [6.0, 3.0, 1.50],
        ], dtype=np.float32)
        points = np.vstack((current_ground, disappeared_object, static_structure))
        polar = BinDisappearanceConfig(
            rings=10, sectors=72, cell_size=0.2, min_votes=2,
            max_removal_fraction=0.8,
        )
        with tempfile.TemporaryDirectory() as tmp:
            history = Path(tmp)
            records = [(index, origin, current_ground) for index in range(3)]
            write_keyframe_history_chunk(history / "chunk_000000.bin", records)
            write_keyframe_history_metadata(history, {"written_frames": len(records)})
            filtered, result = filter_dynamic_with_history_consensus(
                points, history, 0.1, free_frame_threshold=5,
                max_removal_fraction=0.8, polar_config=polar,
            )

        self.assertTrue(result["applied"], result)
        self.assertEqual(result["ray"]["removed_points"], 0)
        self.assertTrue(result["polar"]["applied"], result["polar"])
        self.assertEqual(result["polar"]["removed_points"], len(disappeared_object))
        np.testing.assert_allclose(filtered[:len(current_ground)], current_ground)
        self.assertFalse(np.isin(
            voxel_keys(disappeared_object, 0.1), voxel_keys(filtered, 0.1)
        ).any())
        self.assertTrue(np.isin(
            voxel_keys(static_structure, 0.1), voxel_keys(filtered, 0.1)
        ).all())

    def test_polar_vote_reverts_when_combined_removal_exceeds_the_global_cap(self):
        origin = np.zeros(3, dtype=np.float64)
        current_ground = np.array([
            [3.0, 0.0, 0.00], [3.0, 0.0, 0.10],
        ], dtype=np.float32)
        disappeared_object = np.array([
            [3.0, 0.0, 0.60], [3.0, 0.0, 1.40],
        ], dtype=np.float32)
        static_structure = np.array([
            [6.0, 3.0, 0.20], [6.0, 3.0, 0.80], [6.0, 3.0, 1.50],
        ], dtype=np.float32)
        points = np.vstack((current_ground, disappeared_object, static_structure))
        polar = BinDisappearanceConfig(
            rings=10, sectors=72, cell_size=0.2, min_votes=2,
            max_removal_fraction=0.8,
        )
        with tempfile.TemporaryDirectory() as tmp:
            history = Path(tmp)
            records = [(index, origin, current_ground) for index in range(3)]
            write_keyframe_history_chunk(history / "chunk_000000.bin", records)
            write_keyframe_history_metadata(history, {"written_frames": len(records)})
            filtered, result = filter_dynamic_with_history_consensus(
                points, history, 0.1, free_frame_threshold=5,
                max_removal_fraction=0.20, polar_config=polar,
            )

        np.testing.assert_allclose(filtered, points)
        self.assertFalse(result["polar"]["applied"], result["polar"])
        self.assertIn("总安全上限", str(result["polar"]["reason"]))
        self.assertEqual(result["removed_points"], 0)

    def test_mapping_report_is_atomic_json_beside_its_pcd(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            report = {"schema_version": 1, "dynamic_filter": {"removed_points": 12}}
            path = write_mapping_report(root, "cleaned", report)
            self.assertEqual(path, root / "pcd" / "cleaned.mapping-report.json")
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")), report)
            self.assertFalse((path.parent / ".cleaned.mapping-report.json.tmp").exists())

    def test_ray_budget_rotates_until_every_angular_bin_is_covered(self):
        rng = np.random.default_rng(5)
        azimuth = rng.uniform(-math.pi, math.pi, 6000)
        elevation = rng.uniform(-0.6, 0.6, 6000)
        ranges = rng.uniform(1.0, 6.0, 6000)
        cloud = np.column_stack((
            ranges * np.cos(elevation) * np.cos(azimuth),
            ranges * np.cos(elevation) * np.sin(azimuth),
            0.2 + ranges * np.sin(elevation),
        )).astype(np.float32)
        origin = np.zeros(3, dtype=np.float64)
        angular = math.radians(0.5)

        full = select_visibility_endpoints(origin, cloud, max_rays=10 ** 6, angular_resolution=angular)
        budget = 200
        steps = int(math.ceil(len(full) / budget)) + 2
        union: set[tuple[float, float, float]] = set()
        sets = []
        for phase in range(steps):
            part = select_visibility_endpoints(
                origin, cloud, max_rays=budget, angular_resolution=angular, phase=phase,
            )
            self.assertLessEqual(len(part), budget)
            sets.append({tuple(float(value) for value in row) for row in part})
            union |= sets[-1]

        self.assertGreater(len(full), budget * 4)
        self.assertEqual(union, {tuple(float(value) for value in row) for row in full})
        # 固定相位才会只覆盖同一个梳齿；轮换让相邻帧取到不同方向。
        self.assertNotEqual(sets[0], sets[1])

    def test_ray_phase_rejects_negative_values(self):
        cloud = np.array([[1.0, 0.0, 0.2]], dtype=np.float32)
        with self.assertRaisesRegex(ValueError, "相位"):
            select_visibility_endpoints(np.zeros(3), cloud, phase=-1)

    def test_top_down_projection_ignores_points_above_selected_height(self):
        below = np.array([
            [0.0, 0.0, 0.4],
            [0.5, 0.0, 0.6],
            [0.0, 0.5, 0.8],
        ], dtype=np.float32)
        above = np.array([[50.0, 50.0, 1.8]], dtype=np.float32)
        points = np.vstack((below, above))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            result = write_occupancy_map(
                root / "cutoff.pgm",
                root / "cutoff.yaml",
                points,
                MapExportConfig(
                    resolution=0.1,
                    z_min=0.05,
                    z_max=1.0,
                    radius=0.0,
                    min_neighbors=0,
                    padding=0.0,
                ),
            )
            self.assertEqual(result["slice_points"], 3)
            self.assertEqual(result["z_max"], 1.0)
            self.assertLess(result["width"], 10)
            self.assertLess(result["height"], 10)

    def test_export_config_rejects_non_finite_height(self):
        with self.assertRaisesRegex(ValueError, "有限数值"):
            MapExportConfig(z_max=float("nan")).validate()

    def test_existing_ascii_pcd_with_extra_field_converts_without_rewriting_source(self):
        rows = [
            f"{x:.3f} {y:.3f} 0.600 {index}"
            for index, (x, y) in enumerate(
                (xy for x in np.linspace(-0.5, 0.5, 5) for xy in ((x, y) for y in np.linspace(-0.4, 0.4, 5)))
            )
        ]
        header = (
            "# external PCD with intensity\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 2\n"
            "TYPE F F F U\n"
            "COUNT 1 1 1 1\n"
            f"WIDTH {len(rows)}\nHEIGHT 1\nPOINTS {len(rows)}\nDATA ascii\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "pcd" / "external.pcd"
            source.parent.mkdir(parents=True)
            source.write_text(header + "\n".join(rows) + "\n", encoding="ascii")
            original = source.read_bytes()

            result = convert_pcd_to_map(
                root,
                "external",
                "external",
                MapExportConfig(resolution=0.1, radius=0.0, min_neighbors=0),
            )

            self.assertEqual(result["point_count"], len(rows))
            self.assertEqual(source.read_bytes(), original)
            self.assertTrue((root / "map" / "external.pgm").is_file())
            yaml = (root / "map" / "external.yaml").read_text(encoding="utf-8")
            self.assertIn("image: external.pgm", yaml)

    def test_binary_pcd_extra_fields_are_ignored_and_renamed_bundle_is_canonical(self):
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<u2")])
        records = np.zeros(4, dtype=dtype)
        records["x"] = [0.0, 0.3, 0.0, 0.3]
        records["y"] = [0.0, 0.0, 0.3, 0.3]
        records["z"] = 0.6
        records["intensity"] = [10, 20, 30, 40]
        header = (
            "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 2\n"
            "TYPE F F F U\nCOUNT 1 1 1 1\nWIDTH 4\nHEIGHT 1\n"
            "POINTS 4\nDATA binary\n"
        ).encode("ascii")
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "pcd" / "external.pcd"
            source.parent.mkdir(parents=True)
            source.write_bytes(header + records.tobytes())

            points = read_binary_pcd(source)
            np.testing.assert_allclose(points[:, 2], 0.6)
            result = convert_pcd_to_map(
                root,
                "external",
                "external_copy",
                MapExportConfig(resolution=0.1, radius=0.0, min_neighbors=0, padding=0.0),
            )

            canonical = root / "pcd" / "external_copy.pcd"
            self.assertEqual(Path(result["pcd"]), canonical)
            self.assertIn(b"FIELDS x y z\n", canonical.read_bytes()[:200])
            np.testing.assert_allclose(read_binary_pcd(canonical), points)

    def test_failed_existing_pcd_slice_leaves_existing_outputs_untouched(self):
        points = np.array([[0.0, 0.0, 3.0], [0.2, 0.0, 3.0], [0.0, 0.2, 3.0]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "pcd" / "high.pcd"
            write_binary_pcd(source, points)
            map_dir = root / "map"
            map_dir.mkdir(parents=True)
            old_pgm = map_dir / "protected.pgm"
            old_yaml = map_dir / "protected.yaml"
            old_pgm.write_bytes(b"old-pgm")
            old_yaml.write_bytes(b"old-yaml")

            with self.assertRaisesRegex(ValueError, "切片后"):
                convert_pcd_to_map(
                    root,
                    "high",
                    "new_output",
                    MapExportConfig(z_min=0.05, z_max=1.0, radius=0.0, min_neighbors=0),
                )

            self.assertEqual(old_pgm.read_bytes(), b"old-pgm")
            self.assertEqual(old_yaml.read_bytes(), b"old-yaml")
            self.assertFalse((map_dir / "new_output.pgm").exists())
            self.assertFalse((root / "pcd" / "new_output.pcd").exists())


    def test_cloud_frame_header_decimates_without_touching_the_source(self):
        points = np.arange(3000, dtype=np.float32).reshape(1000, 3)

        body, sent, total = build_cloud_frame(points, 100)

        self.assertEqual(total, 1000)
        self.assertEqual(sent, 100)
        self.assertEqual(body[:4], b"MAP1")
        self.assertEqual(struct.unpack("<I", body[4:8])[0], 100)
        self.assertEqual(len(body), 8 + 100 * 12)
        decoded = np.frombuffer(body, dtype="<f4", count=300, offset=8).reshape(100, 3)
        self.assertTrue(np.allclose(decoded, points[::10]))
        self.assertEqual(points.shape, (1000, 3))

    def test_cloud_frame_keeps_every_point_below_the_limit(self):
        points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float64)

        body, sent, total = build_cloud_frame(points, 1000)

        self.assertEqual((sent, total), (2, 2))
        self.assertEqual(len(body), 8 + 2 * 12)
        self.assertEqual(np.frombuffer(body, dtype="<f4", offset=8).tolist(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_preview_input_resolution_rejects_traversal_and_missing_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pcd_dir = root / "pcd"
            pcd_dir.mkdir(parents=True)
            write_binary_pcd(pcd_dir / "venue.pcd", np.zeros((3, 3), dtype=np.float32))
            (root / "outside.pcd").write_bytes(b"not reachable")
            resolved = resolve_pcd_input(pcd_dir, "venue")

            self.assertEqual(resolved, pcd_dir.resolve() / "venue.pcd")
            for bad in ("../outside", "/etc/passwd", "..", "", "venue/../outside", "a b"):
                with self.assertRaises(ValueError):
                    resolve_pcd_input(pcd_dir, bad)
            with self.assertRaises(FileNotFoundError):
                resolve_pcd_input(pcd_dir, "missing")

    def test_preview_payload_matches_the_browser_protocol(self):
        points = np.array([[float(x), float(y), 0.5] for x in range(6) for y in range(6)], dtype=np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            pcd_dir = Path(tmp) / "pcd"
            write_binary_pcd(pcd_dir / "small.pcd", points)

            path = resolve_pcd_input(pcd_dir, "small")
            body, sent, total = build_cloud_frame(read_binary_pcd(path), 12)

            self.assertEqual(total, 36)
            self.assertEqual(sent, 12)
            self.assertEqual(body[:4], b"MAP1")
            self.assertEqual(len(body), 8 + 12 * 12)
            self.assertEqual(struct.unpack("<I", body[4:8])[0], 12)
            self.assertAlmostEqual(float(np.frombuffer(body, dtype="<f4", offset=8)[2]), 0.5)


if __name__ == "__main__":
    unittest.main()
