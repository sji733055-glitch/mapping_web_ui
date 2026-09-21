from io import BytesIO
import unittest

from backend.mapping_core import MapExportConfig
from backend.mapping_server import (
    RequestHandler,
    export_config_from_overrides,
    occupancy_preview_headers,
)


class RequestHandlerTests(unittest.TestCase):
    def test_export_overrides_accept_only_known_height_modes(self):
        base = MapExportConfig(height_mode="ground")

        absolute = export_config_from_overrides(base, {"height_mode": "absolute"})

        self.assertEqual(absolute.height_mode, "absolute")
        with self.assertRaisesRegex(ValueError, "ground 或 absolute"):
            export_config_from_overrides(base, {"height_mode": "local"})

    def test_export_overrides_accept_structure_voxel_filter(self):
        base = MapExportConfig(filter_mode="radius", filter_voxel_size=0.1)

        config = export_config_from_overrides(
            base, {"filter_mode": "voxel", "filter_voxel_size": "0.08"}
        )

        self.assertEqual(config.filter_mode, "voxel")
        self.assertEqual(config.filter_voxel_size, 0.08)
        with self.assertRaisesRegex(ValueError, "voxel、radius 或 none"):
            export_config_from_overrides(base, {"filter_mode": "sor"})
        with self.assertRaisesRegex(ValueError, "0.02–1.0"):
            export_config_from_overrides(base, {"filter_voxel_size": 0.01})

    def test_preview_headers_report_structure_voxel_filter_statistics(self):
        headers = occupancy_preview_headers({
            "width": 10, "height": 8, "resolution": 0.05,
            "origin_x": -1.0, "origin_y": -2.0,
            "z_min": 0.05, "z_max": 1.5, "height_mode": "absolute",
            "slice_points": 100, "filtered_points": 93,
            "occupied_cells": 20, "preview_stride": 1,
            "preview_occupied_cells": 20,
            "filter_mode": "voxel", "filter_removed_points": 7,
            "filter_voxel_size": 0.1, "filter_voxels": 80,
            "filter_columns": 30, "filter_removed_columns": 4,
        })

        self.assertEqual(headers["X-Map-Filter-Mode"], "voxel")
        self.assertEqual(headers["X-Map-Filter-Removed"], "7")
        self.assertEqual(headers["X-Map-Filter-Voxel-Size"], "0.1")
        self.assertEqual(headers["X-Map-Filter-Removed-Columns"], "4")

    def test_error_headers_do_not_require_a_parsed_request_path(self):
        handler = RequestHandler.__new__(RequestHandler)
        handler._headers_buffer = []
        handler.request_version = "HTTP/1.1"
        handler.wfile = BytesIO()

        self.assertFalse(hasattr(handler, "path"))
        handler.end_headers()

        headers = handler.wfile.getvalue()
        self.assertIn(b"X-Content-Type-Options: nosniff\r\n", headers)
        self.assertIn(b"Referrer-Policy: no-referrer\r\n", headers)
        self.assertIn(b"Cache-Control: no-store\r\n", headers)


if __name__ == "__main__":
    unittest.main()
