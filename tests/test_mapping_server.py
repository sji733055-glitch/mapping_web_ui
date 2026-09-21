from io import BytesIO
import unittest

from backend.mapping_core import MapExportConfig
from backend.mapping_server import RequestHandler, export_config_from_overrides


class RequestHandlerTests(unittest.TestCase):
    def test_export_overrides_accept_only_known_height_modes(self):
        base = MapExportConfig(height_mode="ground")

        absolute = export_config_from_overrides(base, {"height_mode": "absolute"})

        self.assertEqual(absolute.height_mode, "absolute")
        with self.assertRaisesRegex(ValueError, "ground 或 absolute"):
            export_config_from_overrides(base, {"height_mode": "local"})

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
