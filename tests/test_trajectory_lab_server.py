"""Backend coverage for the isolated trajectory laboratory.

The laboratory is the only feature in this project that launches additional
real ROS child processes, so these tests pin down the properties that matter
most and that can still be checked without a ROS installation or a radar:

* the scene guard rejects the start/goal/obstacle input that the real planner
  must never receive, and its world->cell mapping agrees with the browser;
* stopping the laboratory signals exactly its own process groups and leaves
  unrelated processes running;
* the HTTP surface routes to the laboratory controller and turns its failures
  into the documented status codes.
"""

from __future__ import annotations

from functools import partial
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

import numpy as np

from backend.mapping_server import MappingHttpServer, MappingSupervisor, RequestHandler, WebSocketHub
from backend.terrain_core import (
    EditorMap,
    LAYER_TERRAIN,
    MapMetadata,
    write_pgm,
    write_terrain_msgpack,
)

MAP_NAME = "trajectory_lab_fixture"
WIDTH, HEIGHT, RESOLUTION = 8, 6, 0.5
ORIGIN_X, ORIGIN_Y = -2.0, 1.0
# Cell (2, 3) is a hard obstacle; every other cell is free.
OBSTACLE_CELL = (2, 3)


def _world_center(cell_x: int, cell_y: int, yaw: float):
    """Mirror the browser's ``cellToWorld`` for a cell centre."""
    local_x, local_y = (cell_x + 0.5) * RESOLUTION, (cell_y + 0.5) * RESOLUTION
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        ORIGIN_X + cosine * local_x - sine * local_y,
        ORIGIN_Y + sine * local_x + cosine * local_y,
    )


class _Settings:
    trajectory_lab_log_dir = "/tmp/trajectory_lab_logs"


def _make_lab_controller(output_root) -> MappingSupervisor:
    """Build the node object without rclpy, carrying the real lab state.

    ``MappingSupervisor.__init__`` needs a live ROS context, so these tests bind
    the same attributes it installs and exercise the real methods on top.
    """
    controller = MappingSupervisor.__new__(MappingSupervisor)
    controller.output_root = Path(output_root)
    controller.settings = _Settings()
    controller._trajectory_lock = threading.RLock()
    controller._trajectory_processes = {}
    controller._trajectory_state = "STOPPED"
    controller._trajectory_message = "隔离规划器尚未启动"
    controller._trajectory_map_name = ""
    controller._trajectory_start = (0.0, 0.0, 0.0)
    controller._trajectory_goal = (0.0, 0.0, 0.0)
    controller._trajectory_obstacles = []
    controller._trajectory_parameters = {}
    controller._trajectory_global_path = []
    controller._trajectory_minco_path = []
    controller._trajectory_plan_started_at = 0.0
    controller._trajectory_global_received_at = 0.0
    controller._trajectory_minco_received_at = 0.0
    controller._trajectory_constraints = {"free": 0, "directional": 0, "blocked": 0, "unknown": 0}
    controller._trajectory_cmd = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0, "received": 0}
    controller._trajectory_generation = 0
    controller._trajectory_last_cloud_publish = 0.0
    controller._trajectory_last_goal_publish = 0.0
    return controller


class TrajectorySceneGuardTests(unittest.TestCase):
    """The guard is what keeps garbage away from the real planner."""

    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.root = Path(self._temporary.name)
        self.map_dir = self.root / "map"
        self.map_dir.mkdir(parents=True)

    def tearDown(self):
        self._temporary.cleanup()

    def _write_scene(self, *, yaw: float = 0.0, with_yaml: bool = True) -> Path:
        values = np.zeros(WIDTH * HEIGHT, dtype=np.uint8)
        values[OBSTACLE_CELL[1] * WIDTH + OBSTACLE_CELL[0]] = 1
        direction = np.zeros(WIDTH * HEIGHT, dtype=np.uint8)
        editor_map = EditorMap(
            LAYER_TERRAIN,
            MapMetadata(WIDTH, HEIGHT, RESOLUTION, ORIGIN_X, ORIGIN_Y, yaw),
            values,
            direction,
        )
        terrain_path = self.map_dir / f"{MAP_NAME}_terrain.msgpack"
        write_terrain_msgpack(terrain_path, editor_map)
        if with_yaml:
            write_pgm(self.map_dir / f"{MAP_NAME}.pgm", np.full((HEIGHT, WIDTH), 254, dtype=np.uint8))
            (self.map_dir / f"{MAP_NAME}.yaml").write_text(
                f"image: {MAP_NAME}.pgm\n"
                "mode: trinary\n"
                f"resolution: {RESOLUTION}\n"
                f"origin: [{ORIGIN_X}, {ORIGIN_Y}, {yaw}]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.25\n",
                encoding="utf-8",
            )
        return terrain_path

    def _controller(self) -> MappingSupervisor:
        return _make_lab_controller(self.root)

    def test_accepts_free_start_and_goal_and_returns_map_paths(self):
        self._write_scene()

        yaml_path, terrain_path = self._controller()._validate_trajectory_scene(
            MAP_NAME, (0.0, 2.0, 0.0), (1.0, 3.5, 0.0), []
        )

        self.assertEqual(yaml_path, self.map_dir / f"{MAP_NAME}.yaml")
        self.assertEqual(terrain_path, self.map_dir / f"{MAP_NAME}_terrain.msgpack")

    def test_rejects_start_and_goal_on_a_hard_obstacle_cell(self):
        self._write_scene()
        controller = self._controller()
        obstacle_world = _world_center(*OBSTACLE_CELL, 0.0)

        with self.assertRaisesRegex(ValueError, "车体位置落在 label 1 障碍格上"):
            controller._validate_trajectory_scene(
                MAP_NAME, (*obstacle_world, 0.0), (1.0, 3.5, 0.0), []
            )
        with self.assertRaisesRegex(ValueError, "目标位置落在 label 1 障碍格上"):
            controller._validate_trajectory_scene(
                MAP_NAME, (0.0, 2.0, 0.0), (*obstacle_world, 0.0), []
            )

    def test_rejects_poses_and_obstacles_outside_the_terrain_map(self):
        self._write_scene()
        controller = self._controller()

        with self.assertRaisesRegex(ValueError, "车体位置不在 terrain 地图范围内"):
            controller._validate_trajectory_scene(MAP_NAME, (-99.0, 2.0, 0.0), (1.0, 3.5, 0.0), [])
        with self.assertRaisesRegex(ValueError, "目标位置不在 terrain 地图范围内"):
            controller._validate_trajectory_scene(MAP_NAME, (0.0, 2.0, 0.0), (99.0, 3.5, 0.0), [])
        with self.assertRaisesRegex(ValueError, "临时障碍不能放在 terrain 地图范围外"):
            controller._validate_trajectory_scene(
                MAP_NAME, (0.0, 2.0, 0.0), (1.0, 3.5, 0.0), [(99.0, 2.0)]
            )

    def test_rejects_traversal_map_names_and_missing_files(self):
        self._write_scene()
        controller = self._controller()

        with self.assertRaises(ValueError):
            controller._validate_trajectory_scene(
                "../escape", (0.0, 2.0, 0.0), (1.0, 3.5, 0.0), []
            )
        with self.assertRaisesRegex(FileNotFoundError, "找不到地图 YAML"):
            controller._validate_trajectory_scene(
                "absent_map", (0.0, 2.0, 0.0), (1.0, 3.5, 0.0), []
            )

    def test_requires_the_terrain_channel(self):
        self._write_scene(with_yaml=True)
        (self.map_dir / f"{MAP_NAME}_terrain.msgpack").unlink()

        with self.assertRaisesRegex(FileNotFoundError, "找不到 terrain 地图"):
            self._controller()._validate_trajectory_scene(
                MAP_NAME, (0.0, 2.0, 0.0), (1.0, 3.5, 0.0), []
            )

    def test_world_to_cell_agrees_with_the_browser_for_a_rotated_map(self):
        yaw = math.pi / 2
        self._write_scene(yaw=yaw)
        controller = self._controller()
        obstacle_world = _world_center(*OBSTACLE_CELL, yaw)

        # The browser sends cell centres; the backend must floor them back onto
        # the very same cell even when the map origin carries a rotation.
        self.assertEqual(
            controller._trajectory_cell(
                self._load_editor_map(), (*obstacle_world, 0.0)
            ),
            OBSTACLE_CELL,
        )
        with self.assertRaisesRegex(ValueError, "车体位置落在 label 1"):
            controller._validate_trajectory_scene(
                MAP_NAME, (*obstacle_world, 0.0), (0.0, 2.0, 0.0), []
            )

    def _load_editor_map(self):
        from backend.terrain_core import load_terrain_editor_map

        return load_terrain_editor_map(
            self.map_dir / f"{MAP_NAME}_terrain.msgpack",
            yaml_path=self.map_dir / f"{MAP_NAME}.yaml",
        )


class TrajectoryProcessLifecycleTests(unittest.TestCase):
    """Stopping the laboratory must stay surgical."""

    def _controller(self) -> MappingSupervisor:
        return _make_lab_controller(tempfile.gettempdir())

    @staticmethod
    def _spawn(stream) -> subprocess.Popen:
        # start_new_session mirrors the real launcher: the child leads its own
        # process group, so signalling the group reaches exactly this child.
        return subprocess.Popen(
            ["sleep", "60"],
            stdin=subprocess.DEVNULL,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )

    def test_stop_signals_only_its_own_process_groups(self):
        controller = self._controller()
        controller._trajectory_state = "PLANNING"
        bystander = self._spawn(subprocess.DEVNULL)
        streams = []
        children = []
        try:
            for key in ("map_server", "nav_executor"):
                stream = open(os.devnull, "wb")
                streams.append(stream)
                process = self._spawn(stream)
                children.append(process)
                controller._trajectory_processes[key] = (process, stream)

            ok, message = controller.stop_trajectory_lab()

            self.assertTrue(ok)
            self.assertIn("已停止", message)
            self.assertEqual(controller._trajectory_state, "STOPPED")
            self.assertEqual(controller._trajectory_processes, {})
            deadline = time.monotonic() + 5.0
            for process in children:
                while process.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertIsNotNone(process.poll(), "隔离子进程未被停止")
            # A name-based kill would have taken this one down as well.
            self.assertIsNone(bystander.poll(), "停止隔离规划器误杀了无关进程")
        finally:
            bystander.kill()
            bystander.wait(timeout=5)
            for process in children:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
            for stream in streams:
                stream.close()

    def test_stop_is_a_no_op_when_nothing_was_started(self):
        controller = self._controller()

        ok, message = controller.stop_trajectory_lab()

        self.assertTrue(ok)
        self.assertEqual(controller._trajectory_state, "STOPPED")
        self.assertIn("已停止", message)

    def test_status_reports_an_exited_child_as_an_error(self):
        controller = self._controller()
        controller._trajectory_state = "PLANNING"
        stream = open(os.devnull, "wb")
        try:
            process = subprocess.Popen(
                ["true"], stdin=subprocess.DEVNULL, stdout=stream,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            controller._trajectory_processes["nav_executor"] = (process, stream)
            process.wait(timeout=5)

            status = controller.trajectory_status()

            self.assertEqual(status["state"], "ERROR")
            self.assertIn("nav_executor", status["message"])
            self.assertEqual(status["managed_processes"], [])
        finally:
            stream.close()

    def test_obstacle_updates_are_refused_while_stopped(self):
        controller = self._controller()
        controller._trajectory_state = "STOPPED"

        ok, message = controller.update_trajectory_obstacles({"obstacles": [[0.0, 0.0]]})

        self.assertFalse(ok)
        self.assertIn("请先启动", message)


class _StubController:
    """Minimal stand-in mounting the real HTTP dispatch path."""

    def __init__(self):
        self.settings = _Settings()
        self.settings.http_log = False
        self.calls: list[tuple[str, object]] = []
        self.error: Exception | None = None
        self.result = (True, "隔离规划器已启动")

    def trajectory_status(self):
        return {"state": "STOPPED", "isolated": True, "namespace": "/mapping/trajectory_lab"}

    def start_trajectory_lab(self, payload):
        self.calls.append(("start", payload))
        if self.error is not None:
            raise self.error
        return self.result

    def update_trajectory_obstacles(self, payload):
        self.calls.append(("obstacles", payload))
        if self.error is not None:
            raise self.error
        return self.result

    def stop_trajectory_lab(self):
        self.calls.append(("stop", None))
        if self.error is not None:
            raise self.error
        return self.result


class TrajectoryHttpSurfaceTests(unittest.TestCase):
    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.controller = _StubController()
        self.server = MappingHttpServer(
            ("127.0.0.1", 0),
            partial(RequestHandler, directory=self._temporary.name),
            self.controller,
            WebSocketHub(),
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)
        self._temporary.cleanup()

    def _request(self, path: str, payload=None):
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            f"{self.base}{path}",
            data=data,
            method="POST" if data is not None or path != "/api/trajectory" else "GET",
            headers={"Content-Type": "application/json"} if data is not None else {},
        )
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read().decode("utf-8"))

    def test_get_trajectory_reports_the_laboratory_status(self):
        status, body = self._request("/api/trajectory")

        self.assertEqual(status, 200)
        self.assertTrue(body["isolated"])
        self.assertEqual(body["namespace"], "/mapping/trajectory_lab")

    def test_start_and_obstacles_reach_the_controller_with_the_payload(self):
        status, body = self._request(
            "/api/trajectory/start", {"map_name": MAP_NAME, "start": [0, 1]}
        )
        self.assertEqual(status, 200)
        self.assertTrue(body["ok"])
        self.assertEqual(body["message"], "隔离规划器已启动")
        self.assertEqual(body["trajectory"]["state"], "STOPPED")
        self.assertEqual(self.controller.calls[0], ("start", {"map_name": MAP_NAME, "start": [0, 1]}))

        status, _body = self._request("/api/trajectory/obstacles", {"obstacles": [[1.0, 2.0]]})
        self.assertEqual(status, 200)
        self.assertEqual(self.controller.calls[1], ("obstacles", {"obstacles": [[1.0, 2.0]]}))

    def test_stop_dispatches_without_a_body(self):
        status, body = self._request("/api/trajectory/stop", {})

        self.assertEqual(status, 200)
        self.assertTrue(body["ok"])
        self.assertEqual(self.controller.calls, [("stop", None)])

    def test_a_refused_command_becomes_409(self):
        self.controller.result = (False, "隔离规划器未运行，请先启动")

        status, body = self._request("/api/trajectory/obstacles", {"obstacles": []})

        self.assertEqual(status, 409)
        self.assertFalse(body["ok"])
        self.assertIn("请先启动", body["message"])

    def test_validation_and_missing_file_errors_map_to_400_and_404(self):
        self.controller.error = ValueError("车体位置落在 label 1 障碍格上")
        status, body = self._request("/api/trajectory/start", {"map_name": MAP_NAME})
        self.assertEqual(status, 400)
        self.assertIn("label 1", body["message"])

        self.controller.error = FileNotFoundError("找不到地图 YAML：absent.yaml")
        status, body = self._request("/api/trajectory/start", {"map_name": "absent"})
        self.assertEqual(status, 404)
        self.assertIn("找不到地图 YAML", body["message"])

    def test_an_oversized_body_is_rejected_before_it_reaches_the_controller(self):
        status, body = self._request(
            "/api/trajectory/start", {"obstacles": [[0.0, 0.0]] * 200_000}
        )

        self.assertEqual(status, 400)
        self.assertIn("不能超过", body["message"])
        self.assertEqual(self.controller.calls, [])


if __name__ == "__main__":
    unittest.main()
