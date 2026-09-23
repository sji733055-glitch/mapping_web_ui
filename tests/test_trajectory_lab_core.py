from pathlib import Path
import unittest

from backend.trajectory_lab_core import (
    LAB_NAMESPACE,
    LAB_TOPICS,
    MAX_LAB_OBSTACLES,
    build_lab_commands,
    decimate_path,
    validate_obstacles,
    validate_parameters,
    validate_pose,
)


class TrajectoryLabCoreTests(unittest.TestCase):
    def test_all_runtime_topics_are_isolated(self):
        commands = build_lab_commands(
            terrain_path=Path("/tmp/arena_terrain.msgpack"),
            yaml_path=Path("/tmp/arena.yaml"),
            config_dir=Path("/tmp/config"),
            parameters=validate_parameters({}),
        )
        arguments = [argument for command in commands.values() for argument in command]
        for unsafe in ("/goal_pose", "/Odometry", "/cloud_registered", "/cmd_vel"):
            self.assertNotIn(unsafe, arguments)
        for topic in LAB_TOPICS.values():
            self.assertTrue(topic.startswith(f"{LAB_NAMESPACE}/"))
        joined = " ".join(arguments)
        self.assertIn("node.topics.cmd_vel_pub:=/mapping/trajectory_lab/cmd_vel", joined)
        self.assertIn("planner.rog_map.ros_callback.cloud_topic:=/mapping/trajectory_lab/obstacles", joined)
        self.assertIn("planner.global_frame:=map", joined)

    def test_parameter_whitelist_and_relationship(self):
        values = validate_parameters({"safe_dist": 0.4, "collision_dist": 0.3, "max_velocity": 1.2})
        self.assertEqual(values["max_velocity"], 1.2)
        with self.assertRaisesRegex(ValueError, "不支持"):
            validate_parameters({"node.topics.cmd_vel_pub": "/cmd_vel"})
        with self.assertRaisesRegex(ValueError, "不能小于"):
            validate_parameters({"safe_dist": 0.2, "collision_dist": 0.3})
        with self.assertRaises(ValueError):
            validate_parameters({"max_velocity": float("nan")})

    def test_pose_obstacle_and_path_bounds(self):
        self.assertEqual(validate_pose([1, 2], "起点"), (1.0, 2.0, 0.0))
        with self.assertRaises(ValueError):
            validate_pose([1], "起点")
        self.assertEqual(validate_obstacles([[1, 2], [3.5, 4]]), [(1.0, 2.0), (3.5, 4.0)])
        with self.assertRaises(ValueError):
            validate_obstacles([[0, 0]] * (MAX_LAB_OBSTACLES + 1))
        reduced = decimate_path(((float(index), 0.0) for index in range(100)), limit=10)
        self.assertLessEqual(len(reduced), 11)
        self.assertEqual(reduced[-1], [99.0, 0.0])


if __name__ == "__main__":
    unittest.main()
