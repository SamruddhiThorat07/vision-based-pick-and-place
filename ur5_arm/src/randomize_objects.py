#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math
import subprocess

class RandomizeObjects(Node):
    def __init__(self):
        super().__init__('randomize_objects')

        # same seed gives the same layout so a failed run can be repeated
        self.declare_parameter('seed', -1)
        self.seed = self.get_parameter('seed').value
        if self.seed < 0:
            self.seed = random.randint(0, 9999)
        random.seed(self.seed)

        self.objects = ["red_sphere", "green_sphere", "blue_sphere", "red_sphere_2", "green_sphere_2", "blue_sphere_2"]

        # table area (world frame) that is inside the point cloud crop and the arm can reach
        self.x_range = [-0.25, 0.25]
        self.y_range = [-0.25, 0.25]
        self.z = 1.05

        # min distance between 2 balls so the gripper fingers dont hit the next one
        self.min_gap = 0.1

    def get_random_points(self):

        points = []

        while len(points) < len(self.objects):
            x = random.uniform(self.x_range[0], self.x_range[1])
            y = random.uniform(self.y_range[0], self.y_range[1])

            # keep the point only if it is far enough from all the others
            if all(math.dist((x, y), p) > self.min_gap for p in points):
                points.append((x, y))

        return points

    def set_pose(self, name, x, y):

        req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {self.z}}}'

        cmd = [
            "ign", "service",
            "-s", "/world/table_world/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "2000",
            "--req", req
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)
        return "data: true" in result.stdout

    def randomize(self):

        self.get_logger().info(f"randomizing objects with seed: {self.seed}")
        points = self.get_random_points()

        for name, (x, y) in zip(self.objects, points):
            if self.set_pose(name, x, y):
                self.get_logger().info(f"{name} placed at x: {x:.2f} y: {y:.2f}")
            else:
                self.get_logger().error(f"could not move {name}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomizeObjects()
    node.randomize()
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
