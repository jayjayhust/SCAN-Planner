#!/usr/bin/env python3
"""Interactively record odometry positions as ROS 2 SCAN-Planner parameters."""

import argparse
import math
import os
import select
import sys
import termios
import tty
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args


DEFAULT_OUTPUT = os.path.join(os.getcwd(), "keypoints.yaml")
DEFAULT_AVG_WINDOW = 0.5
MAX_SAMPLES = 200


def format_float(value):
    text = f"{float(value):.6f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def atomic_write(path, content):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        handle.write(content)
    os.replace(temporary, path)


class KeypointRecorder(Node):
    def __init__(self, odom_topic, output_path, avg_window=DEFAULT_AVG_WINDOW):
        super().__init__("keypoint_recorder")
        self.odom_topic = odom_topic
        self.output_path = os.path.abspath(output_path)
        self.avg_window = max(0.0, float(avg_window))
        self.latest_odom = None
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.waypoints = []
        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)

    def odom_callback(self, message):
        self.latest_odom = message
        position = message.pose.pose.position
        point = (float(position.x), float(position.y), float(position.z))
        if not all(math.isfinite(value) for value in point):
            return
        stamp = message.header.stamp
        sample_time = stamp.sec + stamp.nanosec * 1e-9
        self.samples.append((sample_time, point))
        if self.avg_window > 0.0:
            cutoff = sample_time - self.avg_window
            while self.samples and self.samples[0][0] < cutoff:
                self.samples.popleft()
        else:
            while len(self.samples) > 1:
                self.samples.popleft()

    def current_point(self):
        if self.latest_odom is None or not self.samples:
            return None
        # Keep the most recent horizontal position but average z over a short
        # window: a trotting quadruped bobs its trunk up and down, and a single
        # instantaneous z corrupts floor heights and stair-ramp waypoints.
        latest_x, latest_y = self.samples[-1][1][:2]
        avg_z = sum(sample[1][2] for sample in self.samples) / len(self.samples)
        return (latest_x, latest_y, avg_z)

    def record_current(self):
        point = self.current_point()
        if point is None:
            self.get_logger().warning("No valid odometry received yet")
            return
        self.waypoints.append(point)
        self.get_logger().info(f"Recorded waypoint {len(self.waypoints)}: {point}")

    def replace_current(self, index):
        point = self.current_point()
        if point is None or not 1 <= index <= len(self.waypoints):
            self.get_logger().warning("Invalid waypoint index or odometry")
            return
        self.waypoints[index - 1] = point

    def delete_waypoint(self, index):
        if 1 <= index <= len(self.waypoints):
            self.waypoints.pop(index - 1)
        else:
            self.get_logger().warning("Invalid waypoint index")

    def build_yaml(self):
        values = [format_float(value) for point in self.waypoints for value in point]
        return (
            "scan_planner_node:\n"
            "  ros__parameters:\n"
            f"    fsm.waypoints: [{', '.join(values)}]\n"
        )

    def save(self):
        atomic_write(self.output_path, self.build_yaml())
        self.get_logger().info(f"Saved {len(self.waypoints)} waypoint(s) to {self.output_path}")

    def prompt_index(self, settings, action):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            return int(input(f"{action} waypoint index (1-{len(self.waypoints)}): ").strip())
        except ValueError:
            return -1
        finally:
            tty.setcbreak(sys.stdin.fileno())

    def run(self):
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard control requires a TTY")
        if self.avg_window > 0.0:
            self.get_logger().info(
                "Recording '%s'; z averaged over %.2fs window", self.odom_topic, self.avg_window)
        else:
            self.get_logger().info(
                "Recording '%s'; instantaneous pose (z averaging disabled)", self.odom_topic)
        print("Enter/Space/a: add, r: replace, d: delete, u: undo, l: list, s: save, q: save+quit")
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if not ready:
                    continue
                key = sys.stdin.read(1).lower()
                if key in ("\n", "\r", " ", "a"):
                    self.record_current()
                elif key == "r":
                    self.replace_current(self.prompt_index(settings, "Replace"))
                elif key == "d":
                    self.delete_waypoint(self.prompt_index(settings, "Delete"))
                elif key == "u" and self.waypoints:
                    self.waypoints.pop()
                elif key == "l":
                    print("\n".join(f"{i}: {point}" for i, point in enumerate(self.waypoints, 1)))
                elif key == "s":
                    self.save()
                elif key in ("q", "\x03"):
                    self.save()
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom", default="/LIO/odom_vehicle")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--average-window",
        type=float,
        default=DEFAULT_AVG_WINDOW,
        help="seconds of odometry averaged for z to reject body-bob (0 = instantaneous)",
    )
    arguments = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=sys.argv)
    recorder = KeypointRecorder(arguments.odom, arguments.output, arguments.average_window)
    try:
        recorder.run()
    except (KeyboardInterrupt, RuntimeError) as error:
        recorder.get_logger().error(str(error))
    finally:
        recorder.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
