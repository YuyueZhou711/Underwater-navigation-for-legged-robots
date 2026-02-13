#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import time
from bisect import bisect_left
from statistics import mean, median

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class StereoOffsetSampler(Node):
    def __init__(self, left_topic, right_topic, duration):
        super().__init__('stereo_offset_sampler')
        qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE)
        self.left_ts = []
        self.right_ts = []
        self.duration = duration
        self.start = time.time()
        self.create_subscription(Image, left_topic, self.cb_left, qos)
        self.create_subscription(Image, right_topic, self.cb_right, qos)

    @staticmethod
    def stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def cb_left(self, msg):
        self.left_ts.append(self.stamp_to_sec(msg.header.stamp))

    def cb_right(self, msg):
        self.right_ts.append(self.stamp_to_sec(msg.header.stamp))


def nearest_abs_diffs(a, b):
    if not a or not b:
        return []
    b_sorted = sorted(b)
    diffs = []
    for t in a:
        i = bisect_left(b_sorted, t)
        candidates = []
        if i < len(b_sorted):
            candidates.append(abs(t - b_sorted[i]))
        if i > 0:
            candidates.append(abs(t - b_sorted[i - 1]))
        if candidates:
            diffs.append(min(candidates))
    return diffs


def main():
    parser = argparse.ArgumentParser(description="Measure stereo timestamp offset.")
    parser.add_argument('--left', default='/silver2/stereo_left/image_raw/image_color',
                        help='Left image topic (default: /silver2/stereo_left/image_raw/image_color)')
    parser.add_argument('--right', default='/silver2/stereo_right/image_raw/image_color',
                        help='Right image topic (default: /silver2/stereo_right/image_raw/image_color)')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='Sampling duration in seconds (default: 3.0)')
    args = parser.parse_args()

    rclpy.init()
    node = StereoOffsetSampler(args.left, args.right, args.duration)

    end_time = time.time() + args.duration
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    diffs = nearest_abs_diffs(node.left_ts, node.right_ts)
    print(f"left_samples {len(node.left_ts)} right_samples {len(node.right_ts)}")
    if diffs:
        print(
            "stereo_nearest_dt_mean", round(mean(diffs), 4),
            "median", round(median(diffs), 4),
            "max", round(max(diffs), 4)
        )
    else:
        print("no stereo samples collected; check topics and QoS")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
