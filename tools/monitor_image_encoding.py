#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class EncodingMonitor(Node):
    def __init__(self, topic, duration):
        super().__init__('encoding_monitor')
        qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE)
        self.topic = topic
        self.duration = duration
        self.counts = defaultdict(int)
        self.start = time.time()
        self.create_subscription(Image, topic, self.cb, qos)

    def cb(self, msg):
        self.counts[msg.encoding] += 1


def main():
    parser = argparse.ArgumentParser(description="Monitor image encoding counts.")
    parser.add_argument('--topic', required=True, help='Image topic to monitor')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='Monitoring duration in seconds (default: 3.0)')
    args = parser.parse_args()

    rclpy.init()
    node = EncodingMonitor(args.topic, args.duration)
    end_time = time.time() + args.duration
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    total = sum(node.counts.values())
    print(f"topic {args.topic}")
    print(f"samples {total}")
    if total == 0:
        print("no samples received")
    else:
        for enc, count in sorted(node.counts.items(), key=lambda x: (-x[1], x[0])):
            print(f"encoding {enc}: {count}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
