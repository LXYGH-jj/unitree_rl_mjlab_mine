#!/usr/bin/env python3

import argparse
import array
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def load_depth_values(path: Path, width: int, height: int) -> list[float]:
    values: list[float] = []
    with path.open("r", encoding="utf-8") as stream:
        for token in stream.read().split():
            values.append(float(token))

    expected = width * height
    if len(values) != expected:
        raise ValueError(f"expected {expected} values for {width}x{height}, got {len(values)}")
    return values


def make_image(values: list[float], width: int, height: int, encoding: str, stamp) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = "go2_depth_local_test"
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.is_bigendian = 0

    if encoding == "16UC1":
        clipped = [max(0, min(65535, int(round(value)))) for value in values]
        data = array.array("H", clipped)
        msg.step = width * 2
    elif encoding == "32FC1":
        data = array.array("f", values)
        msg.step = width * 4
    else:
        raise ValueError(f"unsupported encoding for local publisher: {encoding}")

    msg.data = data.tobytes()
    return msg


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish a saved depth txt as a ROS2 sensor_msgs/Image.")
    parser.add_argument("--source", required=True, type=Path, help="Whitespace separated depth txt file.")
    parser.add_argument("--topic", default="/go2_depth/local_depth/image_raw", help="ROS2 topic to publish.")
    parser.add_argument("--width", type=int, default=640, help="Input image width.")
    parser.add_argument("--height", type=int, default=480, help="Input image height.")
    parser.add_argument("--encoding", choices=("16UC1", "32FC1"), default="16UC1", help="ROS2 Image encoding.")
    parser.add_argument("--rate", type=float, default=10.0, help="Publish rate in Hz.")
    args = parser.parse_args()

    values = load_depth_values(args.source, args.width, args.height)

    rclpy.init()
    node = rclpy.create_node("go2_depth_local_image_publisher")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    publisher = node.create_publisher(Image, args.topic, qos)

    period = 1.0 / max(args.rate, 1.0e-6)
    node.get_logger().info(
        f"Publishing {args.width}x{args.height} {args.encoding} depth from {args.source} on {args.topic}"
    )

    def publish_once() -> None:
        publisher.publish(make_image(values, args.width, args.height, args.encoding, node.get_clock().now().to_msg()))

    timer = node.create_timer(period, publish_once)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
