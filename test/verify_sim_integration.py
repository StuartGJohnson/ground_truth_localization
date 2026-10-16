"""Check the live Isaac Sim + ground_truth_localization DDS graph.

Run with system ROS 2 Humble after starting Isaac Sim and the localization
launch file in separate shells. This script starts neither process.
"""

import argparse
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage


def stamp_key(message):
    stamp = message.header.stamp
    return stamp.sec, stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("verify_ground_truth_localization")
    truth = {}
    odom = {}
    dynamic_tf = {}
    seen = {"clock": False, "gps": False, "static_tf": False}
    stream_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    static_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def on_truth(message):
        truth[stamp_key(message)] = message

    def on_odom(message):
        odom[stamp_key(message)] = message

    def on_tf(message):
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                dynamic_tf[stamp_key(transform)] = transform

    def on_static_tf(message):
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                t, q = transform.transform.translation, transform.transform.rotation
                seen["static_tf"] = (
                    (t.x, t.y, t.z) == (0.0, 0.0, 0.0)
                    and (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)
                )

    def on_gps(message):
        seen["gps"] = (
            message.header.frame_id == "gps_link"
            and message.status.status == 0
            and message.status.service == 1
        )

    subscriptions = [
        node.create_subscription(Odometry, "/ground_truth/odom", on_truth, stream_qos),
        node.create_subscription(Odometry, "/odom", on_odom, stream_qos),
        node.create_subscription(TFMessage, "/tf", on_tf, stream_qos),
        node.create_subscription(TFMessage, "/tf_static", on_static_tf, static_qos),
        node.create_subscription(NavSatFix, "/gps/fix", on_gps, stream_qos),
        node.create_subscription(Clock, "/clock", lambda _: seen.__setitem__("clock", True), stream_qos),
    ]
    _ = subscriptions  # Keep subscriptions alive for the duration of the check.

    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            for key in truth.keys() & odom.keys() & dynamic_tf.keys():
                source, output, transform = truth[key], odom[key], dynamic_tf[key]
                if source.header.frame_id != "map" or source.child_frame_id != "base_link":
                    continue
                if output.header.frame_id != "odom" or output.child_frame_id != "base_link":
                    continue
                if source.pose != output.pose or source.twist != output.twist:
                    continue
                p, q = output.pose.pose.position, output.pose.pose.orientation
                t, tq = transform.transform.translation, transform.transform.rotation
                if (p.x, p.y, p.z) != (t.x, t.y, t.z) or q != tq:
                    continue
                if all(seen.values()):
                    print("INTEGRATION_PASS /clock /gps/fix /ground_truth/odom /odom map->odom odom->base_link")
                    return
        raise SystemExit(
            "INTEGRATION_FAIL missing or inconsistent stream: "
            f"seen={seen}, truth={len(truth)}, odom={len(odom)}, tf={len(dynamic_tf)}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
