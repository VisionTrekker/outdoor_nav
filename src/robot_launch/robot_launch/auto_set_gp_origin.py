#!/usr/bin/env python3
"""Auto-set and verify MAVROS global_position/gp_origin."""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geographic_msgs.msg import GeoPointStamped

class AutoSetGpOrigin(Node):
    def __init__(self):
        super().__init__("auto_set_gp_origin")

        self.lat = self.declare_parameter("latitude", 0.0).value
        self.lon = self.declare_parameter("longitude", 0.0).value
        self.alt = self.declare_parameter("altitude", 0.0).value
        self.delay_s = self.declare_parameter("startup_delay_s", 10.0).value
        self.tolerance_deg = self.declare_parameter("verify_tolerance_deg", 1e-6).value
        self.max_retries = self.declare_parameter("max_retries", 3).value
        self.verify_timeout_s = self.declare_parameter("verify_timeout_s", 5.0).value

        if abs(self.lat) > 90.0 or abs(self.lon) > 180.0:
            self.get_logger().error("Invalid latitude/longitude parameters")
            sys.exit(1)

        self.pub = self.create_publisher(GeoPointStamped, "/mavros/global_position/set_gp_origin", 10)
        self.sub = self.create_subscription(
            GeoPointStamped, "/mavros/global_position/gp_origin", self.on_gp_origin, qos_profile_sensor_data
        )

        self.verified = False
        self.retry_count = 0
        self.startup_timer = None
        self.verify_timer = None
        self.shutdown_timer = None

        self.get_logger().info(
            f"Will set gp_origin to lat={self.lat:.8f}, lon={self.lon:.8f}, alt={self.alt:.3f} "
            f"after {self.delay_s}s delay"
        )
        self.startup_timer = self.create_timer(self.delay_s, self.on_startup_delay)

    def on_startup_delay(self):
        if self.startup_timer is not None:
            self.startup_timer.cancel()
        self.publish_gp_origin()

    def publish_gp_origin(self):
        if self.verified:
            return
        self.retry_count += 1
        if self.retry_count > self.max_retries:
            self.get_logger().error("gp_origin setup failed after max retries")
            self.schedule_shutdown(1.0)
            return

        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.position.latitude = self.lat
        msg.position.longitude = self.lon
        msg.position.altitude = self.alt
        self.pub.publish(msg)
        self.get_logger().info(
            f"Published set_gp_origin [{self.lat:.8f}, {self.lon:.8f}, {self.alt:.3f}] (attempt {self.retry_count}/{self.max_retries})"
        )

        # Start verify timeout timer
        if self.verify_timer is not None:
            self.verify_timer.cancel()
        self.verify_timer = self.create_timer(self.verify_timeout_s, self.on_verify_timeout)

    def on_verify_timeout(self):
        if self.verified:
            return
        self.verify_timer.cancel()
        self.get_logger().warn(f"Verify timeout on attempt {self.retry_count}, retrying...")
        self.publish_gp_origin()

    def on_gp_origin(self, msg: GeoPointStamped):
        if self.verified:
            return
        lat = msg.position.latitude
        lon = msg.position.longitude
        alt = msg.position.altitude
        
        if math.isnan(lat) or math.isnan(lon) or math.isnan(alt):
            return
        if (
            abs(lat - self.lat) < self.tolerance_deg
            and abs(lon - self.lon) < self.tolerance_deg
            and abs(alt - self.alt) < 1.0
        ):
            self.verified = True
            if self.verify_timer is not None:
                self.verify_timer.cancel()
            self.get_logger().info(
                f"gp_origin verified successfully! "
                f"Target: [{self.lat:.8f}, {self.lon:.8f}, {self.alt:.3f}] | "
                f"Received: [{lat:.8f}, {lon:.8f}, {alt:.3f}]"
            )
            self.schedule_shutdown(1.0)
        else:
            self.get_logger().warn(
                f"gp_origin mismatch! "
                f"Target: [{self.lat:.8f}, {self.lon:.8f}, {self.alt:.3f}] | "
                f"Received: [{lat:.8f}, {lon:.8f}, {alt:.3f}]"
            )

    def schedule_shutdown(self, delay_s: float):
        if self.shutdown_timer is not None:
            return
        self.shutdown_timer = self.create_timer(delay_s, self.do_shutdown)

    def do_shutdown(self):
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = AutoSetGpOrigin()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
