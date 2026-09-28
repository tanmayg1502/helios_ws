"""ROS 2 adapter: subscribers only, no command publisher or service client."""
import threading

from .server import TelemetryServer, arguments, token_from_environment
from .state import TelemetryState


def main():
    # Keep ROS imports out of state/server so the fixture and tests run anywhere.
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    args, ros_args = arguments("Helios read-only ROS telemetry gateway")
    token = token_from_environment()
    state = TelemetryState()
    rclpy.init(args=ros_args)

    class GatewayNode(Node):
        def __init__(self):
            super().__init__("mobile_gateway")
            self.declare_parameter("odometry_topic", "/odometry/filtered")
            self.declare_parameter("scan_topic", "/scan")
            self.create_subscription(Odometry, self.get_parameter("odometry_topic").value,
                                     self.on_odometry, qos_profile_sensor_data)
            self.create_subscription(LaserScan, self.get_parameter("scan_topic").value,
                                     self.on_scan, qos_profile_sensor_data)

        def on_odometry(self, message):
            pose, twist = message.pose.pose, message.twist.twist
            q = pose.orientation
            state.odometry(frame_id=message.header.frame_id,
                           child_frame_id=message.child_frame_id,
                           x=pose.position.x, y=pose.position.y,
                           qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                           linear_x=twist.linear.x, linear_y=twist.linear.y,
                           angular_z=twist.angular.z)

        def on_scan(self, message):
            state.scan(frame_id=message.header.frame_id, ranges=message.ranges,
                       range_min=message.range_min, range_max=message.range_max)

    node = None
    server = None
    worker = None
    try:
        node = GatewayNode()
        server = TelemetryServer((args.host, args.port), state, token)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        node.get_logger().info("Read-only telemetry gateway started; no motion interfaces exposed")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if worker is not None:
            worker.join(timeout=5)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
