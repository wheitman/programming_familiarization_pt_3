import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class TrajectoryServer(Node):

    def __init__(self):
        super().__init__("trajectory_server")

        self.declareParams()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trajectory_pub = self.create_publisher(Path, "trajectory", 10)
        self.trajectory = Path()

        update_period = 1 / self.get_parameter("trajectory_update_rate").value
        self.create_timer(update_period, self.updateTrajectory)

    def updateTrajectory(self) -> None:

        to_frame_rel = self.get_parameter("target_frame_name").value
        from_frame_rel = self.get_parameter("source_frame_name").value

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = to_frame_rel
            self.trajectory.poses.append(pose)

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )
            return

        self.trajectory.header.stamp = self.get_clock().now().to_msg()
        self.trajectory.header.frame_id = to_frame_rel
        self.trajectory_pub.publish(self.trajectory)

    def declareParams(self) -> None:
        descr = ParameterDescriptor()
        descr.description = "The name of the tf source frame"
        descr.type = ParameterType.PARAMETER_STRING
        self.declare_parameter("source_frame_name", value="base_link", descriptor=descr)

        descr = ParameterDescriptor()
        descr.description = "The name of the tf target frame"
        descr.type = ParameterType.PARAMETER_STRING
        self.declare_parameter("target_frame_name", value="map", descriptor=descr)

        descr = ParameterDescriptor()
        descr.description = "The update rate [hz] for the saved trajectory"
        descr.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("trajectory_update_rate", value=2.0, descriptor=descr)


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryServer()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
