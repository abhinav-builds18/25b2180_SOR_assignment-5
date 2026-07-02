import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


def quaternion_from_yaw(yaw: float):
    """Build a (x, y, z, w) quaternion for a pure yaw rotation about Z.
    Avoids depending on the external tf_transformations package."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WaypointPatrol(Node):

    
    WAYPOINTS = [
        (5.61, 1.5676, 0.7661),    # Waypoint 1
        (1.014, -6.315, -0.8830),  # Waypoint 2
        (-2.897, -5.0238, 3.1271),  # Waypoint 3
        (-0.8368, 0.2818, 0.2687),
    ]

    def __init__(self):
        super().__init__('waypoint_patrol_node')

        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._last_reported_index = -1
        self._num_waypoints = len(self.WAYPOINTS)

        self.get_logger().info('Waiting for the "follow_waypoints" action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available. Building goal...')

        self.send_patrol_goal()

    def build_pose(self, x: float, y: float, theta: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_yaw(theta)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_patrol_goal(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self.build_pose(x, y, theta) for (x, y, theta) in self.WAYPOINTS]

        self.get_logger().info(f'Dispatching {self._num_waypoints} waypoints to Nav2...')
        self.get_logger().info('Navigating to Waypoint 1...')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Patrol goal was rejected by the Nav2 action server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Patrol goal accepted. Robot is now en route.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # current_waypoint = index of the waypoint the robot is currently heading to
        current_waypoint = feedback_msg.feedback.current_waypoint

        if current_waypoint != self._last_reported_index:
            if self._last_reported_index >= 0:
                self.get_logger().info(f'Waypoint {self._last_reported_index + 1} Reached!')

            if current_waypoint < self._num_waypoints:
                self.get_logger().info(f'Navigating to Waypoint {current_waypoint + 1}...')

            self._last_reported_index = current_waypoint

    def get_result_callback(self, future):
        result = future.result().result
        missed = list(result.missed_waypoints)

        if self._last_reported_index >= 0:
            self.get_logger().info(f'Waypoint {self._last_reported_index + 1} Reached!')

        if not missed:
            self.get_logger().info('Patrol complete! All waypoints reached successfully.')
        else:
            self.get_logger().warn(f'Patrol finished, but missed waypoint indices: {missed}')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()


if __name__ == '__main__':
    main()