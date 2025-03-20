#!/usr/bin/python3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import math
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion
import time

# This function provides placeholder inputs to nav2pose.py, which consist of the robot pose and target distance. Using this information, nav2pose can calculate target pose,
# and as a result, the appropriate goal position and robot velocity.
class PoseDistPublisher(Node):
    def __init__(self):
        super().__init__('pose_dist_publisher')
        self.target_dist_pub = self.create_publisher(Float32MultiArray, '/target_distance', 10)
        self.robot_pose_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.target_dist = [3.0,4.0,5.0]

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.ang_z = 0.5
        self.rot = Rotation.from_euler('xyz', [0, 0, self.ang_z], degrees=False)
        self.rot_quat = self.rot.as_quat() # convert angle to quaternion format
        self.rot_quat = Quaternion(x=self.rot_quat[0], y=self.rot_quat[1], z=self.rot_quat[2], w=self.rot_quat[3])
        self.vel_x = 0.2
        self.ang_vel_z = 0.5

    def timer_callback(self):
        target_dist_msg = Float32MultiArray()
        target_dist_msg.data = self.target_dist

        robot_pose_msg = Odometry()
        robot_pose_msg.header.frame_id = "odom"
        robot_pose_msg.pose.pose.position.x = self.pos_x
        robot_pose_msg.pose.pose.position.y = self.pos_y
        robot_pose_msg.pose.pose.position.z = self.pos_z
        robot_pose_msg.pose.pose.orientation = self.rot_quat
        robot_pose_msg.twist.twist.linear.x = self.vel_x   # Linear velocity in x
        robot_pose_msg.twist.twist.angular.z = self.ang_vel_z  # Angular velocity about z

        self.target_dist_pub.publish(target_dist_msg)
        self.get_logger().info('Publishing (1): "%s"' % target_dist_msg.data[0])

        self.robot_pose_pub.publish(robot_pose_msg)
        self.get_logger().info('Publishing (2): "%s"' % robot_pose_msg)

        #self.target_dist_x += 0.1
        #self.pos_x += 0.1


def main(args=None):
    rclpy.init(args=args)

    pose_dist_publisher = PoseDistPublisher()
    rclpy.spin(pose_dist_publisher)

    pose_dist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()