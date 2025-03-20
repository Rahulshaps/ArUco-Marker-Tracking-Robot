#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
import math
from scipy.spatial.transform import Rotation
import time
import random

# This node searches for targets by publishing random goals
# If target is spotted, set center of search area to robot position
# If target is lost, wait 10 seconds, then publish a goal
class TargetSearch(Node):
    def __init__(self):
        super().__init__('target_search')

        # Initialize variables
        self.center = PoseStamped()
        self.current_pose = PoseStamped()
        self.goal = PoseStamped()
        self.scanmsg = LaserScan()
        self.gpose_orient = 0                       # Orientation of goal pose


        time.now = time.time()
        self.wait_start_time = time.time()

        self.odomsub = self.create_subscription(Odometry, '/odometry/filtered', self.set_current_pose, 10)
        self.targetspottedsub = self.create_subscription(Int32, '/target_spotted', self.set_center, 10)
        self.goalsub = self.create_subscription(PoseStamped, "/nav2pose_goal", self.set_nav2pose_goal, 10)

        self.goalposepub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.goaldistpub = self.create_publisher(PoseStamped, "/dist_to_goal", 10)

    # Update current robot pose
    def set_current_pose(self, odommsg : Odometry):
        self.current_pose.header.frame_id = 'odom'
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.pose.position = odommsg.pose.pose.position
        self.current_pose.pose.orientation = odommsg.pose.pose.orientation

        #self.currentposepub.publish(self.current_pose)
    
    # Update goal set by nav2pose
    def set_nav2pose_goal(self, nav2posemsg : PoseStamped):
        self.gpose_orient = Rotation.from_quat([nav2posemsg.pose.orientation.x,nav2posemsg.pose.orientation.y,
                                                nav2posemsg.pose.orientation.z,nav2posemsg.pose.orientation.w]).as_euler("xyz", degrees=False)[2]
        self.goal = nav2posemsg.data

    # If target is spotted, then set the center search pose to the robot pose.
    # Else, start timeout timer
    def set_center(self, spotted : Int32):
        if spotted.data == 1:
            self.center = PoseStamped()
        else:
            self.wait_start_time = time.time()

    # Rewritten from nav_timed_out()
    # Checks if timeout has occurred so that a random goal can be set
    def wait_timeout(self):
        nav_time_now = time.time()
        nav_time = nav_time_now - self.wait_start_time

        if nav_time >= 10:
            return True
        else:
            return False
    
    # Set random goal if no target is spotted after 10 seconds.
    # TODO: Test/adjust random search behavior
    def set_random_goal(self):
        if self.wait_timeout():
            self.goal.header.frame_id = 'odom'
            self.goal.header.stamp = self.get_clock().now().to_msg()
        
        # Go to search center, which was set when target was last spotted
        if self.center != 0.0 and self.current_pose != self.center: 
            self.get_logger().info("Going to last known target position area")
            self.goal = self.center
            self.goalposepub.publish(self.goal)
            self.get_logger().info('Goal position: [%f,%f,%f]' % (self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z))
            self.get_logger().info('Goal orientation: [%f,%f,%f,%f]' % (self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z, self.goal.pose.orientation.w))

        # If at search center, publish a goal 5m ahead
        # TODO: Adjust/optimize random goal setting
        if self.current_pose == self.center:
            # Takes only the last value from the Euler angle conversion (aka rotation about z-axis, since robot cannot rotate about x or y)
            robot_orient = Rotation.from_quat([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,
                                            self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w]).as_euler("xyz",degrees=False)[2]

            self.goal.pose.position.x = self.current_pose.pose.position.x # xf = xi + dsin(phi)
            self.goal.pose.position.y = self.current_pose.pose.position.y + 5 # yf = yi + dcos(phi)
            self.goal.pose.position.z = self.current_pose.pose.position.z

            # Goal orientation = angle from robot's orientation to goal: use inverse tan of x goal and y goal
            self.gpose_orient = robot_orient + math.atan(self.goal.pose.position.y / self.goal.pose.position.x) #+math.pi))-math.pi
            if self.gpose_orient > math.pi: # sanity check to keep it within ROS bounds [-pi,pi]
                self.gpose_orient -= (2*math.pi)
            elif self.gpose_orient < -3.12: # min angle of LiDAR is not exactly -pi (-3.14159)
                self.gpose_orient += 6.26

            rot = Rotation.from_euler('xyz', [0, 0, self.gpose_orient], degrees=False)
            rot_quat = rot.as_quat() # convert angle to quaternion format

            self.goal.pose.orientation.x = rot_quat[0] # set the orientation to be looking at the marker at the end of navigation
            self.goal.pose.orientation.y = rot_quat[1]
            self.goal.pose.orientation.z = rot_quat[2]
            self.goal.pose.orientation.w = rot_quat[3]

            self.goalposepub.publish(self.goal)

def main(args=None):
    rclpy.init(args=args)

    target_search = TargetSearch()

    rclpy.spin(target_search)

    target_search.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()