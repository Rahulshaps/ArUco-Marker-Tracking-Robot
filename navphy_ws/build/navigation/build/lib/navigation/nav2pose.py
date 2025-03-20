#!/usr/bin/python3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Int32, String
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import math
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion
import time

# This script takes in the distance to the target (x, y, d) and the current robot pose and publishes the goal pose and goal distance.
# Terminology: target = ArUco marker position; goal = desired robot position; 
class Nav2Pose(Node):   # Node is the Base class (Parent) for all ROS2 Nodes. Nav2Pose should inherit its properties, so therefore we call it
    def __init__(self):
        super().__init__('nav2pose')    # Equivalent to Node.__init__(). Calls constructor for Node class

        # PoseStamped(): Header + Pose (position [x,y,z] + orientation [x,y,z,w])
        self.current_pose = PoseStamped()  
        self.goal = PoseStamped()           
        self.prev_goal = PoseStamped()     
        
        # Initialize parameters
        self.truncate_dist = 0.85                   # "Filler" distance between robot and goal
        self.gpose_orient = 0                       # Orientation of goal pose
        self.distance = [0.0,0.0,0.0]
        self.prev_time_published = time.time()
        self.target_last_time = time.time()
        self.target_vel = 0 
        self.target_spotted = 0 # False

        # Subscriptions (target distance + robot pose). 
        # update_distance() and set_current_pose() get called when a message is received from their respective topics
        self.create_subscription(Float32MultiArray, '/aruco_pose', self.update_distance, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.update_current_pose, 10)
        self.create_subscription(Int32, '/target_spotted', self.set_goal, 10)

        # Publishers (goal pose + goal distance)
        self.goalposepub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.goaldistpub = self.create_publisher(Float32MultiArray, "/dist_to_goal", 10)

        # Every 0.1 / 0.5 seconds, recalculate goal position & velocity respectively
        self.create_timer(0.1, self.nav2pose_callback)
        self.create_timer(0.5, self.calculate_target_velocity)
        self.prev_goal_time = time.time()                                             

        self.get_logger().info('Nav2Pose Node Ready!')

    # Update distance to target
    def update_distance(self, msg : Float32MultiArray):
        self.distance = msg.data
        self.target_spotted = 1
        #self.get_logger().info('Current target distance: "%s"' % self.distance)

    # Set current robot pose with timestamp
    def update_current_pose(self, odommsg : Odometry):
        self.current_pose.header.frame_id = 'odom'
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.pose.position = odommsg.pose.pose.position
        self.current_pose.pose.orientation = odommsg.pose.pose.orientation
    
    # Check if goal needs to be updated (update every second for now)
    def nav2pose_callback(self):
        if (time.time() - self.prev_goal_time > 1.0):
            self.goalposepub.publish(self.goal)
            self.goaldistpub.publish(Float32MultiArray(data=self.distance))
            self.prev_goal_time = time.time()
            self.get_logger().info('Goal position: [%f,%f,%f]' % (self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z))
            self.get_logger().info('Goal orientation: [%f,%f,%f,%f]' % (self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z, self.goal.pose.orientation.w))
            self.get_logger().info('Distance to goal: [%f,%f,%f]' % (self.distance[0], self.distance[1], self.distance[2]))

    # Calculate target velocity to track it smoothly
    def calculate_target_velocity(self):                                    
        dt = time.time() - self.target_last_time
        self.target_last_time = time.time()
        
        dx = self.goal.pose.position.x - self.prev_goal.pose.position.x     # Change in distance of goal position
        x_vel = dx/dt

        self.target_vel = -1.5*(x_vel**2) if dx < 0 else 2.5*x_vel**2       # Arbitrary values to command robot's velocity. May need to adjust for Turtlebot
        self.target_vel = min(4,max(-4,self.target_vel))
        #self.get_logger().info(str(self.target_vel))                        # Check target velocity value
    
    # Set goal position
    def set_goal(self, check : Int32):
        if check.data:
            self.goal.header.frame_id = 'odom'
            self.goal.header.stamp = self.get_clock().now().to_msg()

            # Takes only the last value from the Euler angle conversion (aka rotation about z-axis, since robot cannot rotate about x or y)
            robot_orient = Rotation.from_quat([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,
                                            self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w]).as_euler("xyz",degrees=False)[2]

            self.goal.pose.position.x = self.distance[0] # xf = xi + dsin(phi)
            self.goal.pose.position.y = self.distance[1] # yf = yi + dcos(phi)
            self.goal.pose.position.z = self.distance[2]

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

def main(args=None):
    rclpy.init(args=args)
    nav2pose = Nav2Pose()
    rclpy.spin(nav2pose)
    nav2pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()