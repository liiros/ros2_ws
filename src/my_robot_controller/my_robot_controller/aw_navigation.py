#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose
import tf_transformations
import time
import math

from tier4_system_msg.srv import ChangeAutowareControl


class CarNavigationNode(Node):
    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("our navigation is started")
        self.goal_poses = [] # List to store goal poses
        self.current_goal_index = 0

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", 10)
        
        self.odom_listener = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10)
        
        ############# [Initial Location] ############
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = 3724.247802734375
        initial_pose.pose.pose.position.y = 73724.640625

        
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.24
        initial_pose.pose.pose.orientation.w = 0.97
        time.sleep(5)
        self.initial_pose_publisher.publish(initial_pose)
        #################################

       
      
      
       # Initialize goal poses as dictionaries {x, y, w}
        self.goal_poses.append({'x': 3787.62841796875, 'y': 73757.6328125, 'xx':  0.0, 'yy': 0.0, 'zz': 0.21, 'w': 0.97})
 
        time.sleep(5)
        self.publish_goal()

    def odom_callback(self, msg: Odometry):
        # Check if current goal pose is reached
        current_pose = msg.pose.pose
        goal_pose = self.goal_poses[self.current_goal_index]
        distance_to_goal = (((current_pose.position.x) - goal_pose['x']) ** 2 +
                            ((current_pose.position.y) - goal_pose['y']) ** 2) ** 0.5
        if distance_to_goal < 0.3: # You can adjust this threshold
            print(distance_to_goal)
            self.publish_next_goal()

    def publish_next_goal(self):
        # Check if there are more goals to explore
        if self.current_goal_index < len(self.goal_poses) - 1:
            self.current_goal_index += 1
            self.publish_goal()

        else:
            self.get_logger().info("All goals explored!")
            self.stop()

    def publish_goal(self):
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = self.goal_poses[self.current_goal_index]['x']
            pose_msg.pose.position.y = self.goal_poses[self.current_goal_index]['y']
            
            pose_msg.pose.orientation.x = self.goal_poses[self.current_goal_index]['xx']
            pose_msg.pose.orientation.y = self.goal_poses[self.current_goal_index]['yy']
            pose_msg.pose.orientation.z = self.goal_poses[self.current_goal_index]['zz']
            pose_msg.pose.orientation.w = self.goal_poses[self.current_goal_index]['w']

            pose_msg.header.frame_id = 'map'
            self.goal_pose_publisher.publish(pose_msg)
            self.get_logger().info("Published goal: {}".format(self.current_goal_index))
    def stop(self):
        self.get_logger().info("stopping the node")
        rclpy.shutdown()
        raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    node = CarNavigationNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt):
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()