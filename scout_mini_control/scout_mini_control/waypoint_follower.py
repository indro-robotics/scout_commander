import time

from nav2_msgs.action import FollowWaypoints

import pandas as pd

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
import numpy as np

import os
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

from .include.robot_navigator import BasicNavigator,  NavigationResult

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        scout_mini_control_dir = get_package_share_directory('scout_mini_control')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoints_file', os.path.join(scout_mini_control_dir, 'maps', 'waypoints.txt')),
            ]
        )
        
        self.navigator = BasicNavigator()


        self._action_follow_waypoints_client = ActionClient(
            self, FollowWaypoints, '/follow_waypoints')

    def send_waypoints(self):

        follow_waypoints_goal = FollowWaypoints.Goal()
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        # self.goal_poses = []

        waypoints = pd.read_csv(waypoints_file).to_numpy()

        self.navigator.waitUntilNav2Active()
        #waypoints = waypoints.values
        pos_x = waypoints[:,0]
        pos_y = waypoints[:,1]
        pos_z = waypoints[:,2]
        or_x = waypoints[:,3]
        or_y = waypoints[:,4]
        or_z = waypoints[:,5]
        or_w = waypoints[:,6]

        goal_poses = []

        for i in range(waypoints.shape[0]):
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = pos_x[i]
            goal.pose.position.y = pos_y[i]
            goal.pose.position.z = pos_z[i]
            goal.pose.orientation.x = or_x[i]
            goal.pose.orientation.y = or_y[i]
            goal.pose.orientation.z = or_z[i]
            goal.pose.orientation.w = or_w[i]
            goal_poses.append(goal)

        nav_start = self.navigator.get_clock().now()
        self.navigator.followWaypoints(goal_poses)

        i = 0

        while not self.navigator.isNavComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = self.navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=250.0):
                    self.navigator.cancelNav()
        
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        
        exit(0)
            
def main():
    rclpy.init()
    waypoint_follower = WaypointFollower()
    waypoint_follower.send_waypoints()
    rclpy.spin(waypoint_follower)

if __name__ == '__main__':
    main()