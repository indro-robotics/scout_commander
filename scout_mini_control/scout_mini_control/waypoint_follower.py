

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import pandas as pd
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped

from .include.robot_navigator import BasicNavigator,  NavigationResult

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        scout_mini_control_dir = get_package_share_directory('scout_mini_control')

        self.declare_parameter("waypoints_file", '/home/ros2/humble_ws/waypoints.txt')
        self.declare_parameter("rth", True)
        self.declare_parameter("home_point", [0.0,0.0,0.0,0.0,0.0,0.0,1.0])

        timer_period = 0.1
        self.i = 0
        self.goal_poses = []
        self.timer_flag = 0
        self.triggered = 0

        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.navigator = BasicNavigator()

        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.rth = self.get_parameter('rth').get_parameter_value().bool_value
        self.home_point = self.get_parameter('home_point').get_parameter_value().double_array_value
        
        # Call the waypoint sender
        self.send_waypoints()


    def send_waypoints(self):
        
        self.navigator.waitUntilNav2Active()
        waypoints = pd.read_csv(self.waypoints_file).to_numpy()

        pos_x = waypoints[:,0]
        pos_y = waypoints[:,1]
        pos_z = waypoints[:,2]
        or_x = waypoints[:,3]
        or_y = waypoints[:,4]
        or_z = waypoints[:,5]
        or_w = waypoints[:,6]
        self.goal_poses = []

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
            self.goal_poses.append(goal)

        # Adding the home initial home point as the final point: 
        if self.rth == True:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = self.home_point[0]
            goal.pose.position.y = self.home_point[1]
            goal.pose.position.z = self.home_point[2]
            goal.pose.orientation.x = self.home_point[3]
            goal.pose.orientation.y = self.home_point[4]
            goal.pose.orientation.z = self.home_point[5]
            goal.pose.orientation.w = self.home_point[6]
            self.goal_poses.append(goal)

        self.nav_start = self.navigator.get_clock().now()
        self.navigator.followWaypoints(self.goal_poses)

        self.timer_flag = 1

    def timer_callback(self):
        if self.timer_flag == 1:
            if self.triggered == 1:
                self.get_logger().info('The service was triggered')
            if self.navigator.isNavComplete() == False:

                self.i = self.i + 1
                feedback = self.navigator.getFeedback()
                if feedback and self.i % 5 == 0:
                    self.get_logger().info('Executing current waypoint: ' +
                                        str(feedback.current_waypoint + 1) + '/' + str(len(self.goal_poses)))
                    now = self.navigator.get_clock().now()
                    if now - self.nav_start  > Duration(seconds=350.0):
                        self.navigator.cancelNav()
            result = self.navigator.getResult()

            if self.navigator.isNavComplete == True:
                self.get_logger().info('Got to this point')
                if result == NavigationResult.SUCCEEDED:
                    self.get_logger().info('Goal succeeded!')
                elif result == NavigationResult.CANCELED:
                    self.get_logger().warn('Goal was cancelled.')
                elif result == NavigationResult.FAILED:
                    self.get_logger().error('Goal failed!')
                else:
                    self.get_logger().warn('Goal has an invalid return status.')
                exit(0)

def main():
    rclpy.init()
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)

if __name__ == '__main__':
    main()