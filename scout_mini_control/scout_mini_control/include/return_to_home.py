import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

from .robot_navigator import BasicNavigator,  NavigationResult

class ReturnToHome(Node):

    def __init__(self):
        super().__init__('return_to_home')

        # Declaring Node Parameters
        self.declare_parameter("home_point", [0.0,0.0,0.0,0.0,0.0,0.0,1.0])
        self.home_point = self.get_parameter('home_point').get_parameter_value().double_array_value

        # Declaring Variables
        self.goal_poses = []
        self.triggered = 0
        self.i = 0

        # Creating ROS Services
        self.subscription = self.create_subscription(
            Bool,
            'return_to_home',
            self.return_to_home_callback,
            10
        )

        self.timer_ = self.create_timer(
            0.1,
              self.timer_callback
        )

        # Creating Navigator object
        self.navigator = BasicNavigator()

    def return_to_home_callback(self, msg):
        if msg.data == True:
            self.get_logger().warn('Return to home triggered')

            self.navigator.cancelNav()
            self.get_logger().warn('Navigation goal cancelled...')

            # Populate the home point array
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

            self.navigator.followWaypoints(self.goal_poses)
            self.triggered = 1
            self.get_logger().warn('Returning to home...')

    def timer_callback(self):
        if self.triggered == 1:
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
                self.get_logger().info('Reached home point.')
                if result == NavigationResult.SUCCEEDED:
                    self.get_logger().info('Return to home succeeded!')
                    self.i = 0
                    self.triggered = 0
                elif result == NavigationResult.CANCELED:
                    self.get_logger().warn('Return to home was cancelled.')
                    self.i = 0
                    self.triggered = 0
                elif result == NavigationResult.FAILED:
                    self.get_logger().error('Return to home failed!')
                    self.i = 0
                    self.triggered = 0
                else:
                    self.get_logger().warn('Goal has an invalid return status.')
                    self.i = 0
                    self.triggered = 0
                exit(0)

def main(args=None):
    rclpy.init(args=args)

    return_to_home = ReturnToHome()

    rclpy.spin(return_to_home)
    return_to_home.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()