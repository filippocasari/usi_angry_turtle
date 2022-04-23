import turtle
from xxlimited import new
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Empty
import sys
from math import pow, atan2, sqrt

from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import numpy as np
from turtlesim.msg import Pose
from turtlesim import srv as srv


class Move2GoalNode(Node):
    def __init__(self, letters, tolerance):

        # Creates a node with name 'move2goal'
        super().__init__('move2goal')

        '''parameters for writing USI'''
        self.goal_poses = letters[0]
        self.letter_count = 0
        self.letters = letters
        self.i = 0
        self.goal_pose = self.goal_poses[self.i]
        self.tolerance = tolerance
        '''###################################################'''

        '''Publishers and Subscribers'''
        self.vel_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        '''Clients and Servers'''
        self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        self.cli_clean = self.create_client(Empty, '/clear')

        '''requests'''
        self.req_pen = SetPen.Request()
        
        self.req_clean = Empty.Request()
        '''poses'''
        
        self.current_pose = Pose()
        
        self.set_width(width=5)
        # cleaning if it is restarted
        self.clean()

    def clean(self):
        self.cli_clean.call_async(self.req_clean)

    def start_moving(self):

        self.timer = self.create_timer(0.15, self.move_callback)

    def set_off_pen(self):
        self.req_pen.off = True
        self.cli.call_async(self.req_pen)

    def set_width(self, width):
        self.req_pen.width = width
        self.cli.call_async(self.req_pen)

    def set_color(self):
        color = list(np.random.choice(range(256), size=3))
        #print("color: ", color)
        self.req_pen.r = int(color[0])
        self.req_pen.g = int(color[1])
        self.req_pen.b = int(color[2])
        self.cli.call_async(self.req_pen)

    def set_on_pen(self):
        self.req_pen.off = False
        self.cli.call_async(self.req_pen)

    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)

    def move_callback(self):
        """Callback called periodically by the timer to publish a new command."""

        if self.current_pose is None:
            # Wait until we receive the current pose of the turtle for the first time
            return

        if self.euclidean_distance(self.goal_pose, self.current_pose) >= self.tolerance:
            # We still haven't reached the goal pose. Use a proportional controller to compute velocities
            # that will move the turtle towards the goal (https://en.wikipedia.org/wiki/Proportional_control)

            # Twist represents 3D linear and angular velocities, in turtlesim we only care about 2 dimensions:
            # linear velocity along the x-axis (forward) and angular velocity along the z-axis (yaw angle)\
            cmd_vel = Twist()

            cmd_vel.linear.x = self.linear_vel(
                self.goal_pose, self.current_pose)

            cmd_vel.angular.z = self.angular_vel(
                self.goal_pose, self.current_pose)

            # Publish the command
            self.vel_publisher.publish(cmd_vel)
            if(self.i == 0):
                self.set_off_pen()
                self.set_color()
            else:
                self.set_on_pen()

        else:
            self.get_logger().info("Goal reached, shutting down...")

            # Stop the turtle
            #cmd_vel = Twist()
            #cmd_vel.linear.x = 0.0
            #cmd_vel.angular.z = 0.0
            # self.vel_publisher.publish(cmd_vel)

            # Mark the future as completed, which will shutdown the node
            # self.done_future.set_result(True)
            self.i += 1
            print(f"point[{self.i}]")
            if(self.i > len(self.goal_poses)-1):
                self.i = 0
                self.letter_count += 1

                self.goal_poses = self.letters[self.letter_count % len(
                    self.letters)]

            self.goal_pose = self.goal_poses[self.i]

    def euclidean_distance(self, goal_pose, current_pose):

        return sqrt(pow((goal_pose.x - current_pose.x), 2) +
                    pow((goal_pose.y - current_pose.y), 2))

    def linear_vel(self, goal_pose, current_pose, constant=2.):

        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):

        return np.arctan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)

    def angular_vel(self, goal_pose, current_pose, constant=8.):

        return constant * (self.steering_angle(goal_pose, current_pose) - current_pose.theta)


def main():
    # Get the input from the user.
    import numpy as np
    #goal_pose = Pose()
    #goal_pose.x = float(input("Set your x goal position: "))
    #goal_pose.y = float(input("Set your y goal position: "))
    I = []
    U = []
    S = []
    # float(input("Set distance tolerance from goal (e.g. 0.01): "))
    tolerance = 0.01
    array_y_i = np.linspace(10., 2., 2)
    array_x_i = np.linspace(9., 9., 2)

    array_y_u = np.concatenate(
        (np.linspace(10., 2., 2), np.linspace(2., 10., 2)))
    array_x_u = np.concatenate(
        (np.linspace(1., 1., 2), np.linspace(3.5, 3.5, 2)))

    array_x_s = np.concatenate(
        (np.linspace(8., 4.5, 2), np.linspace(4.5, 4.5, 2), np.linspace(4.5, 8., 2), np.linspace(8., 8., 2), np.linspace(8., 4.5, 2)))
    array_y_s = np.concatenate(
        (np.linspace(10., 10., 2), np.linspace(10., 6., 2), np.linspace(6., 6., 2), np.linspace(6., 2., 2), np.linspace(2., 2., 2)))
    for i in range(len(array_y_s)):
        new_point = Pose()
        new_point.x = array_x_s[i]
        new_point.y = array_y_s[i]
        S.append(new_point)
    for i in range(len(array_x_u)):
        new_point = Pose()
        new_point.x = array_x_u[i]
        new_point.y = array_y_u[i]
        U.append(new_point)

    rclpy.init(args=sys.argv)

    for i in range(len(array_y_i)):
        new_point = Pose()
        new_point.x = array_x_i[i]
        new_point.y = array_y_i[i]

        I.append(new_point)

    array_final = [U, S, I]

    node = Move2GoalNode(array_final, tolerance)

    done = node.start_moving()
    # Keep processings events until the turtle has reached the goal
    #rclpy.spin_until_future_complete(node, done)
    rclpy.spin(node)

    # Initialize the ROS client library

    # Create an instance of your node class

    # Alternatively, if you don't want to exit unless someone manually shuts down the node


if __name__ == '__main__':
    main()
