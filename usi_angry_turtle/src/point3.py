#import turtle
from urllib.request import Request
from xxlimited import new
import rclpy
from rclpy.node import Node
#from rclpy.task import Future
from std_srvs.srv import Empty
import sys
from math import pow, atan2, sqrt

from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import numpy as np
from turtlesim.msg import Pose
from turtlesim import srv as srv
from turtlesim.srv import Spawn



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
        '''STATES'''
        self.ANGRY = False
        self.RETURNING = False
        self.WRITING = True

        self.current_pose = None
        self.second_turtle_killed = False

        '''Publishers and Subscribers'''
        self.vel_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose_subscriber_target = self.create_subscription(
            Pose, '/turtle2/pose', self.pose_callback_target, 10)

        '''Clients and Servers'''
        self.cli = self.create_client(SetPen, '/turtle1/set_pen')
        self.cli_target_pen = self.create_client(SetPen, '/turtle2/set_pen')
        self.cli_spawn = self.create_client(Spawn, '/spawn')
        self.cli_clean = self.create_client(Empty, '/clear')

        '''requests'''
        self.req_pen = SetPen.Request()
        self.req_pen_2ndturtle = SetPen.Request()
        self.req_spawn = Spawn.Request()
        self.req_clean = Empty.Request()
        self.req_spawn.name = 'turtle2'
        self.req_spawn.x = np.random.uniform(1.0, 10.0)
        self.req_spawn.y = np.random.uniform(1.0, 10.0)
        '''poses'''
        self.pose_target_turtle = Pose()
        self.current_pose = Pose()

        self.set_width(width=5)
        self.k2 = 3.
        self.k1 = 0.5
        self.m = 3.0
        self.tmp = True
        self.clean()
        

    def clean(self):
        self.cli_clean.call_async(self.req_clean)

    def spawn(self):

        self.cli_spawn.call_async(self.req_spawn)
        self.set_off_pen_target_turtle()

    def start_moving(self):

        self.timer = self.create_timer(.1, self.move_callback)

        # return self.done_future

    def set_off_pen(self):
        self.req_pen.off = True
        self.cli.call_async(self.req_pen)

    def set_off_pen_target_turtle(self):

        self.req_pen_2ndturtle.off = True
        self.cli_target_pen.call_async(self.req_pen_2ndturtle)

    def set_width(self, width):
        self.req_pen.width = width
        self.cli.call_async(self.req_pen)

    def set_color(self):
        color = list(np.random.choice(range(256), size=3))
        print("color: ", color)
        self.req_pen.r = int(color[0])
        self.req_pen.g = int(color[1])
        self.req_pen.b = int(color[2])
        self.cli.call_async(self.req_pen)

    def set_on_pen(self):
        self.req_pen.off = False
        self.cli.call_async(self.req_pen)

    def pose_callback(self, msg):

        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)

    def pose_callback_target(self, msg):

        self.pose_target_turtle = msg
        self.pose_target_turtle.x = round(self.pose_target_turtle.x, 4)
        self.pose_target_turtle.y = round(self.pose_target_turtle.y, 4)

    def move_callback(self):
        """Callback called periodically by the timer to publish a new command."""

        if self.current_pose is None:
            # Wait until we receive the current pose of the turtle for the first time
            return

        elif(self.ANGRY == False and self.WRITING == True):

            if self.euclidean_distance(self.pose_target_turtle, self.current_pose) <= self.k2 \
                    and self.pose_target_turtle != None and self.RETURNING == False:

                self.ANGRY = True
                self.WRITING = False
                self.RETURNING = False
                print("ANGRY? YES")
                return

            elif self.euclidean_distance(self.goal_pose, self.current_pose) >= self.tolerance:

                if(self.RETURNING and self.tmp):
                    self.i -= 1 % len(self.letters)
                    #print('counter : ', str(self.i))
                    self.tmp = False
                if(self.i == 0 ):
                    self.set_off_pen()
                    #self.get_logger().info("chosen point= "+str(self.goal_pose))

                elif self.i ==0 or self.tmp == True:
                    self.set_on_pen()
                cmd_vel = Twist()
                cmd_vel.linear.x = self.linear_vel(
                    self.goal_pose, self.current_pose)
                cmd_vel.angular.z = self.angular_vel(
                    self.goal_pose, self.current_pose)

                self.vel_publisher.publish(cmd_vel)
            else:
                self.get_logger().info("CHANGING LETTER, KEEPING WRITING...")

                self.i += 1

                if(self.i > len(self.goal_poses)-1):
                    self.i = 0
                    self.letter_count += 1
                    self.set_color()
                    self.goal_poses = self.letters[self.letter_count % len(
                        self.letters)]

                self.goal_pose = self.goal_poses[self.i]

        elif(self.ANGRY == True and self.WRITING == False and self.RETURNING == False):

            if(self.euclidean_distance(self.pose_target_turtle, self.current_pose) <= self.k1):
                self.ANGRY = False
                self.WRITING = True
                self.RETURNING = True
                print("NOT ANGRY ANYMORE")
                self.req_pen.off=True
                self.set_off_pen()
                return

            else:
                self.req_pen.off=True
                self.set_off_pen()
                cmd_vel = Twist()
                cmd_vel.linear.x = self.linear_vel(
                    self.pose_target_turtle, self.current_pose)
                cmd_vel.angular.z = self.angular_vel(
                    self.pose_target_turtle, self.current_pose)

                self.vel_publisher.publish(cmd_vel)
                return

    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        # self.get_logger().info("EUCLIDEAN DISTANCE: ..."+str(sqrt(pow((goal_pose.x - current_pose.x), 2) +\
        #            pow((goal_pose.y - current_pose.y), 2))))
        return sqrt(pow((goal_pose.x - current_pose.x), 2) +
                    pow((goal_pose.y - current_pose.y), 2))

    def linear_vel(self, goal_pose, current_pose, constant=2.):

        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        '''self.get_logger().info("ANGLE: "+str(atan2(goal_pose.y -
                                                   current_pose.y, goal_pose.x - current_pose.x)))'''
        return atan2((goal_pose.y - current_pose.y), (goal_pose.x - current_pose.x))

    def angular_vel(self, goal_pose, current_pose, constant=8.):

        '''self.get_logger().info("ANGULAR DIFFERENCE: " +
            str((self.steering_angle(goal_pose, current_pose) - current_pose.theta)))'''
        return constant * np.arctan2(np.sin(self.steering_angle(goal_pose, current_pose) - current_pose.theta),
                                     np.cos(self.steering_angle(goal_pose, current_pose) - current_pose.theta))
        # CREDITS FOR BUG (ANGLE FIXING): https://github.com/thecountoftuscany/unicycle-navigation/blob/master/robot.py, go_to_goal function


def main():

    array_final = write_USI()
    tolerance = 0.01

    node = Move2GoalNode(array_final, tolerance)

    # node.start_moving_randomly()
    done = node.start_moving()
    node.spawn()
    rclpy.spin(node)


def write_USI():
    I = []
    U = []
    S = []

    array_y_i = np.linspace(9.5, 2., 2)
    array_x_i = np.linspace(9., 9., 2)

    array_y_u = np.concatenate(
        (np.linspace(10., 2., 2), np.linspace(2., 10., 2)))
    array_x_u = np.concatenate(
        (np.linspace(1.5, 1.5, 2), np.linspace(3., 3., 2)))

    array_x_s = np.concatenate(
        (np.array([8.0, 8.0]), np.linspace(5., 5., 2), np.linspace(5., 8., 2), np.linspace(8., 8., 2), np.linspace(8., 5., 2)))
    array_y_s = np.concatenate(
        (np.linspace(9.5, 9.5, 2), np.linspace(9.5, 6., 2), np.linspace(6., 6., 2), np.linspace(6., 2., 2), np.linspace(2., 2., 2)))
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
    return [U, S, I]


if __name__ == '__main__':
    main()
