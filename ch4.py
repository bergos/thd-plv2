import time
import math
import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from transforms3d.euler import quat2euler
from statistics import mean

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

STATE_SEARCH = 1
STATE_TO_WALL = 2
STATE_ALONG_WALL = 3

DISTANCE_SAFE = 0.4
DISTANCE_CLOSE = 0.3
SPEED = 50
WALL_MEASURE_ANGLE = 10

def find_center(intensities, low, high):
    found = -1

    for i in range(0, 359):
        if intensities[i] > low and intensities[i] <= high:
            found = i
            break

    if found == -1:
        return -1

    found += 360
    start = found
    end = found

    for i in range(0, -359, -1):
        intensity = intensities[(found + i) % 360]

        if intensity <= low or intensity > high:
            start = found + i + 1
            break

    for i in range(0, 359):
        intensity = intensities[(found + i) % 360]

        if intensity <= low or intensity > high:
            end = found + i - 1
            break

    center = (start + end) / 2
    size = end - start

    found = found % 360
    start = start % 360
    end = end % 360
    center = center % 360

    print('center')
    #print(found)
    #print(start)
    #print(end)
    print(center)
    print(size)

    return center

def next_to_wall(ranges, degree=45):
    distance = min(ranges[270-degree:270+degree-1])

    if distance < DISTANCE_SAFE:
        return True

    return False

def min_range_forward(ranges, degree=25):
    return min(
        min(ranges[360-degree:359]),
        min(ranges[0:degree-1])
    )

def min_range_backward(ranges, degree=25):
    return min(ranges[180-degree:179+degree])


class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.state = STATE_SEARCH
        self.targetAngle = -1
        self.orientation = -1
        self.angle_factor = 0

        self.cmd_vel_pub = self.create_publisher(
                Twist,      # message type
                'cmd_vel',  # topic name
                1)          # history depth

        self.scan_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,  # function to run upon message arrival
                qos_profile_sensor_data)  # allows packet loss

        self.odom_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,
                qos_profile_sensor_data)

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        o = msg.pose.pose.orientation
        orientation = quat2euler([o.w, o.x, o.y, o.z])
        self.orientation = (orientation[2] * 180 / math.pi + 360) % 360

    def scan_callback(self, msg):
        print('state: ', self.state)

        min_distance = min(msg.ranges)
        is_next_to_wall = next_to_wall(msg.ranges)

        print('next to wall: ', is_next_to_wall)

        if self.state == STATE_SEARCH:
            if is_next_to_wall:
                self.state = STATE_ALONG_WALL
            else:
                self.state = STATE_TO_WALL

        if self.state == STATE_TO_WALL:
            if min_distance < DISTANCE_SAFE:
                self.vel(0, 0)
                self.state = STATE_ALONG_WALL
            else:
                self.vel(SPEED, 0)

        if self.state == STATE_ALONG_WALL:
            center = find_center(msg.intensities, 1.5, 2.5)

            if center != -1 and abs(center - 270) < 10:
                self.vel(0, 0)
                exit()

            #front_distance = msg.ranges[270 + WALL_MEASURE_ANGLE]
            #back_distance = msg.ranges[270 - WALL_MEASURE_ANGLE]

            front_distance = mean(msg.ranges[270:270+WALL_MEASURE_ANGLE])
            back_distance = mean(msg.ranges[270 - WALL_MEASURE_ANGLE:269])

            if math.isinf(front_distance):
                front_distance = 5

            if math.isinf(back_distance):
                back_distance = 5

            print('front: ', front_distance)
            print('back: ', back_distance)

            angle_factor1 = back_distance - front_distance
            angle_factor2 = DISTANCE_SAFE - front_distance
            angle_factor = angle_factor1 + angle_factor2
            #angle_factor = angle_factor * angle_factor * angle_factor
            angle_factor = angle_factor * 2
            angle_factor = min(1, angle_factor)
            angle_factor = max(-1, angle_factor)

            self.angle_factor = angle_factor * 0.8 + self.angle_factor * 0.2

            print('angle factor: ', angle_factor)

            forward_distance = min_range_forward(msg.ranges,)
            forward_factor = (forward_distance - DISTANCE_CLOSE) / (DISTANCE_SAFE - DISTANCE_CLOSE)
            #forward_factor = forward_factor * 0.5
            forward_factor = max(0, forward_factor)
            forward_factor = min(0.75, forward_factor)

            print('forward factor: ', forward_factor)

            #forward_factor = min(forward_factor, correct_factor)

            if forward_distance < DISTANCE_CLOSE:
                self.vel(0, SPEED)
            else:
                self.vel(forward_factor * SPEED, self.angle_factor * SPEED)

        """ is run whenever a LaserScan msg is received
        """
        print()
        """print('Distances:')
        print('⬆️ :', msg.ranges[0])
        print('⬇️ :', msg.ranges[180])
        print('⬅️ :', msg.ranges[90])
        print('➡️ :', msg.ranges[-90])"""


def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
