import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
import math
import std_msgs.msg
from geometry_msgs.msg import Point32
import pigpio
from rospy.exceptions import ROSException
import random
import time
import matplotlib.pyplot as plt


class Node:

    '''
        This should be run on a Rasperry Pi/ similar device
    '''

    servo_max_pw = 2250.0
    servo_min_pw = 500.0
    min_sweep_angle = 80.0
    max_sweep_angle = 180.0

    def __init__(self, run_ros=True):

        # Servo Variables
        self.print_once = False
        self.printed = False
        self.disable_sweep = False
        self.servo_controller = pigpio.pi()
        self.sweep_min_angle = Node.min_sweep_angle
        self.sweep_max_angle = Node.max_sweep_angle
        self.current_angle = 145

        self.max_dc, self.min_dc = 2150.0, 1250.0
        self.min_angle, self.max_angle = 80.0, 180.0
        self.set_servo_angle(self.current_angle)
        dt = .00001
        self.sweep_time = 5.0
        self.sweep_start_t = -1

        # ROS Variables
        if run_ros:
            rospy.init_node('lidar_3d_data_node')
            rospy.on_shutdown(self.stop)
            rospy.Timer(rospy.Duration(dt), self.update)
            rospy.Subscriber('/scan', LaserScan, self.process_scan)
            self.publisher = rospy.Publisher('/pcloud', PointCloud, queue_size=1)
            rospy.spin()
        else:
            while 1:
                try:
                    self.update(-1)
                    time.sleep(dt)
                except KeyboardInterrupt:
                    break

    def update_servo(self):
        if not self.disable_sweep:
            if self.sweep_start_t < 0:
                self.sweep_start_t = time.time()
            t_elapsed = time.time() - self.sweep_start_t
            if t_elapsed > 2 * self.sweep_time:
                self.sweep_start_t = -1
                self.sweep_min_angle = Node.min_sweep_angle + random.uniform(-2.0, 2.0)
                self.sweep_max_angle = Node.max_sweep_angle + random.uniform(-2.0, 2.0)
                print("min sweep angle:", self.sweep_min_angle, ",max:", self.sweep_max_angle)

            a_f = self.sweep_max_angle
            a_0 = self.sweep_min_angle
            d = self.sweep_time

            if t_elapsed < d:
                self.current_angle = ((a_f - a_0) * t_elapsed) / (d) + a_0
            else:
                self.current_angle = (-1.0 * (a_f - a_0) * (t_elapsed - d)) / (d) + a_f
            self.set_servo_angle(self.current_angle)

    def set_servo_angle(self, angle):
        angle = float(angle)
        if angle < self.min_angle or angle > self.max_angle:
            print("invalid input angle:", angle)
            return
        pw = Node.translate(angle, self.min_angle, self.max_angle, self.min_dc, self.max_dc)
        if pw < Node.servo_min_pw or pw > Node.servo_max_pw:
            print("invalid dc:", pw)
            return
        self.servo_controller.set_servo_pulsewidth(18, pw)

    @staticmethod
    def pp_float(i):
        if int(i) == i:
            return str(i)
        return format(i, '2.2f')

    @staticmethod
    def pp_list(input_list_, round=None):
        ret = "[ "
        if input_list_ is None:
            return "[Empty List]"
        for i in input_list_:
            if type(i) == list:
                ret += Node.pp_list(i)  # recursion, lfg
            else:
                if isinstance(i, (int, float, complex)):
                    if i == int(i):
                        ret += str(i) + ", "
                    else:
                        if i == np.inf:
                            ret += "inf"
                        else:
                            ret += "%.3f" % i
                        ret += ", "
                else:
                    ret += str(i) + ", "
        ret = ret[:-2]
        ret += " ]"
        return ret

    @staticmethod
    def translate(value, leftMin, leftMax, rightMin, rightMax):
        valueScaled = float(value - leftMin) / float(leftMax - leftMin)
        return rightMin + (valueScaled * (rightMax - rightMin))

    @staticmethod
    def spherical_to_cartesian(r, theta, phi):
        x = r * np.cos(phi)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.sin(phi) * np.cos(theta)
        return [x, y, z]

    def process_scan(self, m):

        pcloud = PointCloud()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pcloud.header = header
        lidar_range_rad = np.deg2rad(270)

        step = 8
        theta = np.deg2rad(self.current_angle)
        for i in range(0, len(m.ranges), step):
            d = m.ranges[i]
            phi = np.deg2rad(Node.translate(i, 0.0, float(len(m.ranges)), 0.0, 270)) - .7843
            xyz = Node.spherical_to_cartesian(d, theta, phi)
            pcloud.points.append(Point32(xyz[0], xyz[1], xyz[2]))

        if self.print_once:
            if not self.printed:
                time.sleep(2)

                xyzs = []
                xs = []
                ys = []
                zs = []
                phis = []
                for i in range(len(m.ranges)):
                    phi = np.deg2rad(Node.translate(i, 0.0, float(len(m.ranges)), 0.0, 270)) - .7843
                    phis.append(np.rad2deg(phi))
                    d = m.ranges[i]
                    xyz = Node.spherical_to_cartesian(d, theta, phi)
                    xyzs.append(xyz)
                    xs.append(xyz[0])
                    ys.append(xyz[1])
                    zs.append(xyz[2])

                f, axarr = plt.subplots(2, 2)
                axarr[0, 0].plot(phis, m.ranges)
                axarr[0, 0].set_title("phi vs distance")
                axarr[0, 1].plot(phis, xs)
                axarr[0, 1].set_title("phi vs X values")
                # axarr[0,1].text(0,0,"x = r * np.sin(theta) * np.cos(phi)",fontsize=12)
                axarr[1, 0].plot(phis, zs)
                axarr[1, 0].set_title("phi vs Z values")
                plt.show()

                self.printed = True
                print("done plotting")
                # print Node.pp_list(m.ranges)

        try:
            self.publisher.publish(pcloud)
        except ROSException:
            pass

    def update(self, timer_event):
        self.update_servo()

    def stop(self):
        self.disable_sweep = True
        self.set_servo_angle(91)
        print("done")


if __name__ == '__main__':
    node = Node(run_ros=False)
