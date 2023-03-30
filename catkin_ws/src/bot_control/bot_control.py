import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


# import tf2_ros

class BotController:
    wheel_d = 0.066  # 66 mm wheel diameter
    enc_ppr = 600  # pulses per revolution of virtual encoder
    wheel_cf = np.pi * wheel_d  # wheel circumference
    mpp = wheel_cf / enc_ppr  # meters per pulse
    skip = 10  # skip points between estimations

    def __init__(self):
        self.ts = []  # time values
        self.pos_rw = []  # right wheel angles
        self.pos_lw = []  # left wheel angles
        self.rw_vel_estimated = []
        self.lw_vel_estimated = []
        self.odom_ts = []  # odometry time values
        self.x_lin_vels = []  # odometry x linear velocities
        self.z_ang_vels = []  # odometry z axis angular velocities
        self.i = 0

        rospy.init_node('bot_controller', anonymous=True)
        self.rate = rospy.Rate(30)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.jst_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.on_shutdown(self.shutdown)

        self.start_time = time.time()

    def jst_callback(self, msg):
        self.ts.append(time.time())
        # recalculate angles to encoder pulses
        rw = int(msg.position[0] / (2 * np.pi) * BotController.enc_ppr)  # for right wheel
        lw = int(msg.position[1] / (2 * np.pi) * BotController.enc_ppr)  # for left wheel
        self.pos_rw.append(rw)
        self.pos_lw.append(lw)
        if self.i > (0 + BotController.skip):
            self.rw_vel_estimated.append(
                BotController.mpp * (self.pos_rw[self.i] - self.pos_rw[self.i - 1 - BotController.skip]) / (
                        self.ts[self.i] - self.ts[self.i - 1 - BotController.skip]))
            self.lw_vel_estimated.append(
                BotController.mpp * (self.pos_lw[self.i] - self.pos_lw[self.i - 1 - BotController.skip]) / (
                        self.ts[self.i] - self.ts[self.i - 1 - BotController.skip]))
        self.i += 1

    def odom_callback(self, msg):
        self.odom_ts.append(time.time())
        twist = msg.twist.twist
        self.x_lin_vels.append(twist.linear.x)
        self.z_ang_vels.append(twist.angular.z)

    def go_x(self, seconds, direction=1):
        start_time = time.time()
        end_time = start_time + seconds

        while time.time() < end_time:
            twist = Twist()
            twist.linear.x = 0.1 * direction
            self.cmd_vel_pub.publish(twist)

        self.cmd_vel_pub.publish(Twist())

    # def turn(self, seconds, direction=-1):  # 1 turn left, -1 turn right
    #     start_time = time.time()
    #     end_time = start_time + seconds
    #
    #     while time.time() < end_time:
    #         twist = Twist()
    #         twist.angular.z = 1.0 * direction
    #         self.cmd_vel_pub.publish(twist)
    #
    #     self.cmd_vel_pub.publish(Twist())
    #     ts = np.array(self.ts) - start_time
    #     pos_rw = np.array(self.pos_rw)
    #     pos_lw = np.array(self.pos_lw)
    #     plt.plot(ts, pos_rw)
    #     plt.plot(ts, pos_lw)
    #     plt.grid()
    #     plt.show()

    def plot(self):
        plt.subplot(2, 1, 1)
        ts = np.array(self.ts) - self.start_time
        pos_rw = np.array(self.pos_rw)
        pos_lw = np.array(self.pos_lw)
        rw_vel_estimated = np.array(self.rw_vel_estimated)
        lw_vel_estimated = np.array(self.lw_vel_estimated)
        plt.plot(ts, pos_rw, label='right wheel')
        plt.plot(ts, pos_lw, label='left wheel')
        plt.xlabel('time, [sec]')
        plt.ylabel('encoder pulse')
        # plt.title('position')
        plt.grid()
        plt.legend()

        plt.subplot(2, 1, 2)
        odom_ts = np.array(self.odom_ts) - self.start_time
        x_lin_vels = np.array(self.x_lin_vels)
        plt.plot(odom_ts, x_lin_vels, label='real x velocity')
        plt.plot(ts[:-(1 + BotController.skip)], rw_vel_estimated, label='estimated rw velocity')
        plt.plot(ts[:-(1 + BotController.skip)], lw_vel_estimated, label='estimated lw velocity')
        plt.xlabel('time, [sec]')
        plt.ylabel('speed, [m/s]')
        # plt.title('velocity')
        plt.grid()
        plt.legend()

        # plt.savefig('plots.png')
        plt.show()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    bot_controller = BotController()
    bot_controller.go_x(10)
    bot_controller.plot()
