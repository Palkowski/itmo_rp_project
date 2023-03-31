import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


# import tf2_ros

class BotController:
    wheel_d = 0.066 + 0.014  # 66 mm wheel diameter
    enc_ppr = 600  # pulses per revolution of virtual encoder
    wheel_cf = np.pi * wheel_d  # wheel circumference
    mpp = wheel_cf / enc_ppr  # meters per pulse
    skip = 0  # skip points between estimations
    tb_rad = 0.080  # 80 mm distance from the center of turtlebot to the wheel

    def __init__(self):
        self.ts = []  # time values
        self.pos_rw = []  # right wheel angles
        self.pos_lw = []  # left wheel angles

        self.pos_rw_est = [0]
        self.pos_lw_est = [0]

        self.vel_rw_int = [0]
        self.vel_lw_int = [0]

        self.vel_rw_est = [0]
        self.vel_lw_est = [0]

        self.null_pos_rw = 0
        self.null_pos_lw = 0

        # self.lw_vel_estimated = []
        self.odom_ts = []  # odometry time values
        self.x_lin_vels = []  # odometry x linear velocities
        self.z_ang_vels = []  # odometry z axis angular velocities
        self.i = 0
        self.ki = 20
        self.kp = 10

        rospy.init_node('bot_controller', anonymous=True)
        self.rate = rospy.Rate(30)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.jst_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.on_shutdown(self.shutdown)

        self.start_time = time.time()

    def jst_callback(self, msg):
        self.ts.append(time.time())

        if self.i == 0:
            self.null_pos_rw = msg.position[0]
            self.null_pos_lw = msg.position[1]

        # recalculate angles to encoder pulses (x_real)
        rw = self.mpp * int(
            (msg.position[0] - self.null_pos_rw) / (2 * np.pi) * BotController.enc_ppr)  # for right wheel
        self.pos_rw.append(rw)

        lw = self.mpp * int(
            (msg.position[1] - self.null_pos_lw) / (2 * np.pi) * BotController.enc_ppr)  # for left wheel
        self.pos_lw.append(lw)

        if self.i > 0:
            # calculate delta_t
            delta_t = (self.ts[-1] - self.ts[-2])

            # estimate position with respect to estimated velocity (x_estimated)
            self.pos_rw_est.append(self.pos_rw_est[-1] + self.vel_rw_est[-1] * delta_t)
            self.pos_lw_est.append(self.pos_lw_est[-1] + self.vel_lw_est[-1] * delta_t)

            # estimate error (e = x_real - x_estimated)
            pos_rw_error = self.pos_rw[-1] - self.pos_rw_est[-1]
            pos_lw_error = self.pos_lw[-1] - self.pos_lw_est[-1]

            # velocity integrator (v_int = v_int + ki * x_error * delta_t)
            self.vel_rw_int.append(self.vel_rw_int[-1] + self.ki * pos_rw_error * delta_t)
            self.vel_lw_int.append(self.vel_lw_int[-1] + self.ki * pos_lw_error * delta_t)

            # velocity estimator (v_estimated = kp * x_error + v_int)
            self.vel_rw_est.append(self.kp * pos_rw_error + self.vel_rw_int[-1])
            self.vel_lw_est.append(self.kp * pos_lw_error + self.vel_lw_int[-1])

        self.i += 1
        print('v_lw:', self.vel_lw_int[-1], 'v_rw:', self.vel_rw_int[-1])

    def odom_callback(self, msg):
        self.odom_ts.append(time.time())
        twist = msg.twist.twist
        self.x_lin_vels.append(twist.linear.x)
        self.z_ang_vels.append(twist.angular.z)

    def go_x(self, seconds, speed=0.1, direction=1):
        start_time = time.time()
        end_time = start_time + seconds

        while time.time() < end_time:
            twist = Twist()
            twist.linear.x = speed * direction
            self.cmd_vel_pub.publish(twist)

        self.cmd_vel_pub.publish(Twist())

    def turn(self, seconds, speed=0.1, direction=-1):  # 1 turn left, -1 turn right
        start_time = time.time()
        end_time = start_time + seconds

        while time.time() < end_time:
            twist = Twist()
            twist.angular.z = speed * direction
            self.cmd_vel_pub.publish(twist)

        self.cmd_vel_pub.publish(Twist())

    def plot(self):
        plt.subplot(2, 1, 1)
        ts = np.array(self.ts) - self.start_time
        pos_rw = np.array(self.pos_rw)
        pos_lw = np.array(self.pos_lw)
        x_vel_est = (np.array(self.vel_rw_int) + np.array(self.vel_lw_int)) / 2
        plt.plot(ts, pos_rw, label='right wheel')
        plt.plot(ts, pos_lw, label='left wheel')
        # plt.xlabel('time, [sec]')
        plt.ylabel('position, [m]')
        plt.grid()
        plt.legend()

        plt.subplot(2, 1, 2)
        odom_ts = np.array(self.odom_ts) - self.start_time
        x_lin_vels = np.array(self.x_lin_vels)  # - np.array(self.z_ang_vels) * BotController.tb_rad
        plt.plot(odom_ts, x_lin_vels, label='odom x velocity')
        plt.plot(ts, x_vel_est, label='estimated x velocity')
        plt.xlabel('time, [sec]')
        plt.ylabel('speed, [m/s]')
        plt.grid()
        plt.legend()

        plt.show()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    bot_controller = BotController()  # init robot controller
    bot_controller.go_x(5, 0.2)  # move forward 5 seconds with speed 0.2
    bot_controller.turn(2, 1, -1)  # turn right 2 seconds with angular speed 1
    bot_controller.go_x(5, 0.2)  # move forward 5 seconds with speed 0.2
    bot_controller.plot()  # show the results
