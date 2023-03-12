import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
# from std_msgs.msg import String
import matplotlib.pyplot as plt

class BotController():

    def __init__(self):
        self.ts = []
        self.pos_rw = []
        self.pos_lw = []
        rospy.init_node('bot_controller', anonymous=True)
        self.rate = rospy.Rate(30)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.jst_callback)
        rospy.on_shutdown(self.shutdown)

    def jst_callback(self, msg):
        # self.lock.acquire()
        # self.name = msg.name
        self.ts.append(time.time())
        self.pos_rw.append(msg.position[0])
        self.pos_lw.append(msg.position[1])
        # self.velocity = msg.velocity
        # self.effort = msg.effort
        # self.lock.release()

    def go_forward(self, seconds):
        start_time = time.time()
        end_time = start_time + seconds

        while time.time() < end_time:
        	twist = Twist()
        	twist.linear.x = 0.1
        	self.cmd_vel_pub.publish(twist)

        self.cmd_vel_pub.publish(Twist())
        ts = np.array(self.ts) - start_time
        pos_rw = np.array(self.pos_rw)
        pos_lw = np.array(self.pos_lw)
        plt.plot(ts, pos_rw)
        plt.plot(ts, pos_lw)
        plt.grid()
        plt.show()

        # while not rospy.is_shutdown():            
        #     twist = Twist()
        #     # twist.linear.x = 0.0
        #     self.cmd_vel_pub.publish(twist)
        #     self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__=="__main__":
    bot_controller = BotController()
    bot_controller.go_forward(10)
