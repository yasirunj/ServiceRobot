import rospy
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu

v = 0
acc_0 = 0

global msg

def riemann_sum(acc_x):

    dt = 0.025

    v = v + ((acc_0 + acc_x)/2)*dt

    acc_0 = acc_x
    
    return v

def imuCallBack(imu):

    pub = rospy.Publisher('vel_pub', Twist, queue_size=10)

    msg.linear.x = riemann_sum(imu.linear_acceleration.x)
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.z = imu.angular_acceleration.z

    pub.publish(msg)

def listener():
    rospy.init_node('imutovel', anonymous=True)

    rospy.Subscriber("rtabmap/imu", Imu, imuCallBack)

    rate = rospy.Rate(40)

    rospy.spin()

if __name__ == '__main__':
    listener()