import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from math import *

th1 = th2 = d = None
def get_states(data):
    global th1, th2, d
    th1 = data.position[0]
    th2 = data.position[1]
    d = data.position[2]

def get_pose(th1, th2, d):
    point = Point()
    L1 = 1
    L2 = 1
    L3 = 1.3
    r = 1.5*L2 - 0.5*L3 + 2*d
    x = L1*cos(th1) + r*cos(th1+th2)
    y = L1*sin(th1) + r*sin(th1+th2)

    point.x = -y
    point.z = x

    return point

rospy.init_node("EEposePub")
pub = rospy.Publisher('/EE_pose', Point, queue_size=10)
rospy.Subscriber('/joint_states', JointState, get_states)
rate = rospy.Rate(10)




while not rospy.is_shutdown():
    if None not in (th1, th2, d):
        p = get_pose(th1, th2, d)
        pub.publish(p)
        rate.sleep()

