import rospy
from hw1.msg import robot_position, velocities
import numpy as np

def get_velocities (vel):
    print('Subscribing to Velocities topic')

rospy.init_node('Robot')
rospy.Subscriber('Velocities', velocities, get_velocities)
pub = rospy.Publisher('Position', robot_position, queue_size=100)
rate = rospy.Rate(50)

while not rospy.is_shutdown():
    pos = robot_position()
    pos.xR = 100*np.random.rand()
    pos.yR = 100*np.random.rand()
    pos.thR= np.random.rand()
    pub.publish(pos)
    rate.sleep()

