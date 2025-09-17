import rospy
from hw1.msg import object_position
import numpy as np
import sys

rospy.init_node('Camera')

rate_pub = 100
max_msgs_num = 1000
if len(sys.argv) >= 2:
    rate_pub = int (sys.argv[1])
    if rate_pub > 50:
        rate_pub = 50
elif len(sys.argv) == 3:
    max_msgs_num = int(sys.argv[2])
    if max_msgs_num > 100:
        max_msgs_num = 100

print(f"Rate of Publishing = {rate_pub} and Maximum Number of Messages = {max_msgs_num}")

pub = rospy.Publisher('ObjectPose', object_position ,queue_size=max_msgs_num )

rate = rospy.Rate(rate_pub)

obj_pos = object_position()

while not rospy.is_shutdown():
    obj_pos.xO = 80*np.random.rand()
    obj_pos.yO = 80*np.random.rand()
    pub.publish(obj_pos)
    rate.sleep()