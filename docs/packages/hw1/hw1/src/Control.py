import rospy
from hw1.msg import object_position, robot_position, velocities, feedback
import math

xO = yO = xR = yR = thR = None
K1 = 1.5
K2 = 0.5
v_max = 40
w_max = 40

def get_object_pose(msg):
    global xO, yO 
    xO = msg.xO
    yO = msg.yO
    print('Subscribing to ObjectPose topic')

def get_robot_pose(msg):
    global xR, yR, thR
    xR = msg.xR
    yR = msg.yR
    thR = msg.thR
    print('Subscribing to Position topic')

def cal_vel(xO, yO, xR, yR, thR):
    dis = math.sqrt((xO - xR)**2 + (yO - yR)**2)
    if dis > 20:
        thO = math.atan2(yO - yR, xO - xR)
        e = math.atan2(math.sin(thO - thR), math.cos(thO - thR))
        if e > 0:
            w = min(K1*e, w_max)
        else:
            w = max(K1*e, -w_max)
        v = min(K2*dis, v_max)
    else:
        v = 0
        w = 0
    return v,w


rospy.init_node('Control')

rospy.Subscriber('ObjectPose', object_position, get_object_pose) 
rospy.Subscriber('Position', robot_position, get_robot_pose) 

vel_pub = rospy.Publisher('Velocities', velocities, queue_size=50)
feedback_pub = rospy.Publisher('Feedback', feedback, queue_size=50)
rate = rospy.Rate(50)

while not rospy.is_shutdown():
    if None not in (xO, yO, xR, yR, thR):
        vel = velocities()
        v , w = cal_vel(xO, yO, xR, yR, thR)
        vel.Forward_Vel = v
        vel.Angular_Vel = w
        vel_pub.publish(vel)  
        print("Puplishing velocities")
        data = feedback()
        data.xO = xO
        data.yO = yO
        data.xR = xR
        data.yR = yR
        data.thR = thR
        data.Forward_Vel = v
        data.Angular_Vel = w
        feedback_pub.publish(data)  
        print("Puplishing feedback")
    rate.sleep()

