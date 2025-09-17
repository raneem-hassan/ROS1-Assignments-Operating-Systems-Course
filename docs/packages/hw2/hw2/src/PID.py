import rospy
from turtlesim.msg import Pose
from turtlesim.srv import *
from geometry_msgs.msg import Twist
import math
import time


# Contants
# For Ditstance Controller
kP1 = 0.95
kI1 = 0.00
kD1 = 0.00
# For Angle Controller
kP2 = 1.0
kI2 = 0.00
kD2 = 0.0

pose = Pose()

def get_pose (data):
    global pose 
    pose = data
    print ("turtle's pose is updated")

class PID_controller ():

    def __init__(self, t):
        self.prev_t = t
        self.prev_e_th = 0.0
        self.prev_sum_e_th = 0.0
        self.prev_de_th = 0.0

        self.prev_dis = 0.0
        self.prev_sum_dis = 0.0

    def PID_output(self, pose, target):
        vel = Twist()
        dx = target.x - pose.x
        dy = target.y - pose.y
        dis = math.sqrt(dx**2 + dy**2)
        if dis < 0.5:
            vel.linear.x = 0
            vel.angular.z = 0

        else:
            t = time.time()
            
            angle_to_goal = math.atan2(dy, dx)
            e_th = angle_to_goal - pose.theta

            e_th = math.atan2(math.sin(e_th), math.cos(e_th))

            dt = t - self.prev_t
            print (f"dt = {dt}")

            sum_dis = self.prev_sum_dis + (dis*dt)
            sum_th = self.prev_sum_e_th + (e_th*dt)


            d_dis = (self.prev_dis - dis)/dt
            d_th = (self.prev_de_th  - e_th)/dt

            self.prev_t = t
            self.prev_e_th = e_th
            self.prev_sum_e_th = sum_th
            self.prev_de_th = e_th

            self.prev_dis = dis
            self.prev_sum_dis = sum_dis

            
            vel.linear.x = (kP1 * dis) + (kI1 * sum_dis) + (kD1 * d_dis) 
            vel.angular.z = (kP2 * e_th) + (kI2 * sum_th) + (kD2 * d_th) 
        return vel




rospy.init_node('PID_Contoller')
vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('turtle1/pose', Pose, get_pose)
rate = rospy.Rate(10)
pid = PID_controller(time.time())
# vel = Twist()
rospy.wait_for_service('spawn')

r = 2
th0 = 0.0

d_th = 0.1
target = Pose()

# server = rospy.ServiceProxy('spawn', Spawn)
# request = server(target.x, target.y, target.theta, "target")

x0 = 5.544445
y0 = 5.544445


while not rospy.is_shutdown():
    target.x = x0 + r*math.cos(th0)
    target.y = y0 + r*math.sin(th0)
    target.theta = th0
    th0 += d_th
    # vel.linear.x = 1
    # vel.angular.z = 1

    vel = pid.PID_output(pose, target)
    vel_pub.publish(vel)
    rate.sleep()
    
