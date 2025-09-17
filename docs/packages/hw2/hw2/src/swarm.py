import rospy
from turtlesim.msg import Pose
from turtlesim.srv import *
from geometry_msgs.msg import Twist
import math
import time


# PID Contants
# Ditstance 
kP1 = 1
kI1 = 0.001
kD1 = 0.7
# Orientation
kP2 = 2
kI2 = 0.00
kD2 = 0.0

pose = Pose()



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

class Turtle ():
    def __init__(self, i, x0, y0, th0):
        server = rospy.ServiceProxy('spawn', Spawn)
        request = server(x0, y0, th0, f"Turtle{i}")
        self.vel_pub = rospy.Publisher(f'Turtle{i}/cmd_vel', Twist, queue_size=10)
        self.pos_subs = rospy.Subscriber(f'Turtle{i}/pose', Pose, self.get_pose)
        
        self.pid = PID_controller(time.time())
        self.pose = Pose()
        # self.r = r
        # self.i = i
    
    def get_pose (self, data):
        # global pose 
        self.pose = data
        # print ("turtle's pose is updated")

    def move (self, target):
            vel = Twist()
            vel = self.pid.PID_output(self.pose, target)
            self.vel_pub.publish(vel)




rospy.init_node('SwarmGenerator')
#vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
#rospy.Subscriber('turtle1/pose', Pose, get_pose)

pid = PID_controller(time.time())
# vel = Twist()



x0 = 5.544445
y0 = 5.544445
th0 = 0.0

d_th = 0.1
target = Pose()

n = rospy.get_param('n')
r = rospy.get_param('r')
spacing = rospy.get_param('spacing')

turtles = []
for i in range(n):
    turtles.append(Turtle(i, x0, y0, th0))


rospy.wait_for_service('kill')
kill_turtle = rospy.ServiceProxy('kill', Kill)
kill_turtle('turtle1')





rate = rospy.Rate(10)
rospy.wait_for_service('spawn')

while not rospy.is_shutdown():

    for i in range(n):
        R = r + (spacing*i)
        target.x = x0 + R*math.cos(th0)
        target.y = y0 + R*math.sin(th0)
        target.theta = th0
        turtles[i].move(target)
    th0 += d_th
    # vel.linear.x = 1
    # vel.angular.z = 1

    # vel = pid.PID_output(pose, target)
    # vel_pub.publish(vel)
    rate.sleep()
