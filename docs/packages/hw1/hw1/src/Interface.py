import rospy
from hw1.msg import feedback

def print_status (msg):
    print('Subscribing to Feedback topic')
    xO = msg.xO
    yO = msg.yO
    xR = msg.xR
    yR = msg.yR
    thR = msg.thR
    v = msg.Forward_Vel
    w = msg.Angular_Vel
    print(f"Object Position : [{xO}, {yO}]")
    print(f"Robot Pose : [{xR}, {yR}, {thR}]")
    print(f"Robot Velocities : v = {v}, w = {w}")

rospy.init_node('Interface')
rospy.Subscriber('Feedback', feedback, print_status)

rospy.spin()
