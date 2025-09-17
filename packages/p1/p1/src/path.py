import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped



p = None
def get_pose(data):
    global p
    p = data


rospy.init_node("EEmarker")
pub = rospy.Publisher('/EE_path', Path, queue_size=10)
rospy.Subscriber('/EE_pose', Point, get_pose)
rate = rospy.Rate(10)

path = Path()
path.header.frame_id = "base"
path.poses = []


while not rospy.is_shutdown():
    if p is not None :
        newpose = PoseStamped() 
        newpose.pose.position.x = p.x
        newpose.pose.position.y = p.y
        newpose.pose.position.z = p.z
        path.poses.append(newpose)
        pub.publish(path)
        rate.sleep()






