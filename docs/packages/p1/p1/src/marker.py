import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point




p = None
def get_pose(data):
    global p
    p = data

def create_marker(rgba, p):
    marker = Marker()
    marker.header.frame_id = "base"
    marker.id = 0
    marker.action= Marker.ADD
    marker.type = Marker.SPHERE
    marker.pose.position.x = p.x 
    marker.pose.position.y = p.y 
    marker.pose.position.z = p.z  
    marker.scale.x = marker.scale.y = marker.scale.z = 0.2
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]
    return marker

rospy.init_node("EEmarker")
pub = rospy.Publisher('/EE_visual', Marker, queue_size=10)
rospy.Subscriber('/EE_pose', Point, get_pose)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    rgba = rospy.get_param('rgba')
    if p is not None :
        marker = create_marker(rgba, p)
        pub.publish(marker)
        rate.sleep()




