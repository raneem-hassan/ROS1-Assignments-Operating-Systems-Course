import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Float32MultiArray

POT_value = None
def get_POT_value (msg):
    global POT_value
    POT_value = msg.data

rospy.init_node("Control")
pub_z = rospy.Publisher("/altitude", Float32MultiArray, queue_size=10)
pub_marker = rospy.Publisher("/create_sphere", Marker, queue_size=10)
rospy.Subscriber("/pot", Float32, get_POT_value)
rate = rospy.Rate(1)

zmin = rospy.get_param('zmin')
zmax = rospy.get_param('zmax')
tolerance = rospy.get_param('tolerance')


while not rospy.is_shutdown():
    if POT_value is not None:
        z_value = zmin + (POT_value/ 1023.0) * (zmax - zmin);

        sphere = Marker()
        sphere.action = Marker.ADD
        sphere.type = Marker.SPHERE
        sphere.header.frame_id = "world"
        sphere.header.stamp = rospy.Time.now()
        sphere.pose.position.z = z_value
        sphere.color.r = 1
        sphere.color.a = 1
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 1

        data = Float32MultiArray()
        data.data = [z_value, tolerance]

        pub_z.publish(data)
        pub_marker.publish(sphere)
        rate.sleep()

