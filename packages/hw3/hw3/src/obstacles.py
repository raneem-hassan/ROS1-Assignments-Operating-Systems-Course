import rospy 
from visualization_msgs.msg import Marker, MarkerArray

print("Hey obstacles")

X = rospy.get_param('obstacles_x')
Y = rospy.get_param('obstacles_y')
starting_pose = rospy.get_param('starting_pose')
goal_pose = rospy.get_param('goal_pose')


obstacles = MarkerArray()
for i in range(len(X)):
    obs = Marker()
    obs.header.frame_id = "world"
    # obs.header.stamp = rospy.Time.now()
    obs.id = i
    obs.type = Marker.CUBE
    obs.action = Marker.ADD
    obs.pose.position.x = X[i]
    obs.pose.position.y = Y[i]
    obs.pose.position.z = 0
    obs.pose.orientation.x = 0
    obs.pose.orientation.y = 0
    obs.pose.orientation.z = 0
    obs.pose.orientation.w = 0
    obs.color.r = 0
    obs.color.g = 1
    obs.color.b = 0
    obs.color.a = 1
    obs.scale.x = 1
    obs.scale.y = 1
    obs.scale.z = 1
    obstacles.markers.append(obs)

rospy.init_node('Obstacles_generator')
pub = rospy.Publisher('obstacles', MarkerArray, queue_size=10)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    pub.publish(obstacles)
    rate.sleep()







