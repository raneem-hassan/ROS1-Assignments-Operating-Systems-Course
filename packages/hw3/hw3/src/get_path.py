import rospy
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

print("Hey path")

rospy.init_node('Path_generator')
pub_path = rospy.Publisher('robot_path', Path, queue_size=10, latch=True)
pub_robot = rospy.Publisher('robot_pose', Marker, queue_size=10)
rate = rospy.Rate(1)

def heuristic(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def get_neighbors(node, x_min, x_max, y_min, y_max, obstacles):
    x, y = node
    moves = [(-1,0), (1,0), (0,-1), (0,1)]
    neighbors = []
    for dx, dy in moves:
        nx = x + dx
        ny = y + dy
        if x_min <= nx <= x_max and y_min <= ny <= y_max:
            if [nx, ny] not in obstacles:
                neighbors.append([nx, ny])
    return neighbors

def astar_simple(start, goal, obstacles, x_min, x_max, y_min, y_max):
    open_list = []
    closed_list = []

    open_list.append([start, 0, heuristic(start, goal), None])

    while open_list:
        # Find node with lowest f_cost 
        min_index = 0
        min_f = open_list[0][2]
        for i in range(1, len(open_list)):
            if open_list[i][2] < min_f:
                min_index = i
                min_f = open_list[i][2]
        current = open_list.pop(min_index)

        node, g, f, parent = current
        closed_list.append(current)

        if node == goal:
            path = []
            while current is not None:
                path.append(current[0])
                current = current[3]
            path.reverse()  
            return path

        neighbors = get_neighbors(node, x_min, x_max, y_min, y_max, obstacles)

        for neighbor in neighbors:
            already_closed = False
            for item in closed_list:
                if item[0] == neighbor:
                    already_closed = True
                    break
            if already_closed:
                continue

            g_new = g + 1
            f_new = g_new + heuristic(neighbor, goal)

            in_open = False
            for item in open_list:
                if item[0] == neighbor:
                    in_open = True
                    if g_new < item[1]:
                        item[1] = g_new
                        item[2] = f_new
                        item[3] = current
                    break

            if not in_open:
                open_list.append([neighbor, g_new, f_new, current])

    return []  # no path found

X = rospy.get_param('obstacles_x')
Y = rospy.get_param('obstacles_y')
start = rospy.get_param('starting_pose')
goal = rospy.get_param('goal_pose')

x0, y0 = start
xg, yg = goal

obstacles = [[int(x), int(y)] for x, y in zip(X, Y)]

x_min = min(min(X), int(x0), int(xg)) - 1
x_max = max(max(X), int(x0), int(xg)) + 1
y_min = min(min(Y), int(y0), int(yg)) - 1
y_max = max(max(Y), int(y0), int(yg)) + 1

path_points = astar_simple(start, goal, obstacles, x_min, x_max, y_min, y_max)

path_msg = Path()
path_msg.header.frame_id = "world"
path_msg.header.stamp = rospy.Time.now()

for pt in path_points:
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = pt[0]
    pose.pose.position.y = pt[1]
    path_msg.poses.append(pose)

robot = Marker()
robot.header.frame_id = "world"
robot.id = 0
robot.type = Marker.SPHERE
robot.action = Marker.ADD
robot.pose.orientation.w = 1.0
robot.color.r = 1.0
robot.color.a = 1.0
robot.scale.x = 0.5
robot.scale.y = 0.5
robot.scale.z = 0.5

pub_path.publish(path_msg)
while not rospy.is_shutdown():
    
    for pt in path_points:
        robot.header.stamp = rospy.Time.now()
        robot.pose.position.x = pt[0]
        robot.pose.position.y = pt[1]
        pub_robot.publish(robot)
        rate.sleep()