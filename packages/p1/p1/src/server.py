import rospy
from p1.srv import *


def color_server(req):
    rgba = [req.r, req.g, req.b,req.a]
    rospy.set_param('rgba', rgba)
    res = EEcolorResponse()
    res.success = True
    return res

rospy.init_node("EEcolorServer")
rospy.Service('set_color', EEcolor, color_server)
rospy.spin()