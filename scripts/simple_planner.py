#!/usr/bin/env python3

import rospy
import math
import tf2_ros
from tf.transformations import *

from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool

sphere_params = SphereParams()
received_sphere_params = False
rqt_toggle = False
pause_toggle = False


# /sphere_params subscriber
def get_sphere_params(data: SphereParams):
	global sphere_params
	global received_sphere_params
	sphere_params = data
	received_sphere_params = True

# /rqt_toggle
def rqt_listener(data):
	global rqt
	rqt_toggle = data.data

# /pause_toggle
def pause_listener(data):
	global pause
	pause_toggle = data.data


def setup_point(*points):
	plan_point = Twist()
	plan_point.linear.x, plan_point.linear.y, plan_point.linear.z, plan_point.angular.x, plan_point.angular.y, plan_point.angular.z = points
	return plan_point
	



if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a subscriber to listen for the sphere parameters
	sphere_sub = rospy.Subscriber("/sphere_params", SphereParams, get_sphere_params)
	# add subscribers for cancelling plan (rqt) or pausing movement (pause)
	rqt_sub = rospy.Subscriber("/rqt_toggle", Bool, rqt_listener)
	pause_sub = rospy.Subscriber("/pause_toggle", Bool, rqt_listener)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	

	
	q_rot = Quaternion()
	while not rospy.is_shutdown():
		# ensure we have received parameters to go off of
		if received_sphere_params:
			# try getting the most update transformation between the tool frame and the base frame
			try:
				trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print('Frames not available!!!')
				loop_rate.sleep()
				continue
			# extract the xyz coordinates
			x = trans.transform.translation.x
			y = trans.transform.translation.y
			z = trans.transform.translation.z
			
			# define a testpoint in the tool frame
			pt_in_camera = PointStamped()
			pt_in_camera.header.frame_id = 'camera_color_optical_frame'
			pt_in_camera.header.stamp = rospy.get_rostime()
			
			pt_in_camera.point.x = sphere_params.xc
			pt_in_camera.point.y = sphere_params.yc
			pt_in_camera.point.z = sphere_params.zc
			
			# convert the 3D point to the base frame coordinates
			pt_in_base = tfBuffer.transform(pt_in_camera,'base', rospy.Duration(1.0))
			x, y, z, rad = pt_in_base.point.x, pt_in_base.point.y, pt_in_base.point.z, sphere_params.radius
			plan = Plan()
			roll, pitch, yaw = 3.13, 0.017, 1.52
			plan.points.append(setup_point(-0.0142, -0.4095, 0.2713, roll, pitch, yaw))
			plan.points.append(setup_point(x, y, z + (rad*2), roll, pitch, yaw))
			plan.points.append(setup_point(x, y, z + 0.02, roll, pitch, yaw))
			plan.points.append(setup_point(x, y, z + (rad*2), roll, pitch, yaw))
			plan.points.append(setup_point(x + (rad*4), y, z + (rad*2), roll, pitch, yaw))
			plan.points.append(setup_point(x + (rad*4), y, z + rad, roll, pitch, yaw))
			plan.points.append(setup_point(x + (rad*4), y, z + (rad*2), roll, pitch, yaw))
			# publish the plan
			if not rqt_toggle:
				plan_pub.publish(plan)
			if pause_toggle:
				plan_pub.publish(Plan())
			# wait for 0.1 seconds until the next loop and repeat
			loop_rate.sleep()
