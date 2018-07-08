from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import Pose
from denso_robot_interface.srv import *
from std_msgs.msg import Char
import time

signal = 0
flag = 0

def callback(data):
    rospy.loginfo("Go to Point %c", data.data)
    global signal
    signal = data.data
    
def listener():
	global flag
	flag = 0
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('listener', anonymous=True)


	rospy.Subscriber("control_signal", Char, callback)
	controller = MoveItClient()

	while True:
		# print(signal)
		# print(flag)
		if signal == 49 and flag != 1:
			controller.move_it(1)
		elif signal == 50 and flag != 2:
			controller.move_it(2)
		elif signal == 51 and flag != 3:
			controller.move_it(3)
		elif signal == 52 and flag != 4:
			controller.move_it(4)
		time.sleep(1)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


class MoveItClient():
	def __init__(self):
		self.go_to = rospy.ServiceProxy('/denso_robot_interface/go_to', GoTo)
		self.translate = rospy.ServiceProxy('/denso_robot_interface/translation', Translation)

		approach_position = rospy.get_param('/ps2_2/approach_pose/position')
		approach_orientation = rospy.get_param('/ps2_2/approach_pose/orientation')
		self.approach_pose = Pose()
		# - Translation: [0.376, 0.160, 0.830]
		# - Rotation: in Quaternion [-0.000, 0.704, 0.000, 0.710]
    	# 			  in RPY [0.006, 1.561, 0.006]
		self.approach_pose.position.x = 0.376
		self.approach_pose.position.y = 0.160
		self.approach_pose.position.z = 0.830
		self.approach_pose.orientation.x = 0.000
		self.approach_pose.orientation.y = 0.704
		self.approach_pose.orientation.z = 0.000
		self.approach_pose.orientation.w = 0.710

		self.trajectory = [self.approach_pose]

		# - Translation: [0.376, -0.155, 0.830]
		# - Rotation: in Quaternion [0.000, 0.704, 0.000, 0.710]
		#             in RPY [0.004, 1.562, 0.004]
		add_pose = Pose()
		add_pose.position.x = 0.376
		add_pose.position.y = -0.155
		add_pose.position.z = 0.830
		add_pose.orientation.x = 0.000
		add_pose.orientation.y = 0.704
		add_pose.orientation.z = 0.000
		add_pose.orientation.w = 0.710
		self.trajectory.append(add_pose)

		# - Translation: [0.378, -0.155, 0.574]
		# - Rotation: in Quaternion [0.000, 0.704, 0.000, 0.710]
		#             in RPY [0.004, 1.562, 0.004]
		add_pose = Pose()
		add_pose.position.x = 0.378
		add_pose.position.y = -0.155
		add_pose.position.z = 0.574
		add_pose.orientation.x = 0.000
		add_pose.orientation.y = 0.704
		add_pose.orientation.z = 0.000
		add_pose.orientation.w = 0.710
		self.trajectory.append(add_pose)

		# - Translation: [0.378, 0.183, 0.574]
		# - Rotation: in Quaternion [0.000, 0.704, 0.000, 0.710]
		#             in RPY [0.005, 1.562, 0.005]
		add_pose = Pose()
		add_pose.position.x = 0.378
		add_pose.position.y = 0.183
		add_pose.position.z = 0.574
		add_pose.orientation.x = 0.000
		add_pose.orientation.y = 0.704
		add_pose.orientation.z = 0.000
		add_pose.orientation.w = 0.710
		self.trajectory.append(add_pose)

		print(self.trajectory)
		# self.trajectory = trajectory

	def move_it(self, index):


		try:
			request = GoToRequest()
			# for waypoint in trajectory:
			if index == 1:
				waypoint = self.trajectory[0]
				assert isinstance(waypoint, geometry_msgs.msg._Pose.Pose)
				request.next = waypoint
				respond = self.go_to(request)
				if respond.success == True:
					print ('Done!')
				else:
					print ('Error')
					raise AttributeError
			elif index == 2:
				waypoint = self.trajectory[1]
				assert isinstance(waypoint, geometry_msgs.msg._Pose.Pose)
				request.next = waypoint
				respond = self.go_to(request)
				if respond.success == True:
					print ('Done!')
				else:
					print ('Error')
					raise AttributeError
			elif index == 3:
				waypoint = self.trajectory[2]
				assert isinstance(waypoint, geometry_msgs.msg._Pose.Pose)
				request.next = waypoint
				respond = self.go_to(request)
				if respond.success == True:
					print ('Done!')
				else:
					print ('Error')
					raise AttributeError
			elif index == 4:
				waypoint = self.trajectory[3]
				assert isinstance(waypoint, geometry_msgs.msg._Pose.Pose)
				request.next = waypoint
				respond = self.go_to(request)
				if respond.success == True:
					print ('Done!')
				else:
					print ('Error')
					raise AttributeError

		except (AssertionError):
			print ('[Exception] Trajectory should be Pose object.')
		except (AttributeError):
			print ('[Exception] ERROR occurred during execution.')
		else:
			global flag
			flag = index
			# signal = 5
			# print ('>>> All waypoints executed!!')
			print ('>>> Position', index, 'reached!!')

if __name__ == "__main__":
	rospy.init_node('ps2_2')

	# controller = MoveItClient()
	

	# while True:
	listener()
		
		# controller.move_it()

	sys.exit(0)