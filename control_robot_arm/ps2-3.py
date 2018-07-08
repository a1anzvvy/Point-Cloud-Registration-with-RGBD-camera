#! /usr/bin/python
# This is the script for ps2-2
# In this part, you will need to send pose command to the
# robot controller and move the manipulator according to
# the trajectory in ps1

# if you are using python3, then you don't need this line
from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import Pose
from denso_robot_interface.srv import *

# create a class for this node
class MoveItClient():
  def __init__(self):
    # a service client that connects to the 'go_to' and 'translate' service
    self.go_to = rospy.ServiceProxy('/denso_robot_interface/go_to', GoTo)
    self.translate = rospy.ServiceProxy('/denso_robot_interface/translation', Translation)

    # approach pose (all pose read from your txt file should be relative to this pose)
    approach_position = rospy.get_param('/ps2_2/approach_pose/position')
    approach_orientation = rospy.get_param('/ps2_2/approach_pose/orientation')
    self.approach_pose = Pose()
    self.approach_pose.position.x = approach_position[0]
    self.approach_pose.position.y = approach_position[1]
    self.approach_pose.position.z = approach_position[2]
    self.approach_pose.orientation.x = 0
    self.approach_pose.orientation.y = 0.744
    self.approach_pose.orientation.z = 0.001
    self.approach_pose.orientation.w = 0.668
    # self.approach_pose.orientation.x = approach_orientation[0]
    # self.approach_pose.orientation.y = approach_orientation[1]
    # self.approach_pose.orientation.z = approach_orientation[2]
    # self.approach_pose.orientation.w = approach_orientation[3]
  # this is where we do everything
  def move_it(self):
    ####### INSERT YOUR CODE HERE ########
    # we will move to the approach_pose first
    trajectory = [self.approach_pose]
    # print (trajectory)
    # this is where you read the txt file
    inFile = open('2d-shape-1-output.txt')
    inputString = inFile.read()
    stringList = inputString.splitlines()
    f1 = open('motion-1.yaml','w+')
    f1.write('trajectory:\n')
    # f1.write('	joint_space:\n')
    flag_first = 1
    last_pose_x = 0
    last_pose_y = 0
    move_down = 0
    move_up = 0
    move_to = 0
    i = 0
    # this is where you transfer the X, Y, Z
    # value into a geometry_msgs.msg.Pose object
    # (DON'T FORGET TO TAKE APPROACH_POSE INTO ACCOUNT
    #  WHEN CALCULATING X, Y, Z)
    for line in stringList:
		if (line[0]=='1'):
			line = line.split(',')
			line = map(float,line)
			end_pose = Pose()
      # end_pose.position.x = 1
      # end_pose.position.y = 1
      # end_pose.position.z = 1
			end_pose.position.x = line[1]/1000.0+self.approach_pose.position.x
      end_pose.position.y = line[2]/1000.0+self.approach_pose.position.y

			if (flag_first == 1):
				# first_iteration
				end_pose.position.x = self.approach_pose.position.x
				end_pose.position.y = self.approach_pose.position.y
				end_pose.position.z = self.approach_pose.position.z - 20.0/1000.0
				flag_first = 0

			elif (move_to == 1):
				end_pose.position.x = line[1]/1000.0+self.approach_pose.position.x
				end_pose.position.y = line[2]/1000.0+self.approach_pose.position.y
				end_pose.position.z = self.approach_pose.position.z
				move_to = 0
				move_down = 1

			elif (move_down == 1):
				a=1
				end_pose.position.x = last_pose_x
				end_pose.position.y = last_pose_y
				end_pose.position.z = self.approach_pose.position.z - 20.0/1000.0
				move_down = 0

			elif (abs(last_pose_x - end_pose.position.x) > 10/1000.0):
				# reach last point of a segment, move up
				end_pose.position.x = last_pose_x
				end_pose.position.y = last_pose_y
				end_pose.position.z = self.approach_pose.position.z
				move_to = 1

			else:
				# normal zig zag
				end_pose.position.x = line[1]/1000.0+self.approach_pose.position.x
				end_pose.position.y = line[2]/1000.0+self.approach_pose.position.y
				end_pose.position.z = self.approach_pose.position.z - 20.0/1000.0
			# end_pose.position.z = self.approach_pose.position.z
			
			end_pose.orientation.x = self.approach_pose.orientation.x
			end_pose.orientation.y = self.approach_pose.orientation.y
			end_pose.orientation.z = self.approach_pose.orientation.z
			end_pose.orientation.w = self.approach_pose.orientation.w
			last_pose_x = end_pose.position.x
			last_pose_y = end_pose.position.y
			trajectory.append(end_pose)
			i = i + 1
			f1.write('	position: ['+str(end_pose.position.x)+','+str(end_pose.position.y)+','+str(end_pose.position.z)+']\n')
			f1.write('  orientation: ['+str(end_pose.orientation.x)+','+str(end_pose.orientation.y)+','+str(end_pose.orientation.z)+','+str(end_pose.orientation.w)+']\n')

    # fill the trajectory with a list of Pose object
    # you wish to execute
    ######################################

    # sending the trajectory to the MoveIt interface
    try:
      trj_index = 1
      request = GoToRequest()
      for waypoint in trajectory:
        # make sure we are dealing with a Pose object
        assert isinstance(waypoint, geometry_msgs.msg._Pose.Pose)
        # fill in the request
        request.next = waypoint
        # send the request
        print ('>>> Executing waypoint %d...'.format(trj_index), end = '')
        respond = self.go_to(request)
        # chech whether the execution is a success
        if respond.success == True:
          print ('Done!')
          continue
        else:
          print ('ERROR!')
          print ('<<< %s'.format(respond.reason))
          raise AttributeError
    except (AssertionError):
      print ('[Exception] Trajectory should be Pose object.')
    except (AttributeError):
      print ('[Exception] ERROR occurred during execution.')
    else:
      print ('>>> All waypoints executed!!')


if __name__ == "__main__":
  rospy.init_node('ps2_2')

  controller = MoveItClient()
  controller.move_it()

  sys.exit(0)
