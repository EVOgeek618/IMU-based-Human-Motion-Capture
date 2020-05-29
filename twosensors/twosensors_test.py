#!/usr/bin/env python

# Tasks:
# 1. Find lehgths of body parts according to the data taken from Kinect				DONE
#	a) get rid of extra data from Kinect in the beginning					DONE
# 2. Turn the sensors in their initial conditions by finding transformation matrices		DONE	
# 3. Publisher tf for visualizing data in rviz
#	a) Solve the issue of time equality for sensors

import roslib
import rospy 
import tf
import numpy
import math
from std_msgs.msg import String

if __name__ == '__main__':
	try:
		# Reading Kinect data from file
		Kinect = []
		f = open('DataKinect.txt', 'r')
		Kinect = f.readlines()
		f.close()
		
		IMUs = {
#        		'neck': ['neck_1', 'head_1'],
#        		'tors': ['torso_1', 'neck_1'],
#        		'larm': ['left_shoulder_1', 'left_elbow_1'],
#        		'lforearm': ['left_elbow_1', 'left_hand_1'],
        		'rarm': ['right_shoulder_1', 'right_elbow_1'],
        		'rforearm': ['right_elbow_1', 'right_hand_1'],
#        		'lthigh': ['left_hip_1', 'left_knee_1'],
#        		'lleg': ['left_knee_1', 'left_foot_1'],
# 			'lfoot':['left_foot_1'],
#        		'rthigh': ['right_hip_1', 'right_knee_1'],
#        		'rleg': ['right_knee_1', 'right_foot_1'],
# 			'rfoot':['right_foot_1']
        		}
		
		# Reading Sensors data from files
		# BNO080
		f = open('rarm.txt','r')
		R_Arm_rot = f.readlines()
		f.close()
		# MPU6050
		f = open('rforearm.txt','r')
		R_ForeArm_rot = f.readlines()
		f.close()

		# Getting body parts vectors and lengths from Kinect data
		# i	j	k	w	x	y	z	time	name
		# Finding vectors from coordinate frame to points on body
		#i = 0
		i = -1
		V_Shoulder = []
		V_Elbow = []
		V_Hand = []
		while (len(V_Shoulder) < 1) or (len(V_Elbow) < 1) or (len(V_Hand) < 1):
			i = i+1
			Kinect_time = float(Kinect[i].split(',')[-2].split(' ')[1])
			Sensor_time = float(R_Arm_rot[0].split(' ')[1:][-1])
			#print(Kinect_time, " :: ", Sensor_time)
			if Kinect_time < Sensor_time:
				continue
			if Kinect[i].split(',')[-1].split(' ')[1].strip('"') == 'left_shoulder_1':
				V_Shoulder = [float(Kinect[i].split(', ')[4]), float(Kinect[i].split(', ')[5]), float(Kinect[i].split(', ')[6])]
				#print(V_Shoulder)
			if Kinect[i].split(',')[-1].split(' ')[1].strip('"') == 'left_elbow_1':
				V_Elbow = [float(Kinect[i].split(', ')[4]), float(Kinect[i].split(', ')[5]), float(Kinect[i].split(', ')[6])]
				#print(V_Elbow)
			if Kinect[i].split(',')[-1].split(' ')[1].strip('"') == 'left_hand_1':
				V_Hand = [float(Kinect[i].split(', ')[4]), float(Kinect[i].split(', ')[5]), float(Kinect[i].split(', ')[6])]
				#print(V_Hand)
			#i = i+1
		R_Arm_len = math.sqrt( (V_Shoulder[0]-V_Elbow[0])**2 + (V_Shoulder[1]-V_Elbow[1])**2 + (V_Shoulder[2]-V_Elbow[2])**2 )
		#print(R_Arm_len)
		R_ForeArm_len = math.sqrt( (V_Elbow[0]-V_Hand[0])**2 + (V_Elbow[1]-V_Elbow[1])**2 + (V_Elbow[2]-V_Hand[2])**2 )
		#print(R_ForeArm_len)

		V_Arm = ((V_Shoulder[0]-V_Elbow[0]), (V_Shoulder[1]-V_Elbow[1]), (V_Shoulder[2]-V_Elbow[2]))
		V_ForeArm = ((V_Elbow[0]-V_Hand[0]), (V_Elbow[1]-V_Hand[1]), (V_Elbow[2]-V_Hand[2]))


		#Broadcasting to rviz
		rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)
		rate = rospy.Rate(20)
		br = tf.TransformBroadcaster()

		#Desired initial rotations of Sensors
		R_Arm_Desired_rot = numpy.array(((-1,0,0,0),(0,1,0,0),(0,0,-1,0),(0,0,0,1)), dtype=numpy.float64)
		R_ForeArm_Desired_rot = numpy.array(((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1)), dtype=numpy.float64)
		
		for i in range(min(len(R_Arm_rot),len(R_ForeArm_rot))):

			# Right Arm Data
			R_Arm_data = R_Arm_rot[i].split(" ")[1:5]
			R_Arm_fact_quat = (float(R_Arm_data[0]), float(R_Arm_data[1]), float(R_Arm_data[2]), float(R_Arm_data[3]))
			R_Arm_fact_matrix = tf.transformations.quaternion_matrix(R_Arm_fact_quat)

			# Right Forearm Data
			R_ForeArm_data = R_ForeArm_rot[i].split(" ")[1:5]
			R_ForeArm_fact_quat = (float(R_ForeArm_data[0]), float(R_ForeArm_data[1]), float(R_ForeArm_data[2]), float(R_ForeArm_data[3]))
			R_ForeArm_fact_matrix = tf.transformations.quaternion_matrix(R_ForeArm_fact_quat)

			R_ForeArm_relative_matrix = numpy.dot( numpy.linalg.inv(R_Arm_fact_matrix), R_ForeArm_fact_matrix)

			if i == 0:
				# Finding the initial rotation matrices
				
				# For Right Arm
				#R_Arm_init_fact_quat = (float(R_Arm_rot[0].split(' ')[1:-1][0]),float(R_Arm_rot[0].split(' ')[1:-1][1]),float(R_Arm_rot[0].split(' ')[1:-1][2]),float(R_Arm_rot[0].split(' ')[1:-1][3]))
				R_Arm_init_fact_quat = R_Arm_fact_quat
				#print("R_Arm_init_fact_quat = ",R_Arm_init_fact_quat)
				R_Arm_rot_matrix = tf.transformations.quaternion_matrix(R_Arm_init_fact_quat)
				#print("R_Arm_rot_matrix = ",R_Arm_rot_matrix)
				R_Arm_rot_matrix_inv = numpy.linalg.inv(R_Arm_rot_matrix)
				#print("R_Arm_rot_matrix_inv = ",R_Arm_rot_matrix_inv)
				R_Arm_init_desired_rot = numpy.dot(R_Arm_rot_matrix_inv, R_Arm_Desired_rot)
				#print("R_Arm_init_desired_rot = ",R_Arm_init_desired_rot)

				#R_Arm_init_desired_matrix = numpy.dot(R_Arm_rot_matrix, R_Arm_init_desired_rot)
				#print("R_Arm_init_desired_matrix = ",R_Arm_init_desired_matrix)

				
				# For Right ForeArm
				#R_ForeArm_init_fact_quat = (float(R_ForeArm_rot[0].split(' ')[1:-1][0]),float(R_ForeArm_rot[0].split(' ')[1:-1][1]),float(R_ForeArm_rot[0].split(' ')[1:-1][2]),float(R_ForeArm_rot[0].split(' ')[1:-1][3]))
				#R_ForeArm_init_fact_quat = R_ForeArm_fact_quat
				#print("R_ForeArm_init_fact_quat = ",R_ForeArm_init_fact_quat)
				#R_ForeArm_rot_matrix = tf.transformations.quaternion_matrix(R_ForeArm_init_fact_quat)
				#print("R_ForeArm_rot_matrix = ",R_ForeArm_rot_matrix)
				
				R_ForeArm_init_relative_matrix = R_ForeArm_relative_matrix

				#R_ForeArm_rot_matrix_inv = numpy.linalg.inv(R_ForeArm_rot_matrix)
				#print("R_ForeArm_rot_matrix_inv = ",R_ForeArm_rot_matrix_inv)
				#R_ForeArm_init_desired_rot = numpy.dot(R_ForeArm_rot_matrix_inv, R_ForeArm_Desired_rot)
				#print("R_ForeArm_init_desired_rot = ",R_ForeArm_init_desired_rot)

				R_ForeArm_rot_matrix_inv = numpy.linalg.inv(R_ForeArm_init_relative_matrix)
				R_ForeArm_init_desired_rot = numpy.dot(R_ForeArm_rot_matrix_inv, R_ForeArm_Desired_rot)

				#R_ForeArm_init_desired_matrix = numpy.dot(R_ForeArm_rot_matrix, R_ForeArm_init_desired_rot)
				#print("R_ForeArm_init_desired_matrix = ",R_ForeArm_init_desired_matrix)

				#R_ForeArm_init_desired_matrix = numpy.dot(R_ForeArm_init_relative_matrix, R_ForeArm_init_desired_rot)
				#print("R_ForeArm_init_desired_matrix = ",R_ForeArm_init_desired_matrix)


        		R_Arm_desired_matrix = numpy.dot(R_Arm_fact_matrix, R_Arm_init_desired_rot)
			R_Arm_desired_quat = tf.transformations.quaternion_from_matrix(R_Arm_desired_matrix)
			
			
        		#R_ForeArm_desired_matrix = numpy.dot(R_ForeArm_fact_matrix, R_ForeArm_init_desired_rot)
			#R_ForeArm_desired_quat = tf.transformations.quaternion_from_matrix(R_ForeArm_desired_matrix)

			R_ForeArm_desired_matrix = numpy.dot(R_ForeArm_relative_matrix, R_ForeArm_init_desired_rot)
			R_ForeArm_desired_quat = tf.transformations.quaternion_from_matrix(R_ForeArm_desired_matrix)

			#R_ForeArm_desired_quat = tf.transformations.quaternion_from_matrix(numpy.dot(numpy.dot(R_Arm_fact_matrix, numpy.linalg.inv(R_ForeArm_fact_matrix)), R_ForeArm_desired_matrix))
			#R_ForeArm_desired_quat = tf.transformations.quaternion_from_matrix(numpy.dot(numpy.linalg.inv(R_Arm_desired_matrix), R_ForeArm_desired_matrix))
			
			
			# Sending Transforms
			br.sendTransform((0, 0, 0), R_Arm_desired_quat, rospy.Time.now(), "Shoulder", "map")
			#br.sendTransform((0, 0, R_Arm_len), R_ForeArm_desired_quat, rospy.Time.now(), "Elbow", "Shoulder")
			br.sendTransform((V_Arm[0], V_Arm[1], V_Arm[2]), R_ForeArm_desired_quat, rospy.Time.now(), "Elbow", "Shoulder")
			#br.sendTransform((0, 0, R_ForeArm_len), (0, 0, 0, 1), rospy.Time.now(), "Hand", "Elbow")
			br.sendTransform((-V_ForeArm[2], V_ForeArm[1], V_ForeArm[0]), (0, 0, 0, 1), rospy.Time.now(), "Hand", "Elbow")
			#br.sendTransform((V_ForeArm[0], V_ForeArm[1], V_ForeArm[2]), (0, 0, 0, 1), rospy.Time.now(), "Hand", "Elbow")
			
			rate.sleep()
		#for line in R_Arm_rot:
			#data = line.split(" ")[1:5]
			#fact_quat = (float(data[0]), float(data[1]), float(data[2]), float(data[3]))
			#print(quat)
			#R_Arm_fact_matrix = tf.transformations.quaternion_matrix(fact_quat)
        		#R_Arm_desired_matrix = numpy.dot(R_Arm_fact_matrix, R_Arm_init_desired_rot)
			#R_Arm_desired_quat = tf.transformations.quaternion_from_matrix(R_Arm_desired_matrix)
			#br.sendTransform((0, 0, 0), R_Arm_desired_quat, rospy.Time.now(), "Shoulder", "map")
			#br.sendTransform((R_Arm_len, 0, 0), (0, 0, 1, 0), rospy.Time.now(), "Elbow", "Shoulder")
			#br.sendTransform((R_ForeArm_len, 0, 0), (1, 0, 0, 0), rospy.Time.now(), "Hand", "Elbow")
			#rate.sleep()

		#for line in R_ForeArm_rot:
			#data = line.split(" ")[1:5]
			#fact_quat = (float(data[0]), float(data[1]), float(data[2]), float(data[3]))
			#print(quat)
			#R_ForeArm_fact_matrix = tf.transformations.quaternion_matrix(fact_quat)
        		#R_ForeArm_desired_matrix = numpy.dot(R_ForeArm_fact_matrix, R_ForeArm_init_desired_rot)
			#R_ForeArm_desired_quat = tf.transformations.quaternion_from_matrix(R_ForeArm_desired_matrix)
			#br.sendTransform((0, 0, 0), R_ForeArm_desired_quat, rospy.Time.now(), "Elbow", "map")
			#br.sendTransform((R_Arm_len, 0, 0), (0, 0, 1, 0), rospy.Time.now(), "Elbow", "Shoulder")
			#rate.sleep()

	except rospy.ROSInterruptException:
			pass
