#!/usr/bin/env python
import roslib
import rospy 
import tf
import numpy
import math
from std_msgs.msg import String

#Reading data from Kinect
lis=[]
f = open('DataKinect.txt', 'r')
lis=f.readlines()
f.close()

IMUs = {
    'rarm':['left_shoulder_1', 'left_elbow_1'],
    'rforearm':['left_elbow_1', 'left_hand_1'],
}

if __name__ == '__main__':
	try:
		#Readind data from sensors
		initstates = {}
		keys = []
		length = dict.fromkeys(IMUs.keys())
		time = []
		desiredrot = numpy.array(((-1,0,0,0),(0,1,0,0),(0,0,-1,0),(0,0,0,1)), dtype=numpy.float64)
		initrots = dict.fromkeys(IMUs.keys())
		for i in IMUs:
		    IMU=[]
		    f = open((i + '.txt'),'r')
		    IMU=f.readlines()
		    f.close()
		    for frame in IMUs[i]:
		        k = 0
		        while float(lis[k].split(',')[7]) < float(IMU[0].split(' ')[-1]):
		            k = k + 1
		        while lis[k].split(',')[8].split(' ')[1].strip('"') != frame:
		            k = k - 1
		        if abs(float(IMU[0].split(' ')[-1])-float(lis[k+15].split(',')[7])) < abs(float(IMU[0].split(' ')[-1])-float(lis[k].split(',')[7])):
		            k = k + 15
		        if frame not in keys:
		            initstates.update({frame: k})
		            keys = initstates.keys()
		        elif (frame in keys) and (k < initstates[frame]):
		            initstates[frame] = k
		    line0 = IMU[0].split(' ')
		    rot = line0[1:-1]
		    quat = (float(rot[0]),float(rot[1]),float(rot[2]),float(rot[3]))
		    time.append(line0[-1])
		    R = tf.transformations.quaternion_matrix(quat)
		    Rinv = numpy.linalg.inv(R)
		    initrots[i] = numpy.dot(Rinv, desiredrot)
		for frame in initstates:
		    point = lis[initstates[frame]].split(',')
		    initstates[frame] = [-float(point[4]), float(point[5]), -float(point[6])]
		for i in length:
		    first = IMUs[i][0]
		    second = IMUs[i][1]
		    firstx = initstates[first][0]
		    firsty = initstates[first][1]
		    firstz = initstates[first][2]
		    secondx = initstates[second][0]
		    secondy = initstates[second][1]
		    secondz = initstates[second][2]
		    length[i] = [(firstx - secondx), (firsty - secondy), (firstz - secondz)]
		intime = float(min(time))

		IMURots = dict.fromkeys(IMUs.keys())
		IMUTimes = dict.fromkeys(IMUs.keys())
		IMUTs = dict.fromkeys(IMUs.keys())
		exptwists = dict.fromkeys(IMUs.keys())

		T = {
		    'left_shoulder_1': [0, 0, 0],
		    'left_elbow_1':length['rarm'],
		    'left_hand_1':(length['rforearm']+length['rarm']),
		}

		M = dict.fromkeys(T.keys())
		for i in T:
		    P = numpy.identity(4);
		    P[0][3] = T[i][0]
		    P[1][3] = T[i][1]
		    P[2][3] = T[i][2]
		    M[i] = P
		joints = {
		    'left_shoulder_1':['rarm'],
		    'left_elbow_1':['rarm', 'rforearm'],
		    'left_hand_1':['rarm', 'rforearm'],
		}
		kinematics = dict.fromkeys(joints.keys())
		for i in IMUs:
		    IMU=[]
		    f = open((i + '.txt'),'r')
		    IMU=f.readlines()
		    f.close()
		    IMURots[i] = []
		    IMUTimes[i] = []
		    IMUTs[i] = []
		    for j in range(len(IMU)):
		        line = IMU[j].split(' ')
		        rot = line[1:-1]
		        #Finding quaternion and rotation matrix
		        quat = (float(rot[0]),float(rot[1]),float(rot[2]),float(rot[3]))
		        RIMU = tf.transformations.quaternion_matrix(quat)
		        R = numpy.dot(RIMU, initrots[i])
		        #Time
		        time = float(line[-1]) - intime
		        t = rospy.Time.from_sec(time)
		        IMURots[i].append(R)
		        IMUTimes[i].append(t)
		        IMUTs[i].append(time)
		    exptwists[i] = []

		    for j in range(len(IMURots[i]) - 1):
		        q1 = tf.transformations.quaternion_from_matrix(IMURots[i][j])
		        q2 = tf.transformations.quaternion_from_matrix(IMURots[i][j+1])
		        q = q2*tf.transformations.quaternion_conjugate(q1)
		        leng = math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2])
		        angle = 2*math.atan2(leng,q[3])
		        if leng > 0:
		            axis = numpy.array((q[0]/leng, q[1]/leng, q[2]/leng))
		        else:
		            axis = numpy.array((1,0,0))
		        w = tf.transformations.unit_vector(axis*angle/(IMUTs[i][j+1]-IMUTs[i][j]))
		        r = initstates[IMUs[i][0]]
		        v = numpy.cross(-w, r)
		        wwedge = numpy.array(((0, -w[2], w[1]), (w[2], 0, -w[0]), (-w[1], w[0], 0)))
		        twist = numpy.array(((wwedge[0][0], wwedge[0][1], wwedge[0][2], v[0]), (wwedge[1][0], wwedge[1][1], wwedge[1][2], v[1]), (wwedge[2][0], wwedge[2][1], wwedge[2][2], v[2]), (0, 0, 0, 0)))
		        expw = numpy.identity(3) + wwedge * math.sin(angle) + numpy.dot(wwedge, wwedge) * (1 - math.cos(angle))
		        exptw = numpy.dot((numpy.identity(3)*angle+(1-math.cos(angle))*wwedge+(angle-math.sin(angle))*numpy.dot(wwedge, wwedge)), v)
		        exptwist = numpy.array(((expw[0][0], expw[0][1], expw[0][2], exptw[0]),(expw[1][0], expw[1][1], expw[1][2], exptw[1]),(expw[2][0], expw[2][1], expw[2][2], exptw[2]),(0,0,0,1)))
		        exptwists[i].append(exptwist)
		tor = min([len(exptwists['rarm']), len(exptwists['rforearm'])])
		print(tor)
		for joint in kinematics:
		    kinematics[joint] = []
		    for i in range(tor):
		        trans = numpy.identity(4)
		        for j in joints[joint]:
		            trans = numpy.dot(trans, exptwists[j][i])
		        kinematics[joint].append(numpy.dot(trans, M[joint]))
		    Tinit = desiredrot
		    Tinit[:2,2] = initstates[joint][0]
		    kinematics[joint].insert(0, Tinit)

		#Broadcasting to rviz
		rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)
		rate = rospy.Rate(20)

		br = tf.TransformBroadcaster()
		while not rospy.is_shutdown():
			j = 0
			k = 0
			for i in range(tor+tor+2):
		    		if (IMUTimes['rarm'][j] <= IMUTimes['rforearm'][k] or k == (tor+1)) and j != (tor+1):
		        		br.sendTransform(kinematics['left_shoulder_1'][j][:3, 3], tf.transformations.quaternion_from_matrix(kinematics['left_shoulder_1'][j]), rospy.Time.now(), 'right_shoulder', 'map')
		        		if j < (tor+1):
		            			j = j+1
		    		elif (IMUTimes['rarm'][j] > IMUTimes['rforearm'][k] or j == (tor+1)) and k != (tor+1):
		        		br.sendTransform(kinematics['left_elbow_1'][k][:3,3], tf.transformations.quaternion_from_matrix(kinematics['left_elbow_1'][k]), rospy.Time.now(), 'right_elbow', 'map')
		        		br.sendTransform(kinematics['left_hand_1'][k][:3,3], tf.transformations.quaternion_from_matrix(kinematics['left_hand_1'][k]), rospy.Time.now(), 'right_hand', 'map')
		        		if k < (tor+1):
		            			k = k+1
		    		print('Sending transform!')
		    		rate.sleep()
	except rospy.ROSInterruptException:
			pass
