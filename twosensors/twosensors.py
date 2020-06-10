#!/usr/bin/env python
import roslib
import rospy 
import tf
import numpy
import math

rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)
starttime = rospy.Time.now()
#Reading data from Kinect
lis=[]
f = open('DataKinect.txt', 'r')
lis=f.readlines()
f.close()

#Initial dictionaries and lists
IMUs = {
    'rarm':['left_shoulder_1', 'left_elbow_1'],
    'rforearm':['left_elbow_1', 'left_hand_1'],
}
length = dict.fromkeys(IMUs.keys())
initrots = dict.fromkeys(IMUs.keys())

IMURots = dict.fromkeys(IMUs.keys())
IMUTimes = dict.fromkeys(IMUs.keys())
IMUTs = dict.fromkeys(IMUs.keys())
IMURraw = dict.fromkeys(IMUs.keys())

lIMU = ['rarm', 'rforearm']
initstates = {}
keys = []
time = []

jointstoIMU = {
    'left_shoulder_1':'rarm',
    'left_elbow_1':'rforearm',
    'left_hand_1':'none',
}
prjoint = {
	'left_shoulder_1':'none',
	'left_elbow_1':'left_shoulder_1',
	'left_hand_1':'left_elbow_1'
}
joints = {
	'left_shoulder_1':['left_shoulder_1'],
	'left_elbow_1':['left_shoulder_1', 'left_elbow_1'],
	'left_hand_1':['left_shoulder_1', 'left_elbow_1', 'left_hand_1'],
}
kinematics = dict.fromkeys(joints.keys())
exptwists = dict.fromkeys(joints.keys())

#Finding initial positions and rotations
desiredrot = numpy.array(((-1,0,0,0),(0,1,0,0),(0,0,-1,0),(0,0,0,1)), dtype=numpy.float64)
for i in IMUs:
    IMU=[]
    f = open((i + '.txt'),'r')
    IMU=f.readlines()
    f.close()
    if i == 'rarm':
        tor = len(IMU)
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
    length[i] = numpy.array(((firstx - secondx), (firsty - secondy), (firstz - secondz)))
intime = float(min(time))

T = {
    'left_shoulder_1': numpy.array((0, 0, 0)),
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

for i in lIMU:
    IMU=[]
    f = open((i + '.txt'),'r')
    IMU=f.readlines()
    f.close()
    IMURots[i] = []
    IMUTimes[i] = []
    IMUTs[i] = []
    IMURraw[i] = []

#Synchronizing data from two sensors
k=0
IMUarm = []
f = open('rarm.txt','r')
IMUarm=f.readlines()
f.close()
for i in range(len(IMUarm)):
    IMUarm[i] =IMUarm[i].split(' ')
IMUforearmold = []
f = open('rforearm.txt','r')
IMUforearmold=f.readlines()
f.close()
for i in range(len(IMUforearmold)):
    IMUforearmold[i] =IMUforearmold[i].split(' ')

IMUforearm = []
for i in range(tor):
    con = False
    while IMUarm[i][-1] >= IMUforearmold[k][-1]:
        k = k + 1
        con = True
    if con:
        k =  k - 1
    IMUforearm.append(IMUforearmold[k])
    
IMUlines = [IMUarm, IMUforearm]

#Reading data from IMU
for i in IMUlines:
    if i == IMUlines[0]:
        im = 'rarm'
    else:
        im = 'rforearm'
    for j in range(tor):
        IMU = i
        line = IMU[j]
        rot = line[1:-1]
        #Finding quaternion and rotation matrix
        quat = (float(rot[0]),float(rot[1]),float(rot[2]),float(rot[3]))
        RIMU = tf.transformations.quaternion_matrix(quat)
        if (im == 'rforearm') and (j < len(IMURraw['rarm'])):
            Rdiff = numpy.dot(numpy.linalg.inv(IMURraw['rarm'][j]), RIMU)
        elif im == 'rforearm':
            continue
        else:
            Rdiff = RIMU
        R = numpy.dot(Rdiff, initrots[im])
        time = float(line[-1]) - intime
        t = rospy.Time.from_sec(time)
        IMURots[im].append(R)
        IMUTimes[im].append(t)
        IMUTs[im].append(time)
        IMURraw[im].append(RIMU)

#Calculating exponential twist
for i in joints:
	exptwists[i] = []
	im = jointstoIMU[i]
	Ra = []
	if im != 'none':
		for j in range(len(IMURots[im])):
			q1 = tf.transformations.quaternion_from_matrix(IMURots[im][0])
			q2 = tf.transformations.quaternion_from_matrix(IMURots[im][j])
			q = tf.transformations.quaternion_multiply(q2,tf.transformations.quaternion_conjugate(q1))
			leng = math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2])
			angle = 2*math.atan2(leng,q[3])
			if leng > 10**(-10):
				axis = numpy.array((q[0]/leng, q[1]/leng, q[2]/leng))
			else:
				axis = numpy.array((1,0,0))
			wwedge = numpy.array(((0, -axis[2], axis[1]), (axis[2], 0, -axis[0]), (-axis[1], axis[0], 0)))
			expw = numpy.identity(3) + wwedge * math.sin(angle) + numpy.dot(wwedge, wwedge) * (1 - math.cos(angle))
			exptwists[i].append(expw)

jointorder = ['left_shoulder_1', 'left_elbow_1','left_hand_1']

for joint in jointorder:
    kinematics[joint] = []

if __name__ == '__main__':
    rate = rospy.Rate(20)
    br = tf.TransformBroadcaster()
    print('Beginning of broadcasting')
    try:
        for i in range(tor):
            if i == 0:
                print('Beginning the last math part')
	#Calculating transformation marix for every joint
            for joint in jointorder:
                trans = numpy.identity(3)
                for j in joints[joint]:
                    if j != joints[joint][-1]:
                        trans = numpy.dot(trans, exptwists[j][i])
                if prjoint[joint] != 'none':
                    prjo = prjoint[joint]
                    instpr = numpy.array(((0),(0),(0),(1)), dtype=numpy.float64)
                    expmap = numpy.identity(4)
                    expmap[:3,:3] = trans
                    expmap[:3,3] = kinematics[prjo][i][:3,3]
                    dif = numpy.array(((0),(0),(0),(1)), dtype=numpy.float64)
                    if i == 0:
                        instpr[:3] = T[prjo]
                        inst = M[joint]
                        dif[:3] = (inst[:,3]-instpr)[:3]              
                    else:
                        instpr[:3] = T[prjo]
                        inst = M[joint]
                        dif[:3] = (inst[:,3]-instpr)[:3]    
                    kinem = numpy.identity(4)
                    kinem[:3,3] = numpy.dot(expmap, dif)[:3]
                    kinem[:3,:3] = IMURots['rforearm'][i][:3,:3]
                else:
                    expmap = numpy.identity(4)
                    expmap[:3,:3] = trans
                    if i == 0:
                        inst = M[joint]
                    else:
                        inst = M[joint]
                    kinem = numpy.identity(4)
                    kinem[:3,3] = numpy.dot(expmap, inst[:,3])[:3]
                    kinem[:3,:3] = IMURots['rarm'][i][:3,:3]
                kinematics[joint].append(kinem)
            #Broacasting to rviz
            br.sendTransform(kinematics['left_shoulder_1'][i][:3, 3], tf.transformations.quaternion_from_matrix(kinematics['left_shoulder_1'][i]), rospy.Time.now(), 'right_shoulder', 'map')
            br.sendTransform(kinematics['left_elbow_1'][i][:3, 3], tf.transformations.quaternion_from_matrix(kinematics['left_elbow_1'][i]), rospy.Time.now(), 'right_elbow', 'map')
            br.sendTransform(kinematics['left_hand_1'][i][:3,3], tf.transformations.quaternion_from_matrix(kinematics['left_hand_1'][i]), rospy.Time.now(), 'right_hand', 'map')
            rate.sleep()
	endtime = rospy.Time.now()
	print(endtime-starttime)
    except rospy.ROSInterruptException:
        pass
	
