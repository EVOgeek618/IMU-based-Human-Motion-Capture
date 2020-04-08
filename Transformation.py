#!/usr/bin/env python
import roslib
import rospy 
import tf
import numpy

#Reading data from Kinect
lis=[]
f = open('DataKinect.txt', 'r')
lis=f.readlines()
f.close()

#Finding start time
lis0 = lis[0].split(', ')
intime = float(lis0[7])

#Working with Kinect data
KinectQuats = []
KinectPoints = []
KinectTrans = []
KinectTimes = []
for i in range(len(lis)):
    lis[i]=lis[i].split(',')
    #Finding quaternion and rotation matrix
    quat = (float(lis[i][0]),float(lis[i][1]),float(lis[i][2]),float(lis[i][3]))
    R = tf.transformations.quaternion_matrix(quat)
    #Finding original point and translation matrix
    point = (float(lis[i][4]),float(lis[i][5]),float(lis[i][6]))
    Trans = tf.transformations.translation_matrix(point)
    #Combining matrices into translation matrix
    T = numpy.dot(Trans,R)
    #Time
    time = float(lis[i][7]) - intime
    t = rospy.Time.from_sec(time)
    #Adding to lists
    KinectQuats.append(quat) 
    KinectPoints.append(point)
    KinectTrans.append(T)
    KinectTimes.append(t)

#Readind data from sensors
IMU=[]
with open('getData.txt','r') as f:
    IMU=f.readlines()

#Working with sensor's data
IMUQuats = []
IMUPoints = []
IMUTrans = []
IMUTimes = []
k = 0
for i in range(len(IMU)):
    line = IMU[i].split(' ')
    rot = line[0:-1]
    #Finding quaternion and rotation matrix
    quat = (float(rot[0][3:]),float(rot[1]),float(rot[2]),float(rot[3][:-4]))
    R = tf.transformations.quaternion_matrix(quat)
    #Time
    time = float(line[-1]) - intime
    t = rospy.Time.from_sec(time)
    #Finding the closest original point and translation matrix
    while KinectTimes[k] < t:
        k = k + 1
    if abs(t-KinectTimes[k+1]) < abs(t-KinectTimes[k]): 
        k = k + 1  
    point = KinectPoints[k]
    Trans = tf.transformations.translation_matrix(point)
    #Combining matrices into translation matrix
    T = numpy.dot(Trans,R)
    #Adding to lists
    IMUQuats.append(quat) 
    IMUPoints.append(point)
    IMUTrans.append(T)
    IMUTimes.append(t)

#Broadcasting to rviz
rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)

br = tf.TransformBroadcaster()

j = 0
k = 0
for i in range(len(lis)+len(IMU)):
    if IMUTimes[j] < KinectTimes[k]:
        br.sendTransform(IMUPoints[j], IMUQuats[j], IMUTimes[j], 'right_elbow_IMU', 'fixed_frame')
        #print(IMUTrans[j])
        print('IMU')
        if j < (len(IMU)-1):
            j = j+1
        rospy.Rate(10).sleep()
    else:
        br.sendTransform(KinectPoints[k], KinectQuats[k], KinectTimes[k], 'right_elbow_Kinect', 'fixed_frame')
        #print(KinectTrans[k])
        print('Kinect')
        if k < (len(lis)-1):
            k = k+1
        rospy.Rate(10).sleep()
    

    
