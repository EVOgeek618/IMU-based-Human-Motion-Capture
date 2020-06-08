#!/usr/bin/env python

import roslib
import rospy 
import tf
import numpy
import math
from std_msgs.msg import String

Transforms = open('Transforms.txt', 'w')

if __name__ == '__main__':
    rospy.init_node('listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
        	(T1,Q1) = listener.lookupTransform('/Shoulder', '/map', rospy.Time(0))
		(T2,Q2) = listener.lookupTransform('/Elbow', '/map', rospy.Time(0))
		(T3,Q3) = listener.lookupTransform('/Hand', '/map', rospy.Time(0))
		print(T1, Q1)
		print(T2, Q2)
		print(T3, Q3)
		Transforms.write("Shoulder: \tTransform: "+str(T1[0])+" "+str(T1[1])+" "+str(T1[2])+"\n")
		Transforms.write("\t\tQuat: "+str(Q1[0])+" "+str(Q1[1])+" "+str(Q1[2])+str(Q1[3])+"\n")
		Transforms.write("Elbow: \t\tTransform: "+str(T2[0])+" "+str(T2[1])+" "+str(T2[2])+"\n")
		Transforms.write("\t\tQuat: "+str(Q2[0])+" "+str(Q2[1])+" "+str(Q2[2])+str(Q2[3])+"\n")
		Transforms.write("Hand: \t\tTransform: "+str(T3[0])+" "+str(T3[1])+" "+str(T3[2])+"\n")
		Transforms.write("\t\tQuat: "+str(Q3[0])+" "+str(Q3[1])+" "+str(Q3[2])+str(Q3[3])+"\n")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

Transforms.close()
