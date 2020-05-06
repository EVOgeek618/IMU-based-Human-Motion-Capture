#!/usr/bin/env python3

import rospy
import tf
import socket
import time
from std_msgs.msg import String

file = open('IMU_main', 'w')

s = socket.socket()         
 
s.bind(('0.0.0.0', 8090 ))
s.listen(0)

vector = (0, 0, 0)

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
	while True:
		client, addr = s.accept()
		pub = rospy.Publisher('IMU_rot', String, queue_size = 10)
		rospy.init_node('IMU', anonymous=False)
		br = tf.TransformBroadcaster()
		#rate = rospy.Rate(20)
		try:
			while True:
				content = client.recv(64)
				if len(content) < 10:
       					break
				else:
					#while not rospy.is_shutdown():
					content = content.replace("\r\n","")
					string =str(rospy.get_time())+": "+str(content[:]) 
					rospy.loginfo(string)
					pub.publish(string)
					quat = content.split(", ")
					t = rospy.Time.from_sec(time.time())
					seconds = t.to_nsec()/1000000000
					nanoseconds = t.to_nsec()%1000000000
					br.sendTransform(vector, (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])), rospy.Time.now(), "IMU_main", "fixed_frame")
					file.write(quat[0]+" "+quat[1]+" "+quat[2]+" "+quat[3]+" "+str(seconds)+"."+ str(nanoseconds)+"\n")
		except rospy.ROSInterruptException:
			client.close()
			file.close()
			pass
		#rate.sleep()              
 
