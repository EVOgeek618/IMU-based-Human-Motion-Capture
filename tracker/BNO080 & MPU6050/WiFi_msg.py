#!/usr/bin/env python3

import rospy
import tf
import socket
import time
from std_msgs.msg import String

BNO = open('BNO080.txt', 'w')
MPU = open('MPU6050.txt', 'w')

s = socket.socket()         
 
s.bind(('0.0.0.0', 8090 ))
s.listen(0)

#time.sleep(1)

vector = (0, 0, 0)
vector1 = (1, 0, 0)

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
				
				#content = content.replace("\r\n","\n")
				#content = content.replace(";\n","\n")
				#content = content.replace("\r","")
				#content = content.replace("\n","")
				#print(content)
				if len(content) < 5:
       					break
				elif content[0] != "#":
					break
				elif content[-1] != ";":
					break
				else:
					#while not rospy.is_shutdown():
					#content = content.replace("\r\n","")
					print(content)
					string =str(rospy.get_time())+": "+str(content[:]) 
					#rospy.loginfo(string)
					pub.publish(string)
					quat = content.split(" ")
					t = rospy.Time.from_sec(time.time())
					seconds = t.to_nsec()/1000000000
					nanoseconds = t.to_nsec()%1000000000
					#print(quat[0])
					#br.sendTransform(vector, (float(quat[1]), float(quat[2]), float(quat[3]), float(quat[4])), rospy.Time.now(), quat[0], "map")
					if quat[0] == "#BNO080":
						BNO.write(quat[0]+" "+quat[1]+" "+quat[2]+" "+quat[3]+" "+quat[4]+" "+str(seconds)+"."+ str(nanoseconds)+"\n")
						br.sendTransform(vector, (float(quat[1]), float(quat[2]), float(quat[3]), float(quat[4])), rospy.Time.now(), quat[0], "map")
					if quat[0] == "#MPU6050":
						MPU.write(quat[0]+" "+quat[1]+" "+quat[2]+" "+quat[3]+" "+quat[4]+" "+str(seconds)+"."+ str(nanoseconds)+"\n")
						br.sendTransform(vector1, (float(quat[1]), float(quat[2]), float(quat[3]), float(quat[4])), rospy.Time.now(), quat[0], "#BNO080")
						br.sendTransform(vector1, (0, 0, 0, 1), rospy.Time.now(), "end_hand", "#MPU6050")
		
		except rospy.ROSInterruptException:
			client.close()
			BNO.close()
			MPU.close()
			pass
		#client.close()
		#rate.sleep()              
 
