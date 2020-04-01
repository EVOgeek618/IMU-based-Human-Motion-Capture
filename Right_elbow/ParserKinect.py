#!/usr/bin/env python
lis=[]
with open('Kinect_TF_msg.txt','r') as f:
    lis=f.readlines()

#Quaternion: x, y, z, w; Translation: x, y, z; Time
with open('DataKinect.txt','w') as f:
    for i in range(len(lis)):
        if lis[i].find("right_elbow") != -1:
            nsec = lis[i-2].split(' ')[-1]
            f.write("{}, {}, {}, {}, {}, {}, {}, {}.{}".format(lis[i+7][11:-1],lis[i+8][11:-1],lis[i+9][11:-1],lis[i+10][11:-1],lis[i+3][11:-1],lis[i+4][11:-1],lis[i+5][11:-1], lis[i-3][14:-1], nsec))


