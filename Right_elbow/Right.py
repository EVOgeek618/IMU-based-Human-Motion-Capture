lis=[]
with open('Kinect_TF_msg.txt','r') as f:
    lis=f.readlines()
with open('Data.txt','w') as f:
    for i in range(len(lis)):
        if lis[i].find("right_elbow") != -1:
            f.write("{}, {}, {}, {}, {}, {}, {}\n".format(lis[i+7][8:-1],lis[i+8][8:-1],lis[i+9][8:-1],lis[i+10][8:-1],lis[i-3][8:-1],lis[i-2][8:-1],lis[i][4:-1]))
