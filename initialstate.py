#Reading data from Kinect
lis=[]
f = open('DataKinect.txt', 'r')
lis=f.readlines()
f.close()

#Finding start time
lis0 = lis[0].split(', ')
intime = float(lis0[7])


IMUs = {
    'neck':['head_1', 'neck_1'],
    'tors':['neck_1', 'torso_1'],
    'larm':['left_shoulder_1', 'left_elbow_1'],
    'lforearm':['left_elbow_1', 'left_hand_1'],
    'rarm':['right_shoulder_1', 'right_elbow_1'],
    'rforearm':['right_elbow_1', 'right_hand_1'],
    'lthigh':['left_hip_1', 'left_knee_1'],
    'lleg':['left_knee_1', 'left_foot_1'],
    'lfoot':['left_foot_1'],
    'rthigh':['right_hip_1', 'right_knee_1'],
    'rleg':['right_knee_1', 'right_foot_1'],
    'rfoot':['right_foot_1']}
    
#Readind data from sensors
initstates = {}
keys = []
length = dict.fromkeys(IMUs.keys())
del length['lfoot']
del length['rfoot']

desiredrot = [[-1,0,0],[0,-1,0],[0,0,1]]
initrots = dict.fromkeys(IMUs.keys())
for i in IMUs:
    IMU=[]
    with open((i + '.txt'),'r') as f:
        IMU=f.readlines()
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
    rot = line0[0:-1]
    quat = (float(rot[0][3:]),float(rot[1]),float(rot[2]),float(rot[3][:-4]))
    R = tf.transformations.quaternion_matrix(quat)
    Rinv = numpy.linalg.inv(R)
    initrots[i] = numpy.dot(Rinv, desiredrot)
    
for i in length:
    first = initstates[i][0]
    second = initstates[i][1]
    firstx = float(lis[first].split(',')[4])
    firsty = float(lis[first].split(',')[5])
    firstz = float(lis[first].split(',')[6])
    secondx = float(lis[second].split(',')[4])
    secondy = float(lis[second].split(',')[5])
    secondz = float(lis[second].split(',')[6])
    length[i] = math.sqrt((firstx - secondx) ** 2 + (firsty - secondy) ** 2 +(firstz - secondz) ** 2)
    