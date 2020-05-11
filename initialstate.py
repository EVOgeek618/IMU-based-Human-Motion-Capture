#Reading data from Kinect
lis=[]
f = open('DataKinect.txt', 'r')
lis=f.readlines()
f.close()

#Finding start time
lis0 = lis[0].split(', ')
intime = float(lis0[7])


IMUs = {
    'neck':['neck_1', 'head_1'],
    'tors':['torso_1', 'neck_1'],
    'larm':['left_shoulder_1', 'left_elbow_1'],
    'lforearm':['left_elbow_1', 'left_hand_1'],
    'rarm':['right_shoulder_1', 'right_elbow_1'],
    'rforearm':['right_elbow_1', 'right_hand_1'],
    'lthigh':['left_hip_1', 'left_knee_1'],
    'lleg':['left_knee_1', 'left_foot_1'],
    #'lfoot':['left_foot_1'],
    'rthigh':['right_hip_1', 'right_knee_1'],
    'rleg':['right_knee_1', 'right_foot_1'],
    #'rfoot':['right_foot_1']}
    
#Readind data from sensors
initstates = {}
keys = []
length = dict.fromkeys(IMUs.keys())
for i in length:
    if len(length[i]) == 1:
        del length[i]

desiredrot = [[-1,0,0],[0,-1,0],[0,0,-1]]
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
for frame in initstates:
    point = lis[initstates[frame]].split(',')
    initstates[frame] = [-float(pont[4]), -float(point[5]), -float(point[6])]
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

firstt = initstates['torso_1']
secondrh = initstates['right_shoulder_1']
secondlh = initstates['left_shoulder_1']
secondrl = initstates['right_hip_1']
secondll = initstates['left_hip_1']
length['rshoulder'] = [(firstt[0] - secondrh[0]), (firstt[1] - secondrh[1]), (firstt[2] - secondrh[2])]
length['lshoulder'] = [(firstt[0] - secondlh[0]), (firstt[1] - secondlh[1]), (firstt[2] - secondlh[2])]
length['rhip'] = [(firstt[0] - secondrl[0]), (firstt[1] - secondrl[1]), (firstt[2] - secondrl[2])]
length['lhip'] = [(firstt[0] - secondll[0]), (firstt[1] - secondll[1]), (firstt[2] - secondll[2])]

#Нужно экспортировать отсюда length, initrots, IMUs, initstates, и все, что ниже, закинуть в другой файл
IMUQuats = dict.fromkeys(IMUs.keys())
IMURots = dict.fromkeys(IMUs.keys())
IMUTimes = dict.fromkeys(IMUs.keys())
exptwists = dict.fromkeys(IMUs.keys())

T = {
    'neck_1':length['tors'],
    'head_1':(length['neck']+length['tors']),
    'right_shoulder_1':length['rshoulder'],
    'right_elbow_1':(length['rarm']+length['rshoulder']),
    'right_hand_1':(length['rforearm']+length['rarm']+length['rshoulder']),
    'left_shoulder_1':length['lshoulder'],
    'left_elbow_1':(length['larm']+length['lshoulder']),
    'left_hand_1':(length['lforearm']+length['larm']+length['lshoulder']),
    'right_hip_1':length['rhip'],
    'right_knee_1':(length['rhip']+length['rthigh']),
    'right_foot_1':(length['rhip']+length['rthigh']+length['rleg']),
    'left_hip_1':length['lhip'],
    'left_knee_1':(length['lhip']+length['lthigh']),
    'left_foot_1':(length['lhip']+length['lthigh']+length['lleg'])
}

M = dict.fromkeys(T.keys())
for i in T:
    P = numpy.identity(4);
    P[0][3] = T[i][0]
    P[1][3] = T[i][1]
    P[2][3] = T[i][2]
    M[i] = P
joints = {
    'neck_1':['tors', 'neck'],
    'head_1':['tors', 'neck'],
    'right_shoulder_1':['tors', 'rarm'],
    'right_elbow_1':['tors', 'rarm', 'rforearm'],
    'right_hand_1':['tors', 'rarm', 'rforearm'],
    'left_shoulder_1':['tors', 'larm'],
    'left_elbow_1':['tors', 'larm', 'lforearm'],
    'left_hand_1':['tors', 'larm', 'lforearm'],
    'right_hip_1':['tors', 'rthigh'],
    'right_knee_1':['tors', 'rthigh', 'rleg'],
    'right_foot_1':['tors', 'rthigh', 'rleg'],
    'left_hip_1':['tors', 'lthigh'],
    'left_knee_1':['tors', 'lthigh', 'lleg'],
    'left_foot_1':['tors', 'lthigh', 'lleg']
}
kinematics = dict.fromkeys(joints.keys())
for i in IMUs:
    IMU=[]
    with open((i + '.txt'),'r') as f:
        IMU=f.readlines()
    IMUQuats[i] = []
    IMURots[i] = []
    IMUTimes[i] = []
    for j in range(len(IMU)):
        line = IMU[j].split(' ')
        rot = line[0:-1]
        #Finding quaternion and rotation matrix
        quat = (float(rot[0][3:]),float(rot[1]),float(rot[2]),float(rot[3][:-4]))
        RIMU = tf.transformations.quaternion_matrix(quat)
        R = numpy.dot(RIMU, initrots[i])
        #Time
        time = float(line[-1]) - intime
        t = rospy.Time.from_sec(time)
        IMUQuats[i].append(quat) 
        IMURots[i].append(R)
        IMUTimes[i].append(t)
    
    exptwists[i] = []

    for j in range(len(IMURots[i]) - 1):
        q1 = tf.transformations.quaternion_from_matrix(IMURots[i][j])
        q2 = tf.transformations.quaternion_from_matrix(IMURots[i][j+1])
        q = q2*tf.transformations.quaternion_conjugate(q1)
        len = math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2])
        angle = 2*math.atan2(len,q[3])
        if len > 0:
            axis = (q[0], q[1], q[2])/len
        else:
            axis = (1,0,0)
        w = tf.transformations.unit_vector(axis*angle/(IMUTimes[i][j+1]-IMUTimes[i][j]))
        r = initstates[IMUs[i][0]]
        v = numpy.cross(-w, r)
        wwedge = numpy.array([0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0])
        twist = numpy.array([wwedge[0][0], wwedge[0][1], wwedge[0][2], v[0]], [wwedge[1][0], wwedge[1][1], wwedge[1][2], v[1]], [wwedge[2][0], wwedge[2][1], wwedge[2][2], v[2]], [0, 0, 0, 0])
        expw = numpy.identity(4) + wwedge * sin(angle) + numpy.dot(wwedge, wwedge) * (1 - cos(angle))
        exptwist = numpy.dot(expw, numpy.dot((numpy.identity(4)-expw),(numpy.dot(wwedge,v)+numpy.dot(numpy.dot(w,w.transpose()),v)*angle)))
        exptwists.append(exptwist)
tor = len(exptwisits['tors']) 

for joint in kinematics:
    kinematics[joint] = []
    for i in range(tor):
        trans = numpy.identity(4)
        for j in joints[joint]:
            trans = numpy.dot(trans, exptwists[j][i])
        kinematics[joint][i] = numpy.dot(trans, M[joint])
