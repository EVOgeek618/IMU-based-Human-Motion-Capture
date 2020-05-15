import numpy,tf
def data(way1):
    lis = []
    f = open(way1, 'r')
    lis = f.readlines()
    f.close()

    lis0 = lis[0].split(', ')
    intime = float(lis0[7])
    IMUs = {
        'neck': ['neck_1', 'head_1'],
        'tors': ['torso_1', 'neck_1'],
        'larm': ['left_shoulder_1', 'left_elbow_1'],
        'lforearm': ['left_elbow_1', 'left_hand_1'],
        'rarm': ['right_shoulder_1', 'right_elbow_1'],
        'rforearm': ['right_elbow_1', 'right_hand_1'],
        'lthigh': ['left_hip_1', 'left_knee_1'],
        'lleg': ['left_knee_1', 'left_foot_1'],
        # 'lfoot':['left_foot_1'],
        'rthigh': ['right_hip_1', 'right_knee_1'],
        'rleg': ['right_knee_1', 'right_foot_1'],
        # 'rfoot':['right_foot_1']
        }

    # Readind data from sensors
    initstates = {}
    keys = []
    length = dict.fromkeys(IMUs.keys())
    for i in length:
        if len(length[i]) == 1:
            del length[i]

    desiredrot = [[-1, 0, 0], [0, -1, 0], [0, 0, -1]]
    initrots = dict.fromkeys(IMUs.keys())
    for i in IMUs:
        IMU = []
        with open((i + '.txt'), 'r') as f:
            IMU = f.readlines()
        for frame in IMUs[i]:
            k = 0
            while float(lis[k].split(',')[7]) < float(IMU[0].split(' ')[-1]):
                k = k + 1
            while lis[k].split(',')[8].split(' ')[1].strip('"') != frame:
                k = k - 1
            if abs(float(IMU[0].split(' ')[-1]) - float(lis[k + 15].split(',')[7])) < abs(
                    float(IMU[0].split(' ')[-1]) - float(lis[k].split(',')[7])):
                k = k + 15
            if frame not in keys:
                initstates.update({frame: k})
                keys = initstates.keys()
            elif (frame in keys) and (k < initstates[frame]):
                initstates[frame] = k
        line0 = IMU[0].split(' ')
        rot = line0[0:-1]
        quat = (float(rot[0][3:]), float(rot[1]), float(rot[2]), float(rot[3][:-4]))
        R = tf.transformations.quaternion_matrix(quat)
        Rinv = numpy.linalg.inv(R)
        initrots[i] = numpy.dot(Rinv, desiredrot)
    for frame in initstates:
        point = lis[initstates[frame]].split(',')
        initstates[frame] = [-float(point[4]), -float(point[5]), -float(point[6])]
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
    return {'IMUs':IMUs,'length':length, 'initrots':initrots, 'initstates':initstates}
