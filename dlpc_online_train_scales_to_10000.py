import numpy as np
import time
import matplotlib.pyplot as plt
import copy
import numpy as np
E = {}
U = {}
X = {}
s=0
def draw_fig2Scale(ANN, NIError, State, R_State1, Obstacle, k):
    global ConHor_len, state_bound, Iterations_num, num, DesiredF, State0, E, U, X
    if k == 0:

        for i in range(1, num + 1):
            E[i] = np.zeros((4, Iterations_num))
            U[i] = np.zeros((2, Iterations_num))
            X[i] = np.zeros((4, Iterations_num))

        PresentState = State0
        R_State1 = np.array([[0], [-1], [0], [1]])
        draw_fig2Scale.PE = {}
        for i in range(1, num + 1):
            draw_fig2Scale.PE[i] = copy.deepcopy(NIError[i])

        draw_fig2Scale.Present_x1 = np.array([[0], [-1], [0], [1]])

    V = np.diag([1, 1, 0, 0])
    V1 = np.diag([0, 0, 1, 1])
    virtual = np.array([[0], [0], [0], [0]])

    timesteps = 0

    PresentError = copy.deepcopy(NIError)
    PresentState = copy.deepcopy(State)

    state_bound = np.array([[2], [2], [2], [2]])
    Umax = np.array([[1], [1]])
    Umin = np.array([[-1], [-1]])

    while timesteps < ConHor_len:
        c_input = {}
        for i in range(1, num + 1):
            c_input[i] = PresentError[i] / state_bound

        ANN[1] = NNProcess1(ANN[1], np.concatenate((c_input[1], c_input[2], c_input[num])))

        for i in range(2, num):
            ANN[i] = NNProcess1(ANN[i], np.concatenate((c_input[i], c_input[i - 1], c_input[i + 1])))

        ANN[num] = NNProcess1(ANN[num], np.concatenate((c_input[num], c_input[1], c_input[num - 1])))

        for i in range(1, num + 1):
            u[i] = copy.deepcopy(ANN[i]['NetworkOut'])
            FutureState[i] = robot(PresentState[i], u[i])
            Thita[i] = FutureState[i][2, 0]

            T[i] = np.array([[np.cos(Thita[i]), np.sin(Thita[i]), 0, 0],
                             [-np.sin(Thita[i]), np.cos(Thita[i]), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])

        Future_x1 = desired_position(draw_fig2Scale.Present_x1)
        FE = {}
        FE[1] = np.dot(T[1], (Future_x1 - FutureState[1])) + np.dot(T[1], (
                FutureState[num] - FutureState[1] + (-DesiredF[:, 2 * (num - 1) - 1]).reshape(-1, 1)))

        for i in range(2, num):
            FE[i] = np.dot(T[i],
                           np.dot(V, (FutureState[i - 1] - FutureState[i])) + DesiredF[:, 2 * (i - 1) - 2].reshape(-1,
                                                                                                                   1)) + np.dot(
                T[i], np.dot(V, (FutureState[i + 1] - FutureState[i])) + DesiredF[:, 2 * (i - 1) - 1].reshape(-1,
                                                                                                              1)) + np.dot(
                V1, (
                        R_State1 - FutureState[i]))

        FE[num] = np.dot(T[num], (
                FutureState[num - 1] - FutureState[num] + DesiredF[:, 2 * (num - 2) - 2].reshape(-1, 1))) + np.dot(
            T[num], (FutureState[1] - FutureState[num] + DesiredF[:, 2 * (num - 1) - 1].reshape(-1, 1))) + np.dot(V1, (
                R_State1 - FutureState[num]))

        for i in range(1, num + 1):
            FutureError[i] = FE[i]
            E[i][:, k + timesteps ] = draw_fig2Scale.PE[i][:, 0]
            X[i][:, k + timesteps ] = PresentState[i][:, 0]
            U[i][:, k + timesteps ] = u[i][:, 0]
            PresentError[i] = copy.deepcopy(FutureError[i])
            PresentState[i] = copy.deepcopy(FutureState[i])
            draw_fig2Scale.PE[i] = copy.deepcopy(FE[i])

        draw_fig2Scale.Present_x1 = copy.deepcopy(Future_x1)
        timesteps += 1

    for i in range(1, num + 1):
        NIError[i] = copy.deepcopy(draw_fig2Scale.PE[i])
        State[i] = copy.deepcopy(PresentState[i])

    R_State1 = copy.deepcopy(draw_fig2Scale.Present_x1)
    return NIError, State, R_State1


def NNTrain1(NN, NNError):
    Delta2 = copy.deepcopy(NNError)
    dB1 = copy.deepcopy(Delta2)
    dW1 = np.dot(Delta2, np.tanh(NN['NetworkIn']).T)
    NN['W1'] = NN['W1'] - NN['LR'] * dW1
    NN['B1'] = NN['B1'] - NN['LR'] * dB1
    return NN


def CreateANN1(x_dim, u_dim):
    ANN = {}
    ANN['HiddenUnitNum'] = 5
    ANN['InDim'] = x_dim
    ANN['OutDim'] = u_dim
    ANN['LR'] = 0.2
    ANN['W1'] = 1 * np.random.rand(ANN['OutDim'], ANN['InDim']) - 0.5  
    ANN['B1'] = 1 * np.random.rand(ANN['OutDim'], 1) - 0.5
    return ANN


def CreateCNN1(x_dim, y_dim):
    CNN = {}
    CNN['HiddenUnitNum'] = 5
    CNN['InDim'] = x_dim
    CNN['OutDim'] = y_dim
    CNN['LR'] = 0.4
    CNN['W1'] = 1 * np.random.rand(CNN['OutDim'], CNN['InDim']) - 0.5
    CNN['B1'] = 1 * np.random.rand(CNN['OutDim'], 1) - 0.5
    return CNN


def NNProcess1(NN, input):
    NN['NetworkIn'] = copy.deepcopy(input)
    NN['NetworkOut'] = np.dot(NN['W1'], np.tanh(NN['NetworkIn'])) + NN['B1']
    return NN


def desired_position(x):
    global tau
    vr = 1
    wr = 0
    FutureState = np.zeros((4, 1))
    FutureState[0, 0] = x[0, 0] + tau * vr * np.cos(x[2, 0])
    FutureState[1, 0] = x[1, 0] + tau * vr * np.sin(x[2, 0])
    FutureState[2, 0] = x[2, 0] + tau * wr
    FutureState[3, 0] = vr
    if FutureState[2, 0] > np.pi:
        FutureState[2, 0] = FutureState[2, 0] - 2 * np.pi
    elif FutureState[2, 0] < -np.pi:
        FutureState[2, 0] = FutureState[2, 0] + 2 * np.pi
    Future_x = np.array([FutureState[0, 0], FutureState[1, 0], FutureState[2, 0], FutureState[3, 0]]).reshape(-1, 1)
    return Future_x


def robot(x, u):
    global tau

    v = x[3, 0] + tau * u[0, 0]
    FutureState = np.zeros((4, 1))
    FutureState[0, 0] = x[0, 0] + tau * v * np.cos(x[2, 0])
    FutureState[1, 0] = x[1, 0] + tau * v * np.sin(x[2, 0])
    FutureState[2, 0] = x[2, 0] + tau * u[1, 0]
    FutureState[3, 0] = v
    if FutureState[2, 0] > np.pi:
        FutureState[2, 0] = FutureState[2, 0] - 2 * np.pi
    elif FutureState[2, 0] < -np.pi:
        FutureState[2, 0] = FutureState[2, 0] + 2 * np.pi
    return FutureState


def sys_process_four(ep, u, PresentState1, PresentState2, PresentState4):
    global tau

    ep1 = ep[0, 0]
    ep2 = ep[1, 0]
    ep3 = ep[2, 0]
    ep4 = ep[3, 0]
    u1 = u[0, 0]
    u2 = u[1, 0]
    wr = 0
    vr = 1
    v4 = PresentState4[3, 0]
    v2 = PresentState2[3, 0]
    thita2 = PresentState2[2, 0]
    thita1 = PresentState1[2, 0]
    thita4 = PresentState4[2, 0]
    MSJ1 = np.array([[1, tau * u2,
                      -tau * np.sin(ep3) * vr - tau * np.sin(thita2 - thita1) * v2 - tau * np.sin(thita4 - thita1) * v4,
                      2 * tau],
                     [-tau * u2, 1,
                      tau * np.cos(ep3) * vr + tau * np.cos(thita2 - thita1) * v2 + tau * np.cos(thita4 - thita1) * v4,
                      0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    MCJ1 = np.array([[0, ep2 * tau],
                     [0, -ep1 * tau],
                     [0, -tau],
                     [-tau, 0]])

    MSJ12 = np.array([[0, 0, tau * np.sin(thita2 - thita1) * v2, -tau * np.cos(thita2 - thita1)],
                      [0, 0, -tau * np.cos(thita2 - thita1) * v2, -tau * np.sin(thita2 - thita1)],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])

    MSJ14 = np.array([[0, 0, tau * np.sin(thita4 - thita1) * v4, -tau * np.cos(thita4 - thita1)],
                      [0, 0, -tau * np.cos(thita4 - thita1) * v4, -tau * np.sin(thita4 - thita1)],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])

    return MSJ1, MCJ1, MSJ12, MSJ14


tau = 0.05
Gamma = 0.95
state_bound = np.array([[5], [5], [5], [5]])
state_bound1 = np.array([[2], [2], [2], [2]])
Iterations_num = 20  # 180
MaxTrials = 30
ConHor_len = 1
PreHor_len = 10
num = 7500 #robot scales
cost = np.zeros((num, Iterations_num))
Jsum = np.zeros((num, Iterations_num + 1))

# Initialization
ANN = {}
CNN = {}

R_State1 = np.array([[0], [-1], [0], [1]])

for i in range(1, num + 1):
    ANN[i] = CreateANN1(12, 2)
    CNN[i] = CreateCNN1(12, 12)

State = {}
Thita = {}
T = {}

for i in range(1, num + 1):
    if i >= 1 and i <= 0.5 * num:
        State[i] = np.array([[-2 * (i - 1) + 1 * np.random.rand(1)[0]], [-1], [0], [1]])
        # State[i] = np.array([[-2 * (i - 1) + 0.5], [-1], [0], [1]])
    else:
        State[i] = np.array([[2 * (i - num) + 1 * np.random.rand(1)[0]], [1], [0], [1]])
        # State[i] = np.array([[2 * (i - num) + 0.5], [1], [0], [1]])
    Thita[i] = State[i][2, 0]
    T[i] = np.array([[np.cos(Thita[i]), np.sin(Thita[i]), 0, 0],
                     [-np.sin(Thita[i]), np.cos(Thita[i]), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

V = np.diag([1, 1, 0, 0])
V1 = np.diag([0, 0, 1, 1])

DF = np.hstack((np.array([[-2], [0], [0], [0]]), np.zeros((4, 2 * (num - 1) - 1))))
DesiredF = np.array([[-2], [0], [0], [0]])
for i in range(1, 2 * (num - 1)):
    if i < num - 3 or (i > num - 3 and i != num - 1 and i < 2 * (num - 1) - 1):
        DF[:, i] = -DF[:, i - 1]
    elif i == num - 3 or i == 2 * (num - 1) - 1:
        DF[:, i] = np.hstack(([[DF[1, i - 1]], [DF[0, i - 1]], [0], [0]]))
    elif i == num - 1:
        DF[:, i] = np.hstack((-1 * np.array([[DF[1, i - 1]], [DF[0, i - 1]], [0], [0]])))
    DesiredF = np.hstack((DesiredF, DF[:, i].reshape(-1, 1)))

NIError = {}
NIError[1] = T[1].dot(R_State1 - State[1]) + T[1].dot(
    State[num] - State[1] + (-DesiredF[:, 2 * (num - 1) - 1]).reshape(-1, 1))

for i in range(2, num):
    NIError[i] = T[i].dot(V.dot(State[i - 1] - State[i]) + DesiredF[:, 2 * (i - 1) - 2].reshape(-1, 1)) + T[i].dot(
        V.dot(State[i + 1] - State[i]) + DesiredF[:, 2 * (i - 1) - 1].reshape(-1, 1)) + V1.dot(R_State1 - State[i])

NIError[num] = T[num].dot(State[num - 1] - State[num] + DesiredF[:, 2 * (num - 1) - 2].reshape(-1, 1)) + T[num].dot(
    State[1] - State[num] + DesiredF[:, 2 * (num - 1) - 1].reshape(-1, 1)) + V1.dot(R_State1 - State[num])

scale = 0.01
mu = 0.001
Q = 1 * np.eye(12) * scale
R = 0.5 * np.eye(2) * scale
J = []
virtual = np.array([[0], [0], [0], [0]])
Umax = np.array([[1], [1]])
Umin = np.array([[-1], [-1]])
Obstacle = np.array([[30], [1]])
OO = np.zeros((4, 4))

P1 = np.array([[148.133190596417, 2.02583848679917, 3.51323846649974, 46.7730691661438],
               [2.02583848679917, 148.133191490445, 46.7730693299270, 3.51323630780372],
               [3.51323846649974, 46.7730693299270, 92.6842566525951, 8.80444258168160],
               [46.7730691661438, 3.51323630780372, 8.80444258168160, 92.6842564186466]])

P2 = np.array([[148.133190600772, 2.02583848790635, 3.51323846727139, 46.7730691676650],
               [2.02583848790635, 148.133191490058, 46.7730693308098, 3.51323630784338],
               [3.51323846727139, 46.7730693308098, 92.6842566511360, 8.80444258118404],
               [46.7730691676650, 3.51323630784338, 8.80444258118404, 92.6842564014894]])

P3 = np.array([[658.124851396075, 11.5177222865393, 18.0575925561011, 244.738183532405],
               [11.5177222865393, 658.124847239358, 244.738181009361, 18.0575821660109],
               [18.0575925561011, 244.738181009361, 213.345125351539, 21.9218553649452],
               [244.738183532405, 18.0575821660109, 21.9218553649452, 213.345126399689]])

P4 = np.array([[888.158317116642, 14.8674148692541, 21.2150995166181, 285.920616563882],
               [14.8674148692541, 888.158303683769, 285.920611242892, 21.2150929470356],
               [21.2150995166181, 285.920611242892, 214.761632207292, 22.3319105649926],
               [285.920616563882, 21.2150929470356, 22.3319105649926, 214.761634284998]])

P5 = np.array([[888.158317389190, 14.8674148497524, 21.2150995112752, 285.920616599051],
               [14.8674148497524, 888.158303732463, 285.920611276001, 21.2150929405337],
               [21.2150995112752, 285.920611276001, 214.761632233751, 22.3319105640493],
               [285.920616599051, 21.2150929405337, 22.3319105640493, 214.761634279374]])

P6 = np.array([[658.124851366556, 11.5177222796616, 18.0575925467624, 244.738183446075],
               [11.5177222796616, 658.124847194266, 244.738180976648, 18.0575821612168],
               [18.0575925467624, 244.738180976648, 213.345125331307, 21.9218553602577],
               [244.738183446075, 18.0575821612168, 21.9218553602577, 213.345126344712]])

P = {}
P[1] = scale * P1
P[2] = scale * P2
P[3] = scale * P3
P[4] = scale * P4
P[5] = scale * P5
P[6] = scale * P6

if num > 6:
    for i in range(6, num + 1):
        P[i] = scale * P6

# Remaining code is the same in Python

run = 1
R_State10 = copy.deepcopy(R_State1)
State0 = copy.deepcopy(State)

NIError0 = copy.deepcopy(NIError)

J = np.zeros((run, Iterations_num))
tic = time.time()

# Main loop
for iter in range(run):
    print(f"run={iter}")
    R_State1 = copy.deepcopy(R_State10)

    NIError = copy.deepcopy(NIError0)

    State = copy.deepcopy(State0)
    for k in range(0, Iterations_num, ConHor_len):
        print(f"time step={k}")
        Present_rx1 = copy.deepcopy(R_State1)

        RealError = copy.deepcopy(NIError)
        RealState = copy.deepcopy(State)
        f = 1
        # Determine whether the learning is successful in the current prediction time domain
        while f >= 1:
            Present_x1 = copy.deepcopy(Present_rx1)
            PresentError = copy.deepcopy(RealError)
            PresentState = copy.deepcopy(RealState)
            Err1 = 0
            Err2 = 0
            timesteps = 0
            j = 0
            c_input = {}
            u = {}
            FutureState = {}
            MJ = {}
            FutureError = {}
            f_input = {}
            dR_dZ = {}
            FutureLambda = {}
            PresentLambda = {}
            ANNError = {}
            CNNTarget = {}

            CNNError = {}

            while max(abs(PresentError[1]) - state_bound) <= 0 and timesteps < PreHor_len:

                for i in range(1, num + 1):
                    c_input[i] = PresentError[i] / state_bound1

                ANN[1] = NNProcess1(ANN[1], np.concatenate(
                    (c_input[1].reshape(-1, 1), c_input[2].reshape(-1, 1), c_input[num].reshape(-1, 1))))
                if num > 2:
                    for i in range(2, num):
                        ANN[i] = NNProcess1(ANN[i], np.concatenate(
                            (c_input[i].reshape(-1, 1), c_input[i - 1].reshape(-1, 1), c_input[i + 1].reshape(-1, 1))))

                ANN[num] = NNProcess1(ANN[num],
                                      np.concatenate((c_input[num].reshape(-1, 1), c_input[1].reshape(-1, 1),
                                                      c_input[num - 1].reshape(-1, 1))))


                for i in range(1, num + 1):
                    u[i] = copy.deepcopy(ANN[i]['NetworkOut'])
                    FutureState[i] = robot(PresentState[i], u[i])

                Future_x1 = desired_position(Present_x1)


                MJ[1, 1], MJ[1, 2], MJ[1, 3], MJ[1, 4] = sys_process_four(PresentError[1], u[1], PresentState[1],
                                                                          PresentState[num], PresentState[2])
                if num > 2:
                    for i in range(2, num):
                        MJ[i, 1], MJ[i, 2], MJ[i, 3], MJ[i, 4] = sys_process_four(PresentError[i], u[i],
                                                                                  PresentState[i], PresentState[i - 1],
                                                                                  PresentState[i + 1])

                MJ[num, 1], MJ[num, 2], MJ[num, 3], MJ[num, 4] = sys_process_four(PresentError[num],
                                                                                  u[num],
                                                                                  PresentState[num],
                                                                                  PresentState[num - 1],
                                                                                  PresentState[1])

                for i in range(1, num + 1):
                    Thita[i] = FutureState[i][2, 0]
                    T[i] = np.array([[np.cos(Thita[i]), np.sin(Thita[i]), 0, 0],
                                     [-np.sin(Thita[i]), np.cos(Thita[i]), 0, 0],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])


                FutureError[1] = np.dot(T[1], (Future_x1 - FutureState[1])) + np.dot(T[1], (
                        FutureState[num] - FutureState[1]) + (-DesiredF[:, 2 * (num - 1) - 1]).reshape(-1, 1))

                if num > 2:
                    for i in range(2, num):
                        FutureError[i] = np.dot(T[i], (
                                V @ (FutureState[i - 1] - FutureState[i]) + DesiredF[:, 2 * (i - 1) - 2].reshape(-1,
                                                                                                                 1))) + np.dot(
                            T[i], (V @ (FutureState[i + 1] - FutureState[i]) + DesiredF[:, 2 * (i - 1) - 1].reshape(-1,
                                                                                                                    1))) + V1 @ (
                                                 R_State1 - FutureState[i])

                FutureError[num] = np.dot(T[num], (
                        FutureState[num - 1] - FutureState[num] + DesiredF[:, 2 * (num - 1) - 2].reshape(-1,
                                                                                                         1))) + np.dot(
                    T[num],
                    (FutureState[1] - FutureState[num] + DesiredF[:, 2 * (num - 1) - 1].reshape(-1, 1))) + V1 @ (
                                           R_State1 - FutureState[num])


                for i in range(1, num + 1):
                    f_input[i] = FutureError[i] / state_bound1



                dR_dZ[1] = 2 * Q @ np.concatenate(
                    (PresentError[1].reshape(-1, 1), PresentError[2].reshape(-1, 1), PresentError[num].reshape(-1, 1)))

                if num > 2:
                    for i in range(2, num):
                        dR_dZ[i] = 2 * Q @ np.concatenate((PresentError[i].reshape(-1, 1),
                                                           PresentError[i - 1].reshape(-1, 1),
                                                           PresentError[i + 1].reshape(-1, 1)))

                dR_dZ[num] = 2 * Q @ np.concatenate((PresentError[num].reshape(-1, 1), PresentError[1].reshape(-1, 1),
                                                     PresentError[num - 1].reshape(-1, 1)))

                CNN[1] = NNProcess1(CNN[1], np.concatenate(
                    (f_input[1].reshape(-1, 1), f_input[2].reshape(-1, 1), f_input[num].reshape(-1, 1))))
                if num > 2:
                    for i in range(2, num):
                        CNN[i] = NNProcess1(CNN[i], np.concatenate(
                            (f_input[i].reshape(-1, 1), f_input[i - 1].reshape(-1, 1), f_input[i + 1].reshape(-1, 1))))

                CNN[num] = NNProcess1(CNN[num],
                                      np.concatenate((f_input[num].reshape(-1, 1), f_input[1].reshape(-1, 1),
                                                      f_input[num - 1].reshape(-1, 1))))

                if timesteps < PreHor_len - 1:
                    for i in range(1, num + 1):
                        FutureLambda[i] = copy.deepcopy(CNN[i]['NetworkOut'])
                else:

                    for i in range(1, num + 1):
                        FutureLambda[i] = 2 * np.dot(np.concatenate(
                            (np.hstack((P[i], OO, OO)), np.hstack((OO, OO, OO)), np.hstack((OO, OO, OO)))),
                            np.concatenate(
                                (FutureError[i], [[0], [0], [0], [0]], [[0], [0], [0], [0]])))

                CNN[1] = NNProcess1(CNN[1], np.concatenate((c_input[1], c_input[2], c_input[num])))
                if num > 2:
                    for i in range(2, num):
                        CNN[i] = NNProcess1(CNN[i], np.concatenate((c_input[i], c_input[i - 1], c_input[i + 1])))

                CNN[num] = NNProcess1(CNN[num], np.concatenate((c_input[num], c_input[1], c_input[num - 1])))


                for i in range(1, num + 1):
                    PresentLambda[i] = copy.deepcopy(CNN[i]['NetworkOut'])

                # Update ANN and CNN

                ANNError[1] = 2 * (R) @ u[1] + Gamma * (np.dot(MJ[1, 2].T, FutureLambda[1][0:4]) + np.dot(MJ[1, 2].T,
                                                                                                          FutureLambda[
                                                                                                              2][
                                                                                                          4:8]) + np.dot(
                    MJ[1, 2].T, FutureLambda[num][4:8]))
                # ANNError[1]=2*(R)*u[1]+mu*dB_u[1]+Gamma*(MJ[1,2]'*FutureLambda[1](1:4)+MJ[1,2]'*FutureLambda[2](5:8))
                if num > 2:
                    for i in range(2, num):
                        ANNError[i] = 2 * (R) @ u[i] + Gamma * (
                                np.dot(MJ[i, 2].T, FutureLambda[i][0:4]) + np.dot(MJ[i, 2].T, FutureLambda[i-1][
                                                                                              8:12]) + np.dot(
                            MJ[i, 2].T, FutureLambda[i + 1][4:8]))

                ANNError[num] = 2 * R @ u[num] + Gamma * (
                        np.dot(MJ[num, 2].T, FutureLambda[num][0:4]) + np.dot(MJ[num, 2].T,
                                                                              FutureLambda[1][
                                                                              8:12]) + np.dot(
                    MJ[num, 2].T, FutureLambda[num - 1][8:12]))

                for i in range(1, num + 1):
                    ANN[i] = NNTrain1(ANN[i], ANNError[i])

                Z0 = np.zeros((4, 4))
                CNNTarget[1] = dR_dZ[1] + Gamma * np.dot(np.concatenate((np.hstack(
                    (MJ[1, 1].T, MJ[2, 3].T, MJ[num, 3].T)), np.hstack((MJ[1, 3].T, MJ[2, 1].T, Z0)),
                                                                         np.hstack((MJ[1, 4].T, Z0.T, MJ[num, 1].T)))),
                    np.concatenate((FutureLambda[1][0:4], FutureLambda[2][4:8],
                                    FutureLambda[num][4:8])))
                # CNNTarget[1] = dR_dZ[1] + Gamma * np.dot(np.concatenate((MJ[1][0].T, MJ[0][3].T, MJ[2][3].T)), np.concatenate((FutureLambda[1][0:4], FutureLambda[0][8:12], FutureLambda[2][4:8])))
                if num > 2:
                    for i in range(2, num):
                        CNNTarget[i] = dR_dZ[i] + Gamma * np.dot(np.concatenate((np.hstack(
                            (MJ[i, 1].T, MJ[i - 1, 4].T, MJ[i + 1, 3].T)), np.hstack(
                            (MJ[i, 3].T, MJ[i - 1, 1].T, Z0.T)), np.hstack((MJ[i, 4].T, Z0.T, MJ[i + 1, 1].T)))),
                            np.concatenate((FutureLambda[i][0:4],
                                            FutureLambda[i - 1][8:12],
                                            FutureLambda[i + 1][4:8])))

                i = num
                CNNTarget[num] = dR_dZ[num] + Gamma * np.dot(np.concatenate((np.hstack(
                    (MJ[num, 1].T, MJ[1, 4].T, MJ[num - 1, 4].T)), np.hstack((MJ[num, 3].T, MJ[1, 1].T, Z0.T)),
                                                                             np.hstack((MJ[num, 4].T, Z0,
                                                                                        MJ[num - 1, 1].T)))),
                    np.concatenate((
                        FutureLambda[i][0:4], FutureLambda[1][8:12],
                        FutureLambda[i - 1][8:12])))

                for i in range(1, num + 1):
                    CNNError[i] = PresentLambda[i] - CNNTarget[i]
                    CNN[i] = NNTrain1(CNN[i], CNNError[i])
                    PresentState[i] = copy.deepcopy(FutureState[i])
                    PresentError[i] = copy.deepcopy(FutureError[i])

                Present_x1 = copy.deepcopy(Future_x1)
                timesteps = timesteps + 1
                if timesteps == 2:
                    j = j + np.dot(np.concatenate((PresentError[1], PresentError[2], PresentError[num])).T, np.dot(Q,
                                                                                                                   np.concatenate(
                                                                                                                       (
                                                                                                                           PresentError[
                                                                                                                               1],
                                                                                                                           PresentError[
                                                                                                                               2],
                                                                                                                           PresentError[
                                                                                                                               num])))) + np.dot(
                        u[1].T, np.dot(R, u[1]))
                    if num > 2:
                        for i in range(2, num):
                            j = j + np.dot(
                                np.concatenate((PresentError[i], PresentError[i - 1], PresentError[i + 1])).T,
                                np.dot(Q, np.concatenate(
                                    (PresentError[i], PresentError[i - 1], PresentError[i + 1])))) + np.dot(
                                u[i].T, np.dot(R, u[i]))
                    j = j + np.dot(np.concatenate((PresentError[num], PresentError[1], PresentError[num - 1])).T,
                                   np.dot(Q, np.concatenate(
                                       (PresentError[num], PresentError[1], PresentError[num - 1])))) + np.dot(
                        u[num].T, np.dot(R, u[num]))

            #J[iter, k] = j[0, 0] / scale


            # Determine whether learning is successful
            # print(ANN[1], '\n', NIError[1], '\n', State[1], '\n', R_State1, '\n', Obstacle, '\n', k, '\n')
            if timesteps == PreHor_len:
                NIError, State, R_State1 = draw_fig2Scale(ANN, NIError, State, R_State1, Obstacle, k)
                # print(NIError[1], '\n', State[1], '\n', R_State1, '\n')
                f = 0
            else:
                f += 1

            # If learning fails for MaxTrials times, reinitialize the network
            if f > MaxTrials:
                ANN_u = {}
                ANN_x = {}
                CNN_x = {}
                for i in range(1, num + 1):
                    ANN[i] = CreateANN1(12, 2)
                    # ANN_u[i] = CreateANN_x(2, 2)
                    # ANN_x[i] = CreateANN_x(2, 2)
                    CNN[i] = CreateCNN1(12, 12)
                    # CNN_x[0] = CreateCNN_x(2, 2)
                f = 1

toc = time.time()
print("training time\n", toc - tic, "\n")

# plot：
# for timesteps in range(Iterations_num):
#     print(np.concatenate((E[1][:, timesteps].reshape(-1,1), E[num][:, timesteps].reshape(-1,1))).shape)
#     cost[0, timesteps] = np.transpose(np.concatenate((E[1][:, timesteps].reshape(-1,1), E[num][:, timesteps].reshape(-1,1)))) @ np.concatenate((E[1][:, timesteps].reshape(-1,1), E[num][:, timesteps].reshape(-1,1))) + 0.5 * U[1][:, timesteps].reshape(-1,1).T @ U[1][:,timesteps].reshape(-1,1).T
#     Jsum[0, timesteps + 1] = Jsum[0, timesteps] + cost[0, timesteps]
#
#     if num > 2:
#         for i in range(1, num - 1):
#             cost[i, timesteps] = np.transpose(
#                 np.concatenate((E[i][:, timesteps].reshape(-1,1), E[i + 1][:, timesteps].reshape(-1,1), E[i + 2][:, timesteps].reshape(-1,1)))) @ np.concatenate((E[i][:, timesteps].reshape(-1,1), E[i+1][:, timesteps].reshape(-1,1), E[i + 2][:, timesteps].reshape(-1,1))) + 0.5 * U[i + 1][:,timesteps].reshape(-1,1).T @ U[i + 1][:, timesteps].reshape(-1,1)
#             Jsum[i, timesteps + 1] = Jsum[i, timesteps] + cost[i, timesteps]
#
#     cost[num-1, timesteps] = np.transpose(
#         np.concatenate((E[num-1][:, timesteps].reshape(-1,1), E[num][:, timesteps].reshape(-1,1), E[1][:, timesteps].reshape(-1,1)))) @ np.concatenate((E[num-1][:, timesteps].reshape(-1,1), E[num][:, timesteps].reshape(-1,1), E[1][:, timesteps].reshape(-1,1))) + 0.5*U[num][:,timesteps].reshape(-1,1).T @ U[num][:, timesteps].reshape(-1,1)
#     Jsum[num-1, timesteps + 1] = Jsum[num-1, timesteps] + cost[num-1, timesteps]
#     timesteps += 1
#
# # Plotting the results
# plt.figure()
# plt.plot(J[run-1, :])
# plt.title('Cost')
#
# plt.figure()
# for i in range(num):
#     plt.plot(np.arange(tau, tau * (Iterations_num + 1), tau), cost[i, :], linewidth=1.5)
#     plt.hold(True)
# plt.xlabel('Time (s)')
# plt.ylabel('$r_i$', interpreter='latex')
# plt.title('Four robots')
# plt.grid(True)
# plt.show()
#
# plt.figure()
# for i in range(num):
#     plt.plot(np.arange(tau, tau * (Iterations_num + 1), tau), Jsum[i, :Iterations_num], linewidth=1.5)
#     plt.hold(True)
# plt.xlabel('Time (s)')
# plt.ylabel('$r_i$', interpreter='latex')
# plt.title('Two hundred robots')
# plt.grid(True)
# plt.show()
