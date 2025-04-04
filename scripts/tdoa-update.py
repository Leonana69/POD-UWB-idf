from math import isnan, sqrt
import math
import numpy as np

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Tdoa:
    def __init__(self, tdoa):
        self.distanceDiff = tdoa
        self.stdDev = 0.15
        self.anchorIdA = 0
        self.anchorIdB = 1
        self.anchorPositionA = Point(-2, 0, 0)
        self.anchorPositionB = Point(2, 0, 0)

stdDevInitialPosition_xy = 100
stdDevInitialPosition_z = 1
stdDevInitialVelocity = 0.01
stdDevInitialAttitude_rollpitch = 0.01
stdDevInitialAttitude_yaw = 0.01
procNoiseAcc_xy = 0.5
procNoiseAcc_z = 1.0
procNoiseVel = 0
procNoisePos = 0
procNoiseAtt = 0
measNoiseBaro = 2.0           
measNoiseGyro_rollpitch = 0.1
measNoiseGyro_yaw = 0.1

KC_STATE_X = 0
KC_STATE_Y = 1
KC_STATE_Z = 2
KC_STATE_PX = 3
KC_STATE_PY = 4
KC_STATE_PZ = 5
KC_STATE_D0 = 6
KC_STATE_D1 = 7
KC_STATE_D2 = 8
KC_STATE_DIM = 9

def cholesky_decomposition(matrix):
    n = matrix.shape[0]
    lower = np.zeros((n, n), dtype=np.float32)

    for i in range(n):
        for j in range(i + 1):
            sum_val = 0.0

            if j == i:
                for k in range(j):
                    sum_val += lower[j, k] ** 2
                lower[j, j] = np.sqrt(matrix[j, j] - sum_val)
            else:
                for k in range(j):
                    sum_val += lower[i, k] * lower[j, k]
                lower[i, j] = (matrix[i, j] - sum_val) / lower[j, j]

    return lower

def GM_UWB(e):
    sigma = 2.0
    GM_dn = sigma + e * e
    return (sigma * sigma) / (GM_dn * GM_dn)

def GM_state(e):
    sigma = 1.5
    GM_dn = sigma + e * e
    return (sigma * sigma) / (GM_dn * GM_dn)

class KalmanCore:
    def __init__(self):
        self.S = np.zeros((KC_STATE_DIM))
        self.S[KC_STATE_X] = 0
        self.S[KC_STATE_Y] = 0
        self.S[KC_STATE_Z] = 0
        self.q = np.zeros((4))
        self.q[0] = 1
        self.R = np.eye(3)
        self.P = np.zeros((KC_STATE_DIM, KC_STATE_DIM))
        self.P[KC_STATE_X, KC_STATE_X] = stdDevInitialPosition_xy**2
        self.P[KC_STATE_Y, KC_STATE_Y] = stdDevInitialPosition_xy**2
        self.P[KC_STATE_Z, KC_STATE_Z] = stdDevInitialPosition_z**2
        self.P[KC_STATE_PX, KC_STATE_PX] = stdDevInitialVelocity**2
        self.P[KC_STATE_PY, KC_STATE_PY] = stdDevInitialVelocity**2
        self.P[KC_STATE_PZ, KC_STATE_PZ] = stdDevInitialVelocity**2
        self.P[KC_STATE_D0, KC_STATE_D0] = stdDevInitialAttitude_rollpitch**2
        self.P[KC_STATE_D1, KC_STATE_D1] = stdDevInitialAttitude_rollpitch**2
        self.P[KC_STATE_D2, KC_STATE_D2] = stdDevInitialAttitude_yaw**2

        self.x_errm = np.zeros((KC_STATE_DIM, 1))

    def UpdateWithPKE(self, Hm, Km, P_w_m, error):
        for i in range(KC_STATE_DIM):
            self.S[i] += Km[i, 0] * error

        tmpNN1m = Km @ Hm
        tmpNN1m = tmpNN1m * -1 + np.eye(KC_STATE_DIM)
        Ppo = tmpNN1m @ P_w_m
        self.P = Ppo

        self.CapCovariance()

    def CapCovariance(self):
        for i in range(KC_STATE_DIM):
            for j in range(KC_STATE_DIM):
                p = 0.5 * (self.P[i, j] + self.P[j, i])
                if isnan(p) or p > 100:
                    self.P[i, j] = self.P[j, i] = 100
                elif i == j and p < 1e-6:
                    self.P[i, j] = self.P[j, i] = 1e-6
                else:
                    self.P[i, j] = self.P[j, i] = p

    def RobustUpdateWithTdoa(self, tdoa: Tdoa):
        x = self.S[KC_STATE_X]
        y = self.S[KC_STATE_Y]
        z = self.S[KC_STATE_Z]

        x0 = tdoa.anchorPositionA.x
        y0 = tdoa.anchorPositionA.y
        z0 = tdoa.anchorPositionA.z
        x1 = tdoa.anchorPositionB.x
        y1 = tdoa.anchorPositionB.y
        z1 = tdoa.anchorPositionB.z

        d0 = sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
        d1 = sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)

        if d0 != 0.0 and d1 != 0.0:
            predicted = d1 - d0
            measurement = tdoa.distanceDiff
            error_check = measurement - predicted

            P_iter = self.P.copy()
            R_iter = tdoa.stdDev**2
            X_state = self.S.copy()

            H = np.zeros((1, KC_STATE_DIM))

            for _ in range(2):
                Pc_m = cholesky_decomposition(P_iter)
                Pc_tran_m = Pc_m.T.copy()
                R_chol = sqrt(R_iter)

                x_iter = X_state[KC_STATE_X]
                y_iter = X_state[KC_STATE_Y]
                z_iter = X_state[KC_STATE_Z]

                dx0 = x_iter - x0
                dy0 = y_iter - y0
                dz0 = z_iter - z0

                dx1 = x_iter - x1
                dy1 = y_iter - y1
                dz1 = z_iter - z1

                d0 = sqrt(dx0**2 + dy0**2 + dz0**2)
                d1 = sqrt(dx1**2 + dy1**2 + dz1**2)

                predicted_iter = d1 - d0
                error_iter = measurement - predicted_iter
                e_y = error_iter

                if d0 != 0.0 and d1 != 0.0:
                    H[0, KC_STATE_X] = (dx1 / d1) - (dx0 / d0)
                    H[0, KC_STATE_Y] = (dy1 / d1) - (dy0 / d0)
                    H[0, KC_STATE_Z] = (dz1 / d1) - (dz0 / d0)

                    if abs(R_chol - 0.0) < 0.0001:
                        e_y = error_iter / 0.0001
                    else:
                        e_y = error_iter / R_chol

                    for col in range(KC_STATE_DIM):
                        for row in range(col, KC_STATE_DIM):
                            if isnan(Pc_m[row, col]) or Pc_m[row, col] > 100:
                                Pc_m[row, col] = 100
                            elif row != col and Pc_m[row, col] < -100:
                                Pc_m[row, col] = -100
                            elif row == col and Pc_m[row, col] < 0:
                                Pc_m[row, col] = 0

                    Pc_m += np.eye(KC_STATE_DIM) * 1e-9
                    tmp1 = Pc_m.copy()
                    Pc_inv_m = np.linalg.inv(tmp1)
                    e_x_m = Pc_inv_m @ self.x_errm

                    wx_invm = np.zeros((KC_STATE_DIM, KC_STATE_DIM))
                    for i in range(KC_STATE_DIM):
                        wx_invm[i, i] = 1.0 / GM_state(e_x_m[i, 0])

                    Pc_w_invm = Pc_m @ wx_invm
                    P_w_m = Pc_w_invm @ Pc_tran_m
                    R_w = 0.0
                    w_y = GM_UWB(e_y)
                    if abs(w_y - 0.0) < 0.0001:
                        R_w = (R_chol * R_chol) / 0.0001
                    else:
                        R_w = (R_chol * R_chol) / w_y

                    HTm = H.T.copy()

                    PHTm = P_w_m @ HTm

                    print("R_w", R_w)
                    HPHR = R_w + H @ PHTm
                    print("HPHR", HPHR)

                    Kwm = np.zeros((KC_STATE_DIM, 1))
                    for i in range(KC_STATE_DIM):
                        Kwm[i, 0] = PHTm[i, 0] / HPHR[0, 0]
                        self.x_errm[i, 0] = Kwm[i, 0] * error_check
                        X_state[i] = self.S[i] + self.x_errm[i, 0]
                        
                    P_iter = P_w_m.copy()
                    R_iter = R_w

            self.UpdateWithPKE(H, Kwm, P_w_m, error_check)

kalman = KalmanCore()
for i in range(10):
    kalman.RobustUpdateWithTdoa(Tdoa(0.5))

print("Final State:")
print("X:", kalman.S[KC_STATE_X])
print("Y:", kalman.S[KC_STATE_Y])
print("Z:", kalman.S[KC_STATE_Z])