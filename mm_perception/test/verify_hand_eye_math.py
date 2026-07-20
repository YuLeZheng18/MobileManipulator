#!/usr/bin/env python3
"""离线验证手眼标定求解逻辑 (不需真机/相机).

思路 (合成闭环, 与 sim_aruco_camera 同套路):
  1. 设一个"真值" X = Link_29->Link_30 外参 (相机相对腕部);
  2. 设固定标记在基座系的位姿 base->target;
  3. 造 N 组随机腕部位姿 base->gripper(=Link_20->Link_29);
  4. 由链路 target2camera = inv(X) @ inv(base->gripper) @ (base->target)
     反推每组相机看到的标记位姿;
  5. 把 (gripper2base, target2camera) 喂 calibrateHandEye, 解出的应等于真值 X.
验证节点里的 quat/rpy 转换与 calibrateHandEye 调用约定一致.
"""
import math
import sys

import numpy as np
import cv2

from mm_perception.hand_eye_calibrator import rot_matrix_to_rpy


def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def H(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(t).reshape(3)
    return M


def main():
    rng = np.random.default_rng(0)

    # 1) 真值 X = Link_29 -> Link_30 (相机相对腕部)
    X_t = np.array([0.055, 0.007, -0.019])
    X_R = rpy_to_R(0.0, math.pi / 2, -math.pi / 2)
    X = H(X_R, X_t)

    # 2) 固定标记在基座系 base->target
    BT = H(rpy_to_R(0.1, -0.2, 0.3), [0.4, 0.05, 0.3])

    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for _ in range(15):
        # 3) 随机腕部位姿 base->gripper
        rpy = rng.uniform(-0.6, 0.6, 3)
        pos = rng.uniform(-0.15, 0.15, 3) + np.array([0.2, 0.0, 0.25])
        BG = H(rpy_to_R(*rpy), pos)

        # 4) target2camera = inv(X) @ inv(BG) @ BT  (camera<-base<-target)
        T2C = np.linalg.inv(X) @ np.linalg.inv(BG) @ BT

        R_g2b.append(BG[:3, :3]); t_g2b.append(BG[:3, 3])
        R_t2c.append(T2C[:3, :3]); t_t2c.append(T2C[:3, 3])

    # 5) 求解
    R_cg, t_cg = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, method=cv2.CALIB_HAND_EYE_TSAI)

    t = t_cg.reshape(3)
    roll, pitch, yaw = rot_matrix_to_rpy(R_cg)
    gt_r, gt_p, gt_y = rot_matrix_to_rpy(X_R)

    print("真值  xyz =", np.round(X_t, 6), " rpy =", np.round([gt_r, gt_p, gt_y], 6))
    print("解出  xyz =", np.round(t, 6),   " rpy =", np.round([roll, pitch, yaw], 6))

    dt = np.linalg.norm(t - X_t)
    dR = np.linalg.norm(R_cg - X_R)
    print("平移误差 = %.2e m, 旋转矩阵误差 = %.2e" % (dt, dR))
    assert dt < 1e-6 and dR < 1e-6, "标定未解回真值! 求解逻辑或约定有误"
    print("\n通过: calibrateHandEye 调用约定正确, 能精确解回 Link_29->Link_30 外参.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
