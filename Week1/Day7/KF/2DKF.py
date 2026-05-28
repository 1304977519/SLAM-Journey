import numpy as np # type: ignore
import matplotlib.pyplot as plt # type: ignore

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# =========================
# 运动模型 f(x, u)
# =========================
# 状态 x = [px, py, theta]
# 控制 u = [v, omega]
#
# px    : 机器人在世界系下的 x 位置
# py    : 机器人在世界系下的 y 位置
# theta : 机器人朝向
#
# v     : 前进速度
# omega : 角速度
def motion_model(x, u, dt):
    px, py, theta = x
    v, omega = u

    px_new = px + v * dt * np.cos(theta)
    py_new = py + v * dt * np.sin(theta)
    theta_new = theta + omega * dt
    theta_new = normalize_angle(theta_new)
    return np.array([px_new, py_new, theta_new])

# =========================
# 运动模型对状态 x 的雅可比 F
# =========================
# F = df / dx
#
# f1 = px + v dt cos(theta)
# f2 = py + v dt sin(theta)
# f3 = theta + omega dt
#
# 对 theta 求导：
# d f1 / d theta = -v dt sin(theta)
# d f2 / d theta =  v dt cos(theta)
def jecobian_F(x, u, dt):
    _, _, theta = x
    v, _ = u
    F = np.array([
        [1.0, 0.0, -v * dt * np.sin(theta)],
        [0.0, 1.0, v * dt * np.cos(theta)],
        [0.0, 0.0, 1.0]
    ])
    return F

# =========================
# 观测模型 h(x)
# =========================
# 这里先用最简单的观测：
# 传感器可以直接测到机器人的位置 [px, py]
#
# 所以：
# z = [px, py] + noise
def observation_model(x):
    px, py, _ = x
    return np.array([px, py])

# =========================
# 观测模型的雅可比 H
# =========================
# h(x) = [px, py]
#
# 所以：
# dh/dx =
# [1, 0, 0]
# [0, 1, 0]
def jecobian_H():
    H = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ])
    return H

# =========================
# EKF 预测步骤
# =========================
def ekf_predict(x, P, u, Q, dt):
    x_predict = motion_model(x, u, dt)
    F = jecobian_F(x, u, dt)
    P_predict = F @ P @ F.T + Q
    return x_predict, P_predict

# =========================
# EKF 更新步骤
# =========================
def ekf_update(x_predict, P_predict, z, R):
    z_predict = observation_model(x_predict)
    residual = z - z_predict
    H = jecobian_H()
    S = H @ P_predict @ H.T + R
    K = P_predict @ H.T @ np.linalg.inv(S)
    x_new = x_predict + K @ residual
    x_new[2] = normalize_angle(x_new[2])
    I = np.eye(3)
    P_new = (I - K @ H) @ P_predict
    return x_new, P_new

def main():
    np.random.seed(0)
    dt = 0.1
    steps = 200
    x_true = np.array([0.0, 0.0, 0.0])
    x_est = np.array([0.0, 0.0, 0.0])
    P = np.eye(3) * 1.0
    Q = np.diag([0.05, 0.05, np.deg2rad(2.0)]) ** 2
    R = np.diag([0.5, 0.5]) ** 2
    true_history = []
    est_history = []
    obs_history = []

    for i in range(steps):
        u = np.array([1.0, 0.2])
        x_true = motion_model(x_true, u, dt)
        z_true = observation_model(x_true)
        noise = np.random.multivariate_normal(mean=np.zeros(2), cov=R)
        z = z_true + noise
        x_predict, P_predict = ekf_predict(x_est, P, u, Q, dt)
        x_est, P = ekf_update(x_predict, P_predict, z, R)
        true_history.append(x_true.copy())
        est_history.append(x_est.copy())
        obs_history.append(z.copy())

    true_history = np.array(true_history)
    est_history = np.array(est_history)
    obs_history = np.array(obs_history)
    plt.figure(figsize=(8, 6))

    plt.plot(true_history[:, 0], true_history[:, 1], label="True trajectory")
    plt.plot(est_history[:, 0], est_history[:, 1], label="EKF estimated trajectory")
    plt.scatter(obs_history[:, 0], obs_history[:, 1], s=8, label="Noisy observations")

    plt.axis("equal")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("2D EKF Localization")
    plt.legend()
    plt.grid(True)

    plt.savefig("ekf_2d_result.png", dpi=200)
    plt.show()


if __name__ == "__main__":
    main()