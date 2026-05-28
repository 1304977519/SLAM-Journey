# 1D Kalman Filter
# x 表示上一时刻的状态估计
x = 0.0

# P 表示上一时刻状态的不确定性
# P 越大，说明我越不相信当前的 x
P = 1.0

# u 表示控制量/运动量
u = 1.0

# Q 表示预测过程噪声
# Q 越大，说明预测越不可靠
Q = 0.1

# R 表示观测噪声
# R 越大，说明观测越不可靠
R = 0.5

# z 表示当前传感器观测值
z = 1.2

# 根据运动模型预测当前状态
# 这里的模型很简单：当前位置 = 上一时刻位置 + 控制量
x_predict = x + u

# 预测当前状态的不确定性
# 预测以后，不确定性会变大，因为运动过程有噪声 Q
p_predict = P + Q

# K 表示 Kalman 增益
# 它决定这次更新时更相信预测，还是更相信观测
#
# 如果 p_predict 很大：
#   说明预测不可靠，K 会变大，更相信观测 z
#
# 如果 R 很大：
#   说明观测不可靠，K 会变小，更相信预测 x_predict
#
# 对应公式：K = P_predict / (P_predict + R)
K = p_predict / (p_predict + R)

# z - x_predict 是残差，也叫 innovation
# 意思是：观测值和预测值之间差了多少
#
# K * (z - x_predict) 表示：
#   根据残差修正预测值
#
# 对应公式：
# x_new = x_predict + K * (z - x_predict)
x_new = x_predict + K * (z - x_predict)

# 更新后的不确定性
# 因为融合了观测，所以不确定性通常会变小
#
# 在 1D 标量 KF 里，I 就是 1
# 所以写成：
# p_new = (1 - K) * p_predict
p_new = (1 - K) * p_predict

print("预测状态 x_predict =", x_predict)
print("预测不确定性 p_predict =", p_predict)
print("Kalman 增益 K =", K)
print("更新后状态 x_new =", x_new)
print("更新后不确定性 p_new =", p_new)

# 下一轮要把这次更新后的结果保存回去
# 否则下一次还是用旧的 x 和 P
x = x_new
P = p_new