import numpy as np
import matplotlib.pyplot as plt
import warnings

# 定义参数
omega = 10
l_AB = 75
l_BC = 120
l_CD = 135
l_AD = 170
l_BE = 20
l_EF = 15
x_D = 170
y_D = 0
dx_D = 0
dy_D = 0
ddx_D = 0
ddy_D = 0

# 定义角度范围
varphi_range = np.arange(0, 361, 1)
num_points = len(varphi_range)

# 初始化结果数组
x_F_points = np.zeros(num_points)
y_F_points = np.zeros(num_points)
v_Fx_points = np.zeros(num_points)
v_Fy_points = np.zeros(num_points)
a_Fx_points = np.zeros(num_points)
a_Fy_points = np.zeros(num_points)

# 循环计算
for idx in range(num_points):
    varphi = varphi_range[idx]
    x_B = l_AB * np.cos(np.deg2rad(varphi))
    y_B = l_AB * np.sin(np.deg2rad(varphi))
    A_0 = 2 * l_BC * (x_D - x_B)
    B_0 = 2 * l_BC * (y_D - y_B)
    l_BD = np.sqrt((x_D - x_B) ** 2 + (y_D - y_B) ** 2)
    C_0 = l_BC ** 2 + l_BD ** 2 - l_CD ** 2
    controlcent = A_0 ** 2 + B_0 ** 2 - C_0 ** 2
    if controlcent < 0:
        warnings.warn(f'虚根出现于 φ={varphi}°, 跳过该角度')
        x_F_points[idx] = np.nan
        y_F_points[idx] = np.nan
        v_Fx_points[idx] = np.nan
        v_Fy_points[idx] = np.nan
        a_Fx_points[idx] = np.nan
        a_Fy_points[idx] = np.nan
        continue
    varphi_i = 2 * np.arctan2(B_0 + np.sqrt(controlcent), A_0 + C_0)
    x_C = x_B + l_BC * np.cos(varphi_i)
    y_C = y_B + l_BC * np.sin(varphi_i)
    varphi_j = np.arctan2(y_C - y_D, x_C - x_D)
    x_F = x_B + l_BE * np.cos(varphi_i) - l_EF * np.sin(varphi_i)
    y_F = y_B + l_BE * np.sin(varphi_i) + l_EF * np.cos(varphi_i)
    x_F_points[idx] = x_F
    y_F_points[idx] = y_F
    dx_B = -omega * l_AB * np.sin(np.deg2rad(varphi))
    dy_B = omega * l_AB * np.cos(np.deg2rad(varphi))
    C_i = l_BC * np.cos(varphi_i)
    S_i = l_BC * np.sin(varphi_i)
    C_j = l_CD * np.cos(varphi_j)
    S_j = l_CD * np.sin(varphi_j)
    G_1 = C_i * S_j - C_j * S_i
    if abs(G_1) < 1e-6:
        warnings.warn(f'奇异位置: φ={varphi}°, 跳过计算')
        v_Fx_points[idx] = np.nan
        v_Fy_points[idx] = np.nan
        a_Fx_points[idx] = np.nan
        a_Fy_points[idx] = np.nan
        continue
    dvarphi_i = (C_j * (dx_D - dx_B) + S_j * (dy_D - dy_B)) / G_1
    dvarphi_j = (C_i * (dx_D - dx_B) + S_i * (dy_D - dy_B)) / G_1
    v_Fx = -omega * l_AB * np.sin(np.deg2rad(varphi)) - dvarphi_i * l_BE * np.sin(varphi_i) - dvarphi_i * l_EF * np.cos(varphi_i)
    v_Fy = omega * l_AB * np.cos(np.deg2rad(varphi)) + dvarphi_i * l_BE * np.cos(varphi_i) - dvarphi_i * l_EF * np.sin(varphi_i)
    v_Fx_points[idx] = v_Fx
    v_Fy_points[idx] = v_Fy
    ddx_B = -omega ** 2 * l_AB * np.cos(np.deg2rad(varphi))
    ddy_B = -omega ** 2 * l_AB * np.sin(np.deg2rad(varphi))
    G_2 = ddx_D - ddx_B + dvarphi_i ** 2 * C_i - dvarphi_j ** 2 * C_j
    G_3 = ddy_D - ddy_B + dvarphi_i ** 2 * S_i - dvarphi_j ** 2 * S_j
    ddvarphi_i = (G_2 * C_j + G_3 * S_j) / G_1
    a_Fx = -omega ** 2 * l_AB * np.cos(np.deg2rad(varphi)) - ddvarphi_i * l_BE * np.sin(varphi_i) - \
        dvarphi_i ** 2 * l_BE * np.cos(varphi_i) - ddvarphi_i * l_EF * np.cos(varphi_i) + dvarphi_i ** 2 * l_EF * np.sin(varphi_i)
    a_Fy = -omega ** 2 * l_AB * np.sin(np.deg2rad(varphi)) + ddvarphi_i * l_BE * np.cos(varphi_i) - \
        dvarphi_i ** 2 * l_BE * np.sin(varphi_i) - ddvarphi_i * l_EF * np.sin(varphi_i) - dvarphi_i ** 2 * l_EF * np.cos(varphi_i)
    a_Fx_points[idx] = a_Fx
    a_Fy_points[idx] = a_Fy


# print results
for idx in range(num_points):
    if not np.isnan(x_F_points[idx]):
        print(f"angle = {varphi_range[idx]}, x_F = {x_F_points[idx]:.2f}, y_F = {y_F_points[idx]:.2f}, \
              v_Fx = {v_Fx_points[idx]}, v_Fy = {v_Fy_points[idx]}, a_Fx = {a_Fx_points[idx]}, a_Fy = {a_Fy_points[idx]}")


# 绘图
plt.figure(figsize=(12, 8))

plt.subplot(3, 2, 1)
plt.plot(x_F_points, y_F_points, 'r-', linewidth=1.5)
plt.xlabel('x_F (mm)')
plt.ylabel('y_F (mm)')
plt.title('S_F(x,y)')

plt.subplot(3, 2, 3)
plt.plot(varphi_range, v_Fx_points, 'r-', linewidth=1.5)
plt.xlabel(r'$\varphi(^{\circ})$')
plt.ylabel(r'$v_{Fx} (mm/s)$')
plt.title(r'$v_{Fx}(\varphi)$')
plt.grid()

plt.subplot(3, 2, 4)
plt.plot(varphi_range, v_Fy_points, 'r-', linewidth=1.5)
plt.xlabel(r'$\varphi(^{\circ})$')
plt.ylabel(r'$v_{Fy} (mm/s)$')
plt.title(r'$v_{Fy}(\varphi)$')
plt.grid()

plt.subplot(3, 2, 5)
plt.plot(varphi_range, a_Fx_points, 'g-', linewidth=1.5)
plt.xlabel(r'$\varphi(^{\circ})$')
plt.ylabel(r'$a_{Fx} (mm^2/s)$')
plt.title(r'$a_{Fx}(\varphi)$')
plt.grid()

plt.subplot(3, 2, 6)
plt.plot(varphi_range, a_Fy_points, 'g-', linewidth=1.5)
plt.xlabel(r'$\varphi(^{\circ})$')
plt.ylabel(r'$a_{Fy} (mm^2/s)$')
plt.title(r'$a_{Fy}(\varphi)$')
plt.grid()

plt.tight_layout()
plt.show()