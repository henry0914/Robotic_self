# 5-DOF Robot
import numpy as np

def dh_transform(theta, d, a, alpha):
    # generic link-coordination transformation matrix
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [     0       ,            np.sin(alpha)      ,           np.cos(alpha)       ,          d       ],
        [     0       ,                 0             ,                0              ,          1       ]
    ])


# ==============================
# Define DH Parameter
# ==============================

d =     [55, 0, 0, 0, 70]
a =     [0, 110, 60, 0, 0]
alpha = [90, 0, 0, 90, 0]

# convert alpha to radians
alpha = np.deg2rad(alpha)

# ask for joint angles (in deg)
theta = []
for i in range (5):
    angle_deg = float(input(f"Enter theta{i+1} (deg):"))
    theta.append(np.deg2rad(angle_deg))

# Calculate and show each transformation matrix
T_all = []
for i in range (5):
    Ti = dh_transform(d= d[i], a=a[i], alpha=alpha[i], theta=theta[i])
    T_all.append(Ti)
    print(f"\nT{i+1}: \n{np.round(a=Ti, decimals=2)}")

T_total = np.eye(4)
for Ti in T_all:
    T_total = np.dot(T_total, Ti)

print("\nOverall Transformation Marix:")
print(np.round(a=T_total, decimals=2))

#Extract the position of TCP
tcp_Pos = T_total[:3, 3]
print("\nPosition of TCP [x, y, z]:")
print(np.round(a=tcp_Pos, decimals=2))
