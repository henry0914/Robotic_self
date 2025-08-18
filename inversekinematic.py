import numpy as np
import math

# desired tip pose
p_tool = np.array([10, 0.0, 10 ])

# desired tool rotation R05
R_tool = np.array([
    [0.0,  0.0, 1.0],
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
])

# links (mm)
a2, a3, d1 = 110.0, 60.0, 55.0
d5 = 120  # tool length along tool +Z

# wrist center
r3 = R_tool[:, 2] # tool +Z (3rd column)
p_wrist = p_tool - d5 * r3
x_w, y_w, z_w = p_wrist

# shoulder-plane triangle
R = math.hypot(x_w, y_w)
zprime = z_w - d1
RF = math.hypot(R, zprime)

# base/elevation
gamma = math.degrees(math.atan2(y_w, x_w))
beta  = math.degrees(math.atan2(zprime, R))

# cosine-rule angles (with clamping)
def clamp(v): return max(-1.0, min(1.0, v))

cos_alpha = clamp((a2*a2 + RF*RF - a3*a3) / (2*a2*RF))
alpha = math.degrees(math.acos(cos_alpha))

cos_phi = clamp((a3*a3 + a2*a2 - RF*RF) / (2*a3*a2))
phi = math.degrees(math.acos(cos_phi))

# joints
q1 = gamma
q2 = beta + alpha
q3 = 180.0 - phi

def R03(q1_deg, q2_deg, q3_deg):
    q1, q2, q3 = map(math.radians, (q1_deg, q2_deg, q3_deg))
    c1, s1 = math.cos(q1), math.sin(q1)
    c23, s23 = math.cos(q2 - q3), math.sin(q2 - q3)
    return np.array([
        [ c1*c23,  c1*s23, -s1],
        [ s1*c23,  s1*s23,  c1],
        [   s23 ,   -c23 ,   0],
    ])

R03 = R03(q1, q2, q3)

M = R03.T @ R_tool

q4 = 90 - math.degrees(math.atan2(M[0,2], M[0,0]))  # rotation about Y
q5 = math.degrees(math.atan2(M[1,1], M[2,1]))  # rotation about X

print("theta1 =", q1)
print("theta2 =", q2)
print("theta3 =", q3)
print("theta4 =", q4)
print("theta5 =", q5)