import numpy as np

def dh_transform(a, alpha, d, theta):
    # generic link-coordination transformation matrix
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                  np.cos(alpha),                 d],
        [0,              0,                              0,                             1]
    ])


# ==============================
# Define DH Parameter
# ==============================

d =     [55, 110, 0, 0, 120]
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
    Ti = dh_transform(theta[i], d[i], a[i], alpha[i])
    T_all.append(Ti)
    print(f"\nT{i+1}: \n{Ti}")

T_total = np.eye(4)
for Ti in T_all:
    T_total = np.dot(T_total, Ti)

print("\nOverall Transformation Marix:")
print(T_total)

#Extract the position of TCP
tcp_Pos = T_total[:3, 3]
print("\nPosition of TCP [x, y, z]:")
print(tcp_Pos)





















# Example DH parameters for a 5-DOF arm
# [a, alpha, d, theta] -> theta will be replaced by joint variables
# You must modify these based on your arm design
dh_params = [
    [0,      np.pi/2,  0.5,  0],  # Link 1
    [0.5,    0,        0,    0],  # Link 2
    [0.5,    0,        0,    0],  # Link 3
    [0.2,    np.pi/2,  0,    0],  # Link 4
    [0.1,    0,        0,    0]   # Link 5 (wrist / end-effector)
]

# Function to compute forward kinematics
def forward_kinematics(joint_angles):
    """
    joint_angles: list of 5 joint angles in radians [θ1, θ2, θ3, θ4, θ5]
    Returns the homogeneous transformation matrix of the end-effector.
    """
    T = np.eye(4)  # Identity matrix
    for i in range(5):
        a, alpha, d, theta = dh_params[i]
        # Replace theta with actual joint angle
        T_i = dh_transform(a, alpha, d, theta + joint_angles[i])
        T = np.dot(T, T_i)  # Multiply transformations
    return T


# Example usage:
if __name__ == "__main__":
    # Example joint angles in radians
    q = [0, np.pi/4, np.pi/6, -np.pi/3, np.pi/2]

    T_end_effector = forward_kinematics(q)
    print("End-effector Transformation Matrix:\n", T_end_effector)

    # Extract position (x, y, z)
    position = T_end_effector[:3, 3]
    print("End-effector Position (x, y, z):", position)

