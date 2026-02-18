import numpy as np

# Improve print readability
np.set_printoptions(precision=6, suppress=True)


# ---------------------------------------------------
# Standard DH Transformation
# ---------------------------------------------------
def dh_transform(a, alpha, d, theta):
    """
    Standard DH:
    Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    """

    T = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                d],
        [0,              0,                             0,                            1]
    ])

    return T


# ---------------------------------------------------
# Extract RPY from Rotation Matrix (ZYX convention)
# ---------------------------------------------------
def rotation_to_rpy(R):
    """
    Converts rotation matrix to Roll-Pitch-Yaw
    using ZYX convention.
    """

    if abs(R[2, 0]) != 1:
        pitch = -np.arcsin(R[2, 0])
        roll = np.arctan2(R[2, 1] / np.cos(pitch),
                          R[2, 2] / np.cos(pitch))
        yaw = np.arctan2(R[1, 0] / np.cos(pitch),
                         R[0, 0] / np.cos(pitch))
    else:
        # Gimbal lock case
        yaw = 0
        if R[2, 0] == -1:
            pitch = np.pi / 2
            roll = yaw + np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -np.pi / 2
            roll = -yaw + np.arctan2(-R[0, 1], -R[0, 2])

    return roll, pitch, yaw


# ---------------------------------------------------
# Forward Kinematics Function
# ---------------------------------------------------
def forward_kinematics(q, link_params):
    """
    q: list or array of 6 joint angles (radians)
    link_params: dictionary containing d1, a2, a3, d4, d5, d6
    """

    d1 = link_params["d1"]
    a3 = link_params["a3"]
    a4 = link_params["a4"]
    d4 = link_params["d4"]
    d5 = link_params["d5"]
    d6 = link_params["d6"]

    # Standard UR5 DH Table
    dh_table = [
        (0,   0,       d1, q[0]),   
        (0,   np.pi/2, 0,  q[1]),   
        (a3,  0,       0,  q[2]),   
        (a4,  0,       d4, q[3]),   
        (0,  -np.pi/2, d5, q[4]),   
        (0,   np.pi/2, d6, q[5]),
    ]

    T = np.eye(4)

    for a, alpha, d, theta in dh_table:
        T = T @ dh_transform(a, alpha, d, theta)

    return T


# ---------------------------------------------------
# Main Execution
# ---------------------------------------------------
if __name__ == "__main__":

    print("\n========== UR5 Forward Kinematics ==========\n")

    print("Enter Joint Angles (in DEGREES):")

    q = []
    for i in range(1, 7):
        angle_deg = float(input(f"q{i} (deg): "))
        q.append(np.deg2rad(angle_deg))

    print("\nEnter Link Constants:")

    link_params = {
        "d1": float(input("d1: ")),
        "a3": float(input("a3: ")),
        "a4": float(input("a4: ")),
        "d4": float(input("d4: ")),
        "d5": float(input("d5: ")),
        "d6": float(input("d6: "))
    }

    # Compute FK
    T06 = forward_kinematics(q, link_params)

    print("\n========================================")
    print("T_06 (End Effector Pose):")
    print("========================================")
    print(T06)

    # Extract position
    position = T06[0:3, 3]

    # Extract rotation matrix
    R = T06[0:3, 0:3]

    # Convert to RPY
    roll, pitch, yaw = rotation_to_rpy(R)

    print("\n--- Position ---")
    print("x =", position[0])
    print("y =", position[1])
    print("z =", position[2])

    print("\n--- Orientation (Rotation Matrix) ---")
    print(R)

    print("\n--- Orientation (RPY in degrees) ---")
    print("Roll  =", np.rad2deg(roll))
    print("Pitch =", np.rad2deg(pitch))
    print("Yaw   =", np.rad2deg(yaw))
