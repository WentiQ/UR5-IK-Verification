import numpy as np

np.set_printoptions(precision=6, suppress=True)


# ---------------------------------------------------
# Standard DH Transformation
# ---------------------------------------------------
def dh_transform(a, alpha, d, theta):

    T = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                d],
        [0,              0,                             0,                            1]
    ])

    return T


# ---------------------------------------------------
# Compute Forward Transformations T_0i
# ---------------------------------------------------
def compute_transformations(q, link_params):

    d1 = link_params["d1"]
    a2 = link_params["a2"]
    a3 = link_params["a3"]
    d4 = link_params["d4"]
    d5 = link_params["d5"]
    d6 = link_params["d6"]

    dh_table = [
        (0,   np.pi/2, d1, q[0]),
        (a2,  0,       0,  q[1]),
        (a3,  0,       0,  q[2]),
        (0,   np.pi/2, d4, q[3]),
        (0,  -np.pi/2, d5, q[4]),
        (0,   0,       d6, q[5]),
    ]

    T_list = [np.eye(4)]  # T_00

    T = np.eye(4)

    for a, alpha, d, theta in dh_table:
        T = T @ dh_transform(a, alpha, d, theta)
        T_list.append(T)

    return T_list  # Contains T_00 to T_06


# ---------------------------------------------------
# Compute Geometric Jacobian
# ---------------------------------------------------
def compute_jacobian(q, link_params):

    T_list = compute_transformations(q, link_params)

    o_n = T_list[6][0:3, 3]   # End-effector origin

    J_v = np.zeros((3, 6))
    J_w = np.zeros((3, 6))

    for i in range(6):

        z_i = T_list[i][0:3, 2]     # z axis of frame i
        o_i = T_list[i][0:3, 3]     # origin of frame i

        # Revolute joint
        J_v[:, i] = np.cross(z_i, (o_n - o_i))
        J_w[:, i] = z_i

    J = np.vstack((J_v, J_w))

    return J


# ---------------------------------------------------
# Singularity Check
# ---------------------------------------------------
def check_singularity(J):

    det_J = np.linalg.det(J)

    print("\nDeterminant of Jacobian =", det_J)

    if abs(det_J) < 1e-6:
        print("⚠ WARNING: Robot is in a singular configuration!")
    else:
        print("Configuration is NOT singular.")

    return det_J


# ---------------------------------------------------
# Main Execution
# ---------------------------------------------------
if __name__ == "__main__":

    print("\n========== UR5 Jacobian Computation ==========\n")

    print("Enter Joint Angles (in DEGREES):")

    q = []
    for i in range(1, 7):
        angle_deg = float(input(f"q{i} (deg): "))
        q.append(np.deg2rad(angle_deg))

    print("\nEnter Link Constants:")

    link_params = {
        "d1": float(input("d1: ")),
        "a2": float(input("a2: ")),
        "a3": float(input("a3: ")),
        "d4": float(input("d4: ")),
        "d5": float(input("d5: ")),
        "d6": float(input("d6: "))
    }

    # Compute Jacobian
    J = compute_jacobian(q, link_params)

    print("\n========================================")
    print("Jacobian Matrix (6x6):")
    print("========================================")
    print(J)

    # Check singularity
    check_singularity(J)
