import numpy as np

np.set_printoptions(precision=6, suppress=True)


# ---------------------------------------------------
# DH Transform
# ---------------------------------------------------
def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),               d],
        [0,              0,                           0,                           1]
    ])


# ---------------------------------------------------
# RPY to Rotation
# ---------------------------------------------------
def rpy_to_rotation(roll, pitch, yaw):

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0,              1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx


# ---------------------------------------------------
# Forward Kinematics
# ---------------------------------------------------
def forward_kinematics(q, link_params):

    d1, a3, a4, d4, d5, d6 = (
        link_params["d1"],
        link_params["a3"],
        link_params["a4"],
        link_params["d4"],
        link_params["d5"],
        link_params["d6"],
    )

    dh_table = [
        (0,   0,        d1, q[0]),
        (0,   np.pi/2,  0,  q[1]),
        (a3,  0,        0,  q[2]),
        (a4,  0,        d4, q[3]),
        (0,  -np.pi/2,  d5, q[4]),
        (0,   np.pi/2,  d6, q[5]),
    ]

    T = np.eye(4)
    for a, alpha, d, theta in dh_table:
        T = T @ dh_transform(a, alpha, d, theta)

    return T


# ---------------------------------------------------
# Jacobian
# ---------------------------------------------------
def compute_jacobian(q, link_params):

    T_list = [np.eye(4)]
    T = np.eye(4)

    d1, a3, a4, d4, d5, d6 = (
        link_params["d1"],
        link_params["a3"],
        link_params["a4"],
        link_params["d4"],
        link_params["d5"],
        link_params["d6"],
    )

    dh_table = [
        (0,   0,        d1, q[0]),
        (0,   np.pi/2,  0,  q[1]),
        (a3,  0,        0,  q[2]),
        (a4,  0,        d4, q[3]),
        (0,  -np.pi/2,  d5, q[4]),
        (0,   np.pi/2,  d6, q[5]),
    ]


    for a, alpha, d, theta in dh_table:
        T = T @ dh_transform(a, alpha, d, theta)
        T_list.append(T)

    o_n = T_list[-1][0:3, 3]

    J_v = np.zeros((3, 6))
    J_w = np.zeros((3, 6))

    for i in range(6):
        z_i = T_list[i][0:3, 2]
        o_i = T_list[i][0:3, 3]
        J_v[:, i] = np.cross(z_i, (o_n - o_i))
        J_w[:, i] = z_i

    return np.vstack((J_v, J_w))


# ---------------------------------------------------
# Orientation Error
# ---------------------------------------------------
def orientation_error(R_current, R_desired):

    return 0.5 * (
        np.cross(R_current[:, 0], R_desired[:, 0]) +
        np.cross(R_current[:, 1], R_desired[:, 1]) +
        np.cross(R_current[:, 2], R_desired[:, 2])
    )


# ---------------------------------------------------
# Numerical IK with Automatic Guessing
# ---------------------------------------------------
def inverse_kinematics(T_desired, link_params,
                       max_iter=1000,
                       pos_tol=1e-4,
                       ori_tol=1e-4,
                       attempts=5):

    for attempt in range(attempts):

        # Random initial guess in [-pi, pi]
        q = np.random.uniform(-np.pi, np.pi, 6)

        for iteration in range(max_iter):

            T_current = forward_kinematics(q, link_params)

            p_current = T_current[0:3, 3]
            R_current = T_current[0:3, 0:3]

            p_desired = T_desired[0:3, 3]
            R_desired = T_desired[0:3, 0:3]

            e_p = p_desired - p_current
            e_o = orientation_error(R_current, R_desired)

            if np.linalg.norm(e_p) < pos_tol and np.linalg.norm(e_o) < ori_tol:
                print(f"\nConverged in {iteration} iterations (attempt {attempt+1}).")
                return q

            e = np.hstack((e_p, e_o))

            J = compute_jacobian(q, link_params)
            J_pinv = np.linalg.pinv(J)

            q = q + J_pinv @ e

    print("\nFailed to converge.")
    return None


# ---------------------------------------------------
# MAIN
# ---------------------------------------------------
if __name__ == "__main__":

    print("\n========== Numerical IK Solver ==========\n")

    # Link parameters
    print("Enter Link Constants:")
    link_params = {
        "d1": float(input("d1: ")),
        "a3": float(input("a3: ")),
        "a4": float(input("a4: ")),
        "d4": float(input("d4: ")),
        "d5": float(input("d5: ")),
        "d6": float(input("d6: "))
    }

    print("\nEnter Desired Position:")
    x = float(input("x: "))
    y = float(input("y: "))
    z = float(input("z: "))

    print("\nEnter Desired Orientation (RPY in degrees):")
    roll = np.deg2rad(float(input("Roll: ")))
    pitch = np.deg2rad(float(input("Pitch: ")))
    yaw = np.deg2rad(float(input("Yaw: ")))

    R_desired = rpy_to_rotation(roll, pitch, yaw)

    T_desired = np.eye(4)
    T_desired[0:3, 0:3] = R_desired
    T_desired[0:3, 3] = [x, y, z]

    q_solution = inverse_kinematics(T_desired, link_params)

    if q_solution is not None:
        print("\nRecovered Joint Angles (degrees):")
        print(np.rad2deg(q_solution))
