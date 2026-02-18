import numpy as np

from forward_kinematics import forward_kinematics
from jacobian import compute_jacobian
from inverse_kinematics import inverse_kinematics, rpy_to_rotation
from IK_verification import verify_solution

np.set_printoptions(precision=6, suppress=True)


# ---------------------------------------------------
# Singularity Check Function
# ---------------------------------------------------
def check_singularity(J):

    det_J = np.linalg.det(J)

    print("\n--- Singularity Check ---")
    print("Determinant of Jacobian =", det_J)

    if abs(det_J) < 1e-6:
        print("⚠ Robot is in a SINGULAR configuration")
    else:
        print("Robot is NOT singular")

    return det_J


# ---------------------------------------------------
# MAIN EXECUTION FLOW
# ---------------------------------------------------
if __name__ == "__main__":

    print("\n========== ROBOT KINEMATICS PIPELINE ==========\n")

    # ---------------------------------------------------
    # 1️⃣ Define Link Constants
    # ---------------------------------------------------
    print("Enter Link Constants:")
    link_params = {
        "d1": float(input("d1: ")),
        "a3": float(input("a3: ")),
        "a4": float(input("a4: ")),
        "d4": float(input("d4: ")),
        "d5": float(input("d5: ")),
        "d6": float(input("d6: "))
    }

    # ---------------------------------------------------
    # 2️⃣ Define Desired Pose
    # ---------------------------------------------------
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

    print("\nDesired Transformation Matrix:")
    print(T_desired)

    # ---------------------------------------------------
    # 3️⃣ Call IK
    # ---------------------------------------------------
    q_solution = inverse_kinematics(T_desired, link_params)

    if q_solution is None:
        print("\nIK Failed. Exiting.")
        exit()

    print("\nRecovered Joint Angles (degrees):")
    print(np.rad2deg(q_solution))

    # ---------------------------------------------------
    # 4️⃣ Call Verification (Step 7)
    # ---------------------------------------------------
    verify_solution(T_desired, q_solution, link_params)

    # ---------------------------------------------------
    # 5️⃣ Compute Jacobian
    # ---------------------------------------------------
    J = compute_jacobian(q_solution, link_params)

    print("\nJacobian Matrix:")
    print(J)

    # ---------------------------------------------------
    # 6️⃣ Check Singularity
    # ---------------------------------------------------
    check_singularity(J)

    print("\n========== EXECUTION COMPLETE ==========\n")
