import numpy as np

# Set print options for better readability
np.set_printoptions(precision=6, suppress=True)

def dh_transform(a, alpha, d, theta):
    """
    Compute Standard DH homogeneous transformation matrix.
    Formula:
    Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    """
    T = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                d],
        [0,              0,                             0,                            1]
    ])
    return T

if __name__ == "__main__":
    print("\n========== UR5 DH Transformation Tool ==========\n")

    # Frame selection
    try:
        start = int(input("Enter start frame (0-5): "))
        end = int(input("Enter end frame (1-6): "))
    except ValueError:
        print("Please enter valid integers.")
        exit()

    if start < 0 or end > 6 or start >= end:
        print("Invalid frame selection. Ensure 0 <= start < end <= 6.")
        exit()

    print("\nEnter Joint Angles (in DEGREES):")
    q = [0.0] * 7 # Index 1-6
    for i in range(1, 7):
        q[i] = np.deg2rad(float(input(f"q{i} (deg): ")))

    print("\nEnter Link Constants:")
    d1 = float(input("d1: "))
    a2 = float(input("a2: "))
    a3 = float(input("a3: "))
    d4 = float(input("d4: "))
    d5 = float(input("d5: "))
    d6 = float(input("d6: "))

    # ---------------------------------------------------
    # DH Table (Standard DH Convention)
    # ---------------------------------------------------
    # Indexing: dh_params[i] corresponds to the transformation from i-1 to i
    # i | a_i | alpha_i | d_i | theta_i
    dh_params = [
        None,                   # Offset for 1-based indexing
        (0,   np.pi/2, d1, q[1]), # T_01
        (a2,  0,       0,  q[2]), # T_12
        (a3,  0,       0,  q[3]), # T_23
        (0,   np.pi/2, d4, q[4]), # T_34
        (0,  -np.pi/2, d5, q[5]), # T_45
        (0,   0,       d6, q[6]), # T_56
    ]

    T_cumulative = np.eye(4)

    print("\n--- Step-by-Step Transformations ---")
    for i in range(start + 1, end + 1):
        a, alpha, d, theta = dh_params[i]
        T_i = dh_transform(a, alpha, d, theta)
        
        print(f"\nMatrix A_{i} (T_{i-1}->{i}):")
        print(T_i)
        
        T_cumulative = T_cumulative @ T_i

    print("\n" + "="*40)
    print(f"FINAL RESULT: T_{start}{end}")
    print("="*40)
    print(T_cumulative)
    print("="*40)