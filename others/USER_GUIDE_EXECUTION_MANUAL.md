# UR5 IK Verification Tool - User Guide & Execution Manual
## Complete Step-by-Step Usage Instructions

**Last Updated:** February 18, 2026

---

## TABLE OF CONTENTS

1. [Quick Start](#quick-start)
2. [System Requirements](#system-requirements)
3. [Setting Up the Environment](#setting-up-the-environment)
4. [Running Different Modules](#running-different-modules)
5. [Input Parameters Guide](#input-parameters-guide)
6. [Interpreting Results](#interpreting-results)
7. [Common Workflows](#common-workflows)
8. [Troubleshooting](#troubleshooting)
9. [Example Test Cases](#example-test-cases)
10. [Advanced Usage](#advanced-usage)

---

## QUICK START

### For the Impatient

```bash
# 1. Navigate to code folder
cd code/

# 2. Run the main pipeline
python main.py

# 3. Enter parameters when prompted:
# - Link constants (d1, a3, a4, d4, d5, d6)
# - Desired position (x, y, z)
# - Desired orientation (roll, pitch, yaw in degrees)

# 4. View results including:
# - Computed joint angles
# - Position/orientation errors
# - Singularity status
```

**Expected Runtime:** 0.5-3 seconds

---

## SYSTEM REQUIREMENTS

### Software
- **Python:** 3.7 or higher
- **NumPy:** 1.19+
- **OS:** Windows, Linux, macOS

### Hardware
- **Processor:** Any modern CPU
- **RAM:** 256 MB minimum
- **Disk:** 50 MB for code + results

### Installation

```bash
# Install numpy if not present
pip install numpy

# Verify installation
python -c "import numpy; print(numpy.__version__)"
```

---

## SETTING UP THE ENVIRONMENT

### Step 1: Verify Project Structure

```
UR5_IK_Verification/
├── code/
│   ├── main.py
│   ├── forward_kinematics.py
│   ├── inverse_kinematics.py
│   ├── jacobian.py
│   ├── IK_verification.py
│   └── dh_model.py
├── results/
├── README.md
├── guide.txt
└── DH_Parameters_Table.txt
```

### Step 2: Verify Python Installation

```bash
python --version
# Expected output: Python 3.x.x
```

### Step 3: Test NumPy

```bash
python -c "import numpy as np; print('NumPy OK')"
```

### Step 4: Navigate to Code Directory

```bash
cd code/
```

---

## RUNNING DIFFERENT MODULES

### Module 1: DH Model (dh_model.py)

**Purpose:** Understand and verify DH transformation matrices

**Run:**
```bash
python dh_model.py
```

**Input Required:**
1. Start frame (0-5)
2. End frame (1-6)
3. All joint angles in degrees (q1-q6)
4. Link constants (d1, a3, a4, d4, d5, d6)

**Output:**
- DH parameter table
- Individual transformation matrices T_i
- Cumulative transformation T_start → T_end

**Example:**
```
Start frame: 0
End frame: 1
q1 (deg): 45
q2-q6 (deg): 0
[Then link constants]

Output:
========== DH PARAMETER TABLE ==========
Matrix A_1 (T_0→1):
[[...]]
...
FINAL RESULT: T_01
[[...]]
```

### Module 2: Forward Kinematics (forward_kinematics.py)

**Purpose:** Compute end-effector pose from joint angles

**Run:**
```bash
python forward_kinematics.py
```

**Input Required:**
1. All 6 joint angles (degrees)
2. Link constants (6 values)

**Output:**
- End-effector transformation matrix T_06
- Position (x, y, z)
- Rotation matrix R
- RPY angles (degrees)

**Example Workflow:**
```
Enter Joint Angles (in DEGREES):
q1 (deg): 0
q2 (deg): -90
q3 (deg): 90
q4 (deg): 0
q5 (deg): 0
q6 (deg): 0

Enter Link Constants:
d1: 0.89159
a3: 0.425
a4: 0.39225
d4: 0.13585
d5: 0.08916
d6: 0.0823

========== UR5 Forward Kinematics ==========
T_06 (End Effector Pose):
[[ 1.000  0.000  0.000  0.815]
 [ 0.000  0.000  1.000  0.000]
 [ 0.000 -1.000  0.000  0.892]
 [ 0.000  0.000  0.000  1.000]]

Position:
x = 0.815
y = 0.000
z = 0.892

Orientation (RPY in degrees):
Roll  = 0.000
Pitch = 90.000
Yaw   = 0.000
```

### Module 3: Jacobian (jacobian.py)

**Purpose:** Compute manipulator Jacobian and detect singularities

**Run:**
```bash
python jacobian.py
```

**Input Required:**
1. All 6 joint angles (degrees)
2. Link constants

**Output:**
- Jacobian matrix (6×6)
- Determinant
- Singularity warning (if applicable)

**Example Output:**
```
========== UR5 Jacobian Computation ==========

Jacobian Matrix (6x6):
=========================================
[[ 0.234  -0.156   0.089   0.123  -0.045   0.067]
 [ 0.567   0.234   0.145   0.234   0.123   0.089]
 [ 0.000   0.000   0.000   0.000   0.000   0.000]
 [ 0.000   0.000   1.000   0.000   0.000   0.000]
 [ 0.000   1.000   0.000  -1.000   0.000  -1.000]
 [ 1.000   0.000   0.000   0.000   1.000   0.000]]

Determinant of Jacobian = 0.0856
✓ Configuration is NOT singular
```

**Singularity Interpretation:**
```
det(J) > 0.001  → Good dexterity
det(J) = 0.001 → Fair dexterity
det(J) < 0.001 → Approaching singularity
det(J) < 0.000001 → SINGULAR ⚠️
```

### Module 4: Inverse Kinematics (inverse_kinematics.py)

**Purpose:** Solve for joint angles given desired end-effector pose

**Run:**
```bash
python inverse_kinematics.py
```

**Input Required:**
1. Required position (x, y, z)
2. Required orientation (roll, pitch, yaw in degrees)
3. Link constants

**Output:**
- Joint angles (degrees)
- Convergence iteration count
- Failure message (if unsuccessful)

**Example:**
```
========== Numerical IK Solver ==========

Enter Link Constants:
d1: 0.89159
a3: 0.425
a4: 0.39225
d4: 0.13585
d5: 0.08916
d6: 0.0823

Enter Desired Position:
x: 0.5
y: 0.3
z: 0.6

Enter Desired Orientation (RPY in degrees):
Roll: 0
Pitch: 0
Yaw: 45

Converged in 127 iterations (attempt 1).

Recovered Joint Angles (degrees):
[  -23.456   -45.678   123.456   -67.890    89.012   -23.456]

Verification:
Position Error = 3.2e-05
Orientation Error = 1.5e-05
✓ IK Solution Valid
```

### Module 5: IK Verification (IK_verification.py)

**Purpose:** Validate IK solution through forward kinematics

**Run:**
```bash
python IK_verification.py
```

**Input Required:**
1. Required position (x, y, z)
2. Required orientation (roll, pitch, yaw in degrees)
3. Link constants

**Output:**
- Position error
- Orientation error
- Validity judgment (Valid/Invalid)

### Module 6: Main Pipeline (main.py)

**Purpose:** Complete workflow combining all operations

**Run:**
```bash
python main.py
```

**Full Workflow:**
```
1. Input link parameters
2. Input desired pose
3. Run IK solver
4. Verify IK solution
5. Compute Jacobian
6. Check singularity
7. Print all results
```

**This is the recommended execution method for complete analysis.**

---

## INPUT PARAMETERS GUIDE

### Link Constants

**UR5 Standard Parameters (Real Robot):**

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| d₁ | 0.89159 | m | Base height (vertical offset) |
| a₃ | 0.425 | m | Forearm length (positive) |
| a₄ | 0.39225 | m | Wrist length (positive) |
| d₄ | 0.13585 | m | Wrist offset |
| d₅ | 0.08916 | m | Tool offset (vertical) |
| d₆ | 0.0823 | m | Tool tip offset |

**For Testing (Simplified Values):**
```
d1: 1.0
a3: 0.5
a4: 0.5
d4: 0.1
d5: 0.1
d6: 0.1
```

### Joint Angles

**Format:** Degrees (converted to radians internally)

**Valid Range:**
- Some robots: ±180°
- Some robots: 0° to 360°
- UR5: ±180° per joint

**Example (Home Position):**
```
q1 = 0°
q2 = -90°
q3 = 90°
q4 = 0°
q5 = 0°
q6 = 0°
```

### Position (x, y, z)

**Format:** Meters (assuming metric system)

**Typical UR5 Workspace:**
```
x: -0.85 to +0.85 m
y: -0.85 to +0.85 m
z: 0.0 to +1.5 m
```

**Example (Reachable Pose):**
```
x: 0.5 m
y: 0.0 m
z: 0.9 m
```

### Orientation (Roll, Pitch, Yaw)

**Format:** Degrees (converted to radians internally)

**Convention:** ZYX Euler angles

**Axes:**
- **Roll (φ):** Rotation about X-axis (-180° to +180°)
- **Pitch (θ):** Rotation about Y-axis (-90° to +90°, avoid ±90°)
- **Yaw (ψ):** Rotation about Z-axis (-180° to +180°)

**Example (Vertical Approach):**
```
Roll:  0°
Pitch: 0°
Yaw:   0°
```

**Example (Side Approach):**
```
Roll:  90°
Pitch: 0°
Yaw:   0°
```

---

## INTERPRETING RESULTS

### Forward Kinematics Output

**Transformation Matrix Breakdown:**

```
T_06 = [[R₀₀  R₀₁  R₀₂  x]
        [R₁₀  R₁₁  R₁₂  y]
        [R₂₀  R₂₁  R₂₂  z]
        [0    0    0    1]]

Position: [x, y, z]
Orientation: 3×3 rotation matrix or RPY angles
```

**Interpretation:**
- If values are NaN: Check link parameters
- If position seems too large: May be outside workspace
- If rotation matrix columns aren't orthogonal: Numerical error

### Inverse Kinematics Output

**Joint Angles:**
```
Recovered Joint Angles (degrees):
[  q1    q2      q3       q4       q5     q6  ]
   -5.2  -45.7  123.5   -67.8    89.0  -23.5
```

**Interpretation:**
- q1: Base rotation (pan)
- q2: Shoulder pitch
- q3: Elbow pitch
- q4-q6: Wrist angles (spherical wrist)

### Error Metrics

#### Position Error

```
Position Error = 8.3e-05 m
```

**Interpretation:**
```
< 1e-6 m    → Exact solution (numerical limit)
1e-6 to 1e-4 m  → Excellent (< 0.1 mm)
1e-4 to 1e-3 m  → Good (< 1 mm)
1e-3 to 1e-2 m  → Acceptable (< 1 cm)
> 1e-2 m    → Poor (> 1 cm) - may be invalid
```

#### Orientation Error

```
Orientation Error = 1.2e-05
```

**Interpretation:**
```
< 1e-5      → Excellent alignment
1e-5 to 1e-4 → Good
1e-4 to 1e-3 → Acceptable
> 1e-3      → Check solution validity
```

### Singularity Warnings

#### Determinant Values

```
det(J) = 0.0234
```

**Interpretation:**
```
det(J) > 0.01   → ✓ Good configuration
det(J) ≈ 0.001  → ⚠ Approaching singularity
det(J) < 1e-6   → ✗ SINGULAR
```

**What to Do:**
- If singular: Avoid this configuration in motion planning
- If near-singular: Reduce speed/force application
- Use alternative configurations with higher det(J)

### Solution Validity

**Valid Solution:**
```
✓ IK Solution Valid
(Both position and orientation errors < 1e-4)
```

**Invalid Solution:**
```
✗ IK Solution Invalid
(One or more errors > tolerance)
```

---

## COMMON WORKFLOWS

### Workflow 1: Verify a Specific Joint Configuration

**Goal:** Check where a configuration puts the end-effector

**Steps:**
```
1. Open forward_kinematics.py (or use main.py)
2. Input joint angles (e.g., all zeros)
3. Input link parameters
4. Observe position and orientation output
5. Verify if reachable and sensible
```

### Workflow 2: Find Joint Angles for Desired Pose

**Goal:** Reach a specific position and orientation

**Steps:**
```
1. Run main.py
2. Input link parameters
3. Input desired position (x, y, z)
4. Input desired orientation (roll, pitch, yaw)
5. Wait for IK solution (should take 0.5-3 seconds)
6. Verify solution validity
7. Check if singular
```

### Workflow 3: Check If Configuration Is Singular

**Goal:** Determine if a configuration has mobility issues

**Steps:**
```
1. Run jacobian.py
2. Input the joint angles to check
3. Input link parameters
4. Observe determinant value
5. Check warning message
6. If det(J) < 1e-6: AVOID this configuration
```

### Workflow 4: Compare Multiple Solutions

**Goal:** Find best joint configuration for a pose

**Steps:**
```
1. Run main.py multiple times with same desired pose
2. IK will find different solutions (due to random init)
3. Compare Jacobian determinants
4. Choose solution with highest det(J) (best dexterity)
5. Verify solution has lowest error
```

### Workflow 5: Study DH Transformation Matrices

**Goal:** Understand transformation chain

**Steps:**
```
1. Run dh_model.py
2. Select start/end frames
3. Set specific joint angles
4. Observe individual T matrices
5. Verify multiplication chain: T_start→end = T_s→s+1 × ... × T_end-1→end
```

---

## TROUBLESHOOTING

### Problem 1: "IK Failed to Converge"

**Causes:**
1. Pose outside workspace (unreachable)
2. Bad random initialization
3. Singular configuration selected

**Solutions:**
```python
# Option A: Try running again (different random init)
# Option B: Adjust desired pose closer to arm
# Option C: Check if workspace allows this pose

# Test workspace reach:
# Maximum reach ≈ a3 + a4 + d6 ≈ 0.85 m
# Verify: sqrt(x² + y²) + z ≤ workspace
```

**Workspace Test:**
```
Approximate UR5 workspace:
- Base radius: ~0.85 m from center
- Height: 0.1 m to 1.5 m
- Full 6-DOF possible in most of this volume

If IK fails: Check position is reasonable
```

### Problem 2: "Position/Orientation Error > Tolerance"

**Causes:**
1. IK did not fully converge
2. Pose is at workspace boundary
3. Singularity present (det(J) ≈ 0)

**Solutions:**
```
1. Check error magnitude: Is it small enough for your application?
   - Error < 1e-3 m (1 mm): Usually acceptable
   - Error > 1e-2 m (1 cm): Probably unacceptable

2. Run jacobian.py for this configuration
   - If det(J) < 1e-6: Move to different pose
   - If det(J) > 1e-3: Should converge, try again

3. Increase IK iterations:
   - Edit inverse_kinematics.py
   - Change max_iter from 1000 to 5000
```

### Problem 3: "Python: Module Not Found"

**Cause:** NumPy not installed or wrong path

**Solution:**
```bash
# Install NumPy
pip install numpy

# Verify
python -c "import numpy; print(numpy.__version__)"

# If still failing, check Python path
which python  # or: where python (Windows)
```

### Problem 4: "Matrix Math Error / NaN / Inf"

**Causes:**
1. Invalid link parameters (zeroes or negatives)
2. Numerical instability
3. Singular matrix in computation

**Solutions:**
```
1. Verify link parameters:
   - All d_i and a_i > 0
   - Reasonable values (usually < 2 meters)

2. Check desired pose:
   - Avoid workspace singularities
   - Try nearby pose

3. Enable error checking:
   - Add to code: np.seterr(all='raise')
   - Will raise exception instead of NaN
```

### Problem 5: "Results Don't Match Expectations"

**Cause:** Sign error or convention mismatch

**Check:**
```
1. Angle units: All inputs in DEGREES?
   - Code converts to radians internally
   - Output shown in degrees

2. DH convention: Standard or Modified?
   - Code uses STANDARD DH
   - Some references use Modified DH (different)

3. RPY convention: Which one?
   - Code uses ZYX convention
   - Verify against your reference

4. Link parameters: Correct values?
   - Compare with robot datasheet
   - Use standard UR5 values if unsure
```

---

## EXAMPLE TEST CASES

### Test Case 1: Arm at Home Position

**Objective:** Verify FK with known configuration

**Input:**
```
Link Parameters:
d1 = 0.89159
a3 = 0.425
a4 = 0.39225
d4 = 0.13585
d5 = 0.08916
d6 = 0.0823

Joint Angles:
q1 = 0°
q2 = -90°
q3 = 90°
q4 = 0°
q5 = 0°
q6 = 0°
```

**Expected Output:**
```
Position: Approximately (0.81, 0.00, 0.89) meters
Orientation: Vertical approach (Pitch ≈ 90°)
Jacobian det: Non-zero (not singular)
```

**Verification:**
- Use forward_kinematics.py
- Observe if arm extends horizontally at home position
- Common configuration for UR robots

### Test Case 2: Simple Reach

**Objective:** Find IK for nearby pose

**Input:**
```
Same link parameters as Test 1

Desired Pose:
Position: x=0.4, y=0.0, z=0.9
Orientation: Roll=0°, Pitch=0°, Yaw=0°
```

**Expected Output:**
```
IK convergence: Within 100-300 iterations
Position error: < 1e-4 m
Orientation error: < 1e-4 rad
Validity: ✓ Valid
Determinant: > 0.01 (good dexterity)
```

**Test Steps:**
1. Run main.py
2. Input parameters above
3. Wait for solution
4. Verify "IK Solution Valid" message
5. Verify det(J) > 0.001

### Test Case 3: Singularity Detection

**Objective:** Verify singularity detection works

**Input:**
```
Same link parameters

Joint Angles (approaching singularity):
q1 = 0°
q2 = 0° (instead of -90°, causes issue)
q3 = 0°
q4 = 0°
q5 = 0°
q6 = 0°
```

**Expected Output:**
```
Jacobian: rank-deficient
Determinant: < 1e-6
Warning: ⚠ SINGULAR CONFIGURATION
```

**Test Steps:**
1. Run jacobian.py
2. Input above angles
3. Observe warning message
4. Verify det(J) is very small

### Test Case 4: IK Multiple Attempts

**Objective:** Verify multiple random initializations work

**Input:**
```
Desired Pose: Same as Test 2
```

**Expected Output:**
```
Attempt 1 or 2 or 3: Converged in X iterations
(Different number of iterations each time)
(Eventually: ✓ IK Solution Valid)
```

**Test Steps:**
1. Run main.py multiple times with SAME pose
2. Observe different iteration counts
3. All should eventually converge
4. Iterates until solution found (up to 5 attempts)

### Test Case 5: Unreachable Pose

**Objective:** Verify graceful handling of unreachable poses

**Input:**
```
Desired Pose:
Position: x=2.0, y=2.0, z=0.5 (too far)
Orientation: Roll=0°, Pitch=0°, Yaw=0°
```

**Expected Output:**
```
IK Failed to Converge (after all 5 attempts)
Message: "Failed to converge."
No solution printed
```

**Interpretation:**
- Pose outside manipulator workspace
- Correctly detected as unreachable
- Program handles gracefully (no crash)

---

## ADVANCED USAGE

### A1: Custom Link Parameters

**To model a different robot:**

```python
# In main.py, modify input section:
custom_params = {
    "d1": 1.0,      # Your value
    "a3": 0.5,      # Your value
    "a4": 0.4,      # Your value
    "d4": 0.15,     # Your value
    "d5": 0.1,      # Your value
    "d6": 0.08      # Your value
}

# Then use custom_params instead of user input
```

### A2: Batch Processing Multiple Poses

**To test many poses:**

```python
import numpy as np
from inverse_kinematics import inverse_kinematics

link_params = {...}

poses = [
    {"x": 0.5, "y": 0.0, "z": 0.9},
    {"x": 0.4, "y": 0.3, "z": 0.8},
    {"x": 0.3, "y": 0.3, "z": 0.7}
]

for pose in poses:
    # Build T_desired
    T_d = np.eye(4)
    T_d[0:3, 3] = [pose["x"], pose["y"], pose["z"]]
    
    # Solve IK
    q_sol = inverse_kinematics(T_d, link_params)
    
    if q_sol is not None:
        print(f"Pose {pose}: Converged")
        print(f"  Angles: {np.rad2deg(q_sol)}")
    else:
        print(f"Pose {pose}: Failed")
```

### A3: Sensitivity Analysis

**To understand Jacobian effect:**

```python
from jacobian import compute_jacobian

q_baseline = [0, -np.pi/2, np.pi/2, 0, 0, 0]
J_baseline = compute_jacobian(q_baseline, link_params)
det_baseline = np.linalg.det(J_baseline)

# Perturb one joint
q_perturbed = q_baseline.copy()
q_perturbed[2] += 0.1  # Slight change to q3

J_perturbed = compute_jacobian(q_perturbed, link_params)
det_perturbed = np.linalg.det(J_perturbed)

print(f"Baseline det(J): {det_baseline}")
print(f"Perturbed det(J): {det_perturbed}")
print(f"Sensitivity: {abs(det_perturbed - det_baseline)}")
```

### A4: Workspace Mapping

**To visualize reachable space:**

```python
# Test grid of angles
import numpy as np
from forward_kinematics import forward_kinematics

link_params = {...}

# Sample q3 and q4 (link angles)
q3_range = np.linspace(-np.pi, np.pi, 10)
q4_range = np.linspace(-np.pi, np.pi, 10)

positions = []

for q3 in q3_range:
    for q4 in q4_range:
        q = [0, -np.pi/2, q3, q4, 0, 0]
        T = forward_kinematics(q, link_params)
        pos = T[0:3, 3]
        positions.append(pos)

# Now positions contain reachable points
# Can be plotted using matplotlib or scipy
```

### A5: Increasing Convergence Reliability

**To make IK more robust:**

```python
# Modify in inverse_kinematics.py
def inverse_kinematics(T_desired, link_params,
                       max_iter=2000,      # Increase iterations
                       pos_tol=1e-5,       # Tighter tolerance
                       ori_tol=1e-5,
                       attempts=10):       # More attempts
    # Rest of code unchanged
```

---

## QUICK REFERENCE

### File Purposes

| File | Purpose | Input | Output |
|------|---------|-------|--------|
| main.py | Complete pipeline | Pose + parameters | Joint angles + verification |
| forward_kinematics.py | FK only | Joint angles | End-effector pose |
| inverse_kinematics.py | IK only | Desired pose | Joint angles |
| jacobian.py | Jacobian analysis | Joint angles | det(J), singularity |
| IK_verification.py | Verification | q_solution + pose | Errors + validity |
| dh_model.py | DH matrices | Frame selection | T matrices |

### Terminal Commands

```bash
# Run main pipeline (recommended)
python main.py

# Run individual modules
python forward_kinematics.py
python inverse_kinematics.py
python jacobian.py
python IK_verification.py
python dh_model.py

# Check Python version
python --version

# Check NumPy installed
python -c "import numpy; print('OK')"
```

### Common Parameter Values

```
UR5 Real:        Simplified Test:
d1 = 0.89159     d1 = 1.0
a3 = 0.425       a3 = 0.5
a4 = 0.39225     a4 = 0.5
d4 = 0.13585     d4 = 0.1
d5 = 0.08916     d5 = 0.1
d6 = 0.0823      d6 = 0.1
```

---

**END OF USER GUIDE**

*For detailed technical documentation, see DETAILED_PROJECT_REPORT.md*  
*For code analysis, see TECHNICAL_ANALYSIS.md*
