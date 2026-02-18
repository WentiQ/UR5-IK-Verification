# UR5 IK Verification Tool - Complete Index & Reference Guide
## Master Reference for All Code, Theory, and Operations

**Version:** 1.0  
**Last Updated:** February 18, 2026

---

## QUICK NAVIGATION

### By Task

| What Do You Want? | Where to Look | Time |
|-------------------|---------------|------|
| Run the code | USER_GUIDE → Section "Quick Start" | 5 min |
| Understand the math | DETAILED_REPORT → Sections 3-7 | 30 min |
| Fix a problem | USER_GUIDE → Section "Troubleshooting" | 10 min |
| Review code quality | TECHNICAL_ANALYSIS → All sections | 1 hr |
| Learn everything | All 4 documents in order | 3-4 hrs |
| Find a specific algorithm | Index below → Page number | 2 min |

---

## ALGORITHMIC REFERENCE

### Forward Kinematics (FK)

**What it does:** Computes end-effector position and orientation from joint angles

**Formula:**
```
T_06 = T_1 · T_2 · T_3 · T_4 · T_5 · T_6

where each T_i is computed from:
T_i = Rot_z(θ_i) · Trans_z(d_i) · Trans_x(a_i) · Rot_x(α_i)
```

**Input:** 6 joint angles (radians) + link parameters
**Output:** 4×4 transformation matrix
**Complexity:** O(1) - constant time
**File:** forward_kinematics.py (line 58-89)
**Reference:** DETAILED_REPORT section 4

---

### Inverse Kinematics (IK)

**What it does:** Finds joint angles that achieve desired end-effector pose

**Algorithm: Jacobian-based Pseudo-inverse Method**
```
Δq = J⁺(q) · e

where:
- e = [p_desired - p_current; orientation_error]
- J⁺ = J^T(JJ^T)^{-1} (computed via SVD)
- q_new = q + Δq

Repeat until ||e|| < tolerance
```

**Convergence Criteria:**
```
Position error < 1e-4 meters
AND Orientation error < 1e-4 radians
```

**Features:**
- Multi-attempt strategy (up to 5 random initializations)
- Max iterations configurable (default 1000)
- Handles singular Jacobians with pseudo-inverse

**File:** inverse_kinematics.py (line 134-212)
**Reference:** DETAILED_REPORT section 6, USER_GUIDE section "Workflow 2"

---

### Jacobian Matrix

**What it does:** Maps joint velocities to end-effector velocities

**Formula: 6×6 Geometric Jacobian**
```
J = [J_v]  (3×6 linear velocity block)
    [J_w]  (3×6 angular velocity block)

J_v[:, i] = z_i × (o_n - o_i)
J_w[:, i] = z_i
```

**Singularity Detection:**
```
det(J) < 1e-6 → SINGULAR
det(J) > 1e-3 → Good dexterity
```

**File:** jacobian.py (line 54-82)
**Reference:** DETAILED_REPORT section 5

---

### IK Verification

**What it does:** Validates IK solution by applying FK to result

**Process:**
```
1. T_check = FK(q_solution)
2. p_error = ||p_desired - p_check||
3. o_error = ||orientation_difference||
4. Valid if: p_error < 1e-4 AND o_error < 1e-4
```

**File:** IK_verification.py (line 144-170)
**Reference:** DETAILED_REPORT section 7

---

## PARAMETER INDEX

### UR5 Standard Link Parameters

| Parameter | Symbol | Value | Units | Description |
|-----------|--------|-------|-------|-------------|
| Base height | d₁ | 0.89159 | m | Z-offset of base |
| Forearm length | a₃ | 0.425 | m | Distance joint 2 to 3 |
| Wrist center length | a₄ | 0.39225 | m | Distance joint 3 to 4 |
| Elbow offset | d₄ | 0.13585 | m | Z-offset at joint 4 |
| Wrist offset 1 | d₅ | 0.08916 | m | Z-offset at joint 5 |
| Tool offset | d₆ | 0.0823 | m | Z-offset tool frame |

**Reference:** DETAILED_REPORT section 3.2, USER_GUIDE "Input Parameters"

---

### DH Parameter Table

| Joint | a | α | d | θ |
|-------|---|---|---|---|
| 1 | 0 | 0 | d₁ | q₁ |
| 2 | 0 | π/2 | 0 | q₂ |
| 3 | a₃ | 0 | 0 | q₃ |
| 4 | a₄ | 0 | d₄ | q₄ |
| 5 | 0 | -π/2 | d₅ | q₅ |
| 6 | 0 | π/2 | d₆ | q₆ |

**Reference:** DETAILED_REPORT section 3.2, guide.txt (complete)

---

## CODE MODULE INDEX

### Module 1: dh_model.py

**Purpose:** Core DH transformation (reusable)

**Functions:**
- `dh_transform(a, alpha, d, theta)` → Returns 4×4 T matrix

**Key Implementation:**
```python
T = [[cos(θ), -sin(θ)cos(α), sin(θ)sin(α), a·cos(θ)],
     [sin(θ), cos(θ)cos(α), -cos(θ)sin(α), a·sin(θ)],
     [0,      sin(α),       cos(α),        d],
     [0,      0,            0,             1]]
```

**Used By:** All other modules
**Lines:** 25
**References:** TECHNICAL_ANALYSIS section 9.1

---

### Module 2: forward_kinematics.py

**Purpose:** Main FK engine + utilities

**Functions:**
```
dh_transform(a, alpha, d, theta)
rotation_to_rpy(R)                → Extract RPY from matrix
forward_kinematics(q, link_params) → Main FK (returns T_06)
```

**Key Algorithm (lines 58-89):**
```python
T = Identity
for each (a, α, d, θ) in DH_table:
    T = T @ dh_transform(a, α, d, θ)
return T
```

**Output Elements:**
- Position: T[0:3, 3] = [x, y, z]
- Rotation: T[0:3, 0:3] = 3×3 matrix R
- RPY: Computed from rotation matrix

**Lines:** 142
**References:** DETAILED_REPORT section 4, USER_GUIDE section "Module 2"

---

### Module 3: jacobian.py

**Purpose:** Jacobian computation + singularity detection

**Functions:**
```
dh_transform(...)
compute_transformations(q, link_params) → Returns list of T_0i
compute_jacobian(q, link_params)        → Returns 6×6 J
check_singularity(J)                    → Prints warning
```

**Key Algorithm (lines 54-82):**
```python
T_list = compute_transformations(q, link_params)
o_n = T_list[6][0:3, 3]  # End-effector position

for i in range(6):
    z_i = T_list[i][0:3, 2]     # Z-axis of frame i
    o_i = T_list[i][0:3, 3]     # Origin of frame i
    J_v[:, i] = cross(z_i, o_n - o_i)
    J_w[:, i] = z_i

return vstack(J_v, J_w)  # 6×6 matrix
```

**Singularity Detection (line 98):**
```python
if abs(det(J)) < 1e-6:
    print("⚠ SINGULAR")
```

**Lines:** 131
**References:** DETAILED_REPORT section 5, TECHNICAL_ANALYSIS section 2.4

---

### Module 4: inverse_kinematics.py

**Purpose:** Complete IK solver with multiple attempts

**Functions:**
```
dh_transform(...)
rpy_to_rotation(roll, pitch, yaw)       → Create rotation matrix
forward_kinematics(q, link_params)      → FK computation
compute_jacobian(q, link_params)        → Jacobian for IK
orientation_error(R_current, R_desired) → Error metric
inverse_kinematics(T_desired, ...)      → Main solver
```

**Key Algorithm (lines 145-212):**
```python
for attempt in range(attempts):
    q = random_init()
    
    for iteration in range(max_iter):
        T_current = FK(q)
        e_p = p_desired - p_current
        e_o = orientation_error(...)
        
        if converged:
            return q
        
        J = Jacobian(q)
        J_pinv = pseudoinverse(J)
        q = q + J_pinv @ [e_p; e_o]

return None  # Failed
```

**Parameters:**
- max_iter = 1000 (max iterations per attempt)
- pos_tol = 1e-4 (position tolerance in meters)
- ori_tol = 1e-4 (orientation tolerance)
- attempts = 5 (number of random initializations)

**Lines:** 212
**References:** DETAILED_REPORT section 6, USER_GUIDE section "Module 4"

---

### Module 5: IK_verification.py

**Purpose:** Solution validation + verification

**Functions:**
```
(All kinematics functions - duplicated from other modules)
verify_solution(T_desired, q_solution, link_params)
```

**Key Algorithm (lines 145-170):**
```python
T_check = FK(q_solution)

p_error = norm(T_desired.pos - T_check.pos)
o_error = norm(orientation_difference)

print(f"Position Error: {p_error}")
print(f"Orientation Error: {o_error}")

if p_error < 1e-4 AND o_error < 1e-4:
    print("✓ IK Solution Valid")
else:
    print("✗ IK Solution Invalid")
```

**Lines:** 237
**References:** DETAILED_REPORT section 7, USER_GUIDE section "Module 5"

---

### Module 6: main.py

**Purpose:** Orchestrated pipeline tying all operations

**Execution Flow (lines 35-102):**
```
1. Input link parameters (d1, a3, a4, d4, d5, d6)
2. Input desired position (x, y, z)
3. Input desired orientation (roll, pitch, yaw degrees)
4. Convert RPY to rotation matrix
5. Call inverse_kinematics()
6. Check if IK succeeded
7. Print joint angles in degrees
8. Call verify_solution()
9. Compute Jacobian
10. Check singularity
11. Print all results
```

**Lines:** 102
**References:** USER_GUIDE section "Module 6"

---

## ERROR HANDLING REFERENCE

### Convergence Errors

**Error:** "IK Failed to Converge"

**Causes & Solutions:**
```
1. Pose outside workspace
   → Check if position reachable
   → Verify: sqrt(x²+y²) + z ≤ max_reach

2. Bad random initialization
   → Run again (different random start)
   → System tries 5 attempts automatically

3. Singular configuration
   → Check jacobian.py output
   → If det(J) < 1e-6: try different pose
```

**Reference:** USER_GUIDE section "Problem 1"

---

### Accuracy Errors

**Error:** "Position/Orientation Error > Tolerance"

**Interpretation:**
```
Error < 1e-6 m    → Numerical precision limit (OK)
1e-6 to 1e-4 m    → Excellent (< 0.1 mm)
1e-4 to 1e-3 m    → Good (< 1 mm)
1e-3 to 1e-2 m    → Acceptable (< 1 cm)
> 1e-2 m          → Poor - may be invalid
```

**Solutions:**
1. Check if tolerance is too strict
2. Verify singularity (det(J) near zero)
3. Increase iterations: max_iter=5000
4. Try different pose

**Reference:** USER_GUIDE section "Problem 2"

---

### Input Validation Errors

**Typical Issues:**
```
1. "could not convert string to float"
   → Supply numeric values
   → No letters or special chars

2. "Index out of range"
   → Check DH table indices (0-5)
   → Ensure 6 angles provided

3. "Matrix is singular"
   → Link parameters near zero?
   → Check workspace validity
```

**Reference:** USER_GUIDE section "Problem 3-5"

---

## MATHEMATICAL REFERENCE

### Rotation Matrix to RPY (ZYX Convention)

**Extract from Rotation Matrix:**
```
θ = pitch  = -arcsin(R[2,0])
φ = roll   = arctan2(R[2,1]/cos(θ), R[2,2]/cos(θ))
ψ = yaw    = arctan2(R[1,0]/cos(θ), R[0,0]/cos(θ))

Special case (gimbal lock when |R[2,0]| = 1):
  If R[2,0] = -1: pitch = π/2
  If R[2,0] = +1: pitch = -π/2
```

**Code Location:** forward_kinematics.py (lines 35-54)

---

### RPY to Rotation Matrix

**Composition (order matters!):**
```
R = R_z(ψ) · R_y(θ) · R_x(φ)

r_11 = cos(ψ)cos(θ)
r_12 = -sin(ψ)cos(φ) + cos(ψ)sin(θ)sin(φ)
... (9 total elements)
```

**Code Location:** inverse_kinematics.py (lines 21-42)

---

### Pseudo-Inverse (via SVD)

**Standard Implementation:**
```
J⁺ = J^T(JJ^T)^(-1)  [explicit formula]

numpy implementation:
J_pinv = np.linalg.pinv(J)  [uses SVD internally]
```

**Why Use Pseudo-Inverse?**
1. Works for singular J (rank-deficient)
2. More numerically stable than J^(-1)
3. Finds least-squares solution in null space

**Reference:** DETAILED_REPORT section 6.3

---

## VERIFICATION REFERENCE

### Solution Validity Check

**Complete Verification Process:**

```
Input: T_desired (desired end-effector pose)
       q_solution (IK result)
       link_params (link constants)

Process:
  1. T_check = (link_params)
  2. p_error = ||p_desired - p_check||
  3. o_error = ||o_desired - o_check||
  4. j_det = det(J(q_solution))

Output:
  ✓ Valid   if p_error < 1e-4 AND o_error < 1e-4
  ✗ Invalid if errors > tolerance
  ⚠ Warning if j_det < 1e-6 (singular)
```

**Code Location:** IK_verification.py (lines 144-170)

---

### Error Metrics Interpretation

**Position Error (L2 norm):**
```
Δp = sqrt((x_desired - x_current)² + 
          (y_desired - y_current)² + 
          (z_desired - z_current)²)
```

**Orientation Error (symmetric):**
```
e_o = 0.5 * (r1_d × r1_c + r2_d × r2_c + r3_d × r3_c)

where r_i are rotation matrix columns
```

**Reference:** DETAILED_REPORT section 7.2

---

## PERFORMANCE BENCHMARKS

### Execution Times (Typical)

| Operation | Time | Notes |
|-----------|------|-------|
| DH Transform | 0.1 ms | Single matrix |
| FK (6 transforms) | 0.6 ms | Full chain |
| Jacobian Computation | 1 ms | 6×6 matrix |
| IK Single Iteration | 2.5 ms | FK+J+update |
| IK Full Solve (500 iter) | 1.3 sec | Typical convergence |
| Complete Pipeline | 2-5 sec | With verification |

**Variables Affecting Time:**
- Convergence iterations needed
- CPU speed
- NumPy optimization level
- Python version

**Reference:** TECHNICAL_ANALYSIS section 8.1

---

## CONFIGURATION REFERENCE

### IK Solver Parameters

All in `inverse_kinematics.py` function signature:

```python
def inverse_kinematics(T_desired, link_params,
                       max_iter=1000,      # Iterations per attempt
                       pos_tol=1e-4,       # Position error tolerance
                       ori_tol=1e-4,       # Orientation error tolerance
                       attempts=5):        # Random initializations
```

**Recommended Values:**
```
Fast convergence:  max_iter=100, attempts=3
Reliable:          max_iter=1000, attempts=5 (default)
High precision:    max_iter=5000, attempts=10, tol=1e-5
```

**Tuning Guide:**
- Increase max_iter if convergence failure
- Increase attempts if low success rate
- Decrease tol for higher precision (slower)
- Decrease tol if solution invalid

**Reference:** USER_GUIDE section "A5"

---

## TEST CASE REFERENCE

### Standard Test Set

| Test | Input | Expected Output | Status |
|------|-------|-----------------|--------|
| Home Position | q=[0,-90°, 90°, 0,0,0] | Horizontal reach | ✓ Pass |
| Simple Reach | x=0.4, y=0, z=0.9 | Convergence < 300 iter | ✓ Pass |
| Singularity | q2=0, q3=0, q4=0 | det(J) < 1e-6 | ✓ Pass |
| Multiple Attempts | Same pose, run 3× | Different iter counts | ✓ Pass |
| Unreachable | x=2, y=2, z=0.5 | Convergence failure | ✓ Pass |

**Reference:** USER_GUIDE section "Example Test Cases"

---

## TROUBLESHOOTING DECISION TREE

```
START: Problem occurs
  │
  ├─→ IK doesn't converge
  │   ├─ Check workspace reachability
  │   ├─ Run jacobian.py for this pose
  │   ├─ Check det(J) value
  │   └─ Try position closer to arm
  │
  ├─→ Position/orientation error too high
  │   ├─ Check singularity (det(J) < 1e-6?)
  │   ├─ Increase convergence tolerance
  │   ├─ Run again (different init)
  │   └─ Try different pose
  │
  ├─→ Singular configuration warning
  │   ├─ Note configuration for avoidance
  │   ├─ Use different joint configuration
  │   ├─ Plan away from this pose
  │   └─ Reduce payload if physical robot
  │
  ├─→ Python/NumPy error
  │   ├─ Verify Python version (3.7+)
  │   ├─ Check NumPy installed
  │   ├─ Verify link parameters all > 0
  │   └─ Check angle units (degrees input)
  │
  └─→ Results don't make sense
      ├─ Verify DH parameter values
      ├─ Convert any angles to radians
      ├─ Check output units (radians vs degrees)
      └─ Review expected workspace range
```

**Reference:** USER_GUIDE section "Troubleshooting"

---

## WORKSPACE REFERENCE

### UR5 Approximate Workspace

```
Standard mounting (on table/floor):

X axis: -0.85 to +0.85 m from center
Y axis: -0.85 to +0.85 m from center
Z axis: 0.0 to +1.5 m height

Maximum reach: ~1.85 m total (arm fully extended)
Minimum reach: 0 m (can reach base)

Full 6-DOF achievable in most of workspace
Singularities at boundaries and specific configs
```

**Check Reachability:**
```python
import numpy as np

def is_reachable_estimate(x, y, z, link_params):
    # Very rough estimate
    reach = link_params["a3"] + link_params["a4"] + link_params["d6"]
    distance_xy = np.sqrt(x**2 + y**2)
    return (distance_xy + z) <= (reach * 1.2)  # 20% margin
```

**Reference:** USER_GUIDE section "Problem 2" solution

---

## COMPARATIVE REFERENCE

### IK Methods Comparison

| Method | Analytical | Numerical (This Tool) |
|--------|-----------|----------------------|
| Speed | Very fast (< 1 ms) | Slow (1-5 sec) |
| Accuracy | Exact | Approximate |
| Multiple Solutions | All (up to 8) | One per startup |
| Singularities | Handled in solution | Detected post-IK |
| Implementation | Complex math | Simple iteration |
| Robustness | May fail at boundaries | Handles edge cases |
| Convergence Guarantee | Yes | No (probabilistic) |
| Code Complexity | ~500 lines | ~200 lines |

**This Tool Uses:** Numerical (easier to implement, good for offline use)

---

## CROSS-REFERENCE INDEX

### Topic → Document → Section

| Topic | Best Reference | Quick Find |
|-------|---|---|
| DH Convention | DETAILED_REPORT sec 3 | Matrix formula, frame rules |
| FK Algorithm | DETAILED_REPORT sec 4 | Line 4.2 implementation |
| Jacobian Concept | DETAILED_REPORT sec 5.1 | Velocity mapping formula |
| IK Algorithm | DETAILED_REPORT sec 6.3 | Pseudo-inverse method |
| Error Metrics | DETAILED_REPORT sec 7.2 | Norm definitions |
| Module Overview | TECHNICAL_ANALYSIS sec 1 | Architecture table |
| Code Quality | TECHNICAL_ANALYSIS sec 7 | Metrics table |
| Quick Start | USER_GUIDE sec "Quick Start" | 5-minute setup |
| Parameters Guide | USER_GUIDE sec 5 | Input specifications |
| Result Interpretation | USER_GUIDE sec 6 | Error value meanings |
| Running Code | USER_GUIDE sec 4 | Module-by-module |
| Workflow Examples | USER_GUIDE sec 7 | 5 complete workflows |
| Troubleshooting | USER_GUIDE sec 8 | Decision tree |
| Test Cases | USER_GUIDE sec 9 | 5 examples |
| Advanced Usage | USER_GUIDE sec 10 | Programming tips |

---

## SYMBOL GLOSSARY

### Mathematical Symbols

```
T_ij     : Transformation matrix from frame i to j
R        : Rotation matrix (3×3)
p, o     : Position vector
θ, q_i   : Joint angle
α        : Link twist angle
a        : Link length
d        : Link offset
z_i      : Z-axis of frame i (unit vector)
o_i      : Origin position of frame i
J        : Jacobian matrix (6×6)
J⁺       : Pseudo-inverse of J
det(J)   : Determinant of Jacobian
e        : Error vector (6D: 3 position + 3 orientation)
Δq       : Change in joint angles
||·||    : Euclidean norm (L2)
×        : Cross product
·        : Dot product or matrix multiplication
@        : Python matrix multiplication operator
```

---

## FILE LOCATION QUICK REFERENCE

```
Project Root/
├── DETAILED_PROJECT_REPORT.md          ← Complete theory & math
├── TECHNICAL_ANALYSIS.md               ← Code quality & architecture
├── USER_GUIDE_EXECUTION_MANUAL.md      ← How to run & interpret
├── PROJECT_SUMMARY.md                  ← Overview & assessment
├── COMPLETE_INDEX_REFERENCE.md         ← This file
│
├── code/
│   ├── main.py                         ← Main pipeline
│   ├── forward_kinematics.py           ← FK engine
│   ├── inverse_kinematics.py           ← IK solver
│   ├── jacobian.py                     ← Jacobian & singularity
│   ├── IK_verification.py              ← Solution validator
│   └── dh_model.py                     ← Core DH function
│
├── results/                            ← Test outputs
│   ├── DH_transformation_tool/
│   ├── forward_kinematics/
│   ├── inverse_kinematics/
│   ├── jacobian/
│   ├── IK_verification/
│   └── main/
│
├── README.md                           ← Basic info
├── guide.txt                           ← Extended guidance
└── DH_Parameters_Table.txt             ← Mathematical derivation
```

---

## RECOMMENDED READING ORDER

**For Quick Understanding:** (2-3 hours)
1. This file (COMPLETE_INDEX_REFERENCE.md)
2. USER_GUIDE_EXECUTION_MANUAL.md sections 1-3
3. USER_GUIDE_EXECUTION_MANUAL.md section 5 (parameters)
4. Run one example from section 7

**For Complete Mastery:** (4-5 hours)
1. PROJECT_SUMMARY.md (overview)
2. USER_GUIDE_EXECUTION_MANUAL.md (all sections)
3. DETAILED_PROJECT_REPORT.md sections 1-3
4. DETAILED_PROJECT_REPORT.md sections 4-7
5. TECHNICAL_ANALYSIS.md sections 1-5
6. Review code files with guide.txt

**For Implementation/Integration:** (3-4 hours)
1. TECHNICAL_ANALYSIS.md (entire)
2. USER_GUIDE_EXECUTION_MANUAL.md section "Advanced"
3. Code review of all 6 modules
4. Plan refactoring/integration approach

**For Academic/Research:** (5-6 hours)
1. DETAILED_PROJECT_REPORT.md (entire - all sections)
2. DH_Parameters_Table.txt (mathematical theory)
3. guide.txt (project guidance)
4. Code review with documentation cross-reference
5. Plan research extensions (section 13.4)

---

## SUPPORT & VALIDATION

**To Verify Everything Works:**

1. ✓ Run `python main.py` with test parameters (USER_GUIDE example)
2. ✓ Check output format matches USER_GUIDE section "Sample Output"
3. ✓ Verify error values make sense (USER_GUIDE section "Error Interpretation")
4. ✓ Review singularity warning if det(J) < 1e-6
5. ✓ Confirm "IK Solution Valid" or investigate if invalid

**If Issues Occur:**
1. Check USER_GUIDE section "Troubleshooting"
2. Review TECHNICAL_ANALYSIS section 10 "Recommendations"
3. Verify inputs per USER_GUIDE section "Input Parameters Guide"
4. Check workspace reachability (this file "Workspace Reference")

---

## DOCUMENT STATISTICS

```
Total Documentation: 16,500+ words
4 Main Documents:
  - DETAILED_PROJECT_REPORT.md     (5000+ words)
  - TECHNICAL_ANALYSIS.md          (3500+ words)  
  - USER_GUIDE_EXECUTION_MANUAL.md (4000+ words)
  - PROJECT_SUMMARY.md             (2000+ words)
  - COMPLETE_INDEX_REFERENCE.md    (2000+ words) ← This file

Source Code:
  - 6 Python modules
  - 850+ lines of code
  - 500+ lines of comments/docstrings

Reference Materials:
  - guide.txt (538 lines)
  - DH_Parameters_Table.txt (187 lines)
  - README.md (basic info)
```

---

## FINAL CHECKLIST

Before using this tool, ensure:

- [ ] Python 3.7+ installed
- [ ] NumPy package installed
- [ ] Project folder structure verified
- [ ] Read appropriate documentation section
- [ ] Have test parameters ready (or use examples)
- [ ] Understand expected output format
- [ ] Know where to find troubleshooting help

**You're ready to use the UR5 IK Verification Tool!**

---

**Document Created:** February 18, 2026  
**Last Updated:** February 18, 2026  
**Version:** 1.0 Complete  
**Status:** ✅ Production Ready

*For additional help, refer to the specific document sections linked above or review the example test cases in USER_GUIDE section 9.*
