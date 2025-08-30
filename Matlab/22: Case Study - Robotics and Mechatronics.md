# 🤖 Session 22: Case Study - Robotics and Mechatronics

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Apply control systems to articulated robotic systems, focusing on kinematics and trajectory tracking. This session teaches students to model robot arms, compute kinematics, and design advanced controllers like Model Predictive Control (MPC) for precise motion in mechatronic applications. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: 
- Control systems knowledge (state-space, PID, LQR from previous sessions).
- Linear algebra and differential equations.
- Python with libraries: NumPy, SciPy, Matplotlib, and `control` (install via `pip install control` if needed).

**📚 Resources**:
- 🌐 Python Robotics Tutorials ([wiki.ros.org](https://wiki.ros.org))
- 📖 “Introduction to Robotics: Mechanics and Control” by John J. Craig (Chapters on kinematics and control)
- 🔗 Online: Robotics Academy ([roboticsacademy.fiu.edu](https://roboticsacademy.fiu.edu)), X posts on robot control (#Robotics #Mechatronics)

---

## 🗂️ Topics Covered

1. 📐 **Forward Kinematics**: Compute end-effector position from joint angles.
2. 🔄 **Inverse Kinematics**: Determine joint angles for a desired end-effector position.
3. 🛤️ **Trajectory Tracking**: Plan and follow paths using controllers.
4. 🎮 **Controller Design**: Implement Model Predictive Control (MPC) for optimal tracking.
5. 📊 **Practical Applications**: Robotics in manufacturing, automation, and mechatronics.

---

## 📝 Detailed Lecture Content

### 📐 1. Forward Kinematics
Forward kinematics calculates the position and orientation of a robot's end-effector given joint parameters. 🤖

- **🔍 Why It Matters**: Essential for simulation and path planning. In mechatronics, it predicts tool positions for tasks like welding or assembly.
- **🛠️ Methods**: Use transformation matrices (Denavit-Hartenberg parameters) or geometric equations for simple arms.
- **🌍 Real-World Example**: In a 2-DOF planar arm, compute Cartesian coordinates from joint angles for pick-and-place operations.

**📐 Mathematical Model** (2-DOF Planar Arm):
- Links: \(l_1, l_2\)
- End-effector position:
  \[
  x = l_1 \cos(\theta_1) + l_2 \cos(\theta_1 + \theta_2)
  \]
  \[
  y = l_1 \sin(\theta_1) + l_2 \sin(\theta_1 + \theta_2)
  \]

**💬 Explanation**: These equations map joint space to task space. For more DOF, use homogeneous transformations.

---

### 🔄 2. Inverse Kinematics
Inverse kinematics finds joint angles for a given end-effector position, often non-unique or singular. 🔍

- **🔍 Why It Matters**: Enables robots to reach specific points. Challenges include multiple solutions or unreachable poses.
- **🛠️ Methods**: Closed-form solutions for simple robots; numerical methods (e.g., Jacobian inverse) for complex ones.
- **🌍 Real-World Example**: A surgical robot solving for angles to position a tool at a target tissue.

**📐 Mathematical Model** (2-DOF Planar Arm):
- Given (x, y), solve:
  \[
  \theta_2 = \pm \cos^{-1} \left( \frac{x^2 + y^2 - l_1^2 - l_2^2}{2 l_1 l_2} \right)
  \]
  \[
  \theta_1 = \tan^{-1} \left( \frac{y}{x} \right) - \tan^{-1} \left( \frac{l_2 \sin(\theta_2)}{l_1 + l_2 \cos(\theta_2)} \right)
  \]

**💬 Explanation**: Two solutions (elbow up/down). Check workspace limits to avoid singularities.

---

### 🛤️ 3. Trajectory Tracking
Trajectory tracking ensures the robot follows a planned path smoothly, accounting for dynamics. 🚀

- **🔍 Why It Matters**: Critical for dynamic tasks like drawing or following conveyor belts. Errors lead to inaccuracies or collisions.
- **🛠️ Methods**: Interpolation (linear, cubic splines), feedback control to correct deviations.
- **🌍 Real-World Example**: Autonomous vehicles tracking GPS waypoints or industrial arms following assembly paths.

**📐 Mathematical Model**:
- Path: Parametric functions, e.g., circle: \(x = r \cos(t), y = r \sin(t)\)
- Tracking error: \(e = \| \mathbf{p}_{desired} - \mathbf{p}_{actual} \|\)

**💬 Explanation**: Combine kinematics with dynamics (e.g., torque control) for accurate tracking.

---

### 🎮 4. Controller Design
Design controllers to achieve precise trajectory tracking in articulated systems. 🕹️

- **MPC Overview**: Model Predictive Control optimizes future actions over a horizon, subject to constraints.
  - Predict states using model.
  - Minimize cost (e.g., tracking error + control effort).
  - Apply first optimal input, repeat.
- **Advantages**: Handles constraints (joint limits, velocities); optimal for nonlinear systems.

**💬 Explanation**: For robotics, MPC uses kinematics/dynamics model to predict and correct paths in real-time.

---

### 📊 5. Practical Applications
- **Manufacturing**: Robot arms for welding, painting.
- **Mechatronics**: Integrated systems like drones or prosthetics.
- **Challenges**: Sensor noise, uncertainties; use AI for adaptive control (e.g., via X discussions on #AIinRobotics).

---

## 💻 Code Implementation (Combined Example)
Simulate a 2-DOF robot arm, compute kinematics, and implement simple MPC for trajectory tracking.

```python
# Robotics_MPC.py
# 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
# Simulate 2-DOF arm with MPC for circle trajectory

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# 📐 Kinematics Functions
def forward_kin(theta, l1=1.0, l2=1.0):
    theta1, theta2 = theta
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return np.array([x, y])

def inverse_kin(pos, l1=1.0, l2=1.0):
    x, y = pos
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(cos_theta2) > 1:
        return None  # Unreachable
    theta2 = np.arccos(cos_theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
    return np.array([theta1, theta2])  # Elbow up solution

# 🛤️ Trajectory: Circle
def desired_trajectory(t, r=1.0):
    return r * np.cos(t), r * np.sin(t)

# 🎮 Simple MPC for Tracking
def mpc_cost(u, *args):
    theta_current, pos_desired, N, dt, Q, R = args
    cost = 0
    theta = theta_current.copy()
    for i in range(N):
        # Simple Euler integration (assume velocity model: dtheta/dt = u)
        theta += u[2*i:2*(i+1)] * dt
        pos_pred = forward_kin(theta)
        error = pos_pred - pos_desired[:, i]
        cost += np.dot(error.T, Q @ error) + np.dot(u[2*i:2*(i+1)].T, R @ u[2*i:2*(i+1)])
    return cost

def mpc_controller(theta_current, pos_desired, N=5, dt=0.1, bounds=None):
    Q = np.eye(2)  # Error weight
    R = 0.01 * np.eye(2)  # Control weight
    u0 = np.zeros(2 * N)  # Initial guess
    args = (theta_current, pos_desired, N, dt, Q, R)
    res = minimize(mpc_cost, u0, args=args, bounds=bounds, method='SLSQP')
    return res.x[:2]  # First control action

# 🔄 Simulation
T = 2 * np.pi  # Full circle
dt = 0.1
t_steps = np.arange(0, T, dt)
theta0 = inverse_kin(desired_trajectory(0))
theta = theta0.copy()
positions = []
controls = []

for t in t_steps:
    pos_des = np.array([desired_trajectory(t + i*dt) for i in range(5)]).T  # Horizon positions
    u = mpc_controller(theta, pos_des, N=5, dt=dt, bounds=[(-1,1)]*10)  # Velocity limits
    theta += u * dt
    positions.append(forward_kin(theta))
    controls.append(u)

positions = np.array(positions)

# 📉 Plot Results
plt.figure(figsize=(10,6))
plt.plot([p[0] for p in positions], [p[1] for p in positions], 'b-', label='Actual Path')
plt.plot(np.cos(t_steps), np.sin(t_steps), 'r--', label='Desired Path')
plt.xlabel('X'); plt.ylabel('Y')
plt.title('Trajectory Tracking with MPC')
plt.legend(); plt.grid(True); plt.axis('equal')
plt.show()
```

**💬 Explanation**:
- **Kinematics**: Functions for forward/inverse mapping.
- **Trajectory**: Circle path for tracking.
- **MPC**: Minimizes quadratic cost over horizon using `scipy.optimize`; simple velocity model.
- **Simulation**: Applies MPC repeatedly, plots actual vs. desired path.
- Save as `Robotics_MPC.py` and run to see tracking performance.

---

## 🛠️ Exercise
**Task**: Simulate and control a 2-DOF robot arm with MPC.
1. Run the script and observe tracking accuracy.
2. Tune MPC parameters (N, Q, R) for better performance (e.g., reduce error).
3. Modify trajectory to a line or square; compute inverse kinematics for start/end.
4. Add constraints (e.g., joint limits in bounds).
5. Write a 1-page report analyzing results and discussing MPC advantages over PID.

**💻 Solution Hint**:
```python
# Tuning Example
Q = 2 * np.eye(2)  # Increase error weight
# Re-run MPC in loop
```

**💬 Explanation**:
- Increase horizon N for better prediction, but higher computation.
- Compare with open-loop (no control) to see improvement.

---

## 📌 Additional Notes
- **✅ Best Practices**:
  - Vectorize computations with NumPy for efficiency.
  - Comment code with `#` for clarity.
  - Use virtual environments for library management. 📂
- **🔍 Debugging Tips**:
  - Print intermediate theta/pos in MPC cost function.
  - Check if inverse_kin returns valid angles.
- **🚀 Extensions**:
  - Include dynamics (inertia, torque) using state-space.
  - Explore ROS (Robot Operating System) for real hardware.
- **🖥️ Lab Setup**: Ensure SciPy installed. Test simple optimizations first.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📐 Compute forward and inverse kinematics for robot arms.
2. 🔄 Solve inverse kinematics problems.
3. 🛤️ Design trajectories and track them.
4. 🎮 Implement MPC for optimal control.
5. 📊 Apply concepts to mechatronic systems.

**📅 Next Session Preview**: Case Study - Autonomous Vehicles: Sensor fusion and path planning. 🛠️

**📝 Assignment**: Complete the exercise, submit the modified script, and provide a 1-page report on MPC tuning results and real-world applications. Due by next session. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 31, 2025*
