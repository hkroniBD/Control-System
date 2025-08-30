# 🚗 Session 23: Case Study - Automotive and EV Control

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Explore vehicle dynamics and apply control systems to automotive and electric vehicle (EV) systems. This session focuses on modeling vehicle behavior, implementing safety controls like ABS and traction control, and managing EV-specific systems like batteries, while designing controllers for efficient and stable operation. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: 
- Control systems fundamentals (PID, state-space from prior sessions).
- Basic physics (mechanics, dynamics).
- Python with libraries: NumPy, SciPy, Matplotlib, and `control` (install via `pip install control`).

**📚 Resources**:
- 🌐 Python Control Systems Library ([python-control.readthedocs.io](https://python-control.readthedocs.io))
- 📖 “Vehicle Dynamics and Control” by Rajesh Rajamani (Chapters on ABS, traction, and EV control)
- 🔗 Online: SAE International ([www.sae.org](https://www.sae.org)), X posts on EV control (#ElectricVehicles #AutomotiveControl)

---

## 🗂️ Topics Covered

1. 🚀 **Vehicle Dynamics**: Longitudinal and lateral models for motion control.
2. 🛑 **Anti-lock Braking System (ABS)**: Prevent wheel lockup during braking.
3. 🛣️ **Traction Control**: Maintain grip during acceleration.
4. 🔋 **Battery Management System (BMS)**: Monitor and control EV batteries.
5. 🎮 **EV Controller Design**: Regulate speed and efficiency.

---

## 📝 Detailed Lecture Content

### 🚀 1. Vehicle Dynamics
Vehicle dynamics model the motion of cars and EVs, including forces like traction, drag, and gravity. 🚘

- **🔍 Why It Matters**: Basis for control systems in autonomous driving and EVs. EVs have unique dynamics due to instant torque from electric motors.
- **🛠️ Models**:
  - **Longitudinal**: Speed control, e.g., \( m \dot{v} = F_t - F_d - F_r - m g \sin(\theta) \), where \(F_t\) is traction force, \(F_d\) aerodynamic drag, \(F_r\) rolling resistance, \(\theta\) road incline.
  - **Lateral**: Steering and handling, e.g., bicycle model for yaw rate and sideslip.
- **🌍 Real-World Example**: In EVs, regenerative braking adds negative torque for energy recovery.

**📐 Mathematical Model** (Simplified Longitudinal EV):
- Motor torque: \(\tau = K_t i\) (current-controlled).
- Wheel speed: \(\omega = v / r_w\).
- Dynamics: \(\dot{v} = (\tau / r_w - F_{res}) / m\).

**💬 Explanation**: These equations simulate acceleration/braking. For EVs, include battery voltage drop under load.

---

### 🛑 2. Anti-lock Braking System (ABS)
ABS modulates brake pressure to prevent wheel lockup, maximizing friction. 🛑

- **🔍 Why It Matters**: Improves stopping distance and steering control on slippery surfaces. In EVs, integrates with regenerative braking.
- **🛠️ Methods**: Use wheel slip ratio \(\lambda = (\omega v - v)/v\), target \(\lambda \approx 0.2\) for peak friction (Burckhardt model).
- **🌍 Real-World Example**: During emergency stops, ABS pulses brakes to avoid skids.

**📐 Mathematical Model**:
- Slip dynamics: \(\dot{\lambda} = f(\lambda, \mu(\lambda))\), where \(\mu(\lambda)\) is tire-road friction.
- Control: Bang-bang or PID on brake pressure to track optimal slip.

**💬 Explanation**: Monitors wheel speeds via sensors; controller adjusts hydraulic valves.

---

### 🛣️ 3. Traction Control
Traction control reduces wheel spin during acceleration by limiting torque. ⚙️

- **🔍 Why It Matters**: Enhances stability on low-grip surfaces. In EVs, high torque demands precise control to prevent spin.
- **🛠️ Methods**: Similar to ABS, control slip ratio for drive wheels. Use throttle/brake intervention.
- **🌍 Real-World Example**: Launch control in sports EVs like Tesla for optimal 0-60 mph.

**📐 Mathematical Model**:
- Drive torque: Limited if \(\lambda > \lambda_{max}\).
- Controller: PID on motor current to reduce slip.

**💬 Explanation**: Integrates with vehicle stability control (ESC) for yaw correction.

---

### 🔋 4. Battery Management System (BMS)
BMS monitors and protects EV batteries for safety and longevity. 🔋

- **🔍 Why It Matters**: Prevents overcharge, deep discharge, and thermal runaway. Optimizes range and performance.
- **🛠️ Functions**: State-of-Charge (SOC) estimation, cell balancing, temperature control.
- **🌍 Real-World Example**: In Tesla packs, BMS equalizes cells and manages cooling.

**📐 Mathematical Model**:
- SOC: \(\dot{SOC} = -I / Q_{max}\), where I is current, Q_max capacity.
- Equivalent circuit: Randle's model for voltage prediction.

**💬 Explanation**: Uses Kalman filters for accurate SOC in noisy conditions.

---

### 🎮 5. EV Controller Design
Design controllers for EV systems, focusing on speed regulation. 🕹️

- **Speed Regulation**: Maintain desired velocity despite disturbances (hills, wind).
- **Methods**: PID for simplicity; MPC for constraints (battery limits, torque bounds).

**💬 Explanation**: In EVs, control motor torque directly for fast response.

---

## 💻 Code Implementation (Combined Example)
Simulate EV longitudinal dynamics and design a PID controller for speed regulation.

```python
# EV_Speed_Control.py
# 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
# Simulate EV speed regulation with PID

import numpy as np
import matplotlib.pyplot as plt
from control import tf, feedback, forced_response, pid

# 🧹 Clear plots
plt.close('all')

# 🧮 Vehicle Parameters
m = 1500  # Mass (kg)
r_w = 0.3  # Wheel radius (m)
C_d = 0.3  # Drag coefficient
A = 2.5   # Frontal area (m^2)
rho = 1.2 # Air density (kg/m^3)
C_r = 0.01 # Rolling resistance
g = 9.81  # Gravity (m/s^2)
K_t = 0.5 # Torque constant (Nm/A)
theta_road = 0  # Road incline (rad)

# 📐 EV Dynamics Model (State-Space: state v, input torque)
def ev_dynamics(v, tau, theta_road=0):
    F_drag = 0.5 * C_d * A * rho * v**2
    F_roll = C_r * m * g
    F_grav = m * g * np.sin(theta_road)
    F_res = F_drag + F_roll + F_grav
    accel = (tau / r_w - F_res) / m
    return accel

# 🔄 Simulation Function
def simulate_ev(controller, v_ref, t, dt, disturbance=False):
    v = 0.0  # Initial speed (m/s)
    integral = 0.0
    prev_error = 0.0
    speeds = [v]
    torques = []
    for i in range(1, len(t)):
        error = v_ref - v
        if controller == 'PID':
            # Manual PID implementation
            P = Kp * error
            integral += error * dt
            I = Ki * integral
            D = Kd * (error - prev_error) / dt
            u = P + I + D
            prev_error = error
        else:
            u = 0  # Open-loop or other
        tau = np.clip(u, -500, 500)  # Torque limits (Nm)
        if disturbance and t[i] > 10:
            theta_road = np.deg2rad(5)  # 5 deg hill
        else:
            theta_road = 0
        dv = ev_dynamics(v, tau, theta_road)
        v += dv * dt
        speeds.append(v)
        torques.append(tau)
    return np.array(speeds), np.array(torques)

# 🎮 PID Parameters
Kp = 1000  # Proportional
Ki = 500   # Integral
Kd = 200   # Derivative

# ⏳ Time Vector
dt = 0.01
t = np.arange(0, 30, dt)
v_ref = 20  # Reference speed (m/s ~72 km/h)

# 📉 Simulate Open-Loop (constant torque) vs PID
v_open, tau_open = simulate_ev('None', v_ref, t, dt, disturbance=True)
v_pid, tau_pid = simulate_ev('PID', v_ref, t, dt, disturbance=True)

# 📊 Plot Results
plt.figure(figsize=(10,6))
plt.plot(t, v_open, 'b-', label='Open-Loop')
plt.plot(t, v_pid, 'r--', label='PID Control')
plt.plot(t, v_ref * np.ones_like(t), 'g:', label='Reference')
plt.xlabel('Time (s)'); plt.ylabel('Speed (m/s)')
plt.title('EV Speed Regulation with Hill Disturbance')
plt.legend(); plt.grid(True)
plt.show()
```

**💬 Explanation**:
- **Model**: Nonlinear longitudinal dynamics with resistances.
- **PID**: Manual implementation for torque control; clips to realistic limits.
- **Simulation**: Shows PID maintaining speed during hill (disturbance).
- Save as `EV_Speed_Control.py` and run to visualize.

---

## 🛠️ Exercise
**Task**: Design a controller for EV speed regulation.
1. Run the script and observe PID vs. open-loop.
2. Tune PID gains (Kp, Ki, Kd) for faster response with minimal overshoot.
3. Add battery model (e.g., SOC drain) and limit torque based on SOC.
4. Implement LQR using linearized state-space around operating point.
5. Write a 1-page report comparing controllers under disturbances.

**💻 Solution Hint**:
```python
# LQR Example (Linearize around v=20 m/s)
from control import ss, lqr
v_op = 20
F_drag_op = 0.5 * C_d * A * rho * v_op  # Partial derivative approx
A = np.array([[-F_drag_op / m]])  # dv/dt ≈ - (dF_drag/dv) v / m
B = np.array([[1 / (m * r_w)]])   # Input torque
Q = np.array([[1]])
R = np.array([[0.01]])
K, _, _ = lqr(A, B, Q, R)
# Integrate with u = -K (v - v_ref)
```

**💬 Explanation**:
- Linearize dynamics for LQR; compare settling time.

---

## 📌 Additional Notes
- **✅ Best Practices**:
  - Use `np.clip` for realistic constraints.
  - Comment code with `#` for clarity.
  - Test with different disturbances (e.g., wind).
- **🔍 Debugging Tips**:
  - Print forces in dynamics function.
  - Check for numerical stability with small dt.
- **🚀 Extensions**:
  - Integrate ABS simulation using wheel slip.
  - Explore ADAS (e.g., adaptive cruise control).
- **🖥️ Lab Setup**: Ensure `control` library installed. Use Jupyter for interactive tuning.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 🚀 Model vehicle dynamics for control.
2. 🛑 Design ABS and traction systems.
3. 🛣️ Implement traction control strategies.
4. 🔋 Manage EV batteries with BMS.
5. 🎮 Design controllers for EV speed regulation.

**📅 Next Session Preview**: Case Study - Aerospace Control: Flight dynamics and autopilot systems. 🛠️

**📝 Assignment**: Complete the exercise, submit the script, and provide a 1-page report on controller performance and EV applications. Due by next session. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: September 1, 2025*
