# ⚡ Session 21: Case Study - Power Systems Control

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Apply control systems engineering principles to electrical power grids, focusing on load frequency control (LFC) and voltage regulation. This session equips students with practical skills to model, simulate, and design controllers for stable grid operation, addressing real-world challenges like load variations and renewable integration. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: 
- Understanding of control systems (PID, state-space, LQR from prior sessions).
- Basic electrical engineering concepts (AC power, frequency, voltage, reactive power).
- Python with libraries: NumPy, SciPy, Matplotlib, and `control` (install via `pip install control`).

**📚 Resources**:
- 🌐 Python Control Systems Library ([python-control.readthedocs.io](https://python-control.readthedocs.io))
- 📖 “Power System Stability and Control” by Prabha Kundur (Chapters on LFC and voltage control)
- 🔗 Online: IEEE Power & Energy Society ([www.ieee-pes.org](https://www.ieee-pes.org)), X posts on grid control (#PowerSystems)

---

## 🗂️ Topics Covered

1. ⚖️ **Load Frequency Control (LFC)**: Maintain grid frequency via primary (droop) and secondary (AGC) control.
2. 🔌 **Voltage Regulation**: Stabilize bus voltages using AVRs, tap changers, and reactive power devices.
3. 💻 **Simulation Tools**: Use Python’s `control` library for modeling and simulating power system dynamics.
4. 🎮 **Controller Design**: Tune PID for AGC and design LQR for optimal control.
5. 📊 **Practical Applications**: Address renewable integration and grid stability challenges.

---

## 📝 Detailed Lecture Content

### ⚖️ 1. Load Frequency Control (LFC)
LFC ensures the grid frequency remains close to its nominal value (e.g., 60 Hz in the US, 50 Hz in Europe) despite load-generation imbalances. 🕰️

- **🔍 Why It Matters**: Frequency deviations cause equipment damage or blackouts. Renewables (e.g., solar/wind) introduce variability, making LFC critical.
- **🛠️ Control Levels**:
  - **Primary**: Turbine governors adjust power via droop control (proportional response).
  - **Secondary**: Automatic Generation Control (AGC) corrects steady-state errors using integral or PID control.
  - **Tertiary**: Economic dispatch for long-term optimization.
- **🌍 Real-World Example**: A 500 MW load surge (e.g., EV charging) drops frequency. AGC ramps up generation to restore balance.

**📐 Mathematical Model** (Single-Area):
- Swing equation: \(\Delta f = \frac{1}{2H s + D} (\Delta P_m - \Delta P_L)\)
  - \(H\): Inertia constant (s)
  - \(D\): Damping (pu)
  - \(\Delta P_m\): Mechanical power change
  - \(\Delta P_L\): Load change
- Governor: \(\frac{1}{1 + T_g s}\), Turbine: \(\frac{1}{1 + T_t s}\)
- Droop: \(\frac{1}{R}\) (Hz/pu)
- AGC: Minimizes Area Control Error (ACE) = \(\Delta f + B \Delta P_{tie}\)

**💬 Explanation**: The swing equation models frequency dynamics. AGC uses feedback to eliminate steady-state errors, critical in interconnected grids.

---

### 🔌 2. Voltage Regulation
Voltage regulation maintains bus voltages within ±5% of nominal to ensure reliable power delivery. ⚡️

- **🔍 Why It Matters**: Voltage sags cause brownouts; swells damage insulation. Distributed renewables (e.g., rooftop solar) cause local voltage fluctuations.
- **🛠️ Methods**:
  - **Automatic Voltage Regulators (AVRs)**: Adjust generator excitation.
  - **On-Load Tap Changers (OLTCs)**: Modify transformer ratios.
  - **Reactive Power Devices**: Capacitor banks, SVCs, STATCOMs.
- **🌍 Real-World Example**: Solar farms injecting power can raise local voltages, requiring coordinated VAR control.

**📐 Mathematical Model**:
- Voltage: \(V = f(Q_g - Q_L)\), where \(Q_g\) is reactive power generated, \(Q_L\) is load.
- AVR: \(\frac{K_a}{1 + T_a s}\) (amplifier gain, time constant).
- Use power flow or small-signal models for simulation.

**💬 Explanation**: Voltage control is decoupled from frequency but interacts via active/reactive power. Advanced grids use Volt-VAR optimization.

---

### 💻 3. Simulation Tools
Python’s `control` library enables modeling and simulation of dynamic systems, ideal for power systems. 🐍

- **✅ Installation Check**: Run `import control` in Python. Install with `pip install control` if needed.
- **🔑 Key Functions**:
  - `tf(num, den)`: Transfer function.
  - `ss(A,B,C,D)`: State-space model.
  - `forced_response(sys, T, U)`: Simulate response to input.
- **📉 Analysis**: Plot frequency/voltage deviations under disturbances.

**💻 Example**: Simulate a step response for a first-order system (e.g., turbine model).
```python
from control import tf, forced_response
import matplotlib.pyplot as plt
import numpy as np

sys = tf([1], [0.5, 1])  # Turbine: 1/(0.5s + 1)
t = np.linspace(0, 5, 100)
_, y = forced_response(sys, t, np.ones_like(t))
plt.plot(t, y)
plt.xlabel('Time (s)'); plt.ylabel('Output')
plt.title('Turbine Response'); plt.grid(True)
plt.show()
```

**💬 Explanation**: This models a turbine’s response to a step input, useful for LFC simulations.

---

### 🎮 4. Controller Design
Design controllers to stabilize frequency and voltage. 🕹️

- **PID for AGC**: Adjusts governor setpoints to minimize ACE.
  - Proportional (Kp): Reduces error quickly.
  - Integral (Ki): Eliminates steady-state error.
  - Derivative (Kd): Dampens oscillations.
- **LQR**: Optimal control for state-space models, minimizing a cost function \(J = \int (x^T Q x + u^T R u) dt\).
  - \(Q\): State weighting (e.g., prioritize frequency).
  - \(R\): Control effort penalty.

**💬 Explanation**: PID is simple but requires tuning. LQR is optimal but needs a state-space model.

---

### 📊 5. Practical Applications
- **Renewable Integration**: Handle solar/wind variability with predictive control.
- **Smart Grids**: Use AI for real-time tuning (e.g., forecasting load via X posts on #SmartGrid).
- **Resilience**: Mitigate cyber threats or extreme weather impacts.

---

## 💻 Code Implementation (Combined Example)
Simulate a single-area power system with AGC, comparing PID and LQR controllers.

```python
% PowerSystem_AGC.py
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Simulate LFC with PID and LQR for a single-area grid

import numpy as np
import matplotlib.pyplot as plt
from control import tf, ss, forced_response, feedback, lqr

# 🧹 Clear plots
plt.close('all')

# 🧮 System Parameters
H = 5    # Inertia constant (s)
D = 1    # Damping (pu)
R = 0.05 # Droop (Hz/pu)
Tg = 0.2 # Governor time (s)
Tt = 0.5 # Turbine time (s)

# 📐 Transfer Functions
Ps = tf(1, [2*H, D])         # Power system
Gov = tf(1, [Tg, 1])        # Governor
Turb = tf(1, [Tt, 1])       # Turbine
Droop = tf(1/R, 1)          # Droop
Plant = Gov * Turb * Ps      # Open-loop plant

# 🔄 Primary Control (Droop Feedback)
Sys_primary = feedback(Plant, Droop, sign=-1)

# ⏳ Time Vector and Load Disturbance
t = np.linspace(0, 50, 1000)
u = np.zeros_like(t)
u[t >= 2] = 0.1  # Step load of 0.1 pu at t=2s

# 📉 Simulate Primary Control
_, delta_f_primary = forced_response(Sys_primary, T=t, U=-u)

# 🎮 AGC with PID
Kp, Ki, Kd = 1.0, 0.5, 0.1
PID = tf([Kd, Kp, Ki], [1, 0])  # PID controller
Sys_agc = feedback(Plant * feedback(Gov * Turb, PID, sign=-1), Droop, sign=-1)
_, delta_f_agc = forced_response(Sys_agc, T=t, U=-u)

# 📊 Plot PID Results
plt.figure(figsize=(10,6))
plt.plot(t, delta_f_primary, 'b-', label='Primary Control')
plt.plot(t, delta_f_agc, 'r--', label='AGC with PID')
plt.xlabel('Time (s)'); plt.ylabel('Frequency Deviation (pu)')
plt.title('Frequency Response to Load Disturbance')
plt.legend(); plt.grid(True)
plt.show()

# 🎮 LQR Design
A = np.array([[-D/(2*H), 1/(2*H), 0],
              [0, -1/Tt, 1/Tt],
              [-1/(R*Tg), 0, -1/Tg]])  # State-space
B = np.array([[0], [0], [1/Tg]])
E = np.array([[-1/(2*H)], [0], [0]])
Q = np.diag([10, 1, 1])  # Weight frequency
R = np.array([[1]])
K, _, _ = lqr(A, B, Q, R)
Acl = A - B @ K
sys_lqr = ss(Acl, E, [1,0,0], 0)  # Output: delta_f

# 📉 Simulate LQR
_, delta_f_lqr = forced_response(sys_lqr, T=t, U=u)

# 📊 Plot LQR Results
plt.figure(figsize=(10,6))
plt.plot(t, delta_f_primary, 'b-', label='Primary Control')
plt.plot(t, delta_f_lqr, 'g--', label='LQR Control')
plt.xlabel('Time (s)'); plt.ylabel('Frequency Deviation (pu)')
plt.title('LQR vs. Primary Control Response')
plt.legend(); plt.grid(True)
plt.show()
```

**💬 Explanation**:
- **Parameters**: Realistic values for a small grid (H=5s, D=1pu, etc.).
- **Primary Control**: Droop reduces frequency deviation but leaves steady-state error.
- **PID**: Eliminates error but may oscillate if poorly tuned.
- **LQR**: Optimizes response using state-space, often smoother.
- Save as `PowerSystem_AGC.py` and run to visualize responses.

---

## 🛠️ Exercise
**Task**: Simulate and tune controllers for a single-area grid.
1. Run the provided script and observe frequency responses.
2. Tune PID gains (Kp, Ki, Kd) to achieve settling time <20s, overshoot <0.02 pu.
3. Adjust LQR’s Q matrix (e.g., increase frequency weight) and compare with PID.
4. Extend to a two-area model (add tie-line, ACE = \(\Delta f + B \Delta P_{tie}\)).
5. Write a brief report (1 page) analyzing results.

**💻 Solution Hint**:
```python
# PID Tuning Example
Kp, Ki, Kd = 1.5, 0.8, 0.2  # Try new gains
PID = tf([Kd, Kp, Ki], [1, 0])
# Re-run simulation and plot
```

**💬 Explanation**:
- Increase Ki for faster error correction, but watch for oscillations.
- LQR tuning: Increase Q[0,0] to prioritize frequency stability.

---

## 📌 Additional Notes
- **✅ Best Practices**:
  - Use `plt.close('all')` to clear old plots.
  - Comment code with `%` for clarity.
  - Save scripts in a dedicated folder; use Git for version control. 📂
- **🔍 Debugging Tips**:
  - Check transfer functions with `print(sys)`.
  - Use `plt.pause(0.1)` for real-time plotting if needed.
- **🚀 Extensions**:
  - Simulate voltage regulation with a simple AVR model.
  - Explore Model Predictive Control (MPC) for advanced grids.
- **🖥️ Lab Setup**: Ensure `control` library is installed. Use Anaconda or virtual environments for clean setups.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. ⚖️ Understand LFC and its role in grid stability.
2. 🔌 Model and control voltage dynamics.
3. 💻 Simulate power systems using Python’s `control` library.
4. 🎮 Design and tune PID and LQR controllers.
5. 📊 Apply concepts to real-world grid challenges (e.g., renewables).

**📅 Next Session Preview**: Advanced control for renewable integration, including MPC and AI-based forecasting. 🛠️

**📝 Assignment**: Complete the exercise, submit the script, and provide a 1-page report comparing PID and LQR performance, discussing implications for smart grids. Due by next session. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
