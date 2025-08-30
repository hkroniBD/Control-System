# 🎓 Session 24: Capstone Project and Review

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Integrate all control systems concepts from the course into a comprehensive capstone project. This session guides students in applying nonlinear, optimal, MIMO, adaptive, robust, and MPC techniques to real-world systems, while reviewing debugging, numerical stability, and real-time implementation for practical deployment. 🛠️

**⏳ Duration**: 4-6 hours (project planning, implementation guidance, and review; extend over multiple days for completion)

**📋 Prerequisites**: 
- All prior sessions (PID/LQR, state-space, MPC, case studies in power, robotics, automotive).
- Proficiency in MATLAB/Simulink (or Python equivalent).
- Basic hardware knowledge (e.g., Arduino for interfacing).

**📚 Resources**:
- 🌐 MATLAB Documentation ([www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Modern Control Systems” by Richard C. Dorf (for integration of concepts)
- 🔗 Online: GitHub for repos ([github.com](https://github.com)), Arduino tutorials ([www.arduino.cc](https://www.arduino.cc)), X posts on control projects (#ControlSystems #CapstoneProject)

---

## 🗂️ Topics Covered

1. 💡 **Project Ideas**: Overview of suggested capstone projects integrating course concepts.
2. 📄 **Deliverables**: Requirements for code, report, presentation, and repo.
3. 🔍 **Review and Best Practices**: Debugging, numerical issues, real-time tips.

---

## 📝 Detailed Lecture Content

### 💡 1. Project Ideas
Select one project to demonstrate mastery. Each integrates multiple concepts (e.g., modeling, simulation, control design, analysis). Focus on simulation first, then optional hardware extension.

#### 1.1 Inverted Pendulum Stabilization (Nonlinear + Optimal Control)
Stabilize an inverted pendulum on a cart, a classic nonlinear system. 🤸

- **🔍 Why It Matters**: Tests nonlinear dynamics and optimal control. Real-world: Segway or rocket balancing.
- **🛠️ Concepts Integrated**: Nonlinear modeling, linearization, LQR for optimal gains, state estimation (e.g., Kalman filter).
- **🌍 Real-World Extension**: Interface with Arduino for physical pendulum.
- **📐 Mathematical Model**:
  - States: \([x, \dot{x}, \theta, \dot{\theta}]\) (cart position/velocity, pendulum angle/angular velocity).
  - Nonlinear equations:
    \[
    \ddot{x} = \frac{F + m l \dot{\theta}^2 \sin \theta - m g \sin \theta \cos \theta}{M + m - m \cos^2 \theta}
    \]
    \[
    \ddot{\theta} = \frac{(M + m) g \sin \theta - F \cos \theta - m l \dot{\theta}^2 \sin \theta \cos \theta}{l (M + m - m \cos^2 \theta)}
    \]
    (M: cart mass, m: pendulum mass, l: length, g: gravity, F: force).
  - Linearize around upright (\(\theta = 0\)) for LQR design.

**💬 Explanation**: Simulate swing-up (energy-based) then stabilization (LQR). Add disturbances for robustness.

#### 1.2 Quadcopter Flight Control (MIMO + Adaptive Control)
Control a quadcopter's position and attitude in 3D space. 🚁

- **🔍 Why It Matters**: MIMO systems with coupled dynamics; adaptive for parameter uncertainties (e.g., wind).
- **🛠️ Concepts Integrated**: State-space MIMO modeling, PID cascades, adaptive control (e.g., MRAC), trajectory tracking.
- **🌍 Real-World Extension**: Simulate with Simulink Aerospace Blockset; interface Arduino for mini-drone.
- **📐 Mathematical Model**:
  - States: Position \([x, y, z]\), velocities, Euler angles \([\phi, \theta, \psi]\), angular rates.
  - Dynamics: Newton-Euler equations, e.g., \(\ddot{z} = (\cos \phi \cos \theta) (U_1 / m) - g\), where \(U_1\) is total thrust.
  - Controls: Rotor speeds for thrust/torques.

**💬 Explanation**: Use inner-loop attitude control (PID) and outer-loop position (adaptive). Handle MIMO coupling via decoupling matrices.

#### 1.3 Smart Grid Simulator (Robust + MPC)
Simulate a microgrid with renewables, loads, and storage. ⚡

- **🔍 Why It Matters**: Robust to uncertainties (e.g., solar variability); MPC for optimization under constraints.
- **🛠️ Concepts Integrated**: Power system modeling, robust control (H-infinity), MPC for frequency/voltage regulation.
- **🌍 Real-World Extension**: Use Simulink for grid simulation; Arduino for sensor emulation.
- **📐 Mathematical Model**:
  - Frequency: Similar to LFC (Session 21), with renewables as disturbances.
  - MPC: Minimize cost \(J = \sum (Q e^2 + R u^2)\) subject to constraints (e.g., battery SOC limits).

**💬 Explanation**: Design MPC to optimize energy dispatch; add robust elements for parameter variations.

---

### 📄 2. Deliverables
Submit a complete project package demonstrating integration of concepts.

- **💻 Code**: Full MATLAB/Simulink scripts/models (e.g., .m, .slx files). Include simulations, plots, and analysis.
- **📝 Report**: 10-15 pages (PDF): Introduction, model derivation, control design, results/discussion, conclusions. Use LaTeX or Word.
- **🎤 Presentation**: 10-15 min slides (PowerPoint/Google Slides): Demo simulation, explain concepts, highlight challenges/solutions.
- **📂 GitHub Repo**: Encouraged for version control. Include README.md with setup instructions, code, report, and video demo.

**💬 Explanation**: Repo enhances collaboration; use branches for iterations.

---

### 🔍 3. Review and Best Practices
Review common pitfalls and tips for successful implementation.

- **🐞 Debugging Tips**:
  - Use `dbstop if error` in MATLAB for breakpoints.
  - Check variable scopes and initialization.
  - Simulate step-by-step; use `disp()` or logging for intermediates.
- **🔢 Numerical Issues**:
  - Handle singularities (e.g., division by zero in kinematics) with conditionals.
  - Use `ode45` with tolerances for stiff systems; check for NaN/Inf.
  - Scale states for better conditioning in optimization (e.g., LQR Q/R).
- **🕒 Real-Time Implementation** (e.g., via Arduino):
  - Use MATLAB Arduino Support Package for interfacing.
  - Discretize controllers (e.g., `c2d` for PID/LQR).
  - Account for delays/sampling: Use timers, avoid blocking code.
  - Safety: Add limits on actuators; test in simulation first.

**💬 Explanation**: Real-time adds latency; use External Mode in Simulink for tuning.

---

## 💻 Code Implementation (Combined Example)
Example: Inverted Pendulum Stabilization in MATLAB (nonlinear sim + LQR).

```matlab
% InvertedPendulum_Capstone.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Simulate and control inverted pendulum with LQR

% 🧹 Clear workspace
clear; clc; close all;

% 🧮 Parameters
M = 0.5;   % Cart mass (kg)
m = 0.2;   % Pendulum mass (kg)
l = 0.3;   % Pendulum length (m)
g = 9.81;  % Gravity (m/s^2)
b = 0.1;   % Damping (N/m/s)

% 📐 Nonlinear Model (for simulation)
nonlin_ode = @(t, x, u) [x(2);
    (u - b*x(2) + m*l*x(4)^2*sin(x(3)) - m*g*sin(x(3))*cos(x(3))) / (M + m - m*cos(x(3))^2);
    x(4);
    ((M + m)*g*sin(x(3)) - u*cos(x(3)) + b*x(2)*cos(x(3)) - m*l*x(4)^2*sin(x(3))*cos(x(3))) / (l*(M + m - m*cos(x(3))^2))];

% 🔄 Linearized Model (around theta=0)
A = [0 1 0 0;
     0 -b/M (m*g)/M 0;
     0 0 0 1;
     0 (b*cos(0))/(M*l) -((M+m)*g)/(M*l) 0];
B = [0; 1/M; 0; -1/(M*l)];
C = eye(4); D = zeros(4,1);
sys = ss(A, B, C, D);

% 🎮 LQR Design
Q = diag([1 1 10 1]);  % Weight position and angle
R = 0.1;               % Control effort
[K, ~, ~] = lqr(sys, Q, R);

% 🔄 Closed-Loop Simulation
t = 0:0.01:10;
x0 = [0; 0; pi/6; 0];  % Initial: theta=30 deg
u_func = @(x) -K * x;  % LQR control
[t, x] = ode45(@(t,x) nonlin_ode(t, x, u_func(x)), t, x0);

% 📉 Plot Results
figure; 
subplot(2,1,1); plot(t, x(:,1), 'b-'); xlabel('Time (s)'); ylabel('Cart Position (m)'); grid on;
subplot(2,1,2); plot(t, x(:,3)*180/pi, 'r-'); xlabel('Time (s)'); ylabel('Pendulum Angle (deg)'); grid on;
title('Inverted Pendulum Stabilization with LQR');
```

**💬 Explanation**:
- **Model**: Nonlinear ODE for accurate sim; linearized for LQR.
- **LQR**: Computes optimal gains; applied to nonlinear plant.
- **Simulation**: Uses `ode45` for dynamics; initial upset stabilizes.
- Extend: Add Kalman filter for noisy measurements.

---

## 🛠️ Exercise
**Task**: Implement your chosen project.
1. Model the system (derive equations, simulate open-loop).
2. Design controllers (e.g., LQR/MPC/adaptive).
3. Analyze (stability, robustness to disturbances).
4. Optional: Real-time demo with Arduino (e.g., pendulum sensors).
5. Prepare deliverables.

**💻 Solution Hint** (for Quadcopter):
Use Simulink blocks for MIMO; implement adaptive gains via S-functions.

**💬 Explanation**:
- Start small: Validate model before control.
- Iterate: Tune via simulations.

---

## 📌 Additional Notes
- **✅ Best Practices**:
  - Version control with Git: Commit often.
  - Document code inline (% comments).
  - Validate with hand calculations.
- **🔍 Debugging Tips**:
  - Use Simulink diagnostics for block errors.
  - Profile code with `tic/toc` for bottlenecks.
- **🚀 Extensions**:
  - Hardware-in-the-Loop (HIL) with Simulink.
  - Publish repo publicly for portfolio.
- **🖥️ Lab Setup**: Ensure MATLAB Control Toolbox/Simulink installed. Arduino kits available.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 💡 Integrate multiple control concepts into a project.
2. 📄 Produce professional deliverables (code, report, presentation).
3. 🔍 Troubleshoot numerical and real-time issues.
4. 🛠️ Apply theory to practical systems.
5. 📂 Use tools like GitHub for collaboration.

**📅 Next Session Preview**: Course wrap-up and feedback (optional). 🛠️

**📝 Assignment**: Complete capstone project (choose one idea or propose equivalent). Submit repo link, report, and presentation slides by deadline. Present in final class. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: September 2, 2025*
