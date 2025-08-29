# 🧠 Lecture on Session 14: Optimal Control: LQR and LQG

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Minimize cost functions for optimal control. This session equips students with skills to design Linear Quadratic Regulator (LQR) and Linear Quadratic Gaussian (LQG) controllers to optimize performance in systems like inverted pendulums, using MATLAB tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-13 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation, state-space control, digital control, observer design). Knowledge of state-space models, linear algebra, and stochastic processes. MATLAB with Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapter 7 for optimal control)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Optimal Control section

---

## 🗂️ Topics Covered

1. 📉 **LQR**: Quadratic cost minimization using `lqr`.
2. 📊 **LQG**: Combining LQR with Kalman filter using `lqg`.
3. 💻 **Code Implementation**: `[K,S,E] = lqr(sys,Q,R)`.

---

## 📝 Detailed Lecture Content

### 📉 1. LQR (Linear Quadratic Regulator)
LQR designs a state feedback controller to minimize a quadratic cost function, optimizing performance for systems like inverted pendulums. 📈

- **Concept**: For state-space system dx/dt = Ax + Bu, y = Cx + Du, LQR minimizes the cost J = ∫(x^T Q x + u^T R u)dt, where Q weights state errors and R weights control effort.
- **Control Law**: u = -Kx, where K is computed to minimize J.
- **MATLAB Tool**: `[K,S,E] = lqr(A,B,Q,R)` computes gain K, solution S to the Riccati equation, and closed-loop eigenvalues E.

**💻 Example Code**:
```matlab
% LQR design
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
Q = eye(2); % State weighting
R = 1; % Control effort weighting
[K, S, E] = lqr(A, B, Q, R);
disp('LQR Gain K:'); disp(K);
disp('Closed-Loop Poles:'); disp(E);
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);
figure; step(sys_cl); title('LQR Closed-Loop Response'); grid on;
```

**💬 Explanation**:
- `lqr` computes K to balance state regulation (Q) and control effort (R).
- `step` shows the optimized response; E confirms stable closed-loop poles.

---

### 📊 2. LQG (Linear Quadratic Gaussian)
LQG combines LQR with a Kalman filter to handle noisy systems, ideal for applications with measurement and process noise. 📡

- **Concept**: LQG integrates LQR (for control) with a Kalman filter (for state estimation), using u = -Kx̂, where x̂ is the estimated state.
- **Components**: LQR gain K from `lqr`, Kalman gain L from `kalman`.
- **MATLAB Tool**: `lqg(sys,Q,R,Qn,Rn)` designs a controller combining LQR and Kalman filter.

**💻 Example Code**:
```matlab
% LQG design
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);
Q = eye(2); % LQR state weighting
R = 1; % LQR control weighting
Qn = 0.1*eye(2); % Process noise covariance
Rn = 0.01; % Measurement noise covariance
lqg_sys = lqg(sys, Q, R, Qn, Rn);
disp('LQG Controller:'); disp(lqg_sys);
figure; step(lqg_sys); title('LQG Closed-Loop Response'); grid on;
```

**💬 Explanation**:
- `lqg` designs a controller combining LQR and Kalman filter.
- Q, R weight control performance; Qn, Rn tune the Kalman filter.
- `step` evaluates the closed-loop response with estimated states.

---

### 💻 3. Code Implementation
LQR and LQG implementations optimize control and estimation, using `lqr` for deterministic systems and `lqg` for noisy systems. 🔧

- **Workflow**: Define state-space model, select Q/R for LQR or Q/R/Qn/Rn for LQG, simulate closed-loop performance.
- **MATLAB Tools**: `lqr`, `lqg`, `ss`, `step` for design and analysis.

**💻 Example Code**:
```matlab
% Session14_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% LQR and LQG design

% 🧹 Clear workspace
clear; clc;

% System
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);

% 📉 LQR
Q = eye(2); R = 1;
[K, S, E] = lqr(A, B, Q, R);
disp('LQR Gain K:'); disp(K);
disp('LQR Closed-Loop Poles:'); disp(E);
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);
figure(1); step(sys_cl); title('LQR Closed-Loop Response'); grid on;

% 📊 LQG
Qn = 0.1*eye(2); Rn = 0.01;
lqg_sys = lqg(sys, Q, R, Qn, Rn);
disp('LQG Controller:'); disp(lqg_sys);
figure(2); step(lqg_sys); title('LQG Closed-Loop Response'); grid on;
```

**💬 Explanation**:
- **LQR**: Designs K to minimize quadratic cost, simulates response.
- **LQG**: Combines LQR with Kalman filter for noisy systems.
- Save as `Session14_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Optimize control for an inverted pendulum using LQR. System: dx/dt = Ax + Bu, with A=[0 1 0 0; 0 0 1 0; 0 10.0 0 0; 0 0 0 -1], B=[0; 0; 0; 1], C=[1 0 0 0], stabilize at upright position.

**💻 Solution Example**:
```matlab
% Exercise_Session14.m
% LQR for inverted pendulum

% 🧹 Clear workspace
clear; clc;

% System: Inverted pendulum (linearized)
A = [0 1 0 0; 0 0 1 0; 0 10.0 0 0; 0 0 0 -1]; % State matrix
B = [0; 0; 0; 1]; % Input matrix
C = [1 0 0 0]; % Output matrix (position)
D = 0; % Feedthrough
sys = ss(A, B, C, D);

% 📉 LQR Design
Q = diag([100 1 1 1]); % Weight position heavily
R = 1; % Control effort
[K, S, E] = lqr(A, B, Q, R);
disp('LQR Gain K:'); disp(K);
disp('Closed-Loop Poles:'); disp(E);

% Closed-Loop Simulation
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);
t = 0:0.01:5; u = zeros(size(t)); % No reference input
x0 = [0.1; 0; 0; 0]; % Initial angle = 0.1 rad
[y, t, x] = lsim(sys_cl, u, t, x0);
figure; plot(t, x(:,1), 'b-', t, x(:,2), 'r--');
title('Inverted Pendulum LQR Response'); xlabel('Time (s)'); ylabel('States'); grid on; legend('Angle (rad)', 'Angular Velocity');

% Verify Stability
if isstable(sys_cl)
    disp('System stabilized!');
else
    disp('Adjust Q or R weights.');
end
```

**💬 Explanation**:
- **System**: Inverted pendulum linearized model with states [angle, angular velocity, cart position, cart velocity].
- **LQR**: Q weights angle heavily (100) to prioritize stabilization; R=1 balances control effort.
- **Simulation**: `lsim` simulates response from initial angle (0.1 rad), verifying stabilization.
- **Verification**: `isstable` confirms closed-loop stability.

---

## 📌 Additional Notes
- **✅ Best Practices**: Tune Q, R to balance performance and control effort. Ensure Q is positive semi-definite, R positive definite.
- **🔍 Debugging Tips**: Check system controllability with `ctrb` before LQR. Verify closed-loop poles with `eig(A_cl)`.
- **🚀 Extensions**: Add Kalman filter for LQG or test in Simulink with nonlinear pendulum model.
- **🖥️ Lab Setup**: Control System Toolbox required for `lqr`, `lqg`, `ss`, `lsim`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📉 Design LQR controllers to minimize quadratic costs.
2. 📊 Implement LQG controllers for noisy systems.
3. 💻 Simulate and verify optimal control performance.

**📅 Next Session Preview**: Robust control basics (H-infinity). 🔒

**📝 Assignment**: For A=[0 1; 1 0], B=[0; 1], C=[1 0], design an LQR controller with Q=eye(2), R=1, simulate response, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
