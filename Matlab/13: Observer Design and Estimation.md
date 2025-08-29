# 👀 Lecture on Session 13: Observer Design and Estimation

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Estimate unmeasurable states for control systems. This session equips students with skills to design observers, such as Luenberger and Kalman filters, to estimate unmeasurable states in systems like power systems, using MATLAB tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-12 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation, state-space control, digital control). Knowledge of state-space models, linear algebra, and stochastic processes. MATLAB with Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapter 7 for state-space methods and observers)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Observer Design section

---

## 🗂️ Topics Covered

1. 🔍 **Luenberger Observer**: Full-order and reduced-order observers.
2. 📊 **Kalman Filter**: State estimation for noisy systems using `kalman`.
3. 💻 **Code Implementation**: Observer gain design with `place`.

---

## 📝 Detailed Lecture Content

### 🔍 1. Luenberger Observer
Luenberger observers estimate unmeasurable states by reconstructing system dynamics from available outputs, critical for control in systems like power grids. 📐

- **Concept**: For a state-space system dx/dt = Ax + Bu, y = Cx + Du, the observer estimates state x̂ with dynamics dx̂/dt = Ax̂ + Bu + L(y - C x̂), where L is the observer gain.
- **Full-Order Observer**: Estimates all states, even measurable ones. Error dynamics: d(x-x̂)/dt = (A-LC)(x-x̂).
- **Reduced-Order Observer**: Estimates only unmeasurable states, reducing complexity.
- **Design**: Choose L to place observer poles (eigenvalues of A-LC) for fast error convergence (typically 2-10 times faster than system poles).

**💻 Example Code**:
```matlab
% Full-order Luenberger observer
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);
poles_sys = eig(A); % System poles
poles_obs = poles_sys*2; % Observer poles (faster)
L = place(A', C', poles_obs)'; % Observer gain
disp('Observer Gain L:'); disp(L);
A_obs = A - L*C; % Observer dynamics
```

**💬 Explanation**:
- `place(A',C',poles_obs)'` computes L to place observer poles at desired locations.
- Observer poles are chosen faster than system poles for quick error decay.

---

### 📊 2. Kalman Filter
Kalman filters provide optimal state estimation in noisy systems, ideal for applications like power system monitoring with measurement noise. 📉

- **Concept**: Combines system model (dx/dt = Ax + Bu + w, y = Cx + v) with noisy measurements (w: process noise, v: measurement noise) to estimate states optimally.
- **Algorithm**: Predicts state evolution, updates with measurements using Kalman gain.
- **MATLAB Tool**: `kalman(sys,Q,R)` designs a Kalman filter for given noise covariances Q (process) and R (measurement).

**💻 Example Code**:
```matlab
% Kalman filter design
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);
Q = 0.1*eye(2); % Process noise covariance
R = 0.01; % Measurement noise covariance
[kalman_sys, K_kalman] = kalman(sys, Q, R);
disp('Kalman Gain:'); disp(K_kalman);
```

**💬 Explanation**:
- `kalman` designs an estimator with gain K_kalman, balancing model and measurement trust.
- Q and R are tuned based on noise characteristics (small values assume low noise).

---

### 💻 3. Code Implementation
Observer design involves computing gains and simulating state estimation, with `place` for Luenberger observers and `kalman` for noisy systems. 🔧

- **Workflow**: Define state-space model, check observability, design observer gain, simulate estimation.
- **MATLAB Tools**: `place` for pole placement, `kalman` for optimal filtering, `lsim` for simulation.

**💻 Example Code**:
```matlab
% Observer simulation
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);
% Check observability
Ob = obsv(A, C);
disp('Observability Matrix Rank:'); disp(rank(Ob));
% Luenberger observer
poles_obs = [-4, -5]; % Fast observer poles
L = place(A', C', poles_obs)';
disp('Luenberger Gain L:'); disp(L);
% Simulate
t = 0:0.01:5; u = ones(size(t));
x0 = [0.5; 0]; % Initial state
x0_hat = [0; 0]; % Initial estimate
A_obs = [A-B*L*C, B*L*C; zeros(size(A)), A-L*C];
B_obs = [B; B];
C_obs = [C, zeros(size(C))];
sys_obs = ss(A_obs, B_obs, C_obs, D);
[y, ~, x_est] = lsim(sys_obs, u, t, [x0; x0_hat]);
figure; plot(t, x_est(:,1:2), 'b-', t, x_est(:,3:4), 'r--');
title('State Estimation'); xlabel('Time (s)'); ylabel('States'); grid on; legend('x1', 'x2', 'x1_hat', 'x2_hat');
```

**💬 Explanation**:
- `obsv` verifies observability (full rank).
- `place` designs L for fast error convergence.
- Combined system simulates true and estimated states using `lsim`.

---

## 💻 Code Implementation (Combined Example)

Integrated script for Luenberger observer and Kalman filter design with simulation.

```matlab
% Session13_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Observer design and estimation

% 🧹 Clear workspace
clear; clc;

% System
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);

% 🔍 Luenberger Observer
Ob = obsv(A, C);
disp('Observability Matrix Rank:'); disp(rank(Ob));
poles_obs = [-4, -5]; % Fast observer poles
L = place(A', C', poles_obs)';
disp('Luenberger Gain L:'); disp(L);

% 📊 Kalman Filter
Q = 0.1*eye(2); % Process noise covariance
R = 0.01; % Measurement noise covariance
[kalman_sys, K_kalman] = kalman(sys, Q, R);
disp('Kalman Gain:'); disp(K_kalman);

% Simulation
t = 0:0.01:5; u = ones(size(t));
x0 = [0.5; 0]; x0_hat = [0; 0];
A_obs = [A-B*L*C, B*L*C; zeros(size(A)), A-L*C];
B_obs = [B; B];
C_obs = [C, zeros(size(C))];
sys_obs = ss(A_obs, B_obs, C_obs, D);
[y, ~, x_est] = lsim(sys_obs, u, t, [x0; x0_hat]);
figure; plot(t, x_est(:,1:2), 'b-', t, x_est(:,3:4), 'r--');
title('Luenberger State Estimation'); xlabel('Time (s)'); ylabel('States'); grid on; legend('x1', 'x2', 'x1_hat', 'x2_hat');
```

**💬 Explanation**:
- **Observability**: Verifies system is observable.
- **Luenberger**: Designs observer with fast poles.
- **Kalman**: Computes optimal gain for noisy system.
- **Simulation**: Plots true vs. estimated states.
- Save as `Session13_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Design an observer for a power system state estimation. System: dx/dt = Ax + Bu, y = Cx, with A=[0 1; -1 -1], B=[0; 1], C=[1 0], place observer poles at s=-5,-6.

**💻 Solution Example**:
```matlab
% Exercise_Session13.m
% Observer for power system

% 🧹 Clear workspace
clear; clc;

% System: Power system model
A = [0 1; -1 -1]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);

% 🔍 Check Observability
Ob = obsv(A, C);
disp('Observability Matrix Rank:'); disp(rank(Ob));

% 💻 Luenberger Observer
poles_obs = [-5, -6]; % Fast observer poles
L = place(A', C', poles_obs)';
disp('Observer Gain L:'); disp(L);

% Simulation
t = 0:0.01:5; u = ones(size(t));
x0 = [0.5; 0]; x0_hat = [0; 0];
A_obs = [A-B*L*C, B*L*C; zeros(size(A)), A-L*C];
B_obs = [B; B];
C_obs = [C, zeros(size(C))];
sys_obs = ss(A_obs, B_obs, C_obs, D);
[y, ~, x_est] = lsim(sys_obs, u, t, [x0; x0_hat]);
figure; plot(t, x_est(:,1:2), 'b-', t, x_est(:,3:4), 'r--');
title('Power System State Estimation'); xlabel('Time (s)'); ylabel('States'); grid on; legend('x1', 'x2', 'x1_hat', 'x2_hat');

% Verify Estimation
if max(abs(x_est(end,1:2) - x_est(end,3:4))) < 0.01
    disp('Accurate state estimation achieved!');
else
    disp('Adjust observer poles or simulation time.');
end
```

**💬 Explanation**:
- **System**: Power system model with A=[0 1; -1 -1], unstable (poles at s=-0.5±j√0.5).
- **Observability**: `obsv` confirms full rank (observable).
- **Observer**: `place` designs L for poles at s=-5,-6.
- **Simulation**: `lsim` compares true and estimated states, verifying convergence.

---

## 📌 Additional Notes
- **✅ Best Practices**: Choose observer poles 2-10 times faster than system poles. Tune Q, R in Kalman based on noise levels.
- **🔍 Debugging Tips**: Ensure observability before design. Check `A_obs` eigenvalues for stability.
- **🚀 Extensions**: Add noise to simulate Kalman filter performance or design reduced-order observer.
- **🖥️ Lab Setup**: Control System Toolbox required for `place`, `kalman`, `ss`, `lsim`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 🔍 Design Luenberger observers for state estimation.
2. 📊 Implement Kalman filters for noisy systems.
3. 💻 Simulate and verify observer performance.

**📅 Next Session Preview**: Nonlinear control basics. 🌊

**📝 Assignment**: For A=[0 1; -2 1], B=[0; 1], C=[1 0], design a Luenberger observer with poles at s=-4,-5, simulate estimation, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
