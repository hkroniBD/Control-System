# 📈 Lecture on Session 16: Model Predictive Control (MPC)

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Implement predictive optimization for control systems. This session equips students with skills to design and apply Model Predictive Control (MPC) to optimize system performance under constraints, such as in chemical process reactors, using MATLAB tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-15 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation, state-space control, digital control, observer design, optimal control, robust control). Knowledge of state-space models, optimization, and control theory. MATLAB with Control System Toolbox and Model Predictive Control Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/mpc](https://www.mathworks.com/help/mpc))
- 📖 “Model Predictive Control: Theory and Design” by James B. Rawlings and David Q. Mayne
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – MPC section

---

## 🗂️ Topics Covered

1. 🔍 **MPC Basics**: Prediction horizon, control horizon, and constraints.
2. 🛠️ **Implementation**: Using `mpc` object in MATLAB.
3. 💻 **Code Implementation**: `ctrl = mpc(sys)`.

---

## 📝 Detailed Lecture Content

### 🔍 1. MPC Basics
Model Predictive Control (MPC) optimizes control actions by predicting future system behavior over a finite horizon, respecting constraints, ideal for complex systems like chemical reactors. 📊

- **Concept**: MPC uses a system model to predict outputs over a prediction horizon (Np), computes optimal inputs over a control horizon (Nc), and applies only the first control move, repeating at each time step.
- **Components**:
  - **Prediction Horizon (Np)**: Number of steps to predict future outputs.
  - **Control Horizon (Nc)**: Number of steps to optimize control inputs (Nc ≤ Np).
  - **Constraints**: Limits on inputs, outputs, or states (e.g., actuator limits, safety bounds).
- **Objective**: Minimize a cost function (e.g., tracking error + control effort) subject to constraints.

**💻 Example Code**:
```matlab
% Basic MPC setup
Ts = 0.1; % Sampling time
sys = tf([1], [1 2 1]); % Plant: 1/(s^2 + 2s + 1)
sysd = c2d(sys, Ts, 'zoh'); % Discrete-time model
ctrl = mpc(sysd, Ts); % MPC controller
ctrl.PredictionHorizon = 10; % Np
ctrl.ControlHorizon = 2; % Nc
disp('MPC Controller:'); disp(ctrl);
```

**💬 Explanation**:
- `c2d` discretizes the plant for digital control.
- `mpc(sysd, Ts)` creates an MPC object with default settings.
- Horizons are set to balance performance and computation.

---

### 🛠️ 2. Implementation
MPC implementation in MATLAB uses the `mpc` object to configure and simulate controllers, handling constraints and optimization automatically. ⚙️

- **Workflow**: Discretize plant, configure `mpc` object (horizons, weights, constraints), simulate closed-loop response.
- **MATLAB Tool**: `mpc` object for controller design, `mpcsim` or `sim` for simulation.
- **Features**: Supports input/output constraints, custom weights for tracking vs. control effort.

**💻 Example Code**:
```matlab
% MPC implementation with constraints
Ts = 0.1;
sys = tf([1], [1 2 1]); % Plant
sysd = c2d(sys, Ts, 'zoh');
ctrl = mpc(sysd, Ts);
ctrl.PredictionHorizon = 10;
ctrl.ControlHorizon = 2;
ctrl.Weights.Output = 1; % Tracking weight
ctrl.Weights.Manipulated = 0.1; % Control effort weight
ctrl.MV.Min = -1; ctrl.MV.Max = 1; % Input constraints
t = 0:Ts:10; r = ones(size(t)); % Reference
[y, t, u] = sim(ctrl, t, r);
figure; subplot(2,1,1); plot(t, y, 'b-', t, r, 'r--'); title('Output'); grid on; legend('y', 'r');
subplot(2,1,2); plot(t, u, 'b-'); title('Control Input'); grid on;
```

**💬 Explanation**:
- `mpc` configures controller with horizons and weights.
- Input constraints (-1 ≤ u ≤ 1) ensure practical control actions.
- `sim` runs closed-loop simulation, plotting output and control.

---

### 💻 3. Code Implementation
MPC implementation uses the `mpc` object to design and simulate controllers, leveraging MATLAB’s Model Predictive Control Toolbox. 🔧

- **Key Steps**: Define discrete model, set up MPC parameters, simulate with reference tracking.
- **MATLAB Tools**: `mpc`, `c2d`, `sim` for design and simulation.

**💻 Example Code**:
```matlab
% Session16_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Model Predictive Control

% 🧹 Clear workspace
clear; clc;

% System
Ts = 0.1;
sys = tf([1], [1 2 1]); % Plant: 1/(s^2 + 2s + 1)
sysd = c2d(sys, Ts, 'zoh');
ctrl = mpc(sysd, Ts);
% Configure MPC
ctrl.PredictionHorizon = 10;
ctrl.ControlHorizon = 2;
ctrl.Weights.Output = 1;
ctrl.Weights.Manipulated = 0.1;
ctrl.MV.Min = -1; ctrl.MV.Max = 1; % Input constraints
% Simulation
t = 0:Ts:10; r = ones(size(t));
[y, t, u] = sim(ctrl, t, r);
figure; subplot(2,1,1); plot(t, y, 'b-', t, r, 'r--'); title('Output Response'); grid on; legend('y', 'r');
subplot(2,1,2); plot(t, u, 'b-'); title('Control Input'); grid on;
```

**💬 Explanation**:
- **Discretization**: Converts plant to discrete-time for MPC.
- **MPC Setup**: Configures horizons, weights, and constraints.
- **Simulation**: Tracks reference (r=1) with constrained inputs.
- Save as `Session16_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Apply MPC to a chemical process reactor simulation. System: G(s) = 1/(5s + 1), Ts=0.5s, achieve reference tracking with <5% overshoot, input constraint |u| ≤ 2.

**💻 Solution Example**:
```matlab
% Exercise_Session16.m
% MPC for chemical reactor

% 🧹 Clear workspace
clear; clc;

% System: Chemical reactor
Ts = 0.5; % Sampling time
G = tf([1], [5 1]); % Plant: 1/(5s + 1)
sysd = c2d(G, Ts, 'zoh'); % Discrete model
disp('Discrete Plant:'); disp(sysd);

% 🔍 MPC Design
ctrl = mpc(sysd, Ts);
ctrl.PredictionHorizon = 20; % Long horizon for stability
ctrl.ControlHorizon = 5;
ctrl.Weights.Output = 10; % Emphasize tracking
ctrl.Weights.Manipulated = 0.1; % Moderate control effort
ctrl.MV.Min = -2; ctrl.MV.Max = 2; % Input constraints

% Simulation
t = 0:Ts:30; r = ones(size(t)); % Reference
[y, t, u] = sim(ctrl, t, r);
figure; subplot(2,1,1); plot(t, y, 'b-', t, r, 'r--'); title('Reactor Output'); grid on; legend('y', 'r');
subplot(2,1,2); plot(t, u, 'b-'); title('Control Input'); grid on;

% Verify Overshoot
info = stepinfo(y, t, 1);
if info.Overshoot < 5
    disp('Overshoot < 5% achieved!');
else
    disp('Adjust weights or horizons.');
end
```

**💬 Explanation**:
- **System**: Chemical reactor modeled as first-order system G(s) = 1/(5s + 1).
- **MPC**: Configured with long prediction horizon (20) and input constraints (|u| ≤ 2).
- **Simulation**: Tracks unit step reference, verifying overshoot <5% with `stepinfo`.
- Students can adjust weights or horizons if overshoot exceeds 5%.

---

## 📌 Additional Notes
- **✅ Best Practices**: Choose Ts based on system dynamics (e.g., Ts < 1/ω_max). Tune weights to balance tracking and control effort.
- **🔍 Debugging Tips**: Ensure `sysd` is stable or stabilizable. Check `sim` output for constraint violations.
- **🚀 Extensions**: Add output constraints or test in Simulink for realistic reactor dynamics.
- **🖥️ Lab Setup**: Requires MATLAB with Control System Toolbox and Model Predictive Control Toolbox for `mpc`, `c2d`, `sim`, `stepinfo`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 🔍 Understand MPC concepts, including horizons and constraints.
2. 🛠️ Implement MPC controllers using MATLAB’s `mpc` object.
3. 💻 Simulate and verify MPC performance under constraints.

**📅 Next Session Preview**: Adaptive control basics. 🔄

**📝 Assignment**: For G(s) = 1/(10s + 1), Ts=1s, design an MPC controller for <10% overshoot, |u| ≤ 1, simulate response, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
