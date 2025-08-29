# 🔗 Lecture on Session 20: Multi-Variable and Decentralized Control

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Handle coupled multi-input multi-output (MIMO) systems. This session equips students with skills to analyze and control MIMO systems, such as a two-tank level system, using transfer function matrices and decoupling techniques, with MATLAB tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-19 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation, state-space control, digital control, observer design, optimal control, robust control, MPC, nonlinear control, adaptive control, fuzzy logic control). Knowledge of state-space models, transfer functions, and linear algebra. MATLAB with Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Multivariable Feedback Control” by Sigurd Skogestad and Ian Postlethwaite
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – MIMO Control section

---

## 🗂️ Topics Covered

1. 📉 **MIMO Systems**: Transfer function matrices for multi-input multi-output systems.
2. 🔄 **Decoupling**: Interaction analysis and controller design to minimize coupling.
3. 💻 **Code Implementation**: Using `ss` and `minreal` for MIMO systems.

---

## 📝 Detailed Lecture Content

### 📉 1. MIMO Systems
Multi-input multi-output (MIMO) systems have multiple inputs and outputs, with interactions modeled by transfer function matrices, critical for systems like coupled tank levels. 📊

- **Concept**: A MIMO system is represented as y(s) = G(s)u(s), where G(s) is an m×n transfer function matrix, y is an m×1 output vector, and u is an n×1 input vector.
- **State-Space Model**: dx/dt = Ax + Bu, y = Cx + Du, where B, C, D reflect multiple inputs/outputs.
- **MATLAB Tool**: `ss(A,B,C,D)` for state-space models; `tf` for transfer function matrices.

**💻 Example Code**:
```matlab
% MIMO system modeling
A = [0 1; -2 -3]; B = [0 1; 1 0]; C = [1 0; 0 1]; D = [0 0; 0 0];
sys_mimo = ss(A, B, C, D); % 2 inputs, 2 outputs
sys_mimo_red = minreal(sys_mimo); % Reduce model
disp('MIMO System:'); disp(sys_mimo);
figure; step(sys_mimo); title('MIMO Step Response'); grid on;
```

**💬 Explanation**:
- `ss` creates a MIMO state-space model with 2 inputs and 2 outputs.
- `minreal` eliminates unobservable/uncontrollable states.
- `step` plots responses for each input-output pair.

---

### 🔄 2. Decoupling
Decoupling minimizes interactions between inputs and outputs in MIMO systems, simplifying control design for systems like coupled tanks. 🔧

- **Concept**: Interactions occur when G(s) is not diagonal (off-diagonal terms cause coupling). Decoupling designs a controller or precompensator to make the closed-loop system nearly diagonal.
- **Interaction Analysis**: Use Relative Gain Array (RGA) to assess coupling: RGA = G(0) .* (G(0)^(-1))', where G(0) is the steady-state gain matrix.
- **MATLAB**: Compute RGA manually or use `feedback` for decentralized control.

**💻 Example Code**:
```matlab
% RGA for interaction analysis
G = tf({[1], [0.5]; [0.5], [1]}, {[1 1], [1 1]; [1 1], [1 1]}); % 2x2 transfer function matrix
G0 = dcgain(G); % Steady-state gain
RGA = G0 .* (inv(G0))'; % Relative Gain Array
disp('RGA:'); disp(RGA);
% Decentralized control (simplified)
C = tf({[1 0.1], [0]; [0], [1 0.1]}, [1], [1]); % Diagonal PID controllers
sys_cl = feedback(G*C, eye(2));
figure; step(sys_cl); title('Decentralized Control Response'); grid on;
```

**💬 Explanation**:
- `dcgain` computes steady-state gain matrix G(0).
- RGA values near 1 on diagonal indicate low coupling.
- Diagonal PID controllers reduce interaction in closed-loop.

---

### 💻 3. Code Implementation
MIMO system implementation uses `ss` for modeling and `minreal` for model reduction, with `feedback` for closed-loop analysis. 🔧

- **Workflow**: Model MIMO system, analyze interactions, design decentralized controllers, simulate performance.
- **MATLAB Tools**: `ss`, `minreal`, `tf`, `feedback`, `step`.

**💻 Example Code**:
```matlab
% Session20_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% MIMO and decentralized control

% 🧹 Clear workspace
clear; clc;

% 📉 MIMO System
A = [0 1; -2 -3]; B = [0 1; 1 0]; C = [1 0; 0 1]; D = [0 0; 0 0];
sys_mimo = ss(A, B, C, D);
sys_mimo_red = minreal(sys_mimo);
disp('Reduced MIMO System:'); disp(sys_mimo_red);
figure(1); step(sys_mimo_red); title('MIMO Step Response'); grid on;

% 🔄 Interaction Analysis and Decentralized Control
G = tf({[1], [0.5]; [0.5], [1]}, {[1 1], [1 1]; [1 1], [1 1]});
G0 = dcgain(G);
RGA = G0 .* (inv(G0))';
disp('RGA:'); disp(RGA);
C = tf({[1 0.1], [0]; [0], [1 0.1]}, [1], [1]); % Diagonal PIDs
sys_cl = feedback(G*C, eye(2));
figure(2); step(sys_cl); title('Decentralized Control Response'); grid on;
```

**💬 Explanation**:
- **MIMO System**: Models a 2×2 system, reduced with `minreal`.
- **Decoupling**: Computes RGA and applies diagonal PID controllers.
- Save as `Session20_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Control a two-tank level system (MIMO). System: Two coupled tanks with state-space model A=[-0.02 0.01; 0.01 -0.02], B=[0.1 0; 0 0.1], C=eye(2), D=zeros(2), control liquid levels.

**💻 Solution Example**:
```matlab
% Exercise_Session20.m
% MIMO control for two-tank system

% 🧹 Clear workspace
clear; clc;

% 📉 Two-Tank System
A = [-0.02 0.01; 0.01 -0.02]; % Coupled dynamics
B = [0.1 0; 0 0.1]; % Two inputs
C = eye(2); D = zeros(2); % Two outputs (levels)
sys_mimo = ss(A, B, C, D);
sys_mimo_red = minreal(sys_mimo);
disp('Reduced MIMO System:'); disp(sys_mimo_red);

% 🔄 Interaction Analysis
G = tf(sys_mimo);
G0 = dcgain(G);
RGA = G0 .* (inv(G0))';
disp('RGA:'); disp(RGA);

% Decentralized Control (PI controllers)
C = tf({[1 0.1], [0]; [0], [1 0.1]}, [1], [1]);
sys_cl = feedback(G*C, eye(2));
figure; step(sys_cl); title('Two-Tank Level Control Response'); grid on; xlabel('Time (s)'); ylabel('Tank Levels');
if isstable(sys_cl)
    disp('System stabilized!');
else
    disp('Adjust controller gains.');
end
```

**💬 Explanation**:
- **System**: Two-tank system with coupling (off-diagonal A terms), modeled in state-space.
- **RGA**: Analyzes interaction; values near 1 suggest diagonal control is feasible.
- **Control**: Diagonal PI controllers stabilize tank levels, verified with `step`.
- Students can tune PI gains for better performance.

---

## 📌 Additional Notes
- **✅ Best Practices**: Use RGA to guide controller pairing. Ensure controllability/observability with `ctrb`, `obsv`.
- **🔍 Debugging Tips**: Check `minreal` output for model validity. Verify `feedback` connections for MIMO systems.
- **🚀 Extensions**: Design state feedback or test in Simulink with realistic tank dynamics.
- **🖥️ Lab Setup**: Requires MATLAB with Control System Toolbox for `ss`, `minreal`, `tf`, `feedback`, `dcgain`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📉 Model and analyze MIMO systems using transfer function matrices.
2. 🔄 Design decentralized controllers with interaction analysis.
3. 💻 Simulate MIMO control performance.

**📅 Next Session Preview**: Control system case studies and integration. 🌍

**📝 Assignment**: For a MIMO system with G(s) = {[1/(s+1), 0.2/(s+1)]; [0.2/(s+1), 1/(s+1)]}, design decentralized PI controllers, simulate, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
