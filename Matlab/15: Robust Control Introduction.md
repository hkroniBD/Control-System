# 💪 Lecture on Session 15: Robust Control Introduction

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Handle uncertainties in control systems. This session introduces students to robust control techniques, focusing on H-infinity control and mu-synthesis, to design controllers for systems with uncertainties, such as flexible beams, using MATLAB’s Robust Control Toolbox. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-14 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation, state-space control, digital control, observer design, optimal control). Knowledge of state-space models, frequency-domain analysis, and linear algebra. MATLAB with Control System Toolbox and Robust Control Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/robust](https://www.mathworks.com/help/robust))
- 📖 “Robust Control Design with MATLAB” by Da-Wei Gu, Petko H. Petkov, and Mihail M. Konstantinov
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Robust Control section

---

## 🗂️ Topics Covered

1. 📉 **H-Infinity Control**: Minimizing H-infinity norm for robustness.
2. 📊 **Mu-Synthesis**: Structured singular value for handling uncertainties.
3. 💻 **Code Implementation**: Using `hinfsyn` from Robust Control Toolbox.

---

## 📝 Detailed Lecture Content

### 📉 1. H-Infinity Control
H-infinity control designs controllers to minimize the worst-case gain (H-infinity norm) of a system, ensuring robustness against uncertainties like parameter variations or disturbances. 📈

- **Concept**: For a generalized plant P(s) with inputs (disturbances, controls) and outputs (errors, measurements), H-infinity control finds a controller K(s) to minimize the H-infinity norm of the closed-loop transfer function from disturbances to errors.
- **Objective**: Ensure stability and performance under worst-case scenarios.
- **MATLAB Tool**: `hinfsyn(P,nmeas,ncont)` computes an H-infinity optimal controller, where nmeas is the number of measurements and ncont is the number of control inputs.

**💻 Example Code**:
```matlab
% H-infinity control design
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);
% Generalized plant (augmented with weights)
W1 = 0.1*tf(1,[1 0]); % Error weight
W2 = 0.01; % Control weight
P = augw(sys, W1, W2); % Augmented plant
[K_hinf, CL, gamma] = hinfsyn(P, 1, 1); % nmeas=1, ncont=1
disp('H-infinity Controller:'); disp(K_hinf);
disp('H-infinity Norm (gamma):'); disp(gamma);
figure; step(CL); title('H-infinity Closed-Loop Response'); grid on;
```

**💬 Explanation**:
- `augw` constructs a generalized plant with performance weights W1 (error) and W2 (control effort).
- `hinfsyn` computes controller K_hinf, minimizing the H-infinity norm (gamma).
- `step` evaluates the closed-loop response.

---

### 📊 2. Mu-Synthesis
Mu-synthesis uses the structured singular value (μ) to design controllers for systems with structured uncertainties, ensuring robust performance. 📡

- **Concept**: The structured singular value μ quantifies the smallest uncertainty that destabilizes the system. Mu-synthesis optimizes controllers to minimize μ, ensuring stability against specific uncertainty structures (e.g., parametric, dynamic).
- **MATLAB Tool**: `dksyn(P,nmeas,ncont)` performs mu-synthesis via D-K iteration, combining H-infinity control with scaling to handle structured uncertainties.

**💻 Example Code**:
```matlab
% Mu-synthesis (requires uncertainty model)
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);
% Define uncertainty (e.g., parametric in A(2,2))
unc = udiag(0.3); % 30% uncertainty in A(2,2)
sys_unc = uss(sys, 'a22', unc);
P = augw(sys_unc, 0.1*tf(1,[1 0]), 0.01); % Augmented plant
[K_mu, CL, bnd] = dksyn(P, 1, 1);
disp('Mu-Synthesis Controller:'); disp(K_mu);
disp('Mu Upper Bound:'); disp(bnd);
figure; step(CL); title('Mu-Synthesis Closed-Loop Response'); grid on;
```

**💬 Explanation**:
- `uss` models the system with uncertainty (e.g., 30% variation in A(2,2)).
- `dksyn` computes a controller minimizing μ, ensuring robust stability.
- `bnd` is the upper bound of μ, indicating robustness level.

---

### 💻 3. Code Implementation
Robust control implementation uses `hinfsyn` for H-infinity and `dksyn` for mu-synthesis, requiring the Robust Control Toolbox. 🔧

- **Workflow**: Model system with uncertainties, construct augmented plant, design controller, simulate closed-loop response.
- **MATLAB Tools**: `augw`, `hinfsyn`, `dksyn`, `step` for design and analysis.

**💻 Example Code**:
```matlab
% Session15_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Robust control: H-infinity and mu-synthesis

% 🧹 Clear workspace
clear; clc;

% System
A = [0 1; -2 -3]; B = [0; 1]; C = [1 0]; D = 0;
sys = ss(A, B, C, D);

% 📉 H-infinity Control
W1 = 0.1*tf(1,[1 0]); % Error weight
W2 = 0.01; % Control weight
P = augw(sys, W1, W2);
[K_hinf, CL_hinf, gamma] = hinfsyn(P, 1, 1);
disp('H-infinity Controller:'); disp(K_hinf);
disp('H-infinity Norm (gamma):'); disp(gamma);
figure(1); step(CL_hinf); title('H-infinity Closed-Loop Response'); grid on;

% 📊 Mu-Synthesis
unc = udiag(0.3); % 30% uncertainty
sys_unc = uss(sys, 'a22', unc);
P_unc = augw(sys_unc, W1, W2);
[K_mu, CL_mu, bnd] = dksyn(P_unc, 1, 1);
disp('Mu-Synthesis Controller:'); disp(K_mu);
disp('Mu Upper Bound:'); disp(bnd);
figure(2); step(CL_mu); title('Mu-Synthesis Closed-Loop Response'); grid on;
```

**💬 Explanation**:
- **H-infinity**: Designs controller to minimize worst-case gain.
- **Mu-Synthesis**: Accounts for structured uncertainty, ensuring robust performance.
- Save as `Session15_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Design a robust controller for a flexible beam with uncertainties. System: G(s) = 1/(s² + 0.2s + 1) with ±20% uncertainty in natural frequency (ωn²=1). Achieve stable response.

**💻 Solution Example**:
```matlab
% Exercise_Session15.m
% Robust control for flexible beam

% 🧹 Clear workspace
clear; clc;

% System: Flexible beam with uncertainty
wn_nom = 1; % Nominal natural frequency
zeta = 0.1; % Damping
G_nom = tf([1], [1 2*zeta*wn_nom wn_nom^2]); % Nominal: 1/(s^2 + 0.2s + 1)
unc = udiag(0.2); % ±20% uncertainty in wn^2
G_unc = uss(G_nom, 'wn2', unc); % Uncertain system

% 📉 H-infinity Control
W1 = 0.1*tf(1,[1 0]); % Error weight
W2 = 0.01; % Control weight
P = augw(G_unc, W1, W2);
[K_hinf, CL_hinf, gamma] = hinfsyn(P, 1, 1);
disp('H-infinity Controller:'); disp(K_hinf);
disp('H-infinity Norm (gamma):'); disp(gamma);

% Simulation
figure; step(CL_hinf); title('H-infinity Closed-Loop Response for Flexible Beam'); grid on;
if isstable(CL_hinf)
    disp('System stabilized with H-infinity!');
else
    disp('Adjust weights or model.');
end
```

**💬 Explanation**:
- **System**: Flexible beam with G(s) = 1/(s² + 0.2s + 1), uncertainty in ωn² modeled via `uss`.
- **H-infinity**: `hinfsyn` designs a controller to ensure robust stability.
- **Simulation**: `step` verifies stable response; `isstable` confirms robustness.

---

## 📌 Additional Notes
- **✅ Best Practices**: Tune weights W1, W2 to balance performance and robustness. Ensure system is controllable/observable.
- **🔍 Debugging Tips**: Check `augw` setup for correct input/output structure. Verify `hinfsyn` convergence (gamma < ∞).
- **🚀 Extensions**: Test mu-synthesis or simulate with Simulink for realistic uncertainties.
- **🖥️ Lab Setup**: Requires MATLAB with Control System Toolbox and Robust Control Toolbox for `hinfsyn`, `dksyn`, `augw`, `uss`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📉 Design H-infinity controllers for robust performance.
2. 📊 Apply mu-synthesis for structured uncertainties.
3. 💻 Implement and simulate robust controllers.

**📅 Next Session Preview**: Nonlinear control introduction. 🌊

**📝 Assignment**: For G(s) = 1/(s² + s + 1) with ±10% uncertainty in s² term, design an H-infinity controller, simulate response, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
