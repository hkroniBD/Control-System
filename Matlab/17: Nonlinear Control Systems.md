# 🌪️ Lecture on Session 17: Nonlinear Control Systems

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Address nonlinearities in control systems. This session equips students with techniques to analyze and control nonlinear systems, such as the Van der Pol oscillator, using linearization, describing functions, and phase-plane analysis with MATLAB and Simulink tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-16 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation, state-space control, digital control, observer design, optimal control, robust control, MPC). Knowledge of differential equations, nonlinear dynamics, and Simulink. MATLAB with Control System Toolbox and Simulink installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/simulink](https://www.mathworks.com/help/simulink), [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Nonlinear Systems” by Hassan K. Khalil (Chapter 3 for linearization, Chapter 6 for describing functions)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Nonlinear Control section

---

## 🗂️ Topics Covered

1. 📉 **Linearization**: Around operating points using `linmod`.
2. 📊 **Describing Functions**: For analyzing limit cycles.
3. 📈 **Phase-Plane Analysis**: Custom plots for nonlinear dynamics.

---

## 📝 Detailed Lecture Content

### 📉 1. Linearization
Linearization approximates nonlinear systems around operating points to apply linear control techniques, critical for systems like oscillators near equilibrium. 📐

- **Concept**: For a nonlinear system dx/dt = f(x,u), y = g(x,u), linearize around equilibrium (x₀,u₀) to get dx/dt ≈ Ax + Bu, y ≈ Cx + Du, where A = ∂f/∂x, B = ∂f/∂u, C = ∂g/∂x, D = ∂g/∂u.
- **MATLAB Tool**: `linmod('simulink_model')` extracts a linear state-space model from a Simulink model at a specified operating condition.
- **Use Case**: Design linear controllers (e.g., PID) for nonlinear systems near operating points.

**💻 Example Code**:
```matlab
% Linearization of a Simulink model
% Requires Simulink model 'nonlinear_system' (placeholder)
% Example: Create a Simulink model with nonlinear block
sys_nl = linmod('nonlinear_system'); % Linearize at default operating condition
disp('Linearized System:'); disp(sys_nl);
A = sys_nl.a; B = sys_nl.b; C = sys_nl.c; D = sys_nl.d;
sys = ss(A, B, C, D);
figure; step(sys); title('Linearized System Step Response'); grid on;
```

**💬 Explanation**:
- `linmod` linearizes a Simulink model around the operating condition (e.g., x₀=0, u₀=0).
- `step` evaluates the linearized system’s response.
- Students must create a Simulink model (`nonlinear_system.slx`) with nonlinear dynamics.

---

### 📊 2. Describing Functions
Describing functions approximate nonlinear elements to analyze limit cycles in systems with nonlinearities, such as saturation or hysteresis. 📡

- **Concept**: Represents a nonlinearity (e.g., saturation) as a gain that depends on input amplitude, enabling frequency-domain analysis of oscillations.
- **Application**: Predicts existence and stability of limit cycles by analyzing the intersection of the nonlinearity’s describing function and the system’s frequency response.
- **MATLAB**: Requires custom computation of describing functions (not directly supported; use numerical methods).

**💻 Example Code**:
```matlab
% Describing function for saturation nonlinearity
% Compute describing function for saturation (limit: ±1)
a = 0:0.1:5; % Input amplitude range
N = zeros(size(a)); % Describing function values
for i = 1:length(a)
    if a(i) <= 1
        N(i) = 1; % Linear region
    else
        N(i) = (2/pi)*(asin(1/a(i)) + (1/a(i))*sqrt(1-(1/a(i))^2)); % Saturation
    end
end
figure; plot(a, N, 'b-', 'LineWidth', 2); title('Describing Function for Saturation'); xlabel('Amplitude (A)'); ylabel('N(A)'); grid on;
```

**💬 Explanation**:
- Computes the describing function N(A) for a saturation nonlinearity (limits ±1).
- Plots N(A) vs. amplitude to analyze potential limit cycles when paired with system G(jω).

---

### 📈 3. Phase-Plane Analysis
Phase-plane analysis visualizes nonlinear system trajectories in a state-space plane, revealing behaviors like limit cycles or stability. 📊

- **Concept**: Plots state trajectories (e.g., x₁ vs. x₂) to study dynamics, especially for second-order systems like oscillators.
- **MATLAB**: Use `ode45` to solve nonlinear differential equations and plot trajectories.
- **Use Case**: Analyze stability, limit cycles, or bifurcations in nonlinear systems.

**💻 Example Code**:
```matlab
% Phase-plane analysis for a nonlinear system
% Example: Van der Pol oscillator (dx/dt = y, dy/dt = -x + mu*(1-x^2)*y)
mu = 1;
vdp = @(t,x) [x(2); -x(1) + mu*(1-x(1)^2)*x(2)];
figure; hold on;
x0_vals = [0.5 0; 2 0; 0 2]; % Initial conditions
for i = 1:size(x0_vals,1)
    [t, x] = ode45(vdp, [0 20], x0_vals(i,:));
    plot(x(:,1), x(:,2), 'LineWidth', 2);
end
title('Phase-Plane: Van der Pol Oscillator'); xlabel('x'); ylabel('y'); grid on;
```

**💬 Explanation**:
- Solves the Van der Pol oscillator equations using `ode45`.
- Plots trajectories for different initial conditions, showing convergence to a limit cycle.

---

## 💻 Code Implementation (Combined Example)

Integrated script for linearization, describing function, and phase-plane analysis.

```matlab
% Session17_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Nonlinear control systems

% 🧹 Clear workspace
clear; clc;

% 📉 Linearization (requires Simulink model 'vdp_model')
% Create Simulink model for Van der Pol: dx/dt = y, dy/dt = -x + (1-x^2)y
% sys_nl = linmod('vdp_model'); % Linearize (uncomment with model)
% disp('Linearized System:'); disp(sys_nl);
% sys = ss(sys_nl.a, sys_nl.b, sys_nl.c, sys_nl.d);
% figure(1); step(sys); title('Linearized Step Response'); grid on;

% 📊 Describing Function (Saturation)
a = 0:0.1:5;
N = zeros(size(a));
for i = 1:length(a)
    if a(i) <= 1
        N(i) = 1;
    else
        N(i) = (2/pi)*(asin(1/a(i)) + (1/a(i))*sqrt(1-(1/a(i))^2));
    end
end
figure(2); plot(a, N, 'b-', 'LineWidth', 2); title('Describing Function for Saturation'); xlabel('Amplitude (A)'); ylabel('N(A)'); grid on;

% 📈 Phase-Plane Analysis (Van der Pol)
mu = 1;
vdp = @(t,x) [x(2); -x(1) + mu*(1-x(1)^2)*x(2)];
figure(3); hold on;
x0_vals = [0.5 0; 2 0; 0 2];
for i = 1:size(x0_vals,1)
    [t, x] = ode45(vdp, [0 20], x0_vals(i,:));
    plot(x(:,1), x(:,2), 'LineWidth', 2);
end
title('Phase-Plane: Van der Pol Oscillator'); xlabel('x'); ylabel('y'); grid on;
```

**💬 Explanation**:
- **Linearization**: Commented out (requires Simulink model `vdp_model.slx` for Van der Pol).
- **Describing Function**: Computes N(A) for saturation nonlinearity.
- **Phase-Plane**: Plots Van der Pol trajectories, showing limit cycle behavior.
- Save as `Session17_Example.m` and run to visualize outputs (create `vdp_model.slx` for linearization).

---

## 🛠️ Exercise
**Task**: Analyze the Van der Pol oscillator; linearize around x=0, design a linear controller to stabilize it. System: dx/dt = y, dy/dt = -x + μ(1-x²)y, μ=1.

**💻 Solution Example**:
```matlab
% Exercise_Session17.m
% Van der Pol oscillator analysis and control

% 🧹 Clear workspace
clear; clc;

% 📈 Phase-Plane Analysis
mu = 1;
vdp = @(t,x) [x(2); -x(1) + mu*(1-x(1)^2)*x(2)];
figure(1); hold on;
x0_vals = [0.5 0; 2 0; 0 2];
for i = 1:size(x0_vals,1)
    [t, x] = ode45(vdp, [0 20], x0_vals(i,:));
    plot(x(:,1), x(:,2), 'LineWidth', 2);
end
title('Phase-Plane: Van der Pol Oscillator'); xlabel('x'); ylabel('y'); grid on;

% 📉 Linearization (manual, around x=0, y=0)
A = [0 1; -1 mu]; % Jacobian: df/dx at (0,0)
B = [0; 1]; % Input (assume control u)
C = [1 0]; D = 0;
sys = ss(A, B, C, D);
disp('Linearized System:'); disp(sys);

% 💻 Linear Controller (LQR)
Q = eye(2); R = 1;
[K, ~, ~] = lqr(A, B, Q, R);
disp('LQR Gain K:'); disp(K);
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);
figure(2); step(sys_cl); title('Linearized Closed-Loop Response'); grid on;

% Verify Stability
if isstable(sys_cl)
    disp('Linearized system stabilized!');
else
    disp('Adjust controller gains.');
end
```

**💬 Explanation**:
- **System**: Van der Pol oscillator with μ=1, showing limit cycle behavior.
- **Phase-Plane**: Plots trajectories to visualize limit cycle.
- **Linearization**: Computes Jacobian at (0,0) for linear model.
- **Controller**: LQR stabilizes the linearized system; `step` verifies response.
- Note: Linear control may not fully stabilize nonlinear dynamics far from (0,0).

---

## 📌 Additional Notes
- **✅ Best Practices**: Linearize around correct operating points. Validate describing function with frequency response. Use multiple initial conditions for phase-plane.
- **🔍 Debugging Tips**: Ensure Simulink model for `linmod` is correctly set up. Check `ode45` integration for numerical stability.
- **🚀 Extensions**: Simulate nonlinear control in Simulink or analyze describing function with Nyquist plot.
- **🖥️ Lab Setup**: Requires MATLAB with Control System Toolbox and Simulink for `linmod`, `ode45`, `ss`, `lqr`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📉 Linearize nonlinear systems around operating points.
2. 📊 Analyze limit cycles using describing functions.
3. 📈 Perform phase-plane analysis for nonlinear dynamics.

**📅 Next Session Preview**: Adaptive control introduction. 🔄

**📝 Assignment**: For a nonlinear system dx/dt = -x + x³ + u, dy/dt = x, linearize, design LQR, plot phase-plane, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
