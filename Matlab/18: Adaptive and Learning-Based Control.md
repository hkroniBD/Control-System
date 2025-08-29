# 🤖 Lecture on Session 18: Adaptive and Learning-Based Control

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Design controllers that adapt to system changes. This session equips students with skills to implement adaptive and learning-based control techniques, such as Model Reference Adaptive Control (MRAC) and neural network control, for systems like motors with varying loads, using MATLAB tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-17 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation, state-space control, digital control, observer design, optimal control, robust control, MPC, nonlinear control). Knowledge of control theory, differential equations, and neural networks. MATLAB with Control System Toolbox and Neural Network Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control), [www.mathworks.com/help/deeplearning](https://www.mathworks.com/help/deeplearning))
- 📖 “Adaptive Control” by Karl J. Åström and Björn Wittenmark (Chapter 3 for MRAC)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Adaptive Control section

---

## 🗂️ Topics Covered

1. 🔄 **Model Reference Adaptive Control (MRAC)**: Adapting to track a reference model.
2. 🧠 **Neural Network Control**: Introduction using Neural Network Toolbox.
3. 💻 **Code Implementation**: Custom adaptive scripts and `nntool`.

---

## 📝 Detailed Lecture Content

### 🔄 1. Model Reference Adaptive Control (MRAC)
MRAC adjusts controller parameters in real-time to make the system follow a reference model’s behavior, ideal for systems with varying parameters like motor loads. 📈

- **Concept**: For a plant y = G(s)u, MRAC adjusts the controller so the closed-loop system matches a reference model y_m = G_m(s)r. The adaptation law updates parameters based on the error e = y_m - y.
- **Structure**: Typically uses a control law u = θ^T ω (θ: adjustable parameters, ω: regressor) and an adaptation law like dθ/dt = -γ e ω (γ: adaptation gain).
- **MATLAB**: Implement custom scripts for MRAC, as MATLAB lacks a direct MRAC function.

**💻 Example Code**:
```matlab
% MRAC for a first-order system
% Plant: dy/dt = -a*y + b*u, Reference: dy_m/dt = -am*y_m + bm*r
am = 2; bm = 2; % Reference model
a = 1; b = 1; % Nominal plant
gamma = 1; % Adaptation gain
T = 10; dt = 0.01; t = 0:dt:T;
y = 0; y_m = 0; theta = [0; 0]; % Initial conditions
r = ones(size(t)); % Reference input
y_data = zeros(size(t)); y_m_data = zeros(size(t));
for i = 1:length(t)-1
    % Reference model
    y_m = y_m + dt * (-am*y_m + bm*r(i));
    % Plant
    u = theta(1)*r(i) + theta(2)*y; % Control law
    y = y + dt * (-a*y + b*u);
    % Adaptation law
    e = y_m - y;
    theta = theta + dt * gamma * e * [r(i); y];
    y_data(i) = y; y_m_data(i) = y_m;
end
figure; plot(t, y_data, 'b-', t, y_m_data, 'r--'); title('MRAC Response'); xlabel('Time (s)'); ylabel('Output'); grid on; legend('Plant', 'Reference');
```

**💬 Explanation**:
- Implements MRAC for a first-order plant to track a reference model.
- Adaptation law updates θ to minimize tracking error.
- Plots plant and reference model outputs.

---

### 🧠 2. Neural Network Control
Neural networks provide learning-based control for complex or uncertain systems, using MATLAB’s Neural Network Toolbox to approximate control laws. 🧬

- **Concept**: A neural network is trained to map system states/inputs to control actions, learning from data or simulations.
- **Application**: Control systems with nonlinearities or parameter variations (e.g., varying load motors).
- **MATLAB Tool**: `nntool` or `fitnet` for designing and training neural networks.

**💻 Example Code**:
```matlab
% Neural network controller (basic example)
% Simulate data for training: y = k*u (k varies)
rng(0); % For reproducibility
u = rand(1, 1000); % Random inputs
k = 1 + 0.5*rand(1, 1000); % Varying gain
y = k .* u; % Outputs
net = fitnet(10); % Neural network with 10 hidden neurons
net = train(net, u, y); % Train to map u to y
% Test
u_test = 0:0.01:1; y_test = zeros(size(u_test));
for i = 1:length(u_test)
    y_test(i) = net(u_test(i));
end
figure; plot(u_test, y_test, 'b-', u_test, u_test, 'r--'); title('Neural Network Control'); xlabel('Input'); ylabel('Output'); grid on; legend('NN Output', 'Nominal');
```

**💬 Explanation**:
- Trains a neural network to approximate a varying-gain system.
- `fitnet` creates a network; `train` uses input-output data.
- Tests network prediction against nominal system.

---

### 💻 3. Code Implementation
Adaptive and learning-based control implementation involves custom MRAC scripts and neural network design with `nntool` or programmatic methods. 🔧

- **Workflow**: Model system, implement adaptation law for MRAC, train neural network for control, simulate performance.
- **MATLAB Tools**: `ode45` for MRAC simulation, `fitnet` or `nntool` for neural networks.

**💻 Example Code**:
```matlab
% Session18_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Adaptive and learning-based control

% 🧹 Clear workspace
clear; clc;

% 🔄 MRAC
am = 2; bm = 2; a = 1; b = 1; gamma = 1;
T = 10; dt = 0.01; t = 0:dt:T;
y = 0; y_m = 0; theta = [0; 0];
r = ones(size(t));
y_data = zeros(size(t)); y_m_data = zeros(size(t));
for i = 1:length(t)-1
    y_m = y_m + dt * (-am*y_m + bm*r(i));
    u = theta(1)*r(i) + theta(2)*y;
    y = y + dt * (-a*y + b*u);
    e = y_m - y;
    theta = theta + dt * gamma * e * [r(i); y];
    y_data(i) = y; y_m_data(i) = y_m;
end
figure(1); plot(t, y_data, 'b-', t, y_m_data, 'r--'); title('MRAC Response'); xlabel('Time (s)'); ylabel('Output'); grid on; legend('Plant', 'Reference');

% 🧠 Neural Network Control
rng(0);
u = rand(1, 1000); k = 1 + 0.5*rand(1, 1000); y = k .* u;
net = fitnet(10); net = train(net, u, y);
u_test = 0:0.01:1; y_test = zeros(size(u_test));
for i = 1:length(u_test)
    y_test(i) = net(u_test(i));
end
figure(2); plot(u_test, y_test, 'b-', u_test, u_test, 'r--'); title('Neural Network Control'); xlabel('Input'); ylabel('Output'); grid on; legend('NN Output', 'Nominal');
```

**💬 Explanation**:
- **MRAC**: Implements adaptation for a first-order system to track a reference model.
- **Neural Network**: Trains a network to approximate a varying-gain system.
- Save as `Session18_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Design adaptive control for a motor with varying load. System: dy/dt = -a*y + b*u, a=1+0.5*sin(t) (time-varying), b=1, track reference model dy_m/dt = -2*y_m + 2*r.

**💻 Solution Example**:
```matlab
% Exercise_Session18.m
% Adaptive control for varying load motor

% 🧹 Clear workspace
clear; clc;

% 🔄 MRAC for varying load motor
am = 2; bm = 2; % Reference model
b = 1; gamma = 1; % Plant and adaptation gain
T = 20; dt = 0.01; t = 0:dt:T;
y = 0; y_m = 0; theta = [0; 0];
r = ones(size(t));
y_data = zeros(size(t)); y_m_data = zeros(size(t));
for i = 1:length(t)-1
    a = 1 + 0.5*sin(t(i)); % Time-varying parameter
    y_m = y_m + dt * (-am*y_m + bm*r(i));
    u = theta(1)*r(i) + theta(2)*y;
    y = y + dt * (-a*y + b*u);
    e = y_m - y;
    theta = theta + dt * gamma * e * [r(i); y];
    y_data(i) = y; y_m_data(i) = y_m;
end
figure; plot(t, y_data, 'b-', t, y_m_data, 'r--'); title('MRAC for Varying Load Motor'); xlabel('Time (s)'); ylabel('Output'); grid on; legend('Plant', 'Reference');

% Verify Tracking
e_final = abs(y_data(end) - y_m_data(end));
if e_final < 0.01
    disp('Tracking achieved!');
else
    disp('Adjust gamma or simulation time.');
end
```

**💬 Explanation**:
- **System**: Motor with time-varying parameter a=1+0.5*sin(t).
- **MRAC**: Adapts θ to track reference model, minimizing error e = y_m - y.
- **Simulation**: Plots plant and reference outputs, verifying tracking.
- Students can adjust γ for faster convergence.

---

## 📌 Additional Notes
- **✅ Best Practices**: Choose appropriate reference model for MRAC. Use sufficient training data for neural networks.
- **🔍 Debugging Tips**: Ensure adaptation gain γ is not too large (instability). Check neural network training convergence.
- **🚀 Extensions**: Implement MRAC in Simulink or train neural network for nonlinear plant.
- **🖥️ Lab Setup**: Requires MATLAB with Control System Toolbox and Neural Network Toolbox for `fitnet`, `nntool`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 🔄 Design MRAC systems to track reference models.
2. 🧠 Implement neural network controllers for uncertain systems.
3. 💻 Simulate adaptive and learning-based control.

**📅 Next Session Preview**: Control system case studies and applications. 🌟

**📝 Assignment**: For dy/dt = -a*y + u, a=1+0.3*cos(t), design MRAC to track dy_m/dt = -3*y_m + 3*r, simulate, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
