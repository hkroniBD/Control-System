# 🧩 Lecture on Session 19: Fuzzy Logic and Intelligent Control

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Implement rule-based control to handle uncertainty. This session equips students with skills to design and apply fuzzy logic controllers, such as fuzzy PID, for systems with uncertainty like temperature regulation, using MATLAB’s Fuzzy Logic Toolbox. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-18 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation, state-space control, digital control, observer design, optimal control, robust control, MPC, nonlinear control, adaptive control). Knowledge of control theory and MATLAB programming. MATLAB with Control System Toolbox, Simulink, and Fuzzy Logic Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/fuzzy](https://www.mathworks.com/help/fuzzy))
- 📖 “Fuzzy Logic with Engineering Applications” by Timothy J. Ross
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Fuzzy Logic section

---

## 🗂️ Topics Covered

1. 🧠 **Fuzzy Sets/Membership**: Fuzzification, inference, defuzzification.
2. ⚙️ **Fuzzy PID**: Designing fuzzy logic-based PID controllers.
3. 💻 **Code Implementation**: Using `readfis` and `evalfis`.

---

## 📝 Detailed Lecture Content

### 🧠 1. Fuzzy Sets/Membership
Fuzzy logic handles uncertainty by using fuzzy sets, where variables have membership degrees (0 to 1), ideal for systems with imprecise dynamics like temperature control. 📊

- **Fuzzy Sets**: Variables (e.g., error, change in error) are assigned membership functions (e.g., triangular, Gaussian) defining their degree of belonging to sets like “Low,” “Medium,” “High.”
- **Fuzzification**: Converts crisp inputs (e.g., error) into fuzzy sets using membership functions.
- **Inference**: Applies rule base (e.g., IF error is High AND change is Low THEN output is Medium) using logical operations.
- **Defuzzification**: Converts fuzzy outputs to crisp control signals (e.g., using centroid method).
- **MATLAB Tool**: Fuzzy Logic Toolbox with `fuzzy` GUI or `readfis` to load fuzzy inference systems (FIS).

**💻 Example Code**:
```matlab
% Create and evaluate a simple fuzzy system
fis = newfis('example_fis'); % Create FIS
% Input: Error
fis = addvar(fis, 'input', 'error', [-10 10]);
fis = addmf(fis, 'input', 1, 'Negative', 'trimf', [-10 -5 0]);
fis = addmf(fis, 'input', 1, 'Zero', 'trimf', [-2.5 0 2.5]);
fis = addmf(fis, 'input', 1, 'Positive', 'trimf', [0 5 10]);
% Output: Control
fis = addvar(fis, 'output', 'control', [-1 1]);
fis = addmf(fis, 'output', 1, 'Negative', 'trimf', [-1 -0.5 0]);
fis = addmf(fis, 'output', 1, 'Zero', 'trimf', [-0.25 0 0.25]);
fis = addmf(fis, 'output', 1, 'Positive', 'trimf', [0 0.5 1]);
% Rules
ruleList = [1 1 1 1; 2 2 1 1; 3 3 1 1]; % Negative->Negative, Zero->Zero, Positive->Positive
fis = addrule(fis, ruleList);
% Evaluate
input = 3; % Example input
output = evalfis(input, fis);
disp('Fuzzy Output:'); disp(output);
```

**💬 Explanation**:
- `newfis` creates a fuzzy inference system (FIS).
- `addvar`, `addmf` define inputs/outputs and membership functions.
- `addrule` sets rules; `evalfis` computes output for a given input.

---

### ⚙️ 2. Fuzzy PID
Fuzzy PID controllers combine fuzzy logic with PID principles to handle nonlinearities and uncertainties, improving robustness over traditional PID. 🔧

- **Concept**: Uses fuzzy rules to map error (e), change in error (de/dt), and integral of error to control actions, mimicking P, I, D terms.
- **Design**: Define membership functions for e, de/dt, and control output; create rules like “IF e is Large AND de/dt is Small THEN control is Medium.”
- **MATLAB Tool**: Fuzzy Logic Toolbox to design FIS, integrated with Simulink for simulation.

**💻 Example Code**:
```matlab
% Fuzzy PID setup (requires Simulink model 'fuzzy_pid_model')
% Create FIS for fuzzy PID
fis = newfis('fuzzy_pid');
% Inputs: Error, Change in Error
fis = addvar(fis, 'input', 'error', [-1 1]);
fis = addmf(fis, 'input', 1, 'Negative', 'trimf', [-1 -0.5 0]);
fis = addmf(fis, 'input', 1, 'Zero', 'trimf', [-0.25 0 0.25]);
fis = addmf(fis, 'input', 1, 'Positive', 'trimf', [0 0.5 1]);
fis = addvar(fis, 'input', 'derror', [-1 1]);
fis = addmf(fis, 'input', 2, 'Negative', 'trimf', [-1 -0.5 0]);
fis = addmf(fis, 'input', 2, 'Zero', 'trimf', [-0.25 0 0.25]);
fis = addmf(fis, 'input', 2, 'Positive', 'trimf', [0 0.5 1]);
% Output: Control
fis = addvar(fis, 'output', 'control', [-2 2]);
fis = addmf(fis, 'output', 1, 'Negative', 'trimf', [-2 -1 0]);
fis = addmf(fis, 'output', 1, 'Zero', 'trimf', [-0.5 0 0.5]);
fis = addmf(fis, 'output', 1, 'Positive', 'trimf', [0 1 2]);
% Rules (simplified)
ruleList = [1 1 1 1 1; 2 2 2 1 1; 3 3 3 1 1]; % e,derror -> control
fis = addrule(fis, ruleList);
writefis(fis, 'fuzzy_pid.fis'); % Save FIS
```

**💬 Explanation**:
- Creates a fuzzy PID with inputs (error, change in error) and output (control).
- Rules mimic PID behavior (e.g., large error → large control).
- Save FIS as `fuzzy_pid.fis` for Simulink integration.

---

### 💻 3. Code Implementation
Fuzzy logic control implementation uses `readfis` to load a fuzzy inference system and `evalfis` for evaluation, with Simulink for closed-loop simulation. 🔧

- **Workflow**: Design FIS, save to `.fis` file, load with `readfis`, simulate in Simulink or MATLAB.
- **MATLAB Tools**: `newfis`, `addvar`, `addmf`, `addrule`, `writefis`, `readfis`, `evalfis`.

**💻 Example Code**:
```matlab
% Session19_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Fuzzy logic control

% 🧹 Clear workspace
clear; clc;

% 🧠 Fuzzy System Design
fis = newfis('example_fis');
fis = addvar(fis, 'input', 'error', [-10 10]);
fis = addmf(fis, 'input', 1, 'Negative', 'trimf', [-10 -5 0]);
fis = addmf(fis, 'input', 1, 'Zero', 'trimf', [-2.5 0 2.5]);
fis = addmf(fis, 'input', 1, 'Positive', 'trimf', [0 5 10]);
fis = addvar(fis, 'output', 'control', [-1 1]);
fis = addmf(fis, 'output', 1, 'Negative', 'trimf', [-1 -0.5 0]);
fis = addmf(fis, 'output', 1, 'Zero', 'trimf', [-0.25 0 0.25]);
fis = addmf(fis, 'output', 1, 'Positive', 'trimf', [0 0.5 1]);
ruleList = [1 1 1 1; 2 2 1 1; 3 3 1 1];
fis = addrule(fis, ruleList);
writefis(fis, 'example_fis.fis');

% 💻 Evaluate FIS
input = [3]; % Example input
output = evalfis(input, fis);
disp('Fuzzy Output:'); disp(output);

% ⚙️ Fuzzy PID (requires Simulink model 'fuzzy_pid_model')
% Create Simulink model with Fuzzy Logic Controller block
% fis_pid = readfis('fuzzy_pid.fis');
% disp('Fuzzy PID:'); disp(fis_pid);
```

**💬 Explanation**:
- **Fuzzy System**: Designs and saves a simple FIS, evaluates with `evalfis`.
- **Fuzzy PID**: Prepares FIS for Simulink (commented out, requires model).
- Save as `Session19_Example.m` and run; create Simulink model for fuzzy PID simulation.

---

## 🛠️ Exercise
**Task**: Design a fuzzy controller for temperature regulation. System: G(s) = 1/(10s + 1), setpoint 25°C, inputs: error (e), change in error (de/dt), output: heater power (0 to 100). Achieve stable tracking.

**💻 Solution Example**:
```matlab
% Exercise_Session19.m
% Fuzzy controller for temperature regulation

% 🧹 Clear workspace
clear; clc;

% 🧠 Fuzzy Controller Design
fis = newfis('temp_control');
% Input: Error (e = setpoint - temp)
fis = addvar(fis, 'input', 'error', [-10 10]); % ±10°C
fis = addmf(fis, 'input', 1, 'Negative', 'trimf', [-10 -5 0]);
fis = addmf(fis, 'input', 1, 'Zero', 'trimf', [-2.5 0 2.5]);
fis = addmf(fis, 'input', 1, 'Positive', 'trimf', [0 5 10]);
% Input: Change in Error (de/dt)
fis = addvar(fis, 'input', 'derror', [-5 5]); % ±5°C/s
fis = addmf(fis, 'input', 2, 'Negative', 'trimf', [-5 -2.5 0]);
fis = addmf(fis, 'input', 2, 'Zero', 'trimf', [-1.25 0 1.25]);
fis = addmf(fis, 'input', 2, 'Positive', 'trimf', [0 2.5 5]);
% Output: Heater Power
fis = addvar(fis, 'output', 'power', [0 100]); % 0-100%
fis = addmf(fis, 'output', 1, 'Low', 'trimf', [0 25 50]);
fis = addmf(fis, 'output', 1, 'Medium', 'trimf', [25 50 75]);
fis = addmf(fis, 'output', 1, 'High', 'trimf', [50 75 100]);
% Rules
ruleList = [1 1 3 1 1; 2 2 2 1 1; 3 3 1 1 1]; % e,derror -> power
fis = addrule(fis, ruleList);
writefis(fis, 'temp_control.fis');

% 💻 Simulation (requires Simulink model 'temp_control_model')
% System: G(s) = 1/(10s + 1)
% Create Simulink model with Fuzzy Logic Controller block, setpoint = 25°C
% fis = readfis('temp_control.fis');
% sim('temp_control_model');
% figure; plot(tout, yout, 'b-', tout, 25*ones(size(tout)), 'r--');
% title('Temperature Regulation'); xlabel('Time (s)'); ylabel('Temperature (°C)'); grid on; legend('Output', 'Setpoint');
```

**💬 Explanation**:
- **System**: Temperature process modeled as G(s) = 1/(10s + 1).
- **Fuzzy Controller**: FIS with inputs (error, change in error) and output (heater power), rules for stable tracking.
- **Simulation**: Requires Simulink model (`temp_control_model.slx`) with Fuzzy Logic Controller block to simulate tracking of 25°C setpoint.
- Students should create the Simulink model and verify tracking performance.

---

## 📌 Additional Notes
- **✅ Best Practices**: Design intuitive membership functions and rules. Validate fuzzy controller in Simulink for realistic dynamics.
- **🔍 Debugging Tips**: Check FIS structure with `showfis`. Ensure Simulink model includes proper plant and setpoint.
- **🚀 Extensions**: Add integral action to fuzzy PID or test with disturbances.
- **🖥️ Lab Setup**: Requires MATLAB with Control System Toolbox, Simulink, and Fuzzy Logic Toolbox for `newfis`, `readfis`, `evalfis`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 🧠 Design fuzzy logic systems with fuzzification, inference, and defuzzification.
2. ⚙️ Implement fuzzy PID controllers.
3. 💻 Simulate fuzzy controllers in MATLAB/Simulink.

**📅 Next Session Preview**: Control system integration and real-world applications. 🌍

**📝 Assignment**: For G(s) = 1/(5s + 1), design a fuzzy controller for setpoint 50, inputs: error, change in error, output: control (0-100), simulate in Simulink, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
