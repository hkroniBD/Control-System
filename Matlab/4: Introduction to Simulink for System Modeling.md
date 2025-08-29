# 🛠️ Lecture on Session 4: Introduction to Simulink for System Modeling

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Transition to block-based modeling for complex systems. This session introduces Simulink, a graphical programming environment within MATLAB, for modeling and simulating dynamic systems like electrical circuits, enabling students to build and analyze control systems visually. 🖥️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-3 (MATLAB basics, system modeling, time-domain analysis). Familiarity with transfer functions and state-space models. MATLAB with Simulink and Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/simulink](https://www.mathworks.com/help/simulink))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapter 2 for modeling, Simulink examples)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Simulink Modeling section

---

## 🗂️ Topics Covered

1. 🧩 **Simulink Basics**: Blocks (sources, sinks, math), scopes, simulations.
2. 🏗️ **Building Models**: Transfer function blocks, state-space blocks.
3. 🔗 **Integration with MATLAB**: Importing/exporting models via scripts.
4. 📈 **Simple Simulations**: Step response in Simulink vs. code.

---

## 📝 Detailed Lecture Content

### 🧩 1. Simulink Basics
Simulink is a block-diagram environment for modeling and simulating dynamic systems, widely used in electrical engineering for circuits, control systems, and signal processing. It allows intuitive, visual design compared to code-based modeling. 🌐

- **Accessing Simulink**: In MATLAB, type `simulink` or click the Simulink icon in the MATLAB toolbar.
- **Interface**: Library Browser (blocks), Canvas (model design), Configuration Parameters (solver settings).
- **Key Blocks**:
  - **Sources**: Step, Sine Wave, Constant (inputs like voltage sources).
  - **Sinks**: Scope, To Workspace (outputs for visualization/data).
  - **Math**: Gain, Sum, Integrator (operations like amplification or integration).
- **Simulations**: Run via the “Play” button or `sim` command; set simulation time in Configuration Parameters.

**💡 Example**: Create a new Simulink model (`File > New > Model`), drag a Step block, Gain block, and Scope block, connect them, and run to visualize output.

---

### 🏗️ 2. Building Models
Simulink models represent systems using interconnected blocks, such as transfer functions or state-space for control systems like amplifiers or motors. 🛠️

- **Transfer Function Block**: Found in Simulink > Continuous. Set numerator/denominator coefficients (e.g., [1] / [1 2 1] for 1/(s² + 2s + 1)).
- **State-Space Block**: Also in Continuous. Specify matrices A, B, C, D for state equations dx/dt = Ax + Bu, y = Cx + Du.
- **Connections**: Drag arrows to connect blocks (e.g., Step → Transfer Function → Scope).
- **Parameters**: Double-click blocks to set values; use variables defined in MATLAB workspace for flexibility.

**💡 Example**: Model a first-order system 1/(s + 1) with a step input and scope output.

---

### 🔗 3. Integration with MATLAB
Simulink models can be controlled and analyzed via MATLAB scripts, enabling automation and data exchange. 📜

- **Importing**: Define parameters in MATLAB (e.g., `num = [1]; den = [1 1];`) and use in Simulink blocks.
- **Exporting**: Save outputs to MATLAB workspace using To Workspace block or `sim` command.
- **Simulation via Script**: Use `sim('model_name')` to run a model and retrieve data.

**💻 Example Code**:
```matlab
% Define parameters
num = [1]; den = [1 1]; % TF: 1/(s+1)
set_param('myModel/Transfer Fcn', 'Numerator', 'num', 'Denominator', 'den');
out = sim('myModel'); % Run Simulink model
y = out.simout; t = out.tout; % Extract output/time
plot(t, y); title('Simulink Step Response'); grid on;
```

**💬 Explanation**:
- `set_param` sets block parameters dynamically.
- `sim('myModel')` runs the model, storing outputs in `out`.

---

### 📈 4. Simple Simulations
Compare Simulink and MATLAB code for a step response to understand equivalence and Simulink’s visual advantage. 📉

- **Simulink**: Use Step, Transfer Function, and Scope blocks for visual simulation.
- **MATLAB Code**: Use `step(sys)` for equivalent response.
- **Comparison**: Simulink is intuitive for complex systems; code is faster for simple analysis.

**💻 Example Code**:
```matlab
% MATLAB vs Simulink step response
sys = tf([1], [1 1]); % 1/(s+1)
[y, t] = step(sys); % MATLAB
figure; plot(t, y); title('MATLAB Step Response'); grid on;

% Simulink (assumes model 'firstOrder.slx' with TF block)
out = sim('firstOrder'); % Run Simulink model
figure; plot(out.tout, out.simout); title('Simulink Step Response'); grid on;
```

**💬 Explanation**:
- Both produce identical responses for linear systems.
- Simulink model must be pre-built with matching TF.

---

## 💻 Code Implementation (Combined Example)

Script to define parameters, run a Simulink model, and compare with MATLAB.

```matlab
% Session4_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Simulink for system modeling

% 🧹 Clear workspace
clear; clc;

% 📈 Define system: 1/(s^2 + 2s + 1)
num = [1]; den = [1 2 1];
sys = tf(num, den);

% MATLAB step response
[y, t] = step(sys);
figure; plot(t, y, 'b-', 'LineWidth', 2);
title('MATLAB Step Response'); xlabel('Time (s)'); ylabel('Output'); grid on;

% 🔗 Simulink: Set parameters and simulate (model: secondOrder.slx)
set_param('secondOrder/Transfer Fcn', 'Numerator', 'num', 'Denominator', 'den');
out = sim('secondOrder'); % Run model
figure; plot(out.tout, out.simout, 'r--', 'LineWidth', 2);
title('Simulink Step Response'); xlabel('Time (s)'); ylabel('Output'); grid on;

% 🧮 State-space equivalent
A = [0 1; -1 -2]; B = [0; 1]; C = [1 0]; D = 0;
sys_ss = ss(A, B, C, D);
set_param('secondOrder/State-Space', 'A', mat2str(A), 'B', mat2str(B), ...
    'C', mat2str(C), 'D', mat2str(D));
out_ss = sim('secondOrder'); % Run with SS block
figure; plot(out_ss.tout, out_ss.simout_ss, 'g:', 'LineWidth', 2);
title('Simulink State-Space Response'); grid on;
```

**💬 Explanation**:
- Models a second-order system in both TF and SS.
- MATLAB `step` and Simulink simulations (`secondOrder.slx` with TF and SS blocks) are compared.
- `set_param` links MATLAB variables to Simulink blocks.
- Assumes `secondOrder.slx` has Transfer Fcn and State-Space blocks, with Scope and To Workspace outputs.

---

## 🛠️ Exercise
**Task**: Build and simulate an op-amp integrator circuit in Simulink; compare with MATLAB code. Parameters: R=10 kΩ, C=1 µF, input=1 V step.

**💻 Solution Example**:
```matlab
% Exercise_Session4.m
% Op-Amp Integrator

% Parameters
R = 10e3; C = 1e-6; % R in ohms, C in farads
num = [-1/(R*C)]; den = [1 0]; % TF: -1/(RCs)
sys = tf(num, den);

% 📈 MATLAB: Step response
[y, t] = step(sys);
figure; plot(t, y, 'b-', 'LineWidth', 2);
title('MATLAB Integrator Response'); xlabel('Time (s)'); ylabel('Output (V)'); grid on;

% 🔗 Simulink: Set and simulate (model: opAmpIntegrator.slx)
set_param('opAmpIntegrator/Transfer Fcn', 'Numerator', 'num', 'Denominator', 'den');
out = sim('opAmpIntegrator');
figure; plot(out.tout, out.simout, 'r--', 'LineWidth', 2);
title('Simulink Integrator Response'); xlabel('Time (s)'); ylabel('Output (V)'); grid on;
```

**💬 Simulink Model (opAmpIntegrator.slx)**:
1. Create new model: `simulink > File > New > Model`.
2. Add blocks: Step (set to 1), Transfer Fcn (set Numerator=[-1/(R*C)], Denominator=[1 0]), Scope, To Workspace (variable: `simout`).
3. Connect: Step → Transfer Fcn → Scope/To Workspace.
4. Set simulation time to 0.01s (for RC=0.01).
5. Save as `opAmpIntegrator.slx`.

**💬 Explanation**:
- Integrator TF: V_out(s)/V_in(s) = -1/(RCs) (inverting, integrates input).
- MATLAB plots V_out = -t/RC for step input.
- Simulink model mimics this; compare plots for equivalence.

---

## 📌 Additional Notes
- **✅ Best Practices**: Name blocks clearly (e.g., “Integrator”). Use workspace variables for parameters.
- **🔍 Debugging Tips**: Check block connections (arrows). Verify simulation time matches dynamics (RC time constant).
- **🚀 Extensions**: Add a sinusoidal input to the integrator; observe output phase shift.
- **🖥️ Lab Setup**: Ensure Simulink license. Use `ver` to check.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 🧩 Understand Simulink’s block-based modeling.
2. 🏗️ Build TF and SS models in Simulink.
3. 🔗 Integrate MATLAB scripts with Simulink.
4. 📈 Simulate and compare responses.

**📅 Next Session Preview**: Stability analysis basics (Routh-Hurwitz, root locus). 🎢

**📝 Assignment**: Create a Simulink model for an RC circuit (R=1 kΩ, C=1 µF). Simulate step response, compare with MATLAB, and submit model/script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
