# 🖥️ Lecture on Session 12: Digital Control Systems Basics

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Handle discrete-time systems for digital control applications. This session equips students with skills to discretize continuous-time systems, analyze discrete-time systems, and simulate digital controllers, such as for motor control, using MATLAB tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-11 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation, state-space control). Knowledge of transfer functions, z-transforms, and sampling theory. MATLAB with Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Digital Control Systems” by Benjamin C. Kuo or “Control Systems Engineering” by Norman S. Nise (Chapter 13 for digital control)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Digital Control section

---

## 🗂️ Topics Covered

1. 📐 **Sampling/Z-Transform**: Discretization using `c2d` (continuous to discrete).
2. 📈 **Discrete Analysis**: Discrete step response (`dstep`) and root locus (`drlocus`).
3. 💻 **Code Implementation**: `sysd = c2d(sys,Ts)`.

---

## 📝 Detailed Lecture Content

### 📐 1. Sampling/Z-Transform
Digital control systems operate in discrete time, requiring conversion of continuous-time systems to discrete-time using sampling and the z-transform. 🕒

- **Sampling**: Converts continuous signals to discrete sequences at sampling period Ts, affecting system dynamics (e.g., aliasing if Ts is too large).
- **Z-Transform**: Maps s-plane to z-plane (z = e^(sTs)); used for discrete transfer functions.
- **MATLAB Tool**: `c2d(sys,Ts,method)` discretizes a continuous system, with methods like 'zoh' (zero-order hold) or 'tustin' (bilinear transformation).

**💻 Example Code**:
```matlab
% Discretize a continuous system
Ts = 0.1; % Sampling time (s)
sys = tf([1], [1 2 1]); % Continuous: 1/(s^2 + 2s + 1)
sysd = c2d(sys, Ts, 'zoh'); % Discrete using zero-order hold
disp('Discrete System:'); disp(sysd);
```

**💬 Explanation**:
- `c2d` converts continuous system to discrete with Ts=0.1s using zero-order hold.
- `disp(sysd)` shows the z-domain transfer function.

---

### 📈 2. Discrete Analysis
Discrete-time systems are analyzed using tools analogous to continuous systems, such as step response and root locus, adapted for the z-plane. 📊

- **Discrete Step Response**: `dstep(sysd)` plots the response to a unit step in discrete time.
- **Discrete Root Locus**: `drlocus(sysd)` shows how closed-loop poles move in the z-plane with varying gain, with stability inside the unit circle (|z| < 1).
- **Key Difference**: In z-plane, stable poles lie inside the unit circle, unlike s-plane’s left-half plane.

**💻 Example Code**:
```matlab
% Discrete analysis
Ts = 0.1;
sys = tf([1], [1 2 1]); % Continuous plant
sysd = c2d(sys, Ts, 'zoh');
figure(1); dstep(sysd); title('Discrete Step Response'); grid on;
figure(2); drlocus(sysd); title('Discrete Root Locus'); grid on;
```

**💬 Explanation**:
- `dstep` shows discrete-time step response, approximating continuous response for small Ts.
- `drlocus` plots z-plane root locus; poles inside unit circle indicate stability.

---

### 💻 3. Code Implementation
MATLAB’s `c2d` enables discretization, followed by analysis with `dstep` and `drlocus` for digital control design. 🔧

- **Workflow**: Discretize plant, design discrete controller, analyze closed-loop response.
- **MATLAB Tools**: `c2d` for discretization, `feedback` for closed-loop, `dstep`/`drlocus` for analysis.

**💻 Example Code**:
```matlab
% Discrete closed-loop system
Ts = 0.1;
G = tf([1], [1 2 0]); % Continuous plant: 1/(s(s+2))
Gd = c2d(G, Ts, 'zoh'); % Discrete plant
C = pid(1, 0.1, 0.05); % Continuous PID
Cd = c2d(C, Ts, 'tustin'); % Discrete PID
sys_cl = feedback(Cd*Gd, 1);
figure; dstep(sys_cl); title('Discrete Closed-Loop Response'); grid on;
```

**💬 Explanation**:
- `c2d` discretizes both plant and PID controller.
- `feedback` forms closed-loop discrete system.
- `dstep` evaluates performance in discrete time.

---

## 💻 Code Implementation (Combined Example)

Integrated script for discretization, discrete analysis, and closed-loop simulation.

```matlab
% Session12_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Digital control systems

% 🧹 Clear workspace
clear; clc;

% 📐 Discretization
Ts = 0.1; % Sampling time
G = tf([1], [1 2 1]); % Continuous: 1/(s^2 + 2s + 1)
Gd = c2d(G, Ts, 'zoh'); % Discrete plant
disp('Discrete Plant:'); disp(Gd);

% 📈 Discrete Analysis
figure(1); dstep(Gd); title('Discrete Step Response'); grid on;
figure(2); drlocus(Gd); title('Discrete Root Locus'); grid on;

% 💻 Closed-Loop with Discrete PID
C = pid(2, 1, 0.5); % Continuous PID
Cd = c2d(C, Ts, 'tustin'); % Discrete PID
sys_cl = feedback(Cd*Gd, 1);
figure(3); dstep(sys_cl); title('Discrete Closed-Loop Response'); grid on;
info = stepinfo(sys_cl);
disp('Closed-Loop Metrics:'); disp(info);
```

**💬 Explanation**:
- **Discretization**: Converts plant to discrete using zero-order hold.
- **Analysis**: `dstep` and `drlocus` evaluate open-loop discrete dynamics.
- **Closed-Loop**: Discretizes PID controller and forms closed-loop system.
- Save as `Session12_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Discretize an analog PID controller and simulate sampled DC motor speed control. System: G(s) = 0.01/(0.01s² + 0.1s + 0.0001), Ts=0.01s, target <5% overshoot.

**💻 Solution Example**:
```matlab
% Exercise_Session12.m
% Discrete PID for DC motor

% 🧹 Clear workspace
clear; clc;

% System: DC motor
J = 0.01; b = 0.1; K = 0.01; % J: inertia, b: damping, K: torque constant
G = tf([K], [J b K^2]); % Continuous: K/(Js^2 + bs + K^2)
Ts = 0.01; % Sampling time
Gd = c2d(G, Ts, 'zoh'); % Discrete plant
disp('Discrete Plant:'); disp(Gd);

% 📐 Discrete PID
C = pidtune(G, 'PID', 10, pidtuneOptions('PhaseMargin', 60)); % Continuous PID
Cd = c2d(C, Ts, 'tustin'); % Discrete PID
disp('Discrete PID:'); disp(Cd);

% 💻 Closed-Loop
sys_cl = feedback(Cd*Gd, 1);
figure; dstep(sys_cl); title('Discrete DC Motor Response'); grid on;
info = stepinfo(sys_cl);
disp('Closed-Loop Metrics:'); disp(info);

% Verify Overshoot
if info.Overshoot < 5
    disp('Overshoot < 5% achieved!');
else
    disp('Adjust PID parameters or sampling time.');
end
```

**💬 Explanation**:
- **System**: DC motor model with given parameters, discretized with Ts=0.01s.
- **PID**: `pidtune` designs continuous PID, then `c2d` converts to discrete using Tustin method.
- **Closed-Loop**: `dstep` verifies response; `stepinfo` checks overshoot <5%.
- Students can adjust Ts or PID parameters if needed.

---

## 📌 Additional Notes
- **✅ Best Practices**: Choose Ts small enough to avoid aliasing (Nyquist: Ts < π/ω_max). Use 'zoh' for plant, 'tustin' for controllers.
- **🔍 Debugging Tips**: Verify discrete system stability with `isstable(sys_cl)`. Check z-plane poles inside unit circle.
- **🚀 Extensions**: Test with varying Ts or implement in Simulink for digital control.
- **🖥️ Lab Setup**: Control System Toolbox required for `c2d`, `dstep`, `drlocus`, `pidtune`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📐 Discretize continuous systems using z-transform.
2. 📈 Analyze discrete systems with step response and root locus.
3. 💻 Implement digital controllers for closed-loop systems.

**📅 Next Session Preview**: Digital controller tuning and implementation. 🎛️

**📝 Assignment**: For G(s) = 1/(s(s+1)), discretize with Ts=0.05s, design discrete PID for <10% overshoot, plot dstep response, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
