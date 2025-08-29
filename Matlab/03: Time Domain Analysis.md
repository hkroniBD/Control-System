# ⏱️ Lecture on Session 3: Time Domain Analysis

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Simulate and evaluate system responses to inputs. This session emphasizes time-domain analysis techniques for control systems, enabling students to assess performance in electrical applications like circuits and motors using MATLAB simulations. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1 and 2 (MATLAB basics and system modeling). Familiarity with Laplace transforms and basic control theory. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapter 4 for time-domain analysis)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Focus on Time Response section

---

## 🗂️ Topics Covered

1. 📈 **Standard Inputs**: Step (`step`), impulse (`impulse`), ramp (custom with `lsim`).
2. 📊 **Response Metrics**: Transient: Rise time, peak time, overshoot, settling time (`stepinfo`). Steady-State: Error calculation for type 0/1/2 systems.
3. 🔄 **Custom Simulations**: Arbitrary inputs using `lsim` (linear simulation).
4. 📍 **Pole-Zero Map**: `pzmap` to visualize stability influences.

---

## 📝 Detailed Lecture Content

### 📈 1. Standard Inputs
Time-domain analysis involves exciting systems with standard test signals to observe behavior. These inputs mimic real-world disturbances in electrical systems, like voltage steps in circuits. 🔋

- **Step Input**: Unit step (Heaviside function), simulating sudden changes (e.g., switch on). Use `step(sys)` for response.
- **Impulse Input**: Dirac delta, for instantaneous shocks (e.g., current surge). Use `impulse(sys)`.
- **Ramp Input**: Linear increase (e.g., gradual voltage rise). No built-in; simulate with `lsim` and input `u = t`.

**💻 Example Code**:
```matlab
% Standard inputs for a system
sys = tf([1], [1 0.5 1]); % Second-order: 1/(s^2 + 0.5s + 1)

% Step response
figure; step(sys); title('Step Response'); grid on;

% Impulse response
figure; impulse(sys); title('Impulse Response'); grid on;

% Ramp response (custom)
t = 0:0.01:10; u = t; % Ramp input
[y, t] = lsim(sys, u, t); % Simulate
figure; plot(t, y); xlabel('Time (s)'); ylabel('Output');
title('Ramp Response'); grid on;
```

**💬 Explanation**:
- `step(sys)` and `impulse(sys)` automatically plot responses for the system object `sys`.
- For ramp, define time `t` and input `u = t`, then `lsim` computes the output `y` via convolution.

---

### 📊 2. Response Metrics
Quantify system performance: transient (how fast/oscillatory) and steady-state (accuracy). Essential for tuning electrical systems like amplifiers. 📉

- **Transient Metrics** (for step response):
  - Rise Time: Time from 10% to 90% of final value.
  - Peak Time: Time to first peak.
  - Overshoot: Percentage exceedance of final value.
  - Settling Time: Time to stay within ±2% (or 5%) of final value.
  - Use `stepinfo(sys)` to extract.
- **Steady-State Error**: For unity feedback, e_ss = 1 / (1 + lim G(s) as s->0). Depends on system type:
  - Type 0: Constant error for step.
  - Type 1: Zero for step, constant for ramp.
  - Type 2: Zero for step/ramp, constant for parabola.

**💻 Example Code**:
```matlab
% Metrics for second-order system
sys = tf([1], [1 0.5 1]);
info = stepinfo(sys);
disp('Transient Metrics:'); disp(info); % Shows RiseTime, SettlingTime, etc.

% Steady-state error (manual for type 0 example)
G = tf([1], [1 1]); % Type 0: 1/(s+1)
Kp = dcgain(G); % Position constant
ess_step = 1 / (1 + Kp); % For unit step
disp(['Steady-State Error (Step): ' num2str(ess_step)]);
```

**💬 Explanation**:
- `stepinfo` returns a struct with metrics; access like `info.RiseTime`.
- For error, `dcgain` computes lim G(s) as s->0; classify type by poles at origin.

---

### 🔄 3. Custom Simulations
For non-standard inputs (e.g., sinusoidal disturbances in power systems), use `lsim` to simulate linear systems. 🌊

- **lsim**: Simulates y(t) = integral G(tau) u(t-tau) dtau + initial conditions.
- Syntax: `[y, t, x] = lsim(sys, u, t, x0);` (u: input vector, t: time, x0: initial states).

**💻 Example Code**:
```matlab
% Custom input: Sinusoidal disturbance
sys = tf([1], [1 0.5 1]);
t = 0:0.01:20; u = sin(2*pi*0.1*t); % 0.1 Hz sine
[y, t, x] = lsim(sys, u, t); % x: state trajectory
figure; plot(t, y); hold on; plot(t, u, '--');
xlabel('Time (s)'); ylabel('Output/Input');
title('Response to Sinusoidal Input'); legend('Output', 'Input'); grid on;
```

**💬 Explanation**:
- Define `u` matching length of `t`.
- `lsim` returns output `y` and states `x`; useful for state analysis.

---

### 📍 4. Pole-Zero Map
Visualize poles/zeros in s-plane to infer time response (e.g., real poles: exponential decay; complex: oscillations). 📌

- **pzmap**: Plots zeros (o) and poles (x); left-half plane for stability.

**💻 Example Code**:
```matlab
% Pole-zero map
sys = tf([1 2], [1 3 2]); % (s+2)/(s^2 + 3s + 2)
figure; pzmap(sys); title('Pole-Zero Map'); grid on;
```

**💬 Explanation**:
- Poles at roots of denominator; zeros at numerator.
- Influences: Dominant poles near imaginary axis slow response; zeros near poles cancel effects.

---

## 💻 Code Implementation (Combined Example)

Integrated script: Analyze a system with standard inputs, metrics, custom sim, and pzmap.

```matlab
% Session3_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Time-domain analysis

% 🧹 Clear workspace
clear; clc;

% System: Underdamped second-order
sys = tf([1], [1 0.5 1]); % 1/(s^2 + 0.5s + 1)

% 📈 Standard Inputs
figure; step(sys); title('Step Response'); grid on;
figure; impulse(sys); title('Impulse Response'); grid on;
t = 0:0.01:10; u = t;
[y_ramp, ~] = lsim(sys, u, t);
figure; plot(t, y_ramp); title('Ramp Response'); grid on;

% 📊 Metrics
info = stepinfo(sys);
disp('Step Info:'); disp(info);
ess_ramp = 1 / dcgain(sys * tf([1],[1 0])); % For ramp (type 1)
disp(['Steady-State Error (Ramp): ' num2str(ess_ramp)]);

% 🔄 Custom: Square wave
u = square(2*pi*0.1*t); % 0.1 Hz square
[y_sq, t_sq] = lsim(sys, u, t);
figure; plot(t_sq, y_sq); title('Square Wave Response'); grid on;

% 📍 PZ Map
figure; pzmap(sys); title('Pole-Zero Map'); grid on;
```

**💬 Explanation**:
- Covers all topics for one system.
- `square` generates a square wave for custom input.

---

## 🛠️ Exercise
**Task**: Analyze a second-order RLC circuit's step response; compute metrics and vary damping ratio. Parameters: R=1 Ω, L=1 H, C=1 F (vary R for damping).

**💻 Solution Example**:
```matlab
% Exercise_Session3.m
% RLC Circuit Analysis

% Parameters (underdamped: zeta = R/2 * sqrt(C/L) <1)
R=1; L=1; C=1; % zeta = 0.5
sys = tf([1/(L*C)], [1, R/L, 1/(L*C)]); % 1/(s^2 + (R/L)s + 1/(L*C))

% Step response and metrics
[y, t] = step(sys);
info = stepinfo(sys);
disp('Metrics for zeta=0.5:'); disp(info);
figure; plot(t, y); title('Step Response (zeta=0.5)'); grid on;

% Vary damping (overdamped: R=3, zeta>1)
R=3; sys_over = tf([1/(L*C)], [1, R/L, 1/(L*C)]);
info_over = stepinfo(sys_over);
disp('Metrics for zeta>1:'); disp(info_over);

% PZ Map
figure; pzmap(sys); title('PZ Map (underdamped)'); grid on;
```

**💬 Explanation**:
- TF for series RLC: 1/(L C s^2 + R C s + 1) (voltage across C).
- Vary R to change zeta = R/(2 sqrt(L/C)); compute metrics.
- Compare responses for different damping.

---

## 📌 Additional Notes
- **✅ Best Practices**: Use `initial(sys, x0)` for nonzero initial conditions. Check stability with poles.
- **🔍 Debugging Tips**: Ensure `t` and `u` match in length for `lsim`. Use `isstable(sys)` for quick checks.
- **🚀 Extensions**: Simulate with noise using `u = t + randn(size(t));`.
- **🖥️ Lab Setup**: Control System Toolbox required for `step`, `lsim`, etc.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📈 Simulate responses to standard inputs.
2. 📊 Calculate transient and steady-state metrics.
3. 🔄 Perform custom simulations for arbitrary inputs.
4. 📍 Visualize pole-zero effects on dynamics.

**📅 Next Session Preview**: Stability analysis basics (Routh-Hurwitz, root locus). 🎢

**📝 Assignment**: For an RL circuit (R=1, L=1), compute step/ramp responses, metrics, and error. Vary R; submit script and report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
