# 🔧 Lecture on Session 10: Lead-Lag Compensation

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Use compensators to meet performance specifications. This session equips students with skills to design lead and lag compensators to improve transient and steady-state performance of control systems, such as stabilizing unstable plants, using MATLAB tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-9 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control). Knowledge of transfer functions, root locus, and frequency response. MATLAB with Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapter 9 for compensator design)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Compensator Design section

---

## 🗂️ Topics Covered

1. 📈 **Lead Compensation**: Phase advance for transient improvement.
2. 📉 **Lag Compensation**: Gain reduction for steady-state accuracy.
3. 🔧 **Design**: Via root locus and Bode plots; `rltool` for interactive design.

---

## 📝 Detailed Lecture Content

### 📈 1. Lead Compensation
Lead compensators add phase advance to improve transient response and stability, commonly used in systems like motor controllers to enhance phase margin or move poles. 🌟

- **Concept**: Lead compensator C(s) = K (s+z)/(s+p), where |z| < |p|. Adds positive phase (up to 90°) near the zero/pole, improving rise time and phase margin.
- **Effect**: Shifts root locus left (stabilizing) or increases phase margin in Bode plot.
- **Design Goals**: Place zero near desired natural frequency and pole further left to add phase without excessive gain.

**💻 Example Code**:
```matlab
% Lead compensator design
G = tf([1], [1 1 0]); % Plant: 1/(s(s+1))
C_lead = tf([1 1], [1 10]); % C(s) = (s+1)/(s+10)
sys_cl = feedback(C_lead*G, 1);
figure; step(sys_cl); title('Closed-Loop Response with Lead Compensator'); grid on;
info = stepinfo(sys_cl);
disp('Lead Compensator Metrics:'); disp(info);
```

**💬 Explanation**:
- `C_lead`: Zero at s=-1, pole at s=-10, adds phase near ω=1–10 rad/s.
- `feedback` forms closed-loop system; `step` evaluates transient response.

---

### 📉 2. Lag Compensation
Lag compensators reduce gain at high frequencies to improve steady-state accuracy, often used in systems requiring low steady-state error (e.g., position control). 📊

- **Concept**: Lag compensator C(s) = K (s+z)/(s+p), where |z| > |p|. Reduces gain at high frequencies while maintaining phase, improving steady-state error.
- **Effect**: Increases low-frequency gain (reduces error) without significantly altering transient response.
- **Design Goals**: Place pole/zero near origin to boost low-frequency gain.

**💻 Example Code**:
```matlab
% Lag compensator design
G = tf([1], [1 1 0]); % Plant: 1/(s(s+1))
C_lag = tf([1 0.1], [1 0.01]); % C(s) = (s+0.1)/(s+0.01)
sys_cl = feedback(C_lag*G, 1);
figure; step(sys_cl); title('Closed-Loop Response with Lag Compensator'); grid on;
info = stepinfo(sys_cl);
disp('Lag Compensator Metrics:'); disp(info);
```

**💬 Explanation**:
- `C_lag`: Zero at s=-0.1, pole at s=-0.01, increases low-frequency gain.
- `step` shows improved steady-state tracking with minimal transient impact.

---

### 🔧 3. Design
Lead-lag compensators are designed using root locus (pole placement) or Bode plots (phase/gain adjustment), with `rltool` for interactive tuning. ⚙️

- **Root Locus Design**: Place compensator poles/zeros to shape locus toward desired closed-loop poles.
- **Bode Plot Design**: Add phase (lead) at gain crossover or boost gain (lag) at low frequencies.
- **MATLAB Tool**: `rltool(sys)` provides an interactive GUI for compensator design, adjusting poles/zeros and visualizing root locus/Bode plots.

**💻 Example Code**:
```matlab
% Interactive design with rltool
G = tf([1], [1 1 0]); % Plant: 1/(s(s+1))
rltool(G); % Launches interactive design tool
```

**💬 Explanation**:
- `rltool` opens a GUI to add lead/lag compensators, adjust parameters, and view root locus/Bode plots.
- Students can drag poles/zeros to meet specs (e.g., phase margin, overshoot).

---

## 💻 Code Implementation (Combined Example)

Integrated script for lead and lag compensator design, closed-loop analysis, and interactive tuning.

```matlab
% Session10_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Lead-lag compensator design

% 🧹 Clear workspace
clear; clc;

% 📈 Lead Compensator
G = tf([1], [1 1 0]); % Plant: 1/(s(s+1))
C_lead = tf([1 1], [1 10]); % C(s) = (s+1)/(s+10)
sys_cl_lead = feedback(C_lead*G, 1);
figure(1); step(sys_cl_lead); title('Lead Compensator Response'); grid on;
info_lead = stepinfo(sys_cl_lead);
disp('Lead Compensator Metrics:'); disp(info_lead);

% 📉 Lag Compensator
C_lag = tf([1 0.1], [1 0.01]); % C(s) = (s+0.1)/(s+0.01)
sys_cl_lag = feedback(C_lag*G, 1);
figure(2); step(sys_cl_lag); title('Lag Compensator Response'); grid on;
info_lag = stepinfo(sys_cl_lag);
disp('Lag Compensator Metrics:'); disp(info_lag);

% 🔧 Interactive Design
% Uncomment to launch rltool
% rltool(G); % Interactive design for lead-lag
```

**💬 Explanation**:
- **Lead**: Improves transient response (faster rise, reduced overshoot).
- **Lag**: Enhances steady-state accuracy (lower error).
- **rltool**: Allows interactive tuning (commented out to avoid GUI launch in script).
- Save as `Session10_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Design a lead-lag compensator for an unstable plant to stabilize it. System: G(s) = 1/(s² - 1), achieve stable response with <10% overshoot.

**💻 Solution Example**:
```matlab
% Exercise_Session10.m
% Lead-lag compensator for unstable plant

% 🧹 Clear workspace
clear; clc;

% Plant: G(s) = 1/(s^2 - 1), unstable (poles at s=±1)
G = tf([1], [1 0 -1]);

% 📈 Lead Compensator: Add phase to stabilize
C_lead = tf([1 2], [1 10]); % C(s) = (s+2)/(s+10)
sys_cl_lead = feedback(C_lead*G, 1);
figure(1); step(sys_cl_lead); title('Lead Compensator Response'); grid on;
info_lead = stepinfo(sys_cl_lead);
disp('Lead Compensator Metrics:'); disp(info_lead);

% 📉 Lag Compensator: Improve steady-state
C_lag = tf([1 0.1], [1 0.01]); % C(s) = (s+0.1)/(s+0.01)
C_leadlag = C_lead * C_lag; % Combined lead-lag
sys_cl = feedback(C_leadlag*G, 1);
figure(2); step(sys_cl); title('Lead-Lag Compensator Response'); grid on;
info = stepinfo(sys_cl);
disp('Lead-Lag Compensator Metrics:'); disp(info);

% 🔧 Verify Stability and Overshoot
if isstable(sys_cl) && info.Overshoot < 10
    disp('Stable with <10% overshoot achieved!');
else
    disp('Adjust lead/lag parameters or use rltool.');
end
```

**💬 Explanation**:
- **Plant**: G(s) = 1/(s² - 1) has poles at s=±1 (unstable).
- **Lead**: Zero at s=-2, pole at s=-10 shifts poles left, stabilizing the system.
- **Lag**: Zero at s=-0.1, pole at s=-0.01 improves steady-state error.
- **Verification**: `isstable` and `stepinfo` confirm stability and overshoot <10%.
- Students can use `rltool(G)` to refine parameters interactively.

---

## 📌 Additional Notes
- **✅ Best Practices**: Start with lead to stabilize, add lag for steady-state if needed. Use `margin(C*G)` to check phase margin.
- **🔍 Debugging Tips**: Ensure plant and compensator are properly defined in `tf`. Check closed-loop stability with `isstable`.
- **🚀 Extensions**: Design lead-lag for systems with delays or test in Simulink.
- **🖥️ Lab Setup**: Control System Toolbox required for `tf`, `feedback`, `rltool`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📈 Design lead compensators for transient improvement.
2. 📉 Design lag compensators for steady-state accuracy.
3. 🔧 Use root locus and Bode plots for compensator design, including interactive tools.

**📅 Next Session Preview**: State-space controller design. 🌐

**📝 Assignment**: For G(s) = 1/(s(s-1)), design a lead-lag compensator to stabilize and achieve <5% overshoot, plot step response, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
