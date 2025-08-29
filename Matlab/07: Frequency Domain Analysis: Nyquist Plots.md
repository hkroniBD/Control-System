# 🌀 Lecture on Session 7: Frequency Domain Analysis: Nyquist Plots

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Use contour mapping for stability assessment. This session introduces Nyquist plots to analyze the stability of control systems, such as feedback amplifiers or control loops, by mapping the frequency response in the complex plane using MATLAB tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-6 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots). Knowledge of complex analysis and frequency response. MATLAB with Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapter 8 for frequency response and Nyquist criterion)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Nyquist Plot section

---

## 🗂️ Topics Covered

1. 📐 **Nyquist Criterion**: Encirclements of -1+j0 point; N = P - Z.
2. 📉 **Plot Construction**: Real vs. imaginary parts.
3. 🔍 **Analysis**: Conditional stability, non-minimum phase systems.

---

## 📝 Detailed Lecture Content

### 📐 1. Nyquist Criterion
The Nyquist criterion assesses closed-loop stability by analyzing the open-loop frequency response contour around the critical point (-1+j0) in the complex plane. 🌀

- **Concept**: For an open-loop transfer function G(s)H(s), plot G(jω)H(jω) for ω from -∞ to +∞. Stability is determined by the number of encirclements (N) of -1+j0, where N = P (number of open-loop right-half plane (RHP) poles) - Z (number of closed-loop RHP poles).
- **Stability**: A system is stable if Z = 0 (no closed-loop RHP poles), which requires N = P.
- **MATLAB Tool**: `nyquist(sys)` generates the Nyquist plot, showing the complex plane trajectory.

**💻 Example Code**:
```matlab
% Nyquist plot for a system
sys = tf([1], [1 2 0]); % Open-loop TF: 1/(s(s+2))
figure; nyquist(sys); title('Nyquist Plot of 1/(s(s+2))'); grid on;
```

**💬 Explanation**:
- `nyquist(sys)` plots the real vs. imaginary parts of G(jω) for ω from -∞ to +∞.
- Check encirclements of -1+j0 to determine stability (P = 2 due to poles at s=0, s=-2; count N to find Z).

---

### 📉 2. Plot Construction
Nyquist plots map the frequency response G(jω) in the complex plane, providing a comprehensive view of system dynamics. 📊

- **Axes**: Real part (x-axis) vs. imaginary part (y-axis) of G(jω).
- **Frequency Range**: MATLAB plots ω from 0 to +∞ and mirrors for -∞ to 0 (for real systems, G(-jω) = G(jω)*).
- **Key Features**: Intersections with the real axis (gain crossover), proximity to -1+j0, and contour shape.
- **MATLAB Tool**: `nyquist(sys)` for automatic plotting; `[re, im, w] = nyquist(sys)` for manual plotting.

**💻 Example Code**:
```matlab
% Detailed Nyquist plot
sys = tf([1], [1 1 0]); % Open-loop TF: 1/(s(s+1))
[re, im, w] = nyquist(sys);
figure; plot(squeeze(re), squeeze(im), 'b-', 'LineWidth', 2);
title('Nyquist Plot of 1/(s(s+1))'); xlabel('Real'); ylabel('Imaginary'); grid on; axis equal;
```

**💬 Explanation**:
- `nyquist(sys)` returns real (`re`) and imaginary (`im`) parts of G(jω); `squeeze` removes singleton dimensions.
- `axis equal` ensures accurate contour proportions for encirclement analysis.

---

### 🔍 3. Analysis
Nyquist plots reveal stability and system properties, such as conditional stability and non-minimum phase behavior, critical for feedback systems like amplifiers. 🔎

- **Conditional Stability**: System stable for a specific gain range but unstable for higher/lower gains (e.g., multiple crossings of -1+j0).
- **Non-Minimum Phase Systems**: Systems with RHP zeros, causing phase anomalies (e.g., initial phase increase).
- **Encirclements**: Count clockwise encirclements of -1+j0; use zoom in `nyquist` plot for precision.

**💻 Example Code**:
```matlab
% Nyquist for non-minimum phase system
sys = tf([1 -1], [1 2 0]); % Open-loop TF: (s-1)/(s(s+2))
figure; nyquist(sys); title('Nyquist Plot of (s-1)/(s(s+2))'); grid on;
```

**💬 Explanation**:
- RHP zero at s=1 introduces phase anomalies (phase starts positive).
- Analyze encirclements to determine closed-loop stability (P = 2; count N to find Z).

---

## 💻 Code Implementation (Combined Example)

Integrated script for Nyquist criterion, plot construction, and analysis of a standard and non-minimum phase system.

```matlab
% Session7_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Nyquist plot analysis

% 🧹 Clear workspace
clear; clc;

% 📐 Nyquist Criterion
sys = tf([1], [1 2 0]); % Open-loop TF: 1/(s(s+2))
figure; nyquist(sys); title('Nyquist Plot of 1/(s(s+2))'); grid on;

% 📉 Detailed Plot
[re, im, w] = nyquist(sys);
figure; plot(squeeze(re), squeeze(im), 'b-', 'LineWidth', 2);
title('Nyquist Plot of 1/(s(s+2))'); xlabel('Real'); ylabel('Imaginary'); grid on; axis equal;

% 🔍 Non-Minimum Phase System
sys_nmp = tf([1 -1], [1 2 0]); % Open-loop TF: (s-1)/(s(s+2))
figure; nyquist(sys_nmp); title('Nyquist Plot of (s-1)/(s(s+2))'); grid on;
```

**💬 Explanation**:
- **Nyquist Criterion**: Plots for 1/(s(s+2)) show encirclements; P=2 (poles at s=0, s=-2).
- **Detailed Plot**: Manual plotting with `re`, `im` for clarity.
- **Non-Minimum Phase**: RHP zero affects contour shape, requiring careful stability analysis.
- Save as `Session7_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Plot Nyquist for a delay system; determine stability for varying gains. System: G(s) = K e^(-0.1s)/(s+1), approximate delay as (1-0.05s)/(1+0.05s) using Pade approximation.

**💻 Solution Example**:
```matlab
% Exercise_Session7.m
% Nyquist analysis for delay system

% 🧹 Clear workspace
clear; clc;

% System: G(s) = K e^(-0.1s)/(s+1), Pade approximation
K = 1;
sys_delay = tf([K -K*0.05], [1 1 0.05]); % K (1-0.05s)/(s+1)(1+0.05s)
figure; nyquist(sys_delay); title('Nyquist Plot of Delay System (K=1)'); grid on;

% Vary gain
K = 5;
sys_delay = tf([K -K*0.05], [1 1 0.05]);
figure; nyquist(sys_delay); title('Nyquist Plot of Delay System (K=5)'); grid on;
```

**💬 Explanation**:
- **Pade Approximation**: e^(-0.1s) ≈ (1-0.05s)/(1+0.05s) for small delay.
- **Nyquist Plots**: Analyze encirclements of -1+j0 for K=1 and K=5; higher K may cause instability.
- **Stability**: P=1 (pole at s=-1); count N to find Z (closed-loop RHP poles).

---

## 📌 Additional Notes
- **✅ Best Practices**: Use `axis equal` for accurate Nyquist contours. Verify open-loop poles (P) with `pole(sys)`.
- **🔍 Debugging Tips**: Zoom in on -1+j0 in `nyquist` plot to count encirclements accurately.
- **🚀 Extensions**: Analyze systems with multiple poles/zeros or higher-order delays.
- **🖥️ Lab Setup**: Control System Toolbox required for `nyquist`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📐 Apply the Nyquist criterion to assess stability.
2. 📉 Construct and interpret Nyquist plots.
3. 🔍 Analyze conditional stability and non-minimum phase behavior.

**📅 Next Session Preview**: Advanced stability tools and sensitivity analysis. 🔍

**📝 Assignment**: For G(s) = K/(s(s+2)), plot Nyquist, determine stability for K=1,10, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
