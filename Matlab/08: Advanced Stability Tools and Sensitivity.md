# 🔍 Lecture on Session 8: Advanced Stability Tools and Sensitivity

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Explore robustness and multi-tool integration. This session equips students with advanced tools to assess the robustness of control systems, such as power converters, to parameter variations and integrates multiple stability analysis methods using MATLAB. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-7 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots). Knowledge of control theory, frequency response, and complex analysis. MATLAB with Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapters 6 and 8 for stability and frequency response)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Robustness and Sensitivity sections

---

## 🗂️ Topics Covered

1. 📉 **Sensitivity Analysis**: To parameter variations.
2. 🔄 **Combining Tools**: Root locus with Bode/Nyquist.
3. ✅ **Disk Margins**: `diskmargin` for robust stability.

---

## 📝 Detailed Lecture Content

### 📉 1. Sensitivity Analysis
Sensitivity analysis evaluates how system performance (e.g., stability, response) changes with parameter variations, crucial for robust design in systems like power converters where component values may vary. 📊

- **Concept**: Sensitivity measures the impact of parameter changes (e.g., resistor or capacitor values) on system metrics (e.g., poles, gain/phase margins).
- **Approach**: Vary parameters in transfer functions or state-space models, analyze changes in poles, step response, or margins.
- **MATLAB Tools**: Use `tf`, `pole`, `step`, or `margin` to compare responses across parameter variations.

**💻 Example Code**:
```matlab
% Sensitivity of a second-order system to damping
sys_nom = tf([1], [1 2 1]); % Nominal: 1/(s^2 + 2s + 1), zeta=1
zeta_vals = [0.5, 1, 2]; % Vary damping ratio
figure; hold on;
for zeta = zeta_vals
    den = [1 2*zeta 1]; % s^2 + 2*zeta*wn*s + wn^2, wn=1
    sys = tf([1], den);
    [y, t] = step(sys);
    plot(t, y, 'DisplayName', ['zeta = ' num2str(zeta)]);
end
title('Step Response Sensitivity to Damping'); xlabel('Time (s)'); ylabel('Output'); grid on; legend;
```

**💬 Explanation**:
- Vary damping ratio (ζ) in a second-order system.
- `step` shows how underdamped (ζ=0.5), critically damped (ζ=1), and overdamped (ζ=2) responses differ.
- Sensitivity is evident in overshoot and settling time changes.

---

### 🔄 2. Combining Tools
Integrating root locus, Bode, and Nyquist plots provides a comprehensive stability analysis, leveraging strengths of each method for systems like feedback controllers. 🔗

- **Root Locus**: Tracks pole movement with gain, showing stability boundaries.
- **Bode Plot**: Reveals gain/phase margins and frequency response.
- **Nyquist Plot**: Shows encirclements for absolute stability.
- **Integration**: Use all three to cross-validate stability and performance.

**💻 Example Code**:
```matlab
% Combine root locus, Bode, and Nyquist
sys_ol = tf([1], [1 2 0]); % Open-loop: 1/(s(s+2))
figure(1); rlocus(sys_ol); title('Root Locus'); grid on;
figure(2); bode(sys_ol); title('Bode Plot'); grid on;
figure(3); nyquist(sys_ol); title('Nyquist Plot'); grid on;
[Gm, Pm] = margin(sys_ol);
disp(['Gain Margin: ' num2str(Gm) ' dB, Phase Margin: ' num2str(Pm) ' deg']);
```

**💬 Explanation**:
- `rlocus` shows pole paths with gain K.
- `bode` provides gain/phase margins.
- `nyquist` checks encirclements of -1+j0.
- Combined analysis confirms stability and robustness.

---

### ✅ 3. Disk Margins
Disk margins assess robust stability under gain and phase variations, more conservative than classical margins, ideal for uncertain systems like power electronics. 📏

- **Concept**: Measures the smallest disk-shaped uncertainty in gain/phase that destabilizes the system.
- **MATLAB Tool**: `diskmargin(sys)` computes disk-based gain and phase margins, accounting for simultaneous variations.
- **Interpretation**: Larger margins indicate greater robustness to perturbations.

**💻 Example Code**:
```matlab
% Disk margins for a system
sys_ol = tf([1], [1 2 0]); % 1/(s(s+2))
DM = diskmargin(sys_ol);
disp('Disk Margins:'); disp(DM);
figure; diskmarginplot(DM); title('Disk Margin Plot');
```

**💬 Explanation**:
- `diskmargin` returns a structure with gain margin, phase margin, and disk margin.
- `diskmarginplot` visualizes the stability region in the complex plane.
- Disk margins are stricter than `margin`, ensuring robustness to combined gain/phase changes.

---

## 💻 Code Implementation (Combined Example)

Integrated script combining sensitivity analysis, multi-tool integration, and disk margins.

```matlab
% Session8_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Advanced stability and sensitivity analysis

% 🧹 Clear workspace
clear; clc;

% 📉 Sensitivity Analysis: Vary damping in second-order system
figure(1); hold on;
zeta_vals = [0.5, 1, 2]; % Damping ratios
for zeta = zeta_vals
    sys = tf([1], [1 2*zeta 1]); % 1/(s^2 + 2*zeta*s + 1)
    [y, t] = step(sys);
    plot(t, y, 'DisplayName', ['zeta = ' num2str(zeta)]);
end
title('Step Response Sensitivity to Damping'); xlabel('Time (s)'); ylabel('Output'); grid on; legend;

% 🔄 Combining Tools: Root locus, Bode, Nyquist
sys_ol = tf([1], [1 2 0]); % 1/(s(s+2))
figure(2); rlocus(sys_ol); title('Root Locus'); grid on;
figure(3); bode(sys_ol); title('Bode Plot'); grid on;
figure(4); nyquist(sys_ol); title('Nyquist Plot'); grid on;
[Gm, Pm] = margin(sys_ol);
disp(['Gain Margin: ' num2str(Gm) ' dB, Phase Margin: ' num2str(Pm) ' deg']);

% ✅ Disk Margins
DM = diskmargin(sys_ol);
disp('Disk Margins:'); disp(DM);
figure(5); diskmarginplot(DM); title('Disk Margin Plot');
```

**💬 Explanation**:
- **Sensitivity**: Varies damping to show response changes.
- **Combined Tools**: Uses root locus, Bode, and Nyquist for a holistic stability view.
- **Disk Margins**: Assesses robustness with `diskmargin`.
- Save as `Session8_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Analyze sensitivity in a power converter system; use multiple plots. System: G(s) = K/(s(s+1)) in unity feedback, vary K=1,5,10.

**💻 Solution Example**:
```matlab
% Exercise_Session8.m
% Sensitivity analysis for power converter

% 🧹 Clear workspace
clear; clc;

% 📉 Sensitivity: Vary gain K
K_vals = [1, 5, 10];
figure(1); hold on;
for K = K_vals
    sys_ol = tf([K], [1 1 0]); % K/(s(s+1))
    sys_cl = feedback(sys_ol, 1);
    [y, t] = step(sys_cl);
    plot(t, y, 'DisplayName', ['K = ' num2str(K)]);
end
title('Step Response Sensitivity to Gain'); xlabel('Time (s)'); ylabel('Output'); grid on; legend;

% 🔄 Combined Tools
sys_ol = tf([1], [1 1 0]); % K=1
figure(2); rlocus(sys_ol); title('Root Locus'); grid on;
figure(3); bode(sys_ol); title('Bode Plot'); grid on;
figure(4); nyquist(sys_ol); title('Nyquist Plot'); grid on;

% ✅ Disk Margins
DM = diskmargin(sys_ol);
disp('Disk Margins for K=1:'); disp(DM);
figure(5); diskmarginplot(DM); title('Disk Margin Plot for K=1');
```

**💬 Explanation**:
- **System**: G(s) = K/(s(s+1)), modeling a simplified power converter feedback loop.
- **Sensitivity**: Vary K to observe changes in step response (overshoot, settling time).
- **Combined Tools**: Root locus, Bode, and Nyquist plots for K=1 to assess stability.
- **Disk Margins**: Evaluate robustness; suggest increasing phase margin if low (e.g., add compensator).
- Students should analyze how higher K affects stability.

---

## 📌 Additional Notes
- **✅ Best Practices**: Use consistent parameter ranges for sensitivity analysis. Validate disk margins with classical margins.
- **🔍 Debugging Tips**: Check `sys_ol` is open-loop for `margin`, `diskmargin`. Ensure `tf` denominator is correct order.
- **🚀 Extensions**: Add parameter uncertainty (e.g., pole variation) or test MIMO systems.
- **🖥️ Lab Setup**: Control System Toolbox required for `diskmargin`, `bode`, `nyquist`, `rlocus`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📉 Analyze system sensitivity to parameter changes.
2. 🔄 Integrate root locus, Bode, and Nyquist for comprehensive analysis.
3. ✅ Assess robust stability using disk margins.

**📅 Next Session Preview**: Controller design basics (PID, lead-lag). 🎮

**📝 Assignment**: For G(s) = K/(s(s+2)), vary K=1,2,5, analyze sensitivity with step response, plot root locus/Bode/Nyquist, compute disk margins, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
