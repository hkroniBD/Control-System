# 🎢 Lecture on Session 5: Stability Analysis Basics

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Assess system stability through pole locations and criteria. This session equips students with techniques to evaluate the stability of control systems, such as amplifiers or motor controllers, using MATLAB tools to analyze pole locations and apply stability criteria. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-4 (MATLAB basics, system modeling, time-domain analysis, Simulink). Knowledge of Laplace transforms and transfer functions. MATLAB with Control System Toolbox and Symbolic Math Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapter 6 for stability and root locus)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Stability and Root Locus sections

---

## 🗂️ Topics Covered

1. 📍 **S-Plane Stability**: All poles in left-half plane.
2. 📊 **Routh-Hurwitz Criterion**: Array construction; special cases (row of zeros).
3. 🔄 **Root Locus**:
   - Rules: Asymptotes, break points, angle of departure.
   - Analysis: Gain effects on poles.

---

## 📝 Detailed Lecture Content

### 📍 1. S-Plane Stability
Stability is critical for control systems like amplifiers or power systems to ensure bounded outputs. In the s-plane (Laplace domain), stability is determined by pole locations. 🌐

- **Definition**: A linear time-invariant (LTI) system is stable if all poles of its transfer function have negative real parts (lie in the left-half plane, LHP).
- **Implications**:
  - Real pole (σ < 0): Exponential decay.
  - Complex conjugate poles (σ ± jω, σ < 0): Damped oscillations.
  - Poles on imaginary axis (σ = 0): Marginal stability (sustained oscillations).
  - Poles in right-half plane (RHP, σ > 0): Instability (exponential growth).
- **MATLAB Tool**: `pole(sys)` to find poles; `pzmap(sys)` to visualize.

**💻 Example Code**:
```matlab
% Check stability via poles
sys = tf([1], [1 2 2]); % 1/(s^2 + 2s + 2)
poles = pole(sys);
disp('Poles:'); disp(poles);
figure; pzmap(sys); title('Pole-Zero Map'); grid on;
```

**💬 Explanation**:
- `pole(sys)` returns pole locations (e.g., -1 ± j for above system).
- `pzmap` plots poles (x) and zeros (o); all poles in LHP indicate stability.

---

### 📊 2. Routh-Hurwitz Criterion
The Routh-Hurwitz criterion determines stability without computing roots, ideal for high-order systems like feedback amplifiers. 📉

- **Concept**: Constructs a Routh array from denominator coefficients to count RHP poles.
- **Steps**:
  1. Form array with coefficients of s^n, s^(n-1), ..., s^0.
  2. Compute subsequent rows using formulas.
  3. Stability: No sign changes in first column (no RHP poles).
- **Special Cases**:
  - **Row of Zeros**: Indicates potential imaginary poles; solve auxiliary polynomial.
  - **Zero in First Column**: Replace with small ε, analyze limit.

**💻 Example Code**:
```matlab
% Routh-Hurwitz for s^3 + 3s^2 + 3s + 1
syms s;
den = [1 3 3 1]; % s^3 + 3s^2 + 3s + 1
% Custom Routh function
function routh_table = routh(den)
    n = length(den);
    routh_table = zeros(n, ceil(n/2));
    % Initialize first two rows
    routh_table(1, 1:2:end) = den(1:2:end);
    routh_table(2, 1:2:end) = den(2:2:end);
    % Compute remaining rows
    for i = 3:n
        for j = 1:size(routh_table,2)-1
            if routh_table(i-1,1) == 0, routh_table(i-1,1) = eps; end
            routh_table(i,j) = (routh_table(i-1,1)*routh_table(i-2,j+1) - ...
                routh_table(i-2,1)*routh_table(i-1,j+1)) / routh_table(i-1,1);
        end
    end
end
table = routh(den);
disp('Routh Table:'); disp(table);
sign_changes = sum(diff(sign(table(:,1))) ~= 0);
disp(['Number of RHP poles: ' num2str(sign_changes)]);
```

**💬 Explanation**:
- `den = [1 3 3 1]` corresponds to s^3 + 3s^2 + 3s + 1 (Hurwitz, stable).
- Custom `routh` function builds array; sign changes in first column indicate RHP poles.
- For special cases, manual analysis or Symbolic Toolbox (`solve` for auxiliary polynomial) is needed.

---

### 🔄 3. Root Locus
Root locus plots how closed-loop poles move with varying gain K in a feedback system, critical for designing stable amplifiers or controllers. 📈

- **Concept**: For G(s)H(s) in unity feedback, solve 1 + K G(s)H(s) = 0 for pole paths as K varies.
- **Rules**:
  - **Start/End**: Poles (K=0) to zeros or infinity.
  - **Asymptotes**: Angles (180°(2q+1)/n-m), centroid (sum poles - sum zeros)/(n-m).
  - **Break Points**: Where dK/ds = 0.
  - **Angle of Departure**: From complex poles, based on angles to other poles/zeros.
- **MATLAB Tool**: `rlocus(sys)` plots locus; `rltool` for interactive design.

**💻 Example Code**:
```matlab
% Root locus for G(s) = 1/(s(s+1))
sys = tf([1], [1 1 0]); % Open-loop TF
figure; rlocus(sys); title('Root Locus of 1/(s(s+1))'); grid on;
```

**💬 Explanation**:
- `sys` is open-loop G(s)H(s).
- `rlocus` shows pole paths; click plot to find K at specific points.
- Poles moving to RHP indicate instability at high K.

---

## 💻 Code Implementation (Combined Example)

Below is a complete script integrating s-plane, Routh-Hurwitz, and root locus for a third-order system, with detailed comments for clarity.

```matlab
% Session5_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Stability analysis for a third-order system

% 🧹 Clear workspace and command window
clear; clc;

% 📍 S-Plane: Check stability via pole locations
% Define a third-order system: 1/(s^3 + 3s^2 + 3s + 1)
sys = tf([1], [1 3 3 1]);
poles = pole(sys); % Compute poles
disp('Poles of the system:'); disp(poles); % Display poles
% Plot pole-zero map to visualize stability
figure;
pzmap(sys);
title('Pole-Zero Map of 1/(s^3 + 3s^2 + 3s + 1)');
grid on;

% 📊 Routh-Hurwitz: Construct Routh array to check stability
% Custom function to build Routh table
function routh_table = routh(den)
    n = length(den);
    routh_table = zeros(n, ceil(n/2));
    % Populate first two rows with coefficients
    routh_table(1, 1:2:end) = den(1:2:end);
    routh_table(2, 1:2:end) = den(2:2:end);
    % Compute subsequent rows
    for i = 3:n
        for j = 1:size(routh_table,2)-1
            % Handle zero in first column with small epsilon
            if routh_table(i-1,1) == 0
                routh_table(i-1,1) = eps;
            end
            routh_table(i,j) = (routh_table(i-1,1)*routh_table(i-2,j+1) - ...
                routh_table(i-2,1)*routh_table(i-1,j+1)) / routh_table(i-1,1);
        end
    end
end
% Apply Routh-Hurwitz to system denominator
den = [1 3 3 1]; % s^3 + 3s^2 + 3s + 1
table = routh(den);
disp('Routh-Hurwitz Table:'); disp(table);
% Count sign changes in first column to determine RHP poles
sign_changes = sum(diff(sign(table(:,1))) ~= 0);
disp(['Number of RHP poles: ' num2str(sign_changes)]);

% 🔄 Root Locus: Analyze pole movement with gain
% Open-loop system for feedback: G(s) = 1/(s(s+1))
sys_ol = tf([1], [1 1 0]); % Open-loop TF
figure;
rlocus(sys_ol);
title('Root Locus of 1/(s(s+1))');
grid on;
```

**💬 Explanation**:
- **S-Plane**: The system 1/(s³ + 3s² + 3s + 1) is analyzed; `pzmap` shows all poles in LHP, confirming stability.
- **Routh-Hurwitz**: The custom `routh` function builds the array; no sign changes in the first column indicate stability.
- **Root Locus**: A different system, 1/(s(s+1)), is used to demonstrate pole movement with gain K in a feedback loop, showing potential instability as poles cross into RHP.
- Save as `Session5_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Apply Routh-Hurwitz to a third-order system and plot the root locus for a feedback amplifier. System: G(s) = K/(s³ + 4s² + 5s + 2) in unity feedback.

**💻 Solution Example**:
```matlab
% Exercise_Session5.m
% Stability analysis for feedback amplifier

% 🧹 Clear workspace
clear; clc;

% System: G(s) = K/(s^3 + 4s^2 + 5s + 2)
K = 1; % Initial gain
sys_ol = tf([K], [1 4 5 2]); % Open-loop TF
sys_cl = feedback(sys_ol, 1); % Closed-loop TF

% 📍 Check poles
poles = pole(sys_cl);
disp('Closed-loop poles:'); disp(poles);
figure; pzmap(sys_cl); title('Pole-Zero Map'); grid on;

% 📊 Routh-Hurwitz
den = [1 4 5 2]; % Denominator coefficients
function routh_table = routh(den)
    n = length(den);
    routh_table = zeros(n, ceil(n/2));
    routh_table(1, 1:2:end) = den(1:2:end);
    routh_table(2, 1:2:end) = den(2:2:end);
    for i = 3:n
        for j = 1:size(routh_table,2)-1
            if routh_table(i-1,1) == 0, routh_table(i-1,1) = eps; end
            routh_table(i,j) = (routh_table(i-1,1)*routh_table(i-2,j+1) - ...
                routh_table(i-2,1)*routh_table(i-1,j+1)) / routh_table(i-1,1);
        end
    end
end
table = routh(den);
disp('Routh-Hurwitz Table:'); disp(table);
sign_changes = sum(diff(sign(table(:,1))) ~= 0);
disp(['Number of RHP poles: ' num2str(sign_changes)]);

% 🔄 Root Locus
figure; rlocus(sys_ol); title('Root Locus of K/(s^3 + 4s^2 + 5s + 2)'); grid on;
```

**💬 Explanation**:
- **System**: Closed-loop TF is K/(s³ + 4s² + 5s + 2 + K); analyze for K=1.
- **Poles**: `pole` and `pzmap` show closed-loop pole locations.
- **Routh-Hurwitz**: Apply to characteristic equation s³ + 4s² + 5s + (2+K); check stability for varying K.
- **Root Locus**: `rlocus` shows how poles move with K, identifying critical K for instability.
- Students should modify K (e.g., 10, 20) to observe stability changes.

---

## 📌 Additional Notes
- **✅ Best Practices**: Verify pole locations with `roots(den)` for small systems. Use `rltool` for interactive root locus design.
- **🔍 Debugging Tips**: Ensure correct denominator order in `tf` (highest power first). Check Routh array for numerical errors with `eps`.
- **🚀 Extensions**: Analyze a system with zeros (e.g., (s+1)/(s(s+2))) or special case (row of zeros).
- **🖥️ Lab Setup**: Control System Toolbox required for `rlocus`, `pzmap`. Symbolic Toolbox helps for special Routh cases.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📍 Determine stability via s-plane pole locations.
2. 📊 Apply Routh-Hurwitz to count RHP poles.
3. 🔄 Analyze gain effects using root locus.

**📅 Next Session Preview**: Frequency domain analysis with Bode plots. 🌊

**📝 Assignment**: For G(s) = K/(s(s+1)(s+2)), apply Routh-Hurwitz, plot root locus, find critical K for stability, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
