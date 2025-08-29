# 🚂 Lecture on Session 2: Modeling of Dynamic Systems

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Represent electrical and mechanical systems mathematically in code. This session focuses on modeling dynamic systems relevant to electrical engineering, such as circuits and motors, using MATLAB tools for simulation and analysis. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Session 1 (Introduction to MATLAB). Basic knowledge of differential equations, Laplace transforms, electrical circuits, and mechanical systems. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapters 2-3 for modeling)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – Especially DC Motor Speed Modeling section

---

## 🗂️ Topics Covered

1. 📐 **Differential Equations**: Solving ODEs using `ode45` or symbolic toolbox.
2. 🔄 **Transfer Function (TF)**: Definition: Laplace domain representation (numerator/denominator polynomials). Operations: Series (`series`), parallel (`parallel`), feedback (`feedback`).
3. 🧮 **State-Space (SS)**: Definition: Matrices A, B, C, D for first-order equations. Conversion: `tf2ss`, `ss2tf`, `canon` (canonical forms).
4. 📍 **Zero-Pole-Gain (ZPK)**: Alternative representation; conversions with `zpk`.

---

## 📝 Detailed Lecture Content

### 📐 1. Differential Equations
Dynamic systems in electrical engineering, such as RC circuits or motors, are often described by ordinary differential equations (ODEs). MATLAB provides tools to solve these numerically or symbolically. 🧮

- **Numerical Solver (`ode45`)**: Uses Runge-Kutta method for non-stiff ODEs. Syntax: `[t, y] = ode45(@odefun, tspan, y0);` where `@odefun` is a function handle defining dy/dt = f(t,y).
- **Symbolic Toolbox**: For exact solutions using `dsolve`. Requires Symbolic Math Toolbox; useful for simple systems.

**💡 Example**: Solve a first-order ODE for an RC circuit: dv/dt = -v/RC + u/RC (voltage across capacitor).

**💻 Example Code**:
```matlab
% Numerical: ode45 for RC circuit (R=1, C=1, input u=1)
function dydt = rc_ode(t, y)
    R = 1; C = 1; u = 1; % Step input
    dydt = -y/(R*C) + u/(R*C);
end

tspan = [0 5]; y0 = 0;
[t, y] = ode45(@rc_ode, tspan, y0);
figure; plot(t, y); xlabel('Time (s)'); ylabel('Voltage (V)');
title('RC Circuit Response (ode45)'); grid on;

% Symbolic: dsolve for same ODE
syms v(t) R C u
eqn = diff(v,t) == -v/(R*C) + u/(R*C);
v_sol = dsolve(eqn, v(0)==0);
disp(v_sol); % Output: u - u*exp(-t/(C*R))
```

**💬 Explanation**:
- The function `rc_ode` defines the ODE as a state derivative.
- `ode45` integrates from t=0 to 5 with initial condition y=0, producing time vector `t` and solution `y`.
- Symbolic `dsolve` gives an analytical expression, ideal for verification or simple systems.

---

### 🔄 2. Transfer Function (TF)
Transfer functions represent systems in the Laplace domain, simplifying analysis for linear time-invariant (LTI) systems like filters or amplifiers. 📈

- **Definition**: G(s) = Num(s)/Den(s), where Num and Den are polynomial coefficients (highest power first).
- **Operations**:
  - **Series**: Cascade systems, `series(G1, G2)`.
  - **Parallel**: Sum outputs, `parallel(G1, G2)`.
  - **Feedback**: Closed-loop, `feedback(G, H)` (negative feedback by default).

**💻 Example Code**:
```matlab
% Simple second-order TF: 1/(s^2 + 2s + 1)
num = [1]; den = [1 2 1]; sys = tf(num, den);
disp(sys); % Displays transfer function

% Operations
G1 = tf([1], [1 1]); % 1/(s+1)
G2 = tf([1], [1 2]); % 1/(s+2)
sys_series = series(G1, G2);
sys_parallel = parallel(G1, G2);
sys_feedback = feedback(G1, 1); % Unity feedback
```

**💬 Explanation**:
- `num = [1]; den = [1 2 1];` defines G(s) = 1 / (s^2 + 2s + 1), a damped oscillator.
- `tf(num, den)` creates the TF object.
- Operations combine systems: series multiplies TFs, parallel adds them, feedback computes G/(1 + G*H).

---

### 🧮 3. State-Space (SS)
State-space models use matrices for multi-input/multi-output (MIMO) systems, common in modern control (e.g., robotics). 🔗

- **Definition**: dx/dt = A x + B u; y = C x + D u (x: states, u: inputs, y: outputs).
- **Conversion**:
  - `tf2ss`: TF to SS.
  - `ss2tf`: SS to TF.
  - `canon`: Convert to canonical forms (e.g., controllable, observable).

**💻 Example Code**:
```matlab
% SS for second-order system: dx/dt = [0 1; -1 -2]x + [0;1]u; y = [1 0]x
A = [0 1; -1 -2]; B = [0;1]; C = [1 0]; D = 0;
sys_ss = ss(A, B, C, D);
disp(sys_ss); % Displays SS matrices

% Conversions
sys_tf = ss2tf(sys_ss); % To TF
sys_ss_back = tf2ss(sys_tf); % Back to SS
sys_canon = canon(sys_ss, 'companion'); % Companion form
```

**💬 Explanation**:
- Matrices: A (system dynamics), B (input coupling), C (output mapping), D (feedthrough).
- This models a mass-spring-damper: states [position, velocity].
- Conversions allow switching representations for different analyses.

---

### 📍 4. Zero-Pole-Gain (ZPK)
ZPK form emphasizes system roots (zeros, poles) and gain, useful for stability analysis. 📊

- **Definition**: G(s) = K * (s - z1)(s - z2)... / (s - p1)(s - p2)... 
- **Conversions**: Use `zpk(z, p, k)`; convert with `tf(sys_zpk)` or `ss(sys_zpk)`.

**💻 Example Code**:
```matlab
% ZPK: zeros=[], poles=[-1, -2], gain=1
z = []; p = [-1, -2]; k = 1;
sys_zpk = zpk(z, p, k);
disp(sys_zpk); % Displays ZPK form

% Conversions
sys_tf = tf(sys_zpk); % To TF
sys_ss = ss(sys_zpk); % To SS
```

**💬 Explanation**:
- Empty `z = []` means no zeros.
- ZPK highlights pole locations for root locus or stability (all poles left of imaginary axis).

---

## 💻 Code Implementation (Combined Example)

A script integrating topics: Solve ODE, create TF/SS/ZPK for a system, and convert.

```matlab
% Session2_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% Modeling dynamic systems

% 🧹 Clear workspace
clear; clc;

% 📐 ODE: Solve mass-spring-damper (m=1, c=2, k=1, F=1)
function dxdt = msd_ode(t, x)
    m=1; c=2; k=1; F=1;
    dxdt = [x(2); (F - c*x(2) - k*x(1))/m];
end
tspan = [0 10]; x0 = [0; 0];
[t, x] = ode45(@msd_ode, tspan, x0);
figure; plot(t, x(:,1)); xlabel('Time'); ylabel('Position');
title('Mass-Spring-Damper ODE Response'); grid on;

% 🔄 TF: 1/(s^2 + 2s + 1)
num = [1]; den = [1 2 1]; sys_tf = tf(num, den);

% 🧮 SS: Equivalent to above TF
A = [0 1; -1 -2]; B = [0;1]; C = [1 0]; D = 0;
sys_ss = ss(A, B, C, D);

% 📍 ZPK: From TF
sys_zpk = zpk(sys_tf);

% Conversions
sys_tf_from_ss = ss2tf(sys_ss);
disp('TF from SS:'); disp(sys_tf_from_ss);
```

**💬 Explanation**:
- ODE simulates a forced mass-spring-damper.
- Models the same system in TF, SS, ZPK.
- Conversions verify equivalence.

---

## 🛠️ Exercise
**Task**: Model a DC motor (electrical: armature circuit; mechanical: torque-speed) using TF and SS; convert between forms. Use parameters: J=0.01 kg.m², b=0.1 N.m.s, K=0.01 (torque/EMF constant), R=1 Ω, L=0.5 H.

**💻 Solution Example**:
```matlab
% Exercise_Session2.m
% DC Motor Model

% Parameters
J = 0.01; b = 0.1; K = 0.01; R = 1; L = 0.5;

% 🔄 TF: Speed(s)/Voltage(s) = K / ((J s + b)(L s + R) + K^2)
s = tf('s');
sys_tf = K / ((J*s + b)*(L*s + R) + K^2);
disp('DC Motor TF:'); disp(sys_tf);

% 🧮 SS: States [speed, current]
A = [-b/J, K/J; -K/L, -R/L];
B = [0; 1/L];
C = [1, 0]; D = 0;
sys_ss = ss(A, B, C, D);
disp('DC Motor SS:'); disp(sys_ss);

% Conversions
sys_ss_from_tf = tf2ss(sys_tf);
sys_tf_from_ss = ss2tf(sys_ss);
disp('SS from TF:'); disp(sys_ss_from_tf);
disp('TF from SS:'); disp(sys_tf_from_ss);
```

**💬 Explanation**:
- TF derived from electrical (L di/dt + R i = V - K speed) and mechanical (J d(speed)/dt + b speed = K i).
- SS uses states [speed, current], input V, output speed.
- Conversions confirm models match (minor numerical differences possible).

---

## 📌 Additional Notes
- **✅ Best Practices**: Define parameters symbolically first for clarity. Use `minreal` to simplify models.
- **🔍 Debugging Tips**: Check matrix dimensions (e.g., A n×n for n states). Use `pzmap(sys)` to visualize poles/zeros.
- **🚀 Extensions**: Model with nonzero D or MIMO (e.g., position output).
- **🖥️ Lab Setup**: Verify Symbolic Toolbox if using `dsolve`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📐 Solve ODEs for dynamic systems numerically and symbolically.
2. 🔄 Create and manipulate transfer functions.
3. 🧮 Build state-space models and convert representations.
4. 📍 Use ZPK for root-based analysis.

**📅 Next Session Preview**: Time-domain analysis of systems (responses, metrics). ⏱️

**📝 Assignment**: Modify the exercise for an RL circuit (inductor-resistor). Model with ODE, TF, SS; convert and simulate. Submit script and report by next session. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 29, 2025*
