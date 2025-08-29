# 🎯 Lecture on Session 11: State-Space Control: Pole Placement

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Control states directly using pole placement. This session equips students with state-space techniques to design controllers for systems like magnetic levitation by placing closed-loop poles to achieve desired dynamics, using MATLAB tools. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-10 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability, PID control, lead-lag compensation). Knowledge of state-space models, linear algebra, and control theory. MATLAB with Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapter 7 for state-space methods)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – State-Space Control section

---

## 🗂️ Topics Covered

1. 📏 **Controllability/Observability**: `ctrb`, `obsv` for system analysis.
2. 🔄 **Feedback**: State feedback control law `u = -Kx`.
3. 🔧 **Pole Placement**: Ackermann’s method via `acker`.

---

## 📝 Detailed Lecture Content

### 📏 1. Controllability/Observability
Controllability and observability determine whether a state-space system can be fully controlled or observed, essential for designing controllers in applications like magnetic levitation. 📐

- **Controllability**: Ability to drive all states to any desired value using input u. A system (A, B) is controllable if the controllability matrix `ctrb(A,B)` has full rank (rank = n, where n is the number of states).
- **Observability**: Ability to estimate all states from output y. A system (A, C) is observable if the observability matrix `obsv(A,C)` has full rank.
- **MATLAB Tools**: `ctrb`, `obsv`, and `rank` to check system properties.

**💻 Example Code**:
```matlab
% Check controllability and observability
A = [0 1; -2 -3]; % State matrix
B = [0; 1]; % Input matrix
C = [1 0]; % Output matrix
D = 0; % Feedthrough
Co = ctrb(A, B); % Controllability matrix
Ob = obsv(A, C); % Observability matrix
disp('Controllability Matrix Rank:'); disp(rank(Co));
disp('Observability Matrix Rank:'); disp(rank(Ob));
```

**💬 Explanation**:
- `ctrb(A,B)` computes [B, AB, A²B, ...]; full rank (2 for 2 states) means controllable.
- `obsv(A,C)` computes [C; CA; CA²; ...]; full rank means observable.
- System is fully controllable/observable if ranks equal the state dimension.

---

### 🔄 2. Feedback
State feedback control uses the law `u = -Kx` to place closed-loop poles, adjusting system dynamics for stability and performance. 🔗

- **Concept**: For state-space model dx/dt = Ax + Bu, y = Cx + Du, apply control u = -Kx, resulting in closed-loop dynamics dx/dt = (A - BK)x.
- **Goal**: Choose gain matrix K to place closed-loop poles (eigenvalues of A - BK) at desired locations.
- **Effect**: Modifies system response (e.g., faster response, reduced overshoot).

**💻 Example Code**:
```matlab
% State feedback
A = [0 1; -2 -3]; B = [0; 1]; % System matrices
poles = [-2, -3]; % Desired closed-loop poles
K = acker(A, B, poles); % Compute feedback gain
A_cl = A - B*K; % Closed-loop A matrix
sys_cl = ss(A_cl, B, [1 0], 0); % Closed-loop system
figure; step(sys_cl); title('Closed-Loop Step Response'); grid on;
```

**💬 Explanation**:
- `acker` computes K to place poles of A - BK at specified locations.
- `step` shows the closed-loop response with desired dynamics.

---

### 🔧 3. Pole Placement
Pole placement designs the feedback gain K to achieve specific closed-loop pole locations, using Ackermann’s method for single-input systems. ⚙️

- **Concept**: Select poles to meet performance specs (e.g., settling time, overshoot).
- **Ackermann’s Method**: Computes K ensuring det(sI - (A - BK)) matches the desired characteristic polynomial.
- **MATLAB Tool**: `acker(A,B,poles)` for single-input systems; `place` for multi-input (more robust numerically).

**💻 Example Code**:
```matlab
% Pole placement with acker
A = [0 1; -2 -3]; B = [0; 1];
poles = [-2+2j, -2-2j]; % Complex poles for oscillatory response
K = acker(A, B, poles);
disp('Feedback Gain K:'); disp(K);
A_cl = A - B*K;
sys_cl = ss(A_cl, B, [1 0], 0);
figure; step(sys_cl); title('Closed-Loop Response with Pole Placement'); grid on;
```

**💬 Explanation**:
- `acker` places poles at s = -2 ± 2j, yielding damped oscillations.
- `step` verifies response; adjust poles for different dynamics.

---

## 💻 Code Implementation (Combined Example)

Integrated script for controllability/observability, state feedback, and pole placement.

```matlab
% Session11_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% State-space pole placement

% 🧹 Clear workspace
clear; clc;

% 📏 Controllability/Observability
A = [0 1; -2 -3]; % State matrix
B = [0; 1]; % Input matrix
C = [1 0]; % Output matrix
D = 0; % Feedthrough
Co = ctrb(A, B);
Ob = obsv(A, C);
disp('Controllability Matrix Rank:'); disp(rank(Co));
disp('Observability Matrix Rank:'); disp(rank(Ob));

% 🔄 Feedback and Pole Placement
poles = [-2+2j, -2-2j]; % Desired poles
K = acker(A, B, poles);
disp('Feedback Gain K:'); disp(K);
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);
figure; step(sys_cl); title('Closed-Loop Response with Pole Placement'); grid on;
info = stepinfo(sys_cl);
disp('Closed-Loop Metrics:'); disp(info);
```

**💬 Explanation**:
- **Controllability/Observability**: Verifies system is fully controllable/observable.
- **Feedback/Pole Placement**: Places poles at s = -2 ± 2j for oscillatory response.
- **Analysis**: `step` and `stepinfo` evaluate performance.
- Save as `Session11_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Design a pole placement controller for a magnetic levitation system to stabilize it. System: dx/dt = Ax + Bu, with A=[0 1; 1 0], B=[0; 1], place poles at s=-2,-3.

**💻 Solution Example**:
```matlab
% Exercise_Session11.m
% Pole placement for magnetic levitation

% 🧹 Clear workspace
clear; clc;

% System: Magnetic levitation
A = [0 1; 1 0]; % State matrix (unstable, poles at ±1)
B = [0; 1]; % Input matrix
C = [1 0]; % Output matrix
D = 0; % Feedthrough

% 📏 Check Controllability
Co = ctrb(A, B);
disp('Controllability Matrix Rank:'); disp(rank(Co));

% 🔧 Pole Placement
poles = [-2, -3]; % Desired stable poles
K = acker(A, B, poles);
disp('Feedback Gain K:'); disp(K);
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);
figure; step(sys_cl); title('MagLev Closed-Loop Response'); grid on;
info = stepinfo(sys_cl);
disp('Closed-Loop Metrics:'); disp(info);

% Verify Stability
if isstable(sys_cl)
    disp('System stabilized!');
else
    disp('Adjust pole locations.');
end
```

**💬 Explanation**:
- **System**: A=[0 1; 1 0] has poles at s=±1 (unstable).
- **Controllability**: `ctrb` confirms full rank (controllable).
- **Pole Placement**: `acker` places poles at s=-2, -3 for stability.
- **Verification**: `step` and `isstable` confirm stable response with desired dynamics.

---

## 📌 Additional Notes
- **✅ Best Practices**: Ensure controllability before pole placement. Choose realistic poles (avoid excessive control effort).
- **🔍 Debugging Tips**: Check matrix dimensions in `acker`. Verify pole locations with `eig(A_cl)`.
- **🚀 Extensions**: Add observer design for unmeasurable states or test in Simulink.
- **🖥️ Lab Setup**: Control System Toolbox required for `ctrb`, `obsv`, `acker`, `ss`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 📏 Assess controllability and observability of state-space systems.
2. 🔄 Implement state feedback control.
3. 🔧 Design controllers using pole placement with Ackermann’s method.

**📅 Next Session Preview**: Optimal control basics (LQR). 🌟

**📝 Assignment**: For A=[0 1; -2 1], B=[0; 1], place poles at s=-1±j, plot step response, verify controllability, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
