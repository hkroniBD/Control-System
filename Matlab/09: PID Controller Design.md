# ⚙️ Lecture on Session 9: PID Controller Design

**👨‍🏫 Instructor**: Md. Hassanul Karim Roni, Assistant Professor, Department of Electrical and Electronic Engineering (EEE), Hajee Mohammad Danesh Science and Technology University (HSTU), Dinajpur, Bangladesh 🇧🇩

**🎯 Objective**: Design and tune PID controllers for common applications. This session equips students with skills to design and tune Proportional-Integral-Derivative (PID) controllers for control systems, such as DC motor speed control, using MATLAB for practical implementation. 🛠️

**⏳ Duration**: 2-3 hours (lecture + lab)

**📋 Prerequisites**: Completion of Sessions 1-8 (MATLAB basics, system modeling, time-domain analysis, Simulink, stability basics, Bode plots, Nyquist plots, advanced stability). Knowledge of transfer functions, feedback systems, and stability analysis. MATLAB with Control System Toolbox installed. 💻

**📚 Resources**:
- 🌐 MATLAB Documentation (MathWorks: [www.mathworks.com/help/control](https://www.mathworks.com/help/control))
- 📖 “Control Systems Engineering” by Norman S. Nise (Chapter 9 for PID control)
- 🔗 Online: Control Tutorials for MATLAB and Simulink ([ctms.engin.umich.edu](http://ctms.engin.umich.edu)) – PID Control section

---

## 🗂️ Topics Covered

1. 🧩 **PID Components**: Proportional (P), Integral (I), Derivative (D).
2. 🔄 **Closed-Loop**: `feedback(sys,1)` for unity feedback.
3. 🔧 **Tuning**: Ziegler-Nichols, `pidtune`, trial-and-error methods.
4. 💻 **Code Implementation**: `C = pid(Kp,Ki,Kd); cl = feedback(C*sys,1);`

---

## 📝 Detailed Lecture Content

### 🧩 1. PID Components
PID controllers are widely used in control systems (e.g., motor speed, temperature control) to achieve desired performance by adjusting control inputs based on error. 📈

- **Proportional (P)**: Reduces error proportionally; C(s) = Kp. Improves rise time but may cause steady-state error.
- **Integral (I)**: Eliminates steady-state error by integrating error over time; C(s) = Ki/s. May introduce overshoot.
- **Derivative (D)**: Reduces overshoot and improves stability by predicting error rate; C(s) = Kd·s. Sensitive to noise.
- **Combined PID**: C(s) = Kp + Ki/s + Kd·s, implemented via `pid(Kp,Ki,Kd)`.

**💻 Example Code**:
```matlab
% Define PID controller
Kp = 1; Ki = 0.1; Kd = 0.05;
C = pid(Kp, Ki, Kd); % PID controller: Kp + Ki/s + Kd*s
disp('PID Controller:'); disp(C);
```

**💬 Explanation**:
- `pid(Kp,Ki,Kd)` creates a PID controller object.
- Parameters Kp, Ki, Kd determine control action; tuning adjusts these for desired response.

---

### 🔄 2. Closed-Loop
PID controllers are typically used in closed-loop systems with unity feedback to track reference inputs, such as setpoint speed in motors. 🔗

- **Concept**: For plant G(s) and controller C(s), closed-loop transfer function is T(s) = C(s)G(s)/(1 + C(s)G(s)) for unity feedback.
- **MATLAB Tool**: `feedback(C*G,1)` computes the closed-loop system.
- **Performance**: Evaluate using step response (`step`) and metrics (`stepinfo`).

**💻 Example Code**:
```matlab
% Closed-loop system with PID
G = tf([1], [1 2 0]); % Plant: 1/(s(s+2))
Kp = 1; Ki = 0.1; Kd = 0.05;
C = pid(Kp, Ki, Kd);
sys_cl = feedback(C*G, 1); % Unity feedback
figure; step(sys_cl); title('Closed-Loop Step Response'); grid on;
info = stepinfo(sys_cl);
disp('Closed-Loop Metrics:'); disp(info);
```

**💬 Explanation**:
- `feedback(C*G,1)` forms the closed-loop system T(s).
- `step` plots the response; `stepinfo` provides metrics like overshoot and settling time.

---

### 🔧 3. Tuning
Tuning PID parameters (Kp, Ki, Kd) optimizes performance metrics (e.g., overshoot, settling time). Common methods include Ziegler-Nichols, `pidtune`, and trial-and-error. ⚙️

- **Ziegler-Nichols**: Based on critical gain (Kcr) and period (Pcr) from sustained oscillations.
  - Steps: Set Ki=Kd=0, increase Kp until oscillation (Kcr), note period (Pcr), apply tuning rules (e.g., PID: Kp=0.6Kcr, Ki=2Kp/Pcr, Kd=Kp·Pcr/8).
- **pidtune**: MATLAB’s automated tuning for specified performance (e.g., reference tracking).
- **Trial-and-Error**: Manually adjust Kp, Ki, Kd based on step response observations.

**💻 Example Code**:
```matlab
% PID tuning with pidtune
G = tf([1], [1 2 0]); % Plant: 1/(s(s+2))
C = pidtune(G, 'PID', 1); % Tune for bandwidth = 1 rad/s
disp('Tuned PID:'); disp(C);
sys_cl = feedback(C*G, 1);
figure; step(sys_cl); title('Closed-Loop Response with pidtune'); grid on;
```

**💬 Explanation**:
- `pidtune(G, 'PID', 1)` optimizes Kp, Ki, Kd for a target bandwidth.
- `step` verifies performance; adjust bandwidth or use manual tuning if needed.

---

## 💻 Code Implementation (Combined Example)

Integrated script for PID design, closed-loop analysis, and tuning.

```matlab
% Session9_Example.m
% 👨‍🏫 Instructor: Md. Hassanul Karim Roni, HSTU
% PID controller design

% 🧹 Clear workspace
clear; clc;

% 🧩 PID Components
Kp = 2; Ki = 1; Kd = 0.5;
C = pid(Kp, Ki, Kd); % Manual PID
disp('Manual PID Controller:'); disp(C);

% 🔄 Closed-Loop
G = tf([1], [1 2 0]); % Plant: 1/(s(s+2))
sys_cl = feedback(C*G, 1);
figure(1); step(sys_cl); title('Manual PID Closed-Loop Response'); grid on;
info = stepinfo(sys_cl);
disp('Manual PID Metrics:'); disp(info);

% 🔧 Tuning with pidtune
C_tuned = pidtune(G, 'PID', 2); % Target bandwidth = 2 rad/s
disp('Tuned PID Controller:'); disp(C_tuned);
sys_cl_tuned = feedback(C_tuned*G, 1);
figure(2); step(sys_cl_tuned); title('Tuned PID Closed-Loop Response'); grid on;
info_tuned = stepinfo(sys_cl_tuned);
disp('Tuned PID Metrics:'); disp(info_tuned);
```

**💬 Explanation**:
- **PID Components**: Defines a manual PID controller and displays its structure.
- **Closed-Loop**: Forms and analyzes the closed-loop system with manual PID.
- **Tuning**: Uses `pidtune` for automated tuning, comparing performance.
- Save as `Session9_Example.m` and run to visualize outputs.

---

## 🛠️ Exercise
**Task**: Tune a PID controller for DC motor speed control to achieve <5% overshoot. System: G(s) = K/(Js² + bs + K²), with J=0.01 kg·m², b=0.1 N·m·s, K=0.01 Nm/A.

**💻 Solution Example**:
```matlab
% Exercise_Session9.m
% PID tuning for DC motor speed control

% 🧹 Clear workspace
clear; clc;

% DC Motor Parameters
J = 0.01; b = 0.1; K = 0.01; % J: inertia, b: damping, K: torque constant
G = tf([K], [J b K^2]); % Plant: K/(Js^2 + bs + K^2)

% 🔧 Tune PID with pidtune
C = pidtune(G, 'PID', 10, pidtuneOptions('PhaseMargin', 60)); % Target PM for low overshoot
disp('Tuned PID Controller:'); disp(C);

% 🔄 Closed-Loop
sys_cl = feedback(C*G, 1);
figure; step(sys_cl); title('DC Motor Speed Response'); grid on;
info = stepinfo(sys_cl);
disp('Closed-Loop Metrics:'); disp(info);

% 📉 Verify Overshoot
if info.Overshoot < 5
    disp('Overshoot < 5% achieved!');
else
    disp('Adjust tuning: Increase phase margin or reduce bandwidth.');
end
```

**💬 Explanation**:
- **System**: DC motor transfer function G(s) = K/(Js² + bs + K²) models speed vs. voltage.
- **Tuning**: `pidtune` targets bandwidth=10 rad/s and phase margin=60° to minimize overshoot (<5%).
- **Closed-Loop**: `step` and `stepinfo` verify performance; overshoot checked programmatically.
- Students can adjust `PhaseMargin` or bandwidth if overshoot exceeds 5%.

---

## 📌 Additional Notes
- **✅ Best Practices**: Start with `pidtune` for initial parameters, then refine manually. Check stability with `margin(sys_cl)`.
- **🔍 Debugging Tips**: Ensure plant model is accurate; verify `feedback` uses correct sign (negative for unity feedback).
- **🚀 Extensions**: Test PID with non-unity feedback or add disturbance rejection.
- **🖥️ Lab Setup**: Control System Toolbox required for `pid`, `pidtune`, `feedback`.

---

## 🎓 Learning Outcomes
By the end of this session, students will:
1. 🧩 Understand PID components and their effects.
2. 🔄 Design closed-loop systems with PID controllers.
3. 🔧 Tune PID controllers using automated and manual methods.

**📅 Next Session Preview**: Lead-lag compensator design. 🎛️

**📝 Assignment**: For G(s) = 1/(s(s+1)), design and tune a PID controller for <10% overshoot and settling time <2s, plot step response, and submit script/report. 📄

---

*Prepared by Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU, Dinajpur, Bangladesh.*  
*📅 Date: August 30, 2025*
