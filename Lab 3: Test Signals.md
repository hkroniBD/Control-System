# ⚙️ **Test Signal Analysis in Control Systems**

---

## 🎯 **Purpose**

To understand how different **test inputs (signals)** are applied to a system and how their corresponding **output responses** reveal the **dynamic characteristics** of that system — such as stability, damping, delay, and steady-state behavior.

---

## 📘 **1. Common Test Signals and Their Uses**

| Test Signal         | MATLAB Function             | Mathematical Form | Physical Meaning                                          | Common Application                      |
| ------------------- | --------------------------- | ----------------- | --------------------------------------------------------- | --------------------------------------- |
| **Impulse**         | `impulse(sys)`              | δ(t)              | System’s *instantaneous reaction* to a sudden disturbance | Stability, transient study              |
| **Step**            | `step(sys)`                 | u(t)              | System’s *response to sudden change* in input             | Time-domain specs, control design       |
| **Ramp**            | `lsim(sys, t)` with r(t)=t  | t·u(t)            | System’s *ability to track continuously increasing input* | Tracking error, steady-state error      |
| **Parabolic**       | `lsim(sys, t)` with r(t)=t² | t²·u(t)           | Tests *higher order steady-state accuracy*                | Position/velocity control in servos     |
| **Sinusoidal**      | `lsim(sys, sin(wt), t)`     | sin(ωt)           | Frequency-domain behavior                                 | Frequency response, resonance           |
| **Square / Custom** | `lsim(sys, square(t))`      | periodic square   | Tests switching or relay responses                        | Power electronics, switching converters |

---

## 🧩 **2. Detailed Discussion of Each Test**

### 🔹 **Impulse Response**

**Definition:** The reaction of a system to a *unit impulse input (δ(t))* applied at t=0.

**MATLAB Example:**

```matlab
s = tf('s');
G = 5 / (s^2 + 3*s + 2);
impulse(G);
title('Impulse Response of Second Order System');
```

**Interpretation:**

* Reveals **natural frequency (ωₙ)** and **damping (ζ)** directly from oscillation pattern.
* If oscillations grow → system unstable.
* Used heavily in **transient testing** and **fault analysis** (e.g., mechanical shock, switching surges).

**Real-world example:**
When a car hits a bump (impulse), suspension response shows damping and spring stiffness quality.

---

### 🔹 **Step Response**

**Definition:** The system output when a *sudden constant input* is applied.

**MATLAB Example:**

```matlab
G = tf([10],[1 3 10]);
step(G);
stepinfo(G)
```

**Interpretation:**

* Provides **rise time, overshoot, settling time, steady-state value**.
* Most important for **control design and tuning**.

**Real-world example:**
In an automatic temperature controller, a step test represents switching the heater ON — we observe how quickly temperature stabilizes.

---

### 🔹 **Ramp Response**

**Definition:** Response to continuously increasing input.

**MATLAB Example:**

```matlab
t = 0:0.1:10;
u = t; % ramp input
[y,t,x] = lsim(G,u,t);
plot(t,y); grid on;
title('Ramp Response');
```

**Interpretation:**

* Used to determine **velocity error constant (Kv)**.
* Tells how well system tracks *linearly increasing commands* (e.g., moving object position).

**Real-world example:**
Elevator moving at uniform speed—motor must track increasing position with minimal error.

---

### 🔹 **Parabolic Response**

**Definition:** Response to input increasing quadratically with time.

**Use:** For systems controlling acceleration, where **position reference changes with time²**.

**Interpretation:**

* Tests **acceleration error constant (Ka)**.
* Systems with integral control can handle such inputs with finite error.

**Real-world example:**
Rocket launch trajectory controller—altitude increases roughly parabolically during early stages.

---

### 🔹 **Sinusoidal Response**

**Definition:** System’s response to sinusoidal input sin(ωt).

**MATLAB Example:**

```matlab
t = 0:0.01:10;
u = sin(2*t);
lsim(G,u,t);
title('Sinusoidal Input Response');
```

**Interpretation:**

* Shows **resonant frequency**, **phase lag**, and **gain margin** behavior.
* Key to **frequency response analysis** and **Bode plotting**.

**Real-world example:**
In vibration testing of aircraft wings — sinusoidal forces simulate turbulent air loads.

---

## 📊 **3. Comparative Overview**

| Input Type | Reveals                  | Common Parameter Found    | Importance in Design       |
| ---------- | ------------------------ | ------------------------- | -------------------------- |
| Impulse    | Stability, natural modes | Poles, ζ, ωₙ              | Fast system identification |
| Step       | Transient performance    | Rise, Settling, Overshoot | PID tuning                 |
| Ramp       | Steady-state tracking    | Velocity error            | Position tracking          |
| Parabolic  | Higher-order accuracy    | Acceleration error        | High-speed servo design    |
| Sinusoidal | Frequency sensitivity    | Gain & Phase              | Frequency response         |

---

## ⚙️ **4. When to Apply Which Signal**

| Design Phase          | Test Signal       | Purpose                                  |
| --------------------- | ----------------- | ---------------------------------------- |
| Early system modeling | Impulse           | Identify system order and poles          |
| Controller tuning     | Step              | Adjust transient behavior                |
| Servo design          | Ramp/Parabolic    | Measure tracking and steady-state errors |
| Resonance testing     | Sinusoidal        | Detect frequency peaks                   |
| Validation            | Real/custom input | Compare with experimental data           |

---

## 🔬 **5. MATLAB Demonstration Example**

Let’s use a **DC motor speed control system** as an example.

```matlab
clc; clear; s = tf('s');

% DC motor transfer function parameters
K = 100;      % Gain
tau = 0.5;    % Time constant
G = K / (tau*s + 1);

% Apply different inputs
t = 0:0.01:5;
u_ramp = t;
u_sin = sin(2*pi*1*t);

figure; step(G); title('Step Response - Speed Response to Voltage');
figure; impulse(G); title('Impulse Response - Torque Shock');
figure; lsim(G,u_ramp,t); title('Ramp Response - Constant Acceleration Command');
figure; lsim(G,u_sin,t); title('Sinusoidal Response - Vibration Sensitivity');
```

**Interpretation:**

* Step response shows how quickly the motor speed settles after voltage is applied.
* Impulse response models mechanical shock or sudden torque.
* Ramp response shows how well motor speed tracks changing speed demand.
* Sinusoidal response tests dynamic stability under periodic load disturbances.

---

## 🧮 **6. Connection with System Poles and Time-Domain Behavior**

Poles determine system response type:

| Pole Location     | System Nature     | Typical Test Signal  |
| ----------------- | ----------------- | -------------------- |
| Left-half plane   | Stable            | Step, Ramp           |
| On imaginary axis | Marginally stable | Sinusoidal           |
| Right-half plane  | Unstable          | Impulse (diagnostic) |

Use MATLAB to visualize:

```matlab
pzmap(G);
```

---

## 🧠 **7. Home Task for Students**

1. **Task-1:**
   Define and simulate a second-order system with **ζ = 0.3**, **ωₙ = 4 rad/s**, and generate **step, impulse, ramp, and sinusoidal responses**. Compare the results.

2. **Task-2:**
   For the same system, obtain system properties using:

   ```matlab
   stepinfo(G)
   damp(G)
   isstable(G)
   pole(G)
   zero(G)
   ```

   Prepare a short report interpreting each value physically.

3. **Task-3:**
   For a DC motor (given transfer function), test how increasing damping affects **step** and **ramp** response tracking.

4. **Task-4 (Advanced):**
   Write a MATLAB script that automatically generates **stepinfo results** and **plots for all test signals** for any transfer function entered by the user.

---
