## 🧪 MATLAB Control System Experiments

📚 Course Lab Notebook for Beginners

---

# 🔹 Experiment 1: Modeling Different Types of Systems

Real systems are mathematically modeled to analyze stability, response, and controller design. MATLAB uses the **Control System Toolbox** for such modeling.

### ▶️ Types of System Models Demonstrated

| Representation    | Description                                | MATLAB Object |
| ----------------- | ------------------------------------------ | ------------- |
| Transfer Function | Ratio of output/input in Laplace domain    | `tf()`        |
| Zero-Pole-Gain    | Specified by system zeros, poles, and gain | `zpk()`       |
| State-Space       | Matrix representation                      | `ss()`        |
| Symbolic S-Domain | Using Laplace operator ‘s’                 | `s = tf('s')` |

---

### 🛠 Common Functions

| Function          | Syntax                 |
| ----------------- | ---------------------- |
| Transfer Function | `sys = tf(num, den)`   |
| Zero-Pole-Gain    | `sys = zpk(z, p, k)`   |
| State-Space       | `sys = ss(A, B, C, D)` |
| S-domain object   | `s = tf('s')`          |

---

### 💻 MATLAB Code Example

```matlab
% Transfer Function Example
num = [2];
den = [1 5 6];
sys_tf = tf(num, den)

% Zero Pole Gain Example
z = [-2];       % One zero at -2
p = [-3 -4];    % Two poles
k = 5;          % Gain
sys_zpk = zpk(z, p, k)

% S-domain object modeling (1st order system)
s = tf('s');
sys_s = 10/(s + 5)

% Convert between forms
sys_tf_to_zpk = zpk(sys_tf)
sys_zpk_to_tf = tf(sys_zpk)
```

---

### 📝 Practice Problems (Experiment 1)

1️⃣ Given poles: −1, −3, zero: −5, gain = 4 → Write MATLAB code to form transfer function and convert it to state-space.
2️⃣ Model a system:
Transfer function: (20) / (s² + 6s + 8) using:
a) `tf()` b) `zpk()` c) `s` object

Continue after completing these tasks.

---

# 🔹 Experiment 2: Transforming Block Diagram into Transfer Function

Block diagrams show input–output relations. Mathematical transformation simplifies control design.

### 🔧 Important Functions

| Purpose             | Function           |
| ------------------- | ------------------ |
| Series connection   | `series(G1, G2)`   |
| Parallel connection | `parallel(G1, G2)` |
| Feedback system     | `feedback(G, H)`   |
| Simplify model      | `minreal()`        |

---

### 🧭 Example Block Diagram Structure (Signal Flow)

```
    +─── G1(s) ────+
r → |              |→ y
    +─ H(s) ←──────+
```

This represents standard negative feedback:
G(s) forward path, H(s) feedback path

---

### 💻 MATLAB Code Example

```matlab
s = tf('s');

G1 = 10/(s+2);      % forward path
H = 0.5;            % feedback gain

sys_closed = feedback(G1, H)   % negative feedback
sys_series = series(G1, G1)    % G1*G1 connection
sys_parallel = parallel(G1, 5/(s+4))
```

---

### 📝 Practice Problems (Experiment 2)

Draw signal structure and write MATLAB code:

1️⃣ r → G(s)=30/(s+6) → y ; Feedback H(s)=1
Compute closed-loop TF.

2️⃣ Two blocks in series G1=5/(s+1), G2=10/(s+3).
Form overall TF using MATLAB commands.

---

# 🔹 Experiment 3: Time Domain Analysis of Different Systems

Time-domain analysis shows how systems respond in real-time.

### 🛠 Useful Functions

| Test                   | Command         |
| ---------------------- | --------------- |
| Step Response          | `step(sys)`     |
| Impulse Response       | `impulse(sys)`  |
| Time Domain Parameters | `stepinfo(sys)` |
| Poles & Stability      | `pole(sys)`     |

---

### 📌 Examples Demonstrated

✔ First-order
✔ Second-order: underdamped, critically damped, overdamped
✔ Real Application: **DC Motor speed model approximation**

---

### 💻 MATLAB Code Example

```matlab
s = tf('s');

% First-order system
sys1 = 5/(s+2);
figure; step(sys1); title('First Order System')

% Second-order system (change damping)
wn = 5;     % natural frequency
zeta1 = 0.2; % underdamped
zeta2 = 1.0; % critically damped

sys_underdamped = wn^2/(s^2 + 2*zeta1*wn*s + wn^2);
sys_critical    = wn^2/(s^2 + 2*zeta2*wn*s + wn^2);

figure; step(sys_underdamped); title('Underdamped')
figure; step(sys_critical); title('Critically Damped')

% Real example: DC motor approx
sys_dc = 100/(s*(s+10));
figure; step(sys_dc); title('DC Motor Step Response')
```

---

### 📝 Practice Problems (Experiment 3)

1️⃣ Plot impulse response of a 1st order system:
G(s)= 8/(s + 4) → Compare with step response.

2️⃣ Choose ζ = 0.5, 1.2 and plot step responses for
G(s)= ωₙ² / (s² + 2ζωₙs + ωₙ²), ωₙ = 8
Comment effect of damping.

---

# 🔹 Experiment 4: Frequency Domain Analysis

Frequency response reveals:
• gain/phase variation
• stability margins
• resonant behavior

### 🛠 Frequency Domain Tools

| Analysis       | Function       |
| -------------- | -------------- |
| Bode Plot      | `bode(sys)`    |
| Root Locus     | `rlocus(sys)`  |
| Pole-Zero Plot | `pzmap(sys)`   |
| Nyquist Plot   | `nyquist(sys)` |

---

### 💻 Code Example

```matlab
s = tf('s');
sys = 50 / (s^2 + 6*s + 5);

figure; bode(sys); title('Bode Plot')
figure; rlocus(sys); title('Root Locus')
figure; pzmap(sys); title('Pole-Zero Map')
figure; nyquist(sys); title('Nyquist Plot')
```

Interpretation strongly relates to **phase margin**, **gain margin**, and **closed-loop stability**.

---

### 📝 Practice Problems (Experiment 4)

1️⃣ For G(s)=80/(s(s+4)) → Draw Bode + Nyquist.
2️⃣ Locate stability using root-locus for 100/(s²+10s+25).

---

# 🔹 Experiment 5: PID Control of DC Motor

We first observe open-loop performance → then improve via PID.

### 🛠 Basic PID Function

| Feature              | MATLAB            |
| -------------------- | ----------------- |
| PID controller block | `pid(Kp, Ki, Kd)` |

---

### 💻 Code Example

```matlab
s = tf('s');

% DC Motor Transfer Function Approximation
G = 100/(s*(s+10));

% Open-loop
figure; step(G); title('Open Loop Response')

% PID Controller
C = pid(2, 10, 0.1);
sys_cl = feedback(C*G, 1);

figure;
step(G); hold on;
step(sys_cl);
legend('Open Loop','PID Controlled');
title('DC Motor Speed Control using PID')
```

---

### 📝 Practice Problems (Experiment 5)

1️⃣ Tune PID values to reduce steady-state error further.
2️⃣ Plot poles of open-loop vs closed-loop system.

---

# 🔹 Experiment 6: Simple Buck Converter Analysis + PID Control

Overly complex switching models avoided for beginners → small-signal averaged model.

Example:
Gv(s) ≈ K / (s(1 + s/wo))

```matlab
s = tf('s');
Gv = 20/(s*(1 + s/100));  % small-signal model

figure; step(Gv); title('Open Loop Buck Converter')

C = pid(0.5, 20, 0.01);
sys_cl = feedback(C*Gv, 1);

figure;
step(Gv); hold on;
step(sys_cl);
legend('Open Loop','PID Controlled');
title('Buck Converter Voltage Control')
```

---

### 📝 Practice Problems (Experiment 6)

1️⃣ Improve transient overshoot with different PID values.
2️⃣ Apply bode analysis before and after PID.

---

# 🔹 Experiment 7: AVR System Analysis + PID Control

Automatic Voltage Regulator → maintains generator voltage.

Simplified plant model:
G(s)=10/(s(1+0.1s))

```matlab
s = tf('s');
Gavr = 10/(s*(1+0.1*s));

% Open loop
figure; step(Gavr); title('AVR Open Loop')

% PID-controlled AVR
C = pid(5, 30, 0.01);
sys_cl = feedback(C*Gavr, 1);

figure;
step(Gavr); hold on;
step(sys_cl);
legend('Open Loop','With PID');
title('AVR PID Control Performance')
```

---

### 📝 Practice Problems (Experiment 7)

1️⃣ Compare frequency response (bode) before vs after PID.
2️⃣ Adjust PID to minimize peak overshoot.

---
