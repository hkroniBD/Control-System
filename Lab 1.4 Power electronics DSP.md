Got it 👍 — here’s the **GitHub-safe Markdown version** of your entire lecture document.
All equations are now written in **plain-text / code-friendly format** (no LaTeX), so they’ll render perfectly on GitHub, Obsidian, Notion, or any Markdown viewer.
Everything else — tables, code blocks, and formatting — is preserved cleanly.

---

# ⚡ Advanced Lecture Notes: Power Electronics, Consumer Electronics, and DSP Systems

Below is a complete and advanced lecture continuation — similar in structure to the **Power Systems** section — now extended across three major domains:

* ⚡ **Power Electronics** (Converters, Inverters, Regulators)
* 📱 **Consumer Electronics** (Audio, Sensors, Displays, etc.)
* 🧠 **Digital Signal Processing (DSP)** (Filters, Control Loops, Discrete-Time Systems)

Each includes **realistic parameters**, **transfer functions**, **MATLAB code**, and **physical insight** — all directly simulatable using `tf()` or `tfest()` for control and signal modeling.

---

## ⚡ SECTION A — POWER ELECTRONICS SYSTEMS

---

### 🔋 1. Buck Converter (Step-Down DC–DC Converter)

**Ref:** Erickson & Maksimovic, *Fundamentals of Power Electronics*

| Parameter       | Symbol |  Value | Unit |
| --------------- | :----: | :----: | :--: |
| Input Voltage   |   Vin  |   24   |   V  |
| Inductance      |    L   | 200 µH |   H  |
| Capacitance     |    C   | 470 µF |   F  |
| Load Resistance |    R   |   10   |   Ω  |
| Duty Ratio      |    D   |   0.5  |   —  |

**Transfer Function**

```
Gvd(s) = Vin / [1 + s*(R*C)/(1 - D) + (s^2)*L*C]
```

**MATLAB Code**

```matlab
Vin = 24; L = 200e-6; C = 470e-6; R = 10; D = 0.5;
num = [Vin];
den = [L*C, R*C/(1-D), 1];
G_buck = tf(num, den)
bode(G_buck)
```

**Observation:**
Second-order system; resonance at ~500–800 Hz; damping depends on load R and C.

---

### ⚙️ 2. Boost Converter (Step-Up DC–DC Converter)

**Ref:** Mohan, *Power Electronics: Converters, Applications, and Design*

| Parameter       | Symbol |  Value | Unit |
| --------------- | :----: | :----: | :--: |
| Input Voltage   |   Vin  |   12   |   V  |
| Inductance      |    L   | 150 µH |   H  |
| Capacitance     |    C   | 220 µF |   F  |
| Load Resistance |    R   |   20   |   Ω  |
| Duty Ratio      |    D   |   0.6  |   —  |

**Transfer Function**

```
Gvd(s) = Vin*(1 - D) / [L*C*s^2 + (R*C*(1 - D)^2)*s + 1]
```

**MATLAB Code**

```matlab
Vin = 12; L = 150e-6; C = 220e-6; R = 20; D = 0.6;
num = [Vin*(1-D)];
den = [L*C, R*C*(1-D)^2, 1];
G_boost = tf(num, den)
bode(G_boost)
```

**Observation:**
Underdamped if L or C is large; potential right-half-plane zero (non-minimum phase).

---

### ⚡ 3. Buck–Boost Converter

**Ref:** Rashid, *Power Electronics Handbook*

| Parameter       | Symbol |  Value | Unit |
| --------------- | :----: | :----: | :--: |
| Input Voltage   |   Vin  |   12   |   V  |
| Inductance      |    L   | 100 µH |   H  |
| Capacitance     |    C   | 330 µF |   F  |
| Load Resistance |    R   |   15   |   Ω  |
| Duty Ratio      |    D   |   0.4  |   —  |

**Transfer Function**

```
Gvd(s) = -Vin*D / [L*C*s^2 + (R*C)*s + (1 - D)^2]
```

**MATLAB Code**

```matlab
Vin = 12; L = 100e-6; C = 330e-6; R = 15; D = 0.4;
num = [-Vin*D];
den = [L*C, R*C, (1-D)^2];
G_bb = tf(num, den)
bode(G_bb)
```

**Observation:**
Negative gain; output polarity inverted; moderate resonant peak (~1 kHz).

---

### 🔄 4. Single-Phase Full-Bridge Inverter (with LC Filter)

**Ref:** Rashid, *Power Electronics Handbook*

| Parameter       | Symbol | Value | Unit |
| --------------- | :----: | :---: | :--: |
| Inductance      |    L   |  5 mH |   H  |
| Capacitance     |    C   | 50 µF |   F  |
| Load Resistance | R_load |   20  |   Ω  |

**Transfer Function**

```
G(s) = 1 / [L*C*s^2 + R*C*s + 1]
```

**MATLAB Code**

```matlab
L = 5e-3; C = 50e-6; R = 20;
num = [1];
den = [L*C, R*C, 1];
G_inverter = tf(num, den)
bode(G_inverter)
```

**Observation:**
Natural frequency ≈ 632 rad/s (~100 Hz). LC filter smooths PWM harmonics.

---

### 🔌 5. Single-Phase Rectifier + Filter

| Parameter   | Symbol |  Value | Unit |
| ----------- | :----: | :----: | :--: |
| Resistance  |    R   |   100  |   Ω  |
| Inductance  |    L   |  0.01  |   H  |
| Capacitance |    C   | 470 µF |   F  |

**Transfer Function**

```
G(s) = 1 / [L*C*s^2 + R*C*s + 1]
```

**MATLAB Code**

```matlab
R = 100; L = 0.01; C = 470e-6;
num = [1];
den = [L*C, R*C, 1];
G_rect = tf(num, den)
bode(G_rect)
```

**Observation:**
Low-frequency ripple filtered; time constant ≈ 0.047 s.

---

## 📱 SECTION B — CONSUMER ELECTRONICS SYSTEMS

---

### 🎧 1. Audio Amplifier (Single-Stage)

| Parameter     | Symbol | Value | Unit |
| ------------- | :----: | :---: | :--: |
| Gain          |    K   |   50  |   —  |
| Dominant Pole |   fp   | 2 kHz |  Hz  |

**Transfer Function**

```
G(s) = K / [1 + s/(2π*fp)]
```

**MATLAB Code**

```matlab
K = 50; fp = 2000; wp = 2*pi*fp;
num = [K]; den = [1/wp, 1];
G_amp = tf(num, den)
bode(G_amp)
```

**Observation:**
First-order low-pass; bandwidth ≈ 2 kHz.

---

### 📺 2. LED Driver (Current-Control Loop)

| Parameter     | Symbol | Value | Unit |
| ------------- | :----: | :---: | :--: |
| Control Gain  |    K   |   5   |   —  |
| Time Constant |    τ   |  0.02 |   s  |

**Transfer Function**

```
G(s) = K / (τ*s + 1)
```

**MATLAB Code**

```matlab
K = 5; tau = 0.02;
num = [K]; den = [tau, 1];
G_led = tf(num, den)
step(G_led)
```

**Observation:**
Fast current control; τ = 20 ms ensures flicker-free illumination.

---

### 📷 3. Camera Autofocus System

| Parameter     | Symbol | Value | Unit |
| ------------- | :----: | :---: | :--: |
| Actuator Gain |    K   |  100  |   —  |
| Time Constant |    τ   |  0.1  |   s  |

**Transfer Function**

```
G(s) = K / (τ*s + 1)
```

**MATLAB Code**

```matlab
K = 100; tau = 0.1;
num = [K]; den = [tau, 1];
G_focus = tf(num, den)
step(G_focus)
```

**Observation:**
Fast 0.1 s lens refocus; exponential convergence.

---

### 📶 4. Bluetooth RF Amplifier

| Parameter        | Symbol | Value | Unit |
| ---------------- | :----: | :---: | :--: |
| Gain             |    K   |   20  |   —  |
| Cutoff Frequency |   fc   | 1 MHz |  Hz  |

**Transfer Function**

```
G(s) = K / [1 + s/(2π*fc)]
```

**MATLAB Code**

```matlab
K = 20; fc = 1e6; wc = 2*pi*fc;
num = [K]; den = [1/wc, 1];
G_rf = tf(num, den)
bode(G_rf)
```

**Observation:**
First-order; very high bandwidth (~1 MHz).

---

### 🎮 5. MEMS Accelerometer (Sensor Dynamics)

| Parameter         | Symbol | Value |  Unit |
| ----------------- | :----: | :---: | :---: |
| Sensitivity       |    K   |   1   |   —   |
| Natural Frequency |   wn   |  1000 | rad/s |
| Damping Ratio     |    ζ   |  0.7  |   —   |

**Transfer Function**

```
G(s) = (K*wn^2) / (s^2 + 2*ζ*wn*s + wn^2)
```

**MATLAB Code**

```matlab
K = 1; wn = 1000; zeta = 0.7;
num = [K*wn^2];
den = [1, 2*zeta*wn, wn^2];
G_acc = tf(num, den)
bode(G_acc)
```

**Observation:**
Resonant at ~160 Hz; damping (ζ=0.7) ensures accuracy and stability.

---

## 🧠 SECTION C — DIGITAL SIGNAL PROCESSING (DSP) SYSTEMS

*(Discrete-Time Equivalents; Sampling at 10 kHz)*

---

### 🔉 1. FIR Low-Pass Filter

| Parameter          | Symbol | Value |
| ------------------ | :----: | :---: |
| Cutoff Frequency   |  1 kHz |       |
| Sampling Frequency | 10 kHz |       |
| Filter Order       |    4   |       |

**Transfer Function**

```
H(z) = 0.2*(1 + z^-1 + z^-2 + z^-3 + z^-4)
```

**MATLAB Code**

```matlab
b = 0.2*ones(1,5);
a = 1;
H_fir = tf(b, a, 1/10000)  % Ts = 0.0001 s
freqz(b, a)
```

**Observation:**
Linear phase; unity DC gain; sharp roll-off at ~1 kHz.

---

### 🌀 2. IIR Low-Pass Filter (Butterworth, 2nd-Order)

**Transfer Function**

```
H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
```

**MATLAB Code**

```matlab
[b,a] = butter(2, 0.2);  % 0.2 normalized (1 kHz at 10 kHz Fs)
H_iir = tf(b, a, 1/10000)
freqz(b, a)
```

**Observation:**
Maximally flat in passband; stable poles inside unit circle.

---

### ⏱️ 3. Digital PID Controller

**Transfer Function**

```
H(z) = Kp + Ki*(Ts/(z - 1)) + Kd*((z - 1)/(Ts*z))
```

**MATLAB Code**

```matlab
Kp = 1; Ki = 50; Kd = 0.01; Ts = 0.001;
s = tf('s');
C_pid = Kp + Ki/s + Kd*s;
Cz = c2d(C_pid, Ts, 'tustin')
```

**Observation:**
Proper tuning yields stable, fast digital control loop.

---

### 📡 4. Adaptive Noise Canceller (Simplified LMS Loop)

**Equations**

```
y[n] = x[n] - w[n]*r[n]
w[n+1] = w[n] + 2*mu*r[n]*y[n]
```

**MATLAB Code (Conceptual)**

```matlab
mu = 0.01; w = 0;
for n = 1:length(x)
    y(n) = x(n) - w*r(n);
    w = w + 2*mu*r(n)*y(n);
end
```

**Observation:**
Reduces noise adaptively; convergence speed proportional to mu.

---

### 📶 5. Digital PLL (Phase-Locked Loop)

**Continuous and Discrete Models**

```
H(s) = (Kp*Kv) / [s*(T*s + 1)]
H(z) = (Kp*Kv*(1 - z^-1)) / [T*(1 + z^-1)]
```

**MATLAB Code**

```matlab
Kp = 1; Kv = 100; T = 0.001;
num = [Kp*Kv*(1 - 1)];
den = [T, T];
H_pll = tf(num, den, T)
```

**Observation:**
Tracks phase/frequency variations; stabilizes clock recovery.

---

## 🧾 GRAND COMPARATIVE SUMMARY

| #  | System             | Domain            |   Order  |  τ / f_c | Key Nature        | Response Type         |
| -- | :----------------- | :---------------- | :------: | :------: | :---------------- | :-------------------- |
| 1  | Buck Converter     | Power Electronics |     2    |   ~1 ms  | Underdamped       | Smooth step-down      |
| 2  | Boost Converter    | Power Electronics |     2    |  ~1–2 ms | Non-minimum phase | Resonant              |
| 3  | Buck–Boost         | Power Electronics |     2    |   ~1 ms  | Inverting         | Resonant peak         |
| 4  | Inverter + Filter  | Power Electronics |     2    |  ~10 ms  | Damped            | Sinusoidal shaping    |
| 5  | Rectifier + Filter | Power Electronics |     2    |   50 ms  | Overdamped        | Ripple smoothing      |
| 6  | Audio Amplifier    | Consumer          |     1    | fp=2 kHz | Low-pass          | Fast gain             |
| 7  | LED Driver         | Consumer          |     1    |  0.02 s  | First-order       | Flicker-free          |
| 8  | Camera Focus       | Consumer          |     1    |   0.1 s  | Servo-type        | Fast steady focus     |
| 9  | RF Amplifier       | Consumer          |     1    | fc=1 MHz | Wideband          | Linear gain           |
| 10 | MEMS Accelerometer | Consumer          |     2    |  wn=1000 | Resonant          | Damped sensor         |
| 11 | FIR LPF            | DSP               |     4    | fc=1 kHz | Linear phase      | Flat passband         |
| 12 | IIR LPF            | DSP               |     2    | fc=1 kHz | Butterworth       | Smooth cutoff         |
| 13 | Digital PID        | DSP               |     3    |  Ts=1 ms | Control loop      | Stable servo          |
| 14 | LMS ANC            | DSP               | Adaptive |     —    | Time-varying      | Noise suppression     |
| 15 | Digital PLL        | DSP               |     2    |  fc≈1/T  | Tracking          | Phase synchronization |

---

## 🧠 Cross-Domain Observations

* **Power Electronics:** Typically second-order (L–C energy storage) and fast (ms range).
* **Consumer Electronics:** Often first- or second-order low-pass systems; moderate speed with emphasis on stability and smooth response.
* **DSP Systems:** Use discrete-time (z-domain) transfer functions; digitally controlled and stable under proper sampling.

---
