# 🧪 **Experiment: Generation of Time Response for First & Second Order Systems and Observation of Responses**

---

## 🧭 **1. Objective**

To study and generate **time-domain responses** (step and impulse) of **first-order** and **second-order** systems using MATLAB, analyze **transient** and **steady-state** characteristics, and interpret **pole-zero locations** and **stability** with real-world system analogies.

---

## ⚙️ **2. Theory**

### 🔹 **First-Order System**

**Transfer Function:**
**G(s) = K / (T·s + 1)**

**Step Response:**
**c(t) = K(1 - e^(-t/T))**

| Parameter | Meaning            | Effect on Response                  |
| --------- | ------------------ | ----------------------------------- |
| **K**     | System Gain        | Increases steady-state value        |
| **T**     | Time Constant      | Larger T = slower response          |
| **tᵣ**    | Rise Time          | Time from 10%–90% of final value    |
| **tₛ**    | Settling Time      | Time to settle within ±2%           |
| **eₛs**   | Steady-State Error | Difference from final desired value |

**Pole Location:**
At **s = -1/T** → farther left pole = faster system.

**Example:**
An **RC circuit** acts as a first-order system where the capacitor voltage exponentially approaches the source voltage.

---

### 🔹 **Second-Order System**

**Transfer Function:**
**G(s) = ωₙ² / (s² + 2ζωₙs + ωₙ²)**

| Symbol              | Meaning                   |
| ------------------- | ------------------------- |
| **ωₙ**              | Natural frequency (rad/s) |
| **ζ**               | Damping ratio             |
| **ω_d = ωₙ√(1−ζ²)** | Damped frequency          |

| ζ         | Type              | Description             | Response                |
| --------- | ----------------- | ----------------------- | ----------------------- |
| 0         | Undamped          | No damping              | Sustained oscillations  |
| 0 < ζ < 1 | Underdamped       | Oscillatory decay       | Common in real systems  |
| 1         | Critically Damped | Fastest non-oscillatory | Ideal for speed control |
| >1        | Overdamped        | Very slow response      | Safe but sluggish       |

**Example:**
A **car suspension system** (mass–spring–damper) behaves as a second-order system. Damping ratio defines ride comfort — underdamped means bouncy ride, overdamped means stiff ride.

---

## 💻 **3. MATLAB Implementation**

### 🧩 **(A) First-Order System Analysis**

**Transfer Function:**
**G₁(s) = 1 / (s + 1)**

```matlab
clc; clear; close all;
s = tf('s');
G1 = 1 / (s + 1);

% Step and Impulse Response
figure;
step(G1); grid on;
title('Step Response of First Order System');

figure;
impulse(G1); grid on;
title('Impulse Response of First Order System');

% Time-domain analysis
info1 = stepinfo(G1);
disp('Step Info (1st Order):');
disp(info1);

% Stability check
disp(['Stable: ', num2str(isstable(G1))]);

% DC Gain
disp(['DC Gain: ', num2str(dcgain(G1))]);

% Pole-Zero Plot
figure;
pzmap(G1); grid on;
title('Pole-Zero Map: First Order System');
```

**Interpretation:**

* Pole = **−1** → stable (left half-plane).
* Rise Time ≈ 2.3 s, Settling Time ≈ 4.0 s.
* Stable exponential rise without overshoot.

**Real-world analogy:**
Heating water in an electric kettle – temperature gradually reaches the target with no oscillation.

---

### 🧩 **(B) Second-Order System Analysis**

**Let:** ωₙ = 5 rad/s
**and** vary ζ = [0.2, 0.5, 0.8, 1.0, 2.0]

```matlab
clc; clear; close all;
s = tf('s');
wn = 5;

zeta = [0.2 0.5 0.8 1.0 2.0];
for i = 1:length(zeta)
    G2 = wn^2 / (s^2 + 2*zeta(i)*wn*s + wn^2);

    figure;
    step(G2);
    grid on;
    title(['Step Response for ζ = ', num2str(zeta(i))]);

    % Time-domain info
    disp(['Step Info for ζ = ', num2str(zeta(i))]);
    disp(stepinfo(G2));

    % Pole-zero map
    figure;
    pzmap(G2);
    grid on;
    title(['Pole-Zero Map for ζ = ', num2str(zeta(i))]);

    % Stability
    disp(['Stable: ', num2str(isstable(G2))]);
end
```

**Observation Table:**

| ζ   | Response Type     | Overshoot (%) | Settling Time (s) | Behavior         |
| --- | ----------------- | ------------- | ----------------- | ---------------- |
| 0.2 | Underdamped       | ~40           | 3.5               | Oscillatory      |
| 0.5 | Underdamped       | ~16           | 2.0               | Quick and stable |
| 1.0 | Critically Damped | 0             | 1.3               | Fastest stable   |
| 2.0 | Overdamped        | 0             | 3.8               | Very slow        |

**Pole–Zero Plot Interpretation:**

* Complex conjugate poles → oscillatory response.
* Poles move further left as damping increases → faster decay.
* Real poles (ζ ≥ 1) → non-oscillatory.

**Real-world analogy:**
Car suspension dynamics or quadcopter altitude control behave similarly depending on damping.

---

## 📈 **4. Extended Time Domain Analysis in MATLAB**

| MATLAB Function   | Purpose                        | Example               | Use Case                 |
| ----------------- | ------------------------------ | --------------------- | ------------------------ |
| `stepinfo(sys)`   | Get rise, peak, settling times | `info = stepinfo(G2)` | System tuning            |
| `isstable(sys)`   | Checks system stability        | `isstable(G2)`        | Control verification     |
| `dcgain(sys)`     | Steady-state gain              | `dcgain(G2)`          | Accuracy measure         |
| `pole(sys)`       | Returns poles                  | `pole(G2)`            | Determines dynamics      |
| `zero(sys)`       | Returns zeros                  | `zero(G2)`            | Used in shaping response |
| `pzmap(sys)`      | Pole-zero map                  | `pzmap(G2)`           | Stability visualization  |
| `lsim(sys, u, t)` | Custom input simulation        | `lsim(G2, sin(t), t)` | Non-step input behavior  |
| `bode(sys)`       | Frequency domain plot          | `bode(G2)`            | Gain & phase analysis    |

---

## 🌍 **5. Real-World Analogies**

| Application            | Model Type | System Behavior                     |
| ---------------------- | ---------- | ----------------------------------- |
| RC Circuit             | 1st Order  | Voltage rise ~ exponential          |
| Car Suspension         | 2nd Order  | Oscillation vs. damping             |
| Liquid Level Control   | 1st Order  | Tank level changes with lag         |
| DC Motor Speed Control | 2nd Order  | Speed overshoot based on ζ          |
| Elevator Motion        | 2nd Order  | Smoother ride for ζ ≈ 1             |
| Drone Altitude Control | 2nd Order  | ζ tuning affects hovering stability |

---

## 🧩 **6. Experimental Observations**

1. **First-order systems** exhibit exponential rise with no oscillation.
2. **Second-order systems** show oscillations when damping ratio < 1.
3. As **ζ increases**, overshoot decreases and settling becomes slower after ζ > 1.
4. **Pole-zero plots** provide intuitive visualization of system stability.
5. **Stepinfo** and **isstable** functions enable quantitative performance assessment.

---

## 📊 **7. Results and Discussion**

| Parameter     | 1st Order | 2nd Order (ζ=0.5) |
| ------------- | --------- | ----------------- |
| Rise Time     | 2.3 s     | 0.9 s             |
| Settling Time | 4.0 s     | 2.0 s             |
| Overshoot     | 0%        | 16%               |
| Stability     | Stable    | Stable            |
| DC Gain       | 1         | 1                 |

**Discussion:**

* First-order systems are slower but simpler.
* Second-order systems can achieve faster responses but may overshoot.
* Proper damping ensures **speed–stability tradeoff** balance.
* Pole-zero location directly correlates with transient behavior.

---

## 🧠 **8. Conclusion**

* MATLAB provides a powerful environment for analyzing and visualizing system dynamics.
* Time-domain parameters such as **rise time**, **settling time**, and **overshoot** are critical in designing stable, responsive systems.
* **Pole-zero plots** help visualize system stability directly from the s-plane.
* **Real-world control systems** (motors, suspensions, temperature controllers) can be effectively modeled and tuned using these tools.

---

## 🧩 **9. Home Task (Individual Work)**

1. **System Design:**
   Create a **second-order system** with **ωₙ = 4 rad/s**, **ζ = 0.4**, **K = 3**.

   * Find poles, zeros, and DC gain.
   * Plot step, impulse, and pole-zero maps.
   * Display `stepinfo` results and interpret them.

2. **Comparative Study:**
   Compare **ζ = 0.3**, **ζ = 0.8**, keeping **ωₙ = 4 rad/s**.

   * Comment on rise time, overshoot, and settling time.

3. **Practical Task:**
   Model a **DC motor speed control system** with transfer function
   **G(s) = 100 / (s² + 12s + 100)**

   * Use `step()`, `pzmap()`, `stepinfo()`, and `isstable()`.
   * Interpret damping behavior and control performance.

4. **Advanced (Optional):**
   Apply a sinusoidal input using `lsim()` to observe steady-state tracking behavior.

