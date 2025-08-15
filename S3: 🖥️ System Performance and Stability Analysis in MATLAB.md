---

**Lecture 3: 🖥️ System Performance and Stability Analysis in MATLAB**

**1️⃣ Introduction to Performance and Stability Analysis**

* System performance involves evaluating how fast, accurate, and stable a system responds to inputs.
* Stability ensures the system output remains bounded and predictable under disturbances.
* MATLAB provides tools for time-domain and frequency-domain analysis.

**2️⃣ Time-Domain Performance Metrics**

* **Rise Time (Tr):** Time to reach 90% of final value.
* **Settling Time (Ts):** Time to remain within a certain error band.
* **Peak Overshoot (Mp):** Maximum deviation from final value.
* **Steady-State Error (Ess):** Difference between desired and actual final value.

**3️⃣ MATLAB Example: Step Response Analysis**

```matlab
s = tf('s');
G = 5/(s^2 + 2*s + 5);
step(G);
y = stepinfo(G);
disp(y); % Displays rise time, settling time, overshoot, etc.
```

* `stepinfo()` provides a quick overview of time-domain performance.

**4️⃣ Root Locus Analysis**

* Shows how system poles move with varying gain.
* Helps design for desired stability and transient response.

```matlab
rlocus(G);
title('Root Locus of the System');
```

* Adjust system gain and observe pole locations to ensure stability.

**5️⃣ Bode Plot Analysis**

* Evaluates frequency response: gain margin, phase margin, bandwidth.

```matlab
bode(G);
grid on;
[GM, PM, Wcg, Wcp] = margin(G);
disp(['Gain Margin: ', num2str(GM), ', Phase Margin: ', num2str(PM)]);
```

* Gain Margin (GM) and Phase Margin (PM) indicate how close the system is to instability.

**6️⃣ Nyquist Plot Analysis**

* Determines stability by evaluating encirclements of the critical point (-1,0) in the complex plane.

```matlab
nyquist(G);
title('Nyquist Plot of the System');
```

* Useful for MIMO systems and systems with delays.

**7️⃣ Pole-Zero Map Analysis**

* Visualizes pole and zero locations to infer stability and transient behavior.

```matlab
pzmap(G);
title('Pole-Zero Map');
```

* Poles in the left-half plane indicate stable system.

**8️⃣ MATLAB Commands Summary**

| Tool          | MATLAB Command | Purpose                                  |
| ------------- | -------------- | ---------------------------------------- |
| Step Response | `step(G)`      | Time-domain behavior                     |
| Step Info     | `stepinfo(G)`  | Rise time, settling time, overshoot      |
| Root Locus    | `rlocus(G)`    | Pole movement with gain                  |
| Bode Plot     | `bode(G)`      | Frequency response, GM, PM               |
| Nyquist Plot  | `nyquist(G)`   | Stability assessment in frequency domain |
| Pole-Zero Map | `pzmap(G)`     | Stability and transient behavior         |

**9️⃣ Next Steps:**

* Experiment with different system parameters to study performance metrics.
* Combine stability analysis tools to design robust controllers.
