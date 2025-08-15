---

**Lecture 2: 🖥️ Second-Order and Multi-Order System Simulation in MATLAB**

**1️⃣ Introduction to Higher-Order Systems**

* Second-order systems have dynamics governed by two energy storage elements (e.g., mass-spring-damper, RLC circuits).
* Multi-order systems involve three or more energy storage elements.
* MATLAB allows easy simulation to observe how system parameters affect behavior.

**2️⃣ Standard Second-Order Transfer Function**
$G(s) = \frac{K\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$

* **K:** System gain
* **ω\_n:** Natural frequency
* **ζ:** Damping ratio

**3️⃣ MATLAB Simulation Example: Mass-Spring-Damper System**

```matlab
s = tf('s');
K = 1; wn = 2; zeta = 0.5;
G = K*wn^2/(s^2 + 2*zeta*wn*s + wn^2);
step(G);
title('Step Response of Mass-Spring-Damper System');
xlabel('Time (s)'); ylabel('Displacement');
```

* Observe overshoot, settling time, and steady-state displacement.

**4️⃣ MATLAB Simulation Example: RLC Circuit**

```matlab
s = tf('s');
R = 1; L = 1; C = 0.25;
G = 1/(L*C*s^2 + R*C*s + 1);
step(G);
title('Step Response of RLC Circuit');
xlabel('Time (s)'); ylabel('Current or Voltage');
```

* Examine how resistance and capacitance affect system response.

**5️⃣ Effects of Parameters on System Response**

* **Damping Ratio (ζ):** Affects overshoot and oscillations.
* **Natural Frequency (ω\_n):** Determines speed of response.
* **Gain (K):** Changes steady-state amplitude.
* Use MATLAB to vary these parameters and plot multiple responses for comparison.

```matlab
hold on;
zeta_values = [0.2 0.5 0.7 1];
for zeta = zeta_values
    G = wn^2/(s^2 + 2*zeta*wn*s + wn^2);
    step(G);
end
legend('ζ=0.2','ζ=0.5','ζ=0.7','ζ=1');
```

**6️⃣ Multi-Order System Example: Higher-Order Mechanical System**

```matlab
s = tf('s');
G = 1/((s+1)*(s+2)*(s+3));
step(G);
title('Step Response of Third-Order Mechanical System');
xlabel('Time (s)'); ylabel('Output');
```

* Observe slower response and multiple inflection points due to higher-order poles.

**7️⃣ Next Steps:**

* Encourage experimentation with varying damping ratio, natural frequency, and system order.
* Prepare for state-space modeling and simulation in Lecture 3.
