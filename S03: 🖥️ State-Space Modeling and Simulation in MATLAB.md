---

**Lecture 3: 🖥️ State-Space Modeling and Simulation in MATLAB**

**1️⃣ Introduction to State-Space Representation**

* State-space models describe systems using first-order differential equations.
* Suitable for multi-input, multi-output (MIMO) systems.
* MATLAB can simulate and analyze state-space systems efficiently.

**2️⃣ General Form of State-Space Model**
$\dot{x}(t) = A x(t) + B u(t)$
$y(t) = C x(t) + D u(t)$

* **x(t):** State vector
* **u(t):** Input vector
* **y(t):** Output vector
* **A, B, C, D:** System matrices

**3️⃣ Converting Transfer Function to State-Space in MATLAB**

```matlab
s = tf('s');
G = 5/(s^2 + 2*s + 5);
[A,B,C,D] = tf2ss([5],[1 2 5]);
```

* Observe A, B, C, D matrices representing the system dynamics.

**4️⃣ Direct State-Space Definition in MATLAB**

```matlab
A = [0 1; -5 -2];
B = [0; 5];
C = [1 0];
D = 0;
sys = ss(A,B,C,D);
step(sys);
title('Step Response using State-Space');
xlabel('Time (s)'); ylabel('Output');
```

* Useful for MIMO systems and advanced control design.

**5️⃣ Simulating Response to Different Inputs**

```matlab
t = 0:0.01:10;
u = sin(t); % sinusoidal input
lsim(sys,u,t);
title('System Response to Sinusoidal Input');
xlabel('Time (s)'); ylabel('Output');
```

* Observe how system reacts to time-varying input.

**6️⃣ Analysis of State-Space Models**

* Use `eig(A)` to determine system stability (eigenvalues).
* Use `initial(sys,x0)` to simulate response from non-zero initial states.
* Compare with transfer function response using `step()` or `impulse()`.

**7️⃣ Next Steps:**

* Experiment with different system matrices and initial conditions.
* Prepare for Lecture 4: PID Controller Implementation and Tuning in MATLAB.
