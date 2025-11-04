# 🎓 Lecture: Defining Transfer Functions in MATLAB

### ✴️ Topic Focus: Mathematical Modeling of Dynamic Systems

---

## 🔍 1. Concept of Transfer Function

A **transfer function** expresses the **input–output relationship** of a **linear time-invariant (LTI)** system in the **Laplace domain**.

It is defined as:

**G(s) = Y(s) / U(s) = (b₀sⁿ + b₁sⁿ⁻¹ + ... + bₙ) / (a₀sᵐ + a₁sᵐ⁻¹ + ... + aₘ)**

where

* `Y(s)` = Laplace transform of output
* `U(s)` = Laplace transform of input
* `b` and `a` are coefficients of numerator and denominator polynomials respectively

📘 **Physical Significance:**
The transfer function describes how a system **responds to an excitation**—for example, how a DC motor’s speed changes when voltage is applied, or how a building structure vibrates when an external force acts on it.

---

## ⚙️ 2. Defining Transfer Functions in MATLAB

There are several ways to define a transfer function in MATLAB depending on the available information about the system.

---

### 🧮 2.1 Using Numerator and Denominator Coefficients — `tf()`

**Syntax:**

```matlab
num = [numerator coefficients];
den = [denominator coefficients];
G = tf(num, den);
```

**Example:**

```matlab
num = [1];
den = [1 3 2];
G = tf(num, den)
```

**Output:**

```
G(s) = 1 / (s^2 + 3s + 2)
```

📘 **Real-world Example:**
This model can represent a **mass-spring-damper system**, where:

* Mass (m) = 1 kg
* Damping coefficient (c) = 3 Ns/m
* Spring constant (k) = 2 N/m

Hence, the transfer function shows how the **mass displacement responds** to an applied force.

---

### ⚡ 2.2 Using Zero-Pole-Gain Form — `zpk()`

**Syntax:**

```matlab
G = zpk(z, p, k);
```

**Example:**

```matlab
z = [-2];            % zero at s = -2
p = [-1 -3];         % poles at s = -1 and s = -3
k = 4;               % system gain
G = zpk(z, p, k)
```

**Output:**

```
G(s) = 4*(s + 2) / ((s + 1)*(s + 3))
```

📘 **Real-world Example:**
An **electronic amplifier** where the zeros and poles define the system’s frequency behavior (gain and phase shift).

---

### 🧠 2.3 Using Symbolic Laplace Variable — `s = tf('s')`

This is a very **convenient way** to define and manipulate transfer functions algebraically.

**Example:**

```matlab
s = tf('s');
G = (s + 2) / (s^2 + 3*s + 2)
```

**Output:**

```
G(s) = (s + 2) / (s^2 + 3s + 2)
```

You can perform operations directly:

```matlab
H = G / (1 + G);     % closed-loop feedback
```

📘 **Real-world Example:**
Used in **control system design**—for example, when analyzing closed-loop transfer functions in a **PID-controlled robotic arm**.

---

## 🔁 3. Other Ways to Define Transfer Functions

---

### 🔸 3.1 From State-Space Representation — `tf(ss(A,B,C,D))`

If a system is described by **state-space equations**:

ẋ = A·x + B·u
y = C·x + D·u

You can convert it to TF form using:

```matlab
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

sys_ss = ss(A, B, C, D);
sys_tf = tf(sys_ss)
```

**Output:**

```
G(s) = 1 / (s^2 + 3s + 2)
```

📘 **Physical Example:**
Model of a **mechanical suspension system** described by state-space form (position and velocity as states).

---

### 🔸 3.2 From Differential Equation Coefficients

Given:
**d²y/dt² + 3 dy/dt + 2y = u(t)**

Laplace transform gives:
**G(s) = Y(s)/U(s) = 1 / (s² + 3s + 2)**

MATLAB representation:

```matlab
num = [1];
den = [1 3 2];
G = tf(num, den);
```

📘 **Physical Example:**
A **spring-mass-damper** or **RC circuit** can often be expressed in this differential form.

---

### 🔸 3.3 From Experimental Frequency Response — `tfest()`

When experimental frequency response data is available (measured gain and phase), MATLAB can identify a suitable TF model.

```matlab
freq = logspace(1,3,50);
mag = abs(1 ./ (1j*freq + 10));
phase = angle(1 ./ (1j*freq + 10));
data = idfrd(mag.*exp(1j*phase), freq, 0);

sys_tf = tfest(data, 1, 1);
```

📘 **Real-world Example:**
When modeling a **hydraulic actuator** or **motor drive** from measured step or frequency response data.

---

### 🔸 3.4 From Symbolic Equation (Symbolic Toolbox)

You can manipulate symbolic expressions and convert them to a TF.

```matlab
syms s
G_sym = (s + 2) / (s^2 + 3*s + 2);
[num, den] = numden(G_sym);
G = tf(sym2poly(num), sym2poly(den))
```

📘 **Use Case:**
Useful when deriving system equations symbolically in terms of parameters before numerical substitution.

---

### 🔸 3.5 For Discrete-Time (z-Domain) Systems

To define discrete-time transfer functions:

```matlab
num = [0.1 0.1];
den = [1 -0.9];
Ts = 0.1;
Gz = tf(num, den, Ts)
```

**Output:**

```
G(z) = (0.1z + 0.1) / (z - 0.9)
Sample time: 0.1 seconds
```

📘 **Physical Example:**
A **digital controller** running on a microcontroller for a temperature or motor control loop.

---

### 🔸 3.6 Multi-Input Multi-Output (MIMO) Transfer Function Matrix

For systems with multiple inputs and outputs:

```matlab
G11 = tf([1], [1 1]);
G12 = tf([2], [1 3]);
G21 = tf([3], [1 2]);
G22 = tf([4], [1 4]);
G = [G11 G12; G21 G22];
```

📘 **Example:**
Used in **chemical process plants** or **aircraft dynamics**, where multiple control loops interact.

---

## ⚡ 4. Real-Life Engineering Example: DC Motor Modeling

Consider a **DC motor** with the following parameters:

| Parameter           | Symbol | Value      |
| ------------------- | ------ | ---------- |
| Armature resistance | R      | 2 Ω        |
| Armature inductance | L      | 0.5 H      |
| Motor constant      | K      | 0.1 Nm/A   |
| Moment of inertia   | J      | 0.02 kg·m² |
| Damping coefficient | B      | 0.02 N·m·s |

Transfer function (speed/voltage) is:

**G(s) = K / [(J·L)s² + (J·R + B·L)s + (B·R + K²)]**

**MATLAB Implementation:**

```matlab
R = 2; L = 0.5; K = 0.1; J = 0.02; B = 0.02;

num = [K];
den = [J*L, (J*R + B*L), (B*R + K^2)];
G_motor = tf(num, den)
```

**Result:**

```
G(s) = 0.1 / (0.01s^2 + 0.06s + 0.0402)
```

You can visualize its response:

```matlab
step(G_motor)
```

📊 This step response shows **how quickly the motor reaches its steady-state speed** after a voltage step input.

---

## 🧩 5. Summary Table

| Method             | MATLAB Function       | Type          | Use Case Example              |
| ------------------ | --------------------- | ------------- | ----------------------------- |
| Coefficients       | `tf(num, den)`        | Continuous    | Spring-mass system            |
| Zero-Pole-Gain     | `zpk(z,p,k)`          | Continuous    | Amplifier model               |
| Symbolic `s`       | `tf('s')`             | Continuous    | Feedback control analysis     |
| State-space → TF   | `tf(ss(A,B,C,D))`     | Continuous    | Suspension or robotics        |
| Frequency response | `tfest()`             | Experimental  | Measured system               |
| Symbolic Equation  | `sym2poly()` + `tf()` | Symbolic      | Analytical modeling           |
| Discrete-time      | `tf(num, den, Ts)`    | Digital       | Microcontroller-based control |
| MIMO System        | Matrix of TFs         | Multivariable | Process control systems       |

---

## ❓ Objective Viva Questions

1. Which MATLAB function defines a transfer function from numerator and denominator coefficients?
   a) `zpk()` b) `tf()` c) `tfest()` d) `ss()`

2. What does the variable `s = tf('s')` represent?
   a) Sampling time
   b) Time-domain signal
   c) Laplace variable
   d) System gain

3. Which command converts a state-space model into a transfer function?
   a) `tfest()` b) `tf(ss(A,B,C,D))` c) `zpk()` d) `c2d()`

4. In a DC motor TF, what does the denominator determine?
   a) Input gain
   b) System poles
   c) Motor torque constant
   d) Output noise

---

## ✅ Solutions to Viva Questions

1. **b) tf()** — defines TF using polynomial coefficients
2. **c) Laplace variable** — used for symbolic transfer functions
3. **b) tf(ss(A,B,C,D))** — converts state-space to TF form
4. **b) System poles** — denominator defines pole locations determining stability and response speed

---

## 💬 Follow-up Practice for Students

1. Design and simulate an **RLC circuit** using `tf()` and plot the Bode response.
2. Using `tf('s')`, form a **closed-loop system** for G(s) = 10 / (s(s+2)) and observe stability using `step()` and `pzmap()`.
3. Identify and fit an experimental TF using `tfest()` from measured data of a DC motor lab experiment.

---
