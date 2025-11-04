# 🧩 **Feedback, Series, Parallel, Append, and Connect — Realizing Block Diagrams in MATLAB**

This tutorial teaches how to construct and analyze **control system block diagrams** using MATLAB’s built-in functions like `series()`, `parallel()`, `feedback()`, `append()`, and `connect()`. Each function will be explained with its **physical meaning**, **syntax**, **block interpretation**, and **practical MATLAB examples**.

---

## 🎯 **Objective**

To learn how complex control systems are modeled and simulated in MATLAB by combining simpler transfer functions using **feedback**, **series**, and **parallel** connections — the fundamental operations for realizing block diagrams.

---

## ⚙️ **Key MATLAB Functions Overview**

| Function               | Purpose                                                    | Symbolic Block Representation |
| ---------------------- | ---------------------------------------------------------- | ----------------------------- |
| `series(sys1, sys2)`   | Connects two systems in series (cascade)                   | →                             |
| `parallel(sys1, sys2)` | Connects two systems in parallel                           | → + ←                         |
| `feedback(sys1, sys2)` | Forms a feedback loop between two systems                  | ↺                             |
| `append(sys1, sys2)`   | Combines multiple systems into a block-diagram-ready model | Multiple inputs/outputs       |
| `connect(sys, ...)`    | Connects systems using port interconnections               | General MIMO configuration    |

---

## 🧠 **Background Concept**

A **block diagram** represents a control system as an interconnection of functional blocks, each described by its transfer function.

If the system components are represented as:

* **G(s)** = Forward path transfer function
* **H(s)** = Feedback path transfer function

Then:

**Closed-loop Transfer Function:**
**T(s) = G(s) / (1 + G(s)H(s))**

---

## 🧩 **1️⃣ Series Connection**

### 🔹 Concept:

In a **series** connection, the output of one block becomes the input of another.
Equivalent transfer function:

**T(s) = G₁(s) × G₂(s)**

### 🔹 MATLAB Syntax:

```matlab
sys_series = series(sys1, sys2);
```

### 🔹 Example:

```matlab
G1 = tf([1], [1 2]);       % G1(s) = 1 / (s + 2)
G2 = tf([5], [1 3]);       % G2(s) = 5 / (s + 3)

sys_series = series(G1, G2);
disp('Series Connection Result:');
sys_series
```

🧩 **Interpretation:**
The resulting transfer function represents two cascaded stages (e.g., amplifier + actuator in control).

---

## 🧩 **2️⃣ Parallel Connection**

### 🔹 Concept:

In a **parallel** connection, both systems share the same input, and their outputs are added (or subtracted).

**T(s) = G₁(s) + G₂(s)**

### 🔹 MATLAB Syntax:

```matlab
sys_parallel = parallel(sys1, sys2);
```

### 🔹 Example:

```matlab
G1 = tf([1], [1 2]);
G2 = tf([3], [1 5]);

sys_parallel = parallel(G1, G2);
disp('Parallel Connection Result:');
sys_parallel
```

🧩 **Interpretation:**
Common in redundant control systems or when combining multiple feedback paths.
E.g., two sensors feeding a controller simultaneously.

---

## 🧩 **3️⃣ Feedback Connection**

### 🔹 Concept:

A **feedback** system automatically adjusts its behavior by feeding part of its output back to the input.

For **unity feedback**:
**T(s) = G(s) / (1 + G(s))**

For **non-unity feedback**:
**T(s) = G(s) / (1 + G(s)H(s))**

### 🔹 MATLAB Syntax:

```matlab
sys_feedback = feedback(G, H, +1);   % Positive feedback
sys_feedback = feedback(G, H, -1);   % Negative feedback (default)
```

### 🔹 Example:

```matlab
G = tf([10], [1 2 10]);
H = tf([1], [1 5]);

sys_feedback = feedback(G, H); % Default is negative feedback
disp('Closed-loop System:');
sys_feedback

figure; step(sys_feedback);
title('Step Response of Closed-loop System');
grid on;
```

🧩 **Interpretation:**
Feedback improves **stability**, **reduces steady-state error**, and **rejects disturbances**—the essence of modern control.

---

## 🧩 **4️⃣ Append — Preparing Multi-Input Multi-Output (MIMO) Systems**

### 🔹 Concept:

`append()` stacks multiple systems together **without connecting them**, preparing for complex interconnections using `connect()` later.

### 🔹 MATLAB Syntax:

```matlab
sys_appended = append(sys1, sys2, sys3, ...);
```

### 🔹 Example:

```matlab
G1 = tf([1], [1 2]);
G2 = tf([2], [1 3]);
G3 = tf([3], [1 4]);

sys_all = append(G1, G2, G3);
size(sys_all)
```

🧩 **Interpretation:**
Used as a preparatory step for custom interconnections — similar to defining all blocks in a Simulink model before wiring them.

---

## 🧩 **5️⃣ Connect — Building Complex Block Diagrams**

### 🔹 Concept:

`connect()` defines how blocks are **wired together** through **input-output mapping**.
You first `append` all blocks, then specify **input/output relationships**.

---

### 🔹 Example: Realizing Closed-Loop System using `append()` and `connect()`

**System Structure:**

```
     +----G1(s)----+
     |              |
 r →(+)            Σ→ y
     |←---H(s)------|
```

### 🔹 MATLAB Code:

```matlab
% Define components
G1 = tf([10], [1 2 10]);   % forward path
H  = tf([1], [1 5]);       % feedback path

% Append systems
sys = append(G1, H);

% Define interconnections:
% sys = append(G1, H)
% Inputs:  1=r (reference)
% Outputs: 1=output of G1, 2=output of H
% Interconnections matrix defines how blocks link:
Q = [1 -2];  % input to summing junction: r - feedback
inputs = 1;  % reference input index
outputs = 1; % output (from G1)

% Connect the systems
CL_sys = connect(sys, Q, inputs, outputs);

disp('Closed-loop System using connect():');
CL_sys

figure; step(CL_sys);
title('Closed-loop Response using connect()');
grid on;
```

🧩 **Interpretation:**
Here, MATLAB automatically creates the **closed-loop transfer function** from the defined interconnection, similar to manual wiring in **Simulink**.

---

## ⚖️ **Comparison of Connection Types**

| Connection | Function     | Mathematical Operation     | Common Use Case                  |
| ---------- | ------------ | -------------------------- | -------------------------------- |
| Series     | `series()`   | Multiplication             | Cascaded blocks                  |
| Parallel   | `parallel()` | Addition/Subtraction       | Redundant or combined paths      |
| Feedback   | `feedback()` | Division by (1 ± GH)       | Stability and error reduction    |
| Append     | `append()`   | Combine unconnected blocks | Multi-block prep for `connect()` |
| Connect    | `connect()`  | Custom interconnection     | Complex multi-loop systems       |

---

## 🌍 **Practical Example: DC Motor Control**

| Element         | Function               | MATLAB Representation |
| --------------- | ---------------------- | --------------------- |
| Controller      | `Gc(s)` = K(s+z)/(s+p) | Compensator           |
| Plant           | `Gp(s)` = 1/(Js + b)   | DC Motor Model        |
| Feedback Sensor | `H(s)` = 1             | Position Feedback     |

```matlab
Gc = tf([2 5], [1 10]);    % Controller
Gp = tf([1], [0.5 1]);     % DC Motor
H  = tf([1], [1]);         % Unity feedback

sys_series = series(Gc, Gp);
sys_closed = feedback(sys_series, H);

figure; step(sys_closed);
title('Step Response: DC Motor with Feedback Controller');
grid on;
```

🧩 **Physical Significance:**

* The controller improves **speed** and **damping**.
* Feedback minimizes error and enhances stability.

---

## 🧠 **Home Task**

1. Design a closed-loop system using `series`, `parallel`, and `feedback` for the following:

   * G1(s) = 5 / (s + 2)
   * G2(s) = 3 / (s + 1)
   * H(s) = 1 / (s + 4)

2. Using `append()` and `connect()`, realize the same system and compare responses using `step()`.

3. Add an extra zero to the controller and observe changes in **rise time** and **overshoot**.

---

## ❓ **Objective Viva Questions**

1. (a) What is the difference between `series()` and `parallel()` in MATLAB?
   (b) Why is `append()` used before `connect()`?
2. (a) Write the closed-loop formula for a feedback system.
   (b) How can you create positive feedback in MATLAB?
3. (a) Which function is most useful for multi-loop control systems?
   (b) What happens if a pole lies on the right half-plane?

---

## ✅ **Solutions to Viva Questions**

1. (a) `series()` multiplies transfer functions; `parallel()` adds them.
   (b) `append()` prepares all unconnected subsystems for interconnection using `connect()`.
2. (a) **T(s) = G(s) / (1 + G(s)H(s))** for negative feedback.
   (b) Use `feedback(G, H, +1)`.
3. (a) `connect()` allows flexible interconnections for MIMO systems.
   (b) System becomes **unstable**.

---
