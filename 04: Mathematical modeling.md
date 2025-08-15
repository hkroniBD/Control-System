# **Lecture 4 – Mathematical Modeling of Control Systems**
- 📕Course: Control System Engineering
- 🤖Instructor: Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU,Dinajpur,BD

📌 *How real-world systems are expressed in mathematical form for analysis, simulation, and control design*

---

## **🎯 Ice-Breaker: Why Do We Model?**

Imagine designing a **car suspension** that keeps rides smooth on bumpy roads.
Instead of building countless prototypes, we **create a mathematical model** to test ideas virtually — saving time, money, and effort.

💡 *Even if a model is not 100% accurate, why can it still be valuable for design and prediction?*

---

## **1️⃣ Definition and Purpose**

📖 **Definition:**
Mathematical modeling in control systems is the process of **representing the behavior of a physical system using mathematical equations** that link its **inputs**, **outputs**, and **internal states**.

🔍 **Purpose:**

* **Predict** behavior under different inputs.
* **Design** controllers before physical implementation.
* **Simulate** and test system performance.
* **Optimize** performance and stability.

---

## **2️⃣ Steps in Mathematical Modeling**

1. **Understand the physical system** 🛠️ – Identify components, inputs, and outputs.
2. **Simplify** 📉 – Ignore negligible effects for clarity.
3. **Apply physical laws** 📐 – Newton’s Laws, Kirchhoff’s Laws, Thermodynamics, etc.
4. **Form equations** 📝 – Usually differential or difference equations.
5. **Convert to standard forms** 🔄 – Transfer functions, state-space, etc.

---

## **3️⃣ Common Modeling Domains**

| **Domain**                 | **Typical Elements**                                     | **Modeling Laws**                             |
| -------------------------- | -------------------------------------------------------- | --------------------------------------------- |
| Mechanical (Translational) | Mass (m), spring (k), damper (b)                         | Newton’s 2nd Law (F = ma)                     |
| Mechanical (Rotational)    | Inertia (J), torsional spring (K), rotational damper (B) | Torque balance equations                      |
| Electrical                 | Resistor (R), inductor (L), capacitor (C)                | Kirchhoff’s Voltage & Current Laws (KVL, KCL) |
| Thermal                    | Thermal resistance, thermal capacitance                  | Fourier’s Law, heat balance equations         |
| Hydraulic/Pneumatic        | Pressure, flow rate, orifice area                        | Bernoulli’s equation, continuity equation     |

---

## **4️⃣ Types of Mathematical Models**

| Type                      | Description                                                    | Example                                  |
| ------------------------- | -------------------------------------------------------------- | ---------------------------------------- |
| **Physical Model** 🏗️    | Real or scaled-down replica                                    | Wind tunnel model of an aircraft         |
| **Mathematical Model** 📊 | Equations representing the system                              | Differential equation of an RLC circuit  |
| **Black-box Model** 🎯    | Based on input-output data only, without knowing inner details | Empirical room temperature control model |

---

## **5️⃣ Common Modeling Techniques**

### **a) Differential Equations**

Uses derivatives to describe time-domain behavior.

Example: Mass-spring-damper system

$$
m\frac{d^2x}{dt^2} + c\frac{dx}{dt} + kx = F(t)
$$

---

### **b) Transfer Function**

Laplace-domain ratio of output to input (zero initial conditions):

$$
G(s) = \frac{Y(s)}{U(s)}
$$

Example: RC Circuit:

$$
G(s) = \frac{1}{RCs + 1}
$$

---

### **c) State-Space Models**

Matrix form using **state variables**:

$$
\dot{x} = Ax + Bu
$$

$$
y = Cx + Du
$$

Useful for **multi-input multi-output (MIMO)** systems.

---

### **d) Empirical Models**

Built from measured data without full physical understanding.
Example: Fitting a polynomial to furnace temperature data.

---
### **e) Block Diagram Models**

Visual representation of signal flow and system components.
Helps in system simplification.

---
## **6️⃣ Worked Example – Electrical System**

**System:** Series R–L circuit with input voltage \$V\_{in}(t)\$ and output current \$i(t)\$.

**Step 1 – KVL:**

$$
V_{in}(t) = L \frac{di(t)}{dt} + R i(t)
$$

**Step 2 – Standard form:**

$$
\frac{di(t)}{dt} + \frac{R}{L} i(t) = \frac{1}{L} V_{in}(t)
$$

This is a **first-order linear differential equation**.

---

## **7️⃣ Real-Life Examples**

* **🏢 Elevator Speed Control:** Input voltage → motor dynamics → speed output.
* **🏠 Room Temperature Control:** Heater power → heat transfer equation → temperature output.
* **🚁 Drone Altitude Control:** Thrust command → Newton’s 2nd Law + aerodynamics → height.
* **🚦 Bangladesh Railway Gate:** Train signal → motor torque model → gate position.

---

## **8️⃣ Challenges in Modeling**

* **Unmodeled dynamics** – Missing small effects may cause errors.
* **Parameter uncertainty** – Values like mass or resistance may vary.
* **Nonlinearities** – Many real systems aren’t perfectly linear.

---

## **9️⃣ Advantages of Mathematical Models**

* Predict behavior without physical prototypes.
* Enable systematic controller design.
* Allow safe simulation of dangerous or costly scenarios.

---

## **🔍 Summary Table**

| Method            | Best For                  | Example System     |
| ----------------- | ------------------------- | ------------------ |
| Differential Eqn. | Time-domain analysis      | Mass-spring-damper |
| Transfer Function | Frequency-domain analysis | RLC circuit        |
| State-Space       | Multi-variable systems    | Aircraft control   |
| Empirical         | Data-driven modeling      | Furnace control    |

---

## **❓ Objective Viva Questions**

**Q1:** Which law is used for modeling mechanical translational systems?
(a) Kirchhoff’s Voltage Law
(b) Newton’s Second Law
(c) Fourier’s Law
(d) Ohm’s Law

**Q2:** Which equation represents a state-space model?
(a) \$y = mx + c\$
(b) \$\dot{x} = Ax + Bu\$
(c) \$m\frac{d^2x}{dt^2} + kx = 0\$
(d) \$G(s) = \frac{Y(s)}{U(s)}\$

---

## **✅ Solutions**

* **Q1:** (b) Newton’s Second Law – relates force, mass, and acceleration.
* **Q2:** (b) \$\dot{x} = Ax + Bu\$ – standard state equation.
