**Lecture 5: Transfer Function and Block Diagram Representation**
*Course: Control System Engineering*
*Instructor: HK Roni, Assistant Professor, EEE, HSTU,Dinajpur,BD*

---

## **Icebreaker – A Quick Analogy**

Imagine you are giving someone cooking instructions:

* **Input:** The recipe (ingredients + steps)
* **System:** The cooking process (heat, mixing, timing)
* **Output:** The finished dish

In control systems, the **transfer function** plays the role of a "recipe" for predicting how the system will respond to any given input.

---

## **1. Introduction to Transfer Functions**

A **transfer function** is a mathematical representation that relates the **output** of a system to its **input** in the Laplace transform domain, assuming **zero initial conditions**.

It is defined as:

$$
G(s) = \frac{Y(s)}{X(s)}
$$

Where:

* $Y(s)$ = Laplace transform of the output
* $X(s)$ = Laplace transform of the input
* $G(s)$ = Transfer function

---

## **2. Importance of Transfer Functions**

* **Simplifies system analysis**: Works in the frequency domain rather than time domain.
* **Helps in stability analysis**: Easily find poles & zeros.
* **Aids in design**: Useful for controller design (PID, Lead/Lag, etc.).

---

## **3. Steps to Derive a Transfer Function**

1. Write the governing differential equation of the system.
2. Apply Laplace transform assuming zero initial conditions.
3. Take the ratio $\frac{\text{Output}}{\text{Input}}$ to obtain $G(s)$.

---

## **4. Real-Life Examples**

### Example 1: Electrical RC Circuit

* Governing equation:

  $$
  RC \frac{dy(t)}{dt} + y(t) = x(t)
  $$
* Laplace transform:

  $$
  RC \cdot s Y(s) + Y(s) = X(s)
  $$
* Transfer function:

  $$
  G(s) = \frac{Y(s)}{X(s)} = \frac{1}{RCs + 1}
  $$

### Example 2: Water Tank Level System

* Input: Water inflow rate
* Output: Water level
* Transfer function shows how level changes with inflow changes over time.

---

## **5. Block Diagram Representation**

A **block diagram** is a graphical way to represent the functional relationship between system components.

**Key elements:**

* **Blocks:** Represent transfer functions.
* **Arrows:** Indicate the direction of signal flow.
* **Summing points:** Represent the addition/subtraction of signals.
* **Take-off points:** Split signals to multiple blocks.

---

### **Basic Example**

If we have two blocks in series:

$$
G_1(s) \quad \text{and} \quad G_2(s)
$$

The overall transfer function is:

$$
G(s) = G_1(s) \cdot G_2(s)
$$

If they are in parallel:

$$
G(s) = G_1(s) + G_2(s)
$$

---

## **6. Signal Flow Graphs**

An alternative to block diagrams, using nodes and branches to represent system equations. **Mason’s Gain Formula** can be used to find the overall transfer function.

---

## **7. Practical Applications**

* **Robotics:** Arm positioning control
* **Aerospace:** Aircraft autopilot
* **Audio Systems:** Equalizer filtering
* **Industrial Automation:** Conveyor speed control

---

## **Objective Viva Questions ❓**

1. The transfer function is defined in which domain?
   a) Time domain
   b) Frequency domain
   c) Laplace domain
   d) Both time and frequency domain

2. In a block diagram, two blocks in series have:
   a) Sum of transfer functions
   b) Product of transfer functions
   c) Ratio of transfer functions
   d) None of the above

3. The poles of a transfer function are obtained from:
   a) Zeros of numerator
   b) Zeros of denominator
   c) Both numerator and denominator
   d) None

---

## **Solutions to Viva Questions ✅**

1. **c)** Laplace domain – transfer functions are defined after Laplace transformation.
2. **b)** Product of transfer functions – in series, we multiply them.
3. **b)** Zeros of denominator – poles come from denominator equation roots.
