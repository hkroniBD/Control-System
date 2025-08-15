**Lecture 8: ⏱️ Time Response Analysis**

---

## **🚦 Ice-Breaker**

Imagine you throw a ball against a wall and observe how it bounces back. The way it returns — fast or slow, oscillating or smooth — tells you about the system’s response. Similarly, **time response analysis** tells us how a control system reacts to changes in input over time.

---

## **1️⃣ Introduction to Time Response**

Time response is the output behavior of a system when subjected to a specific input. It helps understand **speed, stability, and accuracy** of the system.

Types of time responses:

* **Transient Response** – behavior before reaching steady state.
* **Steady-State Response** – final output behavior after transients die out.

💡 Note: Both aspects are crucial for designing responsive and reliable controllers.

---

## **2️⃣ Common Input Signals**

* **Step Input** ⬆️: Sudden change, like switching a motor on.
* **Impulse Input** ⚡: Very short, high-intensity input.
* **Ramp Input** ⬆️📈: Gradual increase over time.
* **Sinusoidal Input** 🌊: Periodic input, useful in frequency response analysis.

---

## **3️⃣ First-Order System Response**

A first-order system is described by:
$\tau \frac{dy(t)}{dt} + y(t) = K u(t)$

* **Step Response**: $y(t) = K(1 - e^{-t/\tau})$
* **Time Constant (τ)**: Time for output to reach \~63% of final value.

💡 Note: Smaller τ → faster response.

---

## **4️⃣ Second-Order System Response**

General form:
$\frac{d^2y(t)}{dt^2} + 2\zeta\omega_n \frac{dy(t)}{dt} + \omega_n^2 y(t) = \omega_n^2 u(t)$

* **Damping ratio (ζ)** and **natural frequency (ωn)** define the response type.

### **Types of Second-Order Response**

* **Overdamped (ζ > 1)** – slow, no oscillation.
* **Critically Damped (ζ = 1)** – fastest response without overshoot.
* **Underdamped (0 < ζ < 1)** – oscillatory with decaying amplitude.
* **Undamped (ζ = 0)** – continuous oscillation.

💡 Note: Overshoot, rise time, settling time, and peak time are key parameters.

---

## **5️⃣ Real-Life Examples**

* **Stepper Motor Control** ⚙️ – first-order response to voltage input.
* **Aircraft Altitude Control** ✈️ – second-order underdamped response, oscillates slightly before settling.
* **Room Temperature Regulation** 🌡️ – step input from heater leads to first-order temperature rise.

---

## **6️⃣ Time Response Specifications**

| Specification      | Description                                       |
| ------------------ | ------------------------------------------------- |
| Rise Time (Tr)     | Time to go from 10% to 90% of final value         |
| Peak Time (Tp)     | Time to reach first maximum peak                  |
| Maximum Overshoot  | Amount output exceeds final value                 |
| Settling Time (Ts) | Time to remain within 2%–5% of final value        |
| Steady-State Error | Difference between final output and desired value |

💡 Note: Designers adjust parameters to meet these specifications.

---

## **7️⃣ Methods of Analysis**

* Analytical solution of differential equations.
* Laplace transform methods for transfer function analysis.
* Simulation tools (MATLAB, Simulink) for time-domain response.

---

## **🎯 Objective Viva Questions**

**Q1.** What is the time constant (τ) in a first-order system?
(a) Time to reach 50% of final value
(b) Time to reach 63% of final value ✅
(c) Time to settle within 5%
(d) None of the above

**Q2.** Which damping ratio gives fastest response without overshoot?
(a) ζ < 1
(b) ζ = 1 ✅
(c) ζ > 1
(d) ζ = 0

**Q3.** Which response parameter measures time to remain within 2%–5% of final value?
(a) Rise Time
(b) Peak Time
(c) Settling Time ✅
(d) Overshoot

---

## **✅ Solutions to Viva Questions**

**Q1:** (b) Time to reach \~63% of final value
**Q2:** (b) Critically damped (ζ = 1)
**Q3:** (c) Settling Time

---

💡 **Notes:**

* Time response analysis allows engineers to predict system performance under different inputs.
* Helps in designing controllers for faster and more accurate response.
* Essential for both first-order and higher-order system design.

