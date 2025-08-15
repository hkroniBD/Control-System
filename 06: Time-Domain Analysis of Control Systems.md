# **Lecture 6: Time-Domain Analysis of Control Systems**
- 📕Course: Control System Engineering
- 🤖Instructor: Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU,Dinajpur,BD

---

## **Ice-Breaker Question 🤔**

When you press the accelerator of your car, do you immediately reach your target speed? Or does it take a short time before the car settles at a steady speed?
That *time-dependent behavior* is exactly what we analyze in the **time domain**.

---

## **1. Introduction**

The **time-domain analysis** studies how the output of a control system responds over time to a given input signal.
It is widely used to determine **system performance**, **speed of response**, and **stability**.

---

## **2. Standard Test Input Signals**

To evaluate system performance, we use standard test inputs:

| Input Type          | Mathematical Form    | Shape            | Common Use                                   |
| ------------------- | -------------------- | ---------------- | -------------------------------------------- |
| **Step Input**      | $u(t)$               | Sudden change    | Models sudden changes like a light switch ON |
| **Ramp Input**      | $t \cdot u(t)$       | Sloped line      | Models constant acceleration                 |
| **Parabolic Input** | $\frac{t^2}{2} u(t)$ | Curved           | Models increasing acceleration               |
| **Impulse Input**   | $\delta(t)$          | Very short spike | Tests system’s initial response              |

---

## **3. Time Response Components**

A system’s total time response can be divided into:

1. **Transient Response**

   * The initial part of the output before it settles.
   * Shows how quickly the system reacts.
2. **Steady-State Response**

   * The final value after transients die out.
   * Shows accuracy of the system.

---

## **4. Time-Domain Specifications**

For **second-order systems**, the key performance measures are:

| Parameter                    | Definition                                        | Importance                     |
| ---------------------------- | ------------------------------------------------- | ------------------------------ |
| **Rise Time (Tr)**           | Time to go from 10% to 90% of final value         | Speed of reaching output       |
| **Peak Time (Tp)**           | Time to reach the first maximum overshoot         | Indicates oscillation speed    |
| **Maximum Overshoot (Mp)**   | % amount exceeding final value                    | Indicates oscillation severity |
| **Settling Time (Ts)**       | Time to remain within ±2% (or ±5%) of final value | Speed of stabilization         |
| **Steady-State Error (Ess)** | Difference between desired and actual final value | Accuracy measure               |

---

## **5. First-Order System Time Response**

For a first-order system:

$$
G(s) = \frac{K}{\tau s + 1}
$$

* Time constant ($\tau$): smaller values → faster response.
* Step response:

$$
c(t) = K \left( 1 - e^{-t/\tau} \right)
$$

---

## **6. Second-Order System Time Response**

For a standard second-order system:

$$
G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}
$$

* **$\omega_n$** → natural frequency (speed of oscillation)
* **$\zeta$** → damping ratio (controls oscillation decay)

---

### **Damping Ratio Classification**

| Damping Ratio ($\zeta$) | System Type       | Response                    |
| ----------------------- | ----------------- | --------------------------- |
| $\zeta = 0$             | Undamped          | Sustained oscillations      |
| $0 < \zeta < 1$         | Underdamped       | Oscillatory with decay      |
| $\zeta = 1$             | Critically damped | Fastest without oscillation |
| $\zeta > 1$             | Overdamped        | Slow, no oscillation        |

---

## **7. Real-Life Example 🚗**

Imagine **cruise control** in a car:

* You set the speed to **80 km/h**.
* **Transient Response**: The car accelerates and might slightly overshoot to **82 km/h**.
* **Steady-State Response**: It settles at exactly **80 km/h**.
* The **settling time** is how long it takes to reach and stay near **80 km/h**.
* If the road is uphill or downhill, the **steady-state error** tells us how close we stay to the target speed.

---

## **8. Summary Table**

| Concept                   | Purpose                                |
| ------------------------- | -------------------------------------- |
| **Transient Response**    | Early system reaction                  |
| **Steady-State Response** | Final system behavior                  |
| **Time-Domain Specs**     | Measure speed, accuracy, and stability |
| **First vs Second Order** | Different dynamics and complexity      |

---

## **Objective Viva Questions ❓**

1. Which part of the time response determines system accuracy?
   a) Transient response
   b) Steady-state response
   c) Rise time
   d) Overshoot

2. If damping ratio $\zeta = 1$, the system is:
   a) Overdamped
   b) Critically damped
   c) Underdamped
   d) Unstable

3. The time taken to go from 10% to 90% of the final value is:
   a) Rise time
   b) Settling time
   c) Peak time
   d) Overshoot time

---

## **Solutions to Viva Questions ✅**

1. **b)** Steady-state response → shows final accuracy.
2. **b)** Critically damped → fastest response without oscillations.
3. **a)** Rise time → measures initial speed to reach near-final value.

---
