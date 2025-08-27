# **Lecture 13: 📐 Root Locus and Frequency Domain Techniques**
- 📕Course: Control System Engineering
- 🤖Instructor: Md. Hassanul Karim Roni, Assistant Professor, EEE, HSTU,Dinajpur,BD

---

## **🧩 Ice-Breaker**

Imagine adjusting the tension on a guitar string to achieve the perfect note. The way the string vibrates changes as you tune it. Similarly, **root locus and frequency domain methods** show how system behavior changes when parameters vary.

---

## **1️⃣ Introduction to Root Locus**

**Definition:** Root locus is a graphical method to study how the **poles of a closed-loop system** move in the s-plane as a system parameter (usually gain K) changes.

**Purpose:**

* Analyze **stability** as gain varies
* Design controllers for desired **transient response**
* Predict system behavior before implementation

---

## **2️⃣ Rules for Drawing Root Locus**

1. Root locus starts at **open-loop poles** and ends at **open-loop zeros**
2. Symmetrical about the **real axis**
3. Number of branches = number of poles
4. Real axis segments belong to the root locus if an **odd number of poles and zeros** lie to the right
5. Asymptotes show direction for branches going to infinity

**Example:** Second-order system with transfer function $G(s) = \frac{K}{s(s+2)}$

---

## **3️⃣ Frequency Domain Techniques**

**a) Bode Plot** 📈

* Shows **magnitude** and **phase** vs frequency
* Determines **gain margin** and **phase margin**
* Useful for **stability and performance assessment**

**b) Nyquist Plot** 🔄

* Encodes frequency response in a polar plot
* **Nyquist criterion** predicts closed-loop stability
* Handles **open-loop unstable systems**

**c) Nichols Chart**

* Combines gain and phase information in a single plot
* Useful for **gain and phase margin analysis**

---

## **4️⃣ Advantages**

* Visualizes how pole locations affect system stability
* Guides **controller design** (PID tuning, compensators)
* Helps predict **transient and steady-state behavior**
* Applicable to **both SISO and MIMO systems**

---

## **5️⃣ Limitations / Cons**

* Requires **linear models**
* Complex for high-order systems
* Interpretation may be difficult for beginners
* Frequency domain methods need **accurate system data**

---

## **6️⃣ Real-Life Examples**

* **Cruise Control Design** 🚗: Adjusting gain to ensure smooth acceleration without overshoot
* **Servo Motor Control** ⚙️: Root locus shows effect of controller gain on speed and stability
* **Power System Stabilizer** ⚡: Nyquist plot ensures voltage stability under load changes
* **Aircraft Pitch Control** ✈️: Bode plot helps design compensators for safe maneuvering

---

## **🎯 Objective Viva Questions**

**Q1.** What does a root locus plot show?
(a) Time-domain response
(b) Pole movement as system gain changes
(c) Frequency response magnitude only
(d) Zeros of open-loop system

**Q2.** Gain margin and phase margin can be determined from:
(a) Root locus
(b) Bode plot
(c) Time response
(d) State-space equations

**Q3.** Nyquist plot is primarily used for:
(a) Designing P controllers only
(b) Predicting closed-loop stability
(c) Calculating rise time
(d) Measuring damping ratio

**Q4.** Root locus branches start at:
(a) Open-loop zeros
(b) Open-loop poles
(c) Closed-loop poles
(d) Steady-state value

**Q5.** Which method is best for high-frequency stability analysis?
(a) Root locus
(b) Bode plot
(c) Step response
(d) Trial-and-error

---

## **✅ Solutions to Viva Questions**

**Q1:** (b) Pole movement as system gain changes
**Q2:** (b) Bode plot
**Q3:** (b) Predicting closed-loop stability
**Q4:** (b) Open-loop poles
**Q5:** (b) Bode plot

---

💡 **Additional Notes:**

* Root locus and frequency domain techniques are complementary for control design.
* Gain and phase margins provide safety buffers in practical systems.
* Modern software (MATLAB, Python) allows plotting root locus, Bode, and Nyquist plots efficiently.
