**Lecture 7: 🛠️ Control System Stability**

---

## **🚦 Ice-Breaker**

Imagine you’re driving a car 🚗 and take your hands off the steering wheel for a moment. If the car smoothly keeps going in its lane, it’s “stable.” If it slowly drifts but doesn’t crash immediately, it’s “marginally stable.” But if it swerves wildly and ends up in a ditch, that’s “unstable.”
Control system stability works the same way — it’s about whether the system’s output remains predictable and bounded over time.

---

## **1️⃣ What is Stability in Control Systems**

📌 Stability refers to the ability of a system to return to its equilibrium state after being disturbed.

A system is **stable** if:

* For a bounded input → the output remains bounded (**BIBO stability**) ✅.
* The system eventually returns to its steady state after a disturbance.

💡 Note: Stability ensures safety and predictable operation in real-world systems.

---

## **2️⃣ Types of Stability**

1. **BIBO Stability (Bounded-Input, Bounded-Output)** 🟢

   * Bounded input → bounded output.
   * *Example:* Shock absorber in a car keeps vibrations controlled.

2. **Asymptotic Stability** 🔵

   * Output returns to zero or equilibrium as time → ∞.

3. **Marginal Stability** 🟡

   * Output remains bounded but oscillates indefinitely.

4. **Instability** 🔴

   * Output grows unbounded over time.

💡 Note: Marginal stability is rare in practical systems; usually, we aim for asymptotic stability.

---

## **3️⃣ Mathematical Criteria for Stability**

For **linear time-invariant (LTI)** systems, stability can be checked from system poles:

* All poles in **LHP** → **Stable** ✅
* Any pole in **RHP** → **Unstable** ❌
* Poles on the **imaginary axis** → **Marginally Stable** ⚠️

💡 Note: Pole location directly relates to system response over time.

---

## **4️⃣ Stability in Time Domain**

For a second-order system:

$$
y(t) = A e^{-\zeta\omega_n t} \sin(\omega_d t + \phi)
$$

* \$\zeta > 0\$ → stable ✅
* \$\zeta = 0\$ → marginally stable ⚠️
* \$\zeta < 0\$ → unstable ❌

---

## **5️⃣ Stability in Frequency Domain**

* **Nyquist Criterion** and **Bode Plots** 📊 help assess stability without explicitly solving for poles.
* **Gain Margin (GM)** and **Phase Margin (PM)** indicate how close a system is to instability.

💡 Note: Frequency-domain methods are especially useful for designing feedback controllers.

---

## **6️⃣ Real-Life Examples**

* **Stable** 🟢: Cruise control maintains constant speed even on slopes.
* **Marginally Stable** 🟡: A frictionless pendulum swings forever.
* **Unstable** 🔴: A bicycle with a broken steering mechanism wobbles uncontrollably.

---

## **7️⃣ Methods to Analyze Stability**

1. **Routh-Hurwitz Criterion** 📋
2. **Root Locus** 📈
3. **Nyquist & Bode Plots** 📊

💡 Note: Choosing the right method depends on system complexity and type.

---

## **8️⃣ Improving Stability**

* Increase damping ratio 🛠️
* Feedback design (PID tuning) ⚙️
* Reduce system gain if too high 📉

---

### **📊 Summary Table**

| Stability Type       | Pole Location                  | Output Behavior            |
| -------------------- | ------------------------------ | -------------------------- |
| Stable 🟢            | All poles in LHP               | Decays to equilibrium      |
| Marginally Stable 🟡 | Simple poles on imaginary axis | Oscillates forever         |
| Unstable 🔴          | Any pole in RHP                | Output grows without bound |

---

## **🎯 Objective Viva Questions**

**Q1.** Which criterion checks stability without finding the roots?
(a) Nyquist Criterion
(b) Routh-Hurwitz Criterion ✅
(c) Bode Plot
(d) State-Space Representation

**Q2.** If poles are at \$-2, -3, -5\$, the system is:
(a) Stable ✅
(b) Marginally Stable
(c) Unstable
(d) None of the above

**Q3.** In BIBO stability, the system output must be:
(a) Infinite
(b) Zero
(c) Bounded ✅
(d) Oscillating

---

## **✅ Solutions to Viva Questions**

**Q1:** (b) Routh-Hurwitz Criterion
**Q2:** (a) Stable
**Q3:** (c) Bounded

---

💡 **Notes:**

* Stability is the foundation for designing safe and reliable control systems.
* Understanding pole locations and damping helps in controller tuning.
* Real-world systems often combine time-domain and frequency-domain analysis for stability assessment.
