**Lecture 16: 📏 Root Locus Techniques for Controller Design**

---

## **🧩 Ice-Breaker**

Imagine balancing a stick vertically on your hand. If you slowly move your hand, the stick tilts but you can bring it back. Root locus is like plotting all the possible positions of the stick over time as you change system parameters.

---

## **1️⃣ Introduction to Root Locus**

**Definition:** Root locus is a graphical method to study how the **poles of a closed-loop system** move in the **s-plane** as a system parameter (usually gain) varies.

**Purpose:**

* Analyze **stability changes** with gain
* Design controllers by selecting appropriate gain values
* Visualize system transient behavior

---

## **2️⃣ Key Concepts**

* **Open-loop poles and zeros**: Start and end points of root locus branches
* **Branches**: Paths traced by poles as gain changes
* **Breakaway and break-in points**: Points where poles split or converge
* **Asymptotes**: Directions of branches as gain → ∞

---

## **3️⃣ Steps to Construct Root Locus**

1. Plot open-loop poles (x) and zeros (o) on s-plane
2. Determine number of branches (equal to number of poles)
3. Find real-axis segments where root locus exists
4. Calculate asymptotes and intersection points
5. Locate breakaway/break-in points
6. Sketch the path as gain varies

---

## **4️⃣ Real-Life Examples**

* **Servo Motor Position Control** ⚙️: Adjusting gain to minimize overshoot and settling time
* **Inverted Pendulum Stabilization** 🤹: Choosing gain to stabilize the pendulum
* **DC Motor Speed Control** 🚗: Ensuring smooth acceleration without oscillation
* **Flight Control System** ✈️: Maintaining aircraft stability under varying gains

---

## **5️⃣ Advantages of Root Locus** ✅

* Provides **visual insight** into system stability and transient behavior
* Useful for **controller gain selection**
* Can design compensators to improve performance
* Works for **SISO systems** effectively

---

## **6️⃣ Limitations / Cons** ⚠️

* Primarily for **SISO systems**; MIMO is more complex
* Only shows **pole movement**; doesn’t directly show time-domain response
* High-order systems can be challenging to plot by hand
* Requires knowledge of system poles and zeros

---

## **7️⃣ Controller Design Using Root Locus**

* **Proportional (P) Controller:** Move poles for desired damping and speed
* **PD Controller:** Adds derivative action to shift root locus left and increase damping
* **PI Controller:** Adds integral action to reduce steady-state error
* **Lead/Lag Compensators:** Adjust pole-zero locations to shape root locus for stability and performance

---

## **🎯 Objective Viva Questions**

**Q1.** Root locus plots show:
(a) Step response
(b) Closed-loop poles location as gain varies
(c) Frequency response magnitude only
(d) Open-loop zeros only

**Q2.** Breakaway points occur when:
(a) Poles converge or diverge on the real axis
(b) System gain is zero
(c) Zeros are on the imaginary axis
(d) Poles are purely imaginary

**Q3.** Which controller can be designed using root locus to improve transient response?
(a) PI
(b) PD
(c) P
(d) All of the above

**Q4.** Root locus is most effective for:
(a) MIMO systems
(b) SISO systems
(c) Nonlinear systems
(d) Time-delay systems

**Q5.** Lead compensators affect root locus by:
(a) Shifting poles right
(b) Shifting poles left and improving damping
(c) Adding integrator
(d) Increasing system gain only

---

## **✅ Solutions to Viva Questions**

**Q1:** (b) Closed-loop poles location as gain varies
**Q2:** (a) Poles converge or diverge on the real axis
**Q3:** (d) All of the above
**Q4:** (b) SISO systems
**Q5:** (b) Shifting poles left and improving damping

---

💡 **Additional Notes:**

* Root locus is a powerful tool to **visualize system behavior** as gain changes.
* Enables systematic design of **P, PD, PI, and compensator controllers**.
* Software tools like MATLAB can generate accurate root locus plots for complex systems.

---
