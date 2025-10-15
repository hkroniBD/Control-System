This table is often used by control engineers to quickly judge system performance and stability from a Bode plot.

---

# 📊 Relative Relation Table — Gain Margin (GM) & Phase Margin (PM) vs Stability

| **Case**                              | **Gain Margin (GM)** | **Phase Margin (PM)** | **Stability Condition**         | **Transient Response Nature**                       | **Practical Interpretation**                                                                                                          |
| ------------------------------------- | -------------------- | --------------------- | ------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| 🟢 **Highly Stable**                  | > +10 dB             | > +60°                | Strongly stable                 | Very well damped, slow response                     | System is conservative; sluggish but safe; suitable for safety-critical or precision systems (e.g., autopilot, power supply control). |
| 🟩 **Stable (Good Design Zone)**      | +6 dB to +10 dB      | +30° to +60°          | Stable                          | Moderately damped, quick settling, slight overshoot | Optimum range for most industrial control designs (servo, DC drive, converter control).                                               |
| 🟨 **Marginally Stable**              | 0 to +6 dB           | 0° to +30°            | Near instability                | Underdamped, oscillatory, large overshoot           | Sensitive to parameter variations; minor delay or gain increase may cause oscillations.                                               |
| 🟠 **Critically Stable (Borderline)** | 0 dB                 | 0°                    | Neutral (sustained oscillation) | Continuous oscillation at natural frequency         | Indicates critical boundary between stable and unstable; used in oscillators.                                                         |
| 🔴 **Unstable**                       | Negative (GM < 0 dB) | Negative (PM < 0°)    | Unstable                        | Divergent response, growing oscillations            | Feedback becomes positive; output oscillates or diverges; redesign or compensation required.                                          |

---

## ⚙️ Engineering Notes

1. **Higher PM → more damping → slower but stable response.**
2. **Lower PM → less damping → faster but oscillatory response.**
3. **Negative PM → positive feedback dominates → instability.**
4. **Increasing GM or PM** can be achieved via **lead compensators**, phase advance networks, or by reducing delay.
5. **System delay** (like sensor or actuator lag) reduces PM, moving the system toward instability.

---

## 📈 Graphical Correlation (Conceptual)

As frequency increases:

* If **Phase = -180° before |G(jω)| drops below 0 dB → Unstable.**
* If **|G(jω)| = 0 dB while Phase = -150° → PM = 30° → Stable.**
* If **|G(jω)| = 0 dB while Phase = -120° → PM = 60° → Highly stable but slower.**

---

## 🔎 Practical Design Guidance

| **System Type**                          | **Recommended PM** | **Recommended GM** | **Reasoning**                                 |
| ---------------------------------------- | ------------------ | ------------------ | --------------------------------------------- |
| Power converters / Inverters             | 30°–45°            | ≥6 dB              | Compromise between speed and ripple stability |
| Servo drives / Robotics                  | 45°–60°            | ≥10 dB             | Smooth motion, minimal overshoot              |
| Measurement / Instrumentation amplifiers | 60°–90°            | >10 dB             | Noise immunity and high precision             |
| Process control loops                    | 30°–50°            | ≥6 dB              | Sufficient robustness against time delay      |

---

## 🧠 Quick Engineering Interpretation Summary

| Observation from Bode Plot            | Inference                  |
| ------------------------------------- | -------------------------- |
| Phase curve touches -180° before 0 dB | System is unstable         |
| Phase curve stays above -180° at 0 dB | System is stable           |
| Phase margin small (<30°)             | System oscillates easily   |
| Phase margin large (>60°)             | System sluggish but robust |
| Gain margin small (<6 dB)             | Poor robustness            |
| Gain margin large (>10 dB)            | Strongly stable            |

---
