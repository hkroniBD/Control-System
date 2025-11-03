# PID Control Systems
## Lecture Notes for Control Systems Course

---

## 1. Introduction to Feedback Control

### What is Control?
Control systems are designed to maintain a desired output (controlled variable) by manipulating an input (manipulated variable). The fundamental goal is to minimize the error between the desired setpoint and the actual system output.

### Why PID Control?
- **Most widely used** industrial controller (>90% of control loops)
- **Simple to understand** and implement
- **Effective** for a wide range of processes
- **No process model required** for basic tuning
- **Three-term control** provides comprehensive error correction

---

## 2. The Control Problem

### Basic Feedback Loop
```
         +------+      +------------+      +--------+
r(t) -->| + |----->| Controller |----->| Plant  |----> y(t)
    -   |      |      +------------+      +--------+
    |   +------+                              |
    |                                         |
    +-----------------------------------------+
```

Where:
- **r(t)** = Reference/Setpoint (desired value)
- **y(t)** = Process Variable/Output (actual value)
- **e(t) = r(t) - y(t)** = Error signal
- **u(t)** = Control signal

---

## 3. PID Controller Structure

### The PID Equation

The standard form of PID controller:

**u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·(de(t)/dt)**

Or in terms of individual gains:

**u(t) = Kp·e(t) + (Kp/Ti)·∫e(τ)dτ + Kp·Td·(de(t)/dt)**

Where:
- **Kp** = Proportional gain
- **Ki** = Integral gain (or Kp/Ti)
- **Kd** = Derivative gain (or Kp·Td)
- **Ti** = Integral time constant
- **Td** = Derivative time constant

### Transfer Function Form (Laplace Domain)

**G_c(s) = Kp + Ki/s + Kd·s**

Or:

**G_c(s) = Kp(1 + 1/(Ti·s) + Td·s)**

---

## 4. Understanding Each Control Action

### 4.1 Proportional Control (P)

**Action:** u(t) = Kp·e(t)

**Characteristics:**
- Output is proportional to current error
- Larger Kp → faster response but more overshoot
- Cannot eliminate steady-state error in many systems
- Simple but limited

**Advantages:**
- Easy to understand and implement
- Fast response to changes
- Stable for appropriate gains

**Disadvantages:**
- Steady-state error (offset)
- Trade-off between speed and stability

**Example:** Room temperature control - heater power proportional to temperature error

---

### 4.2 Integral Control (I)

**Action:** u(t) = Ki·∫e(τ)dτ

**Characteristics:**
- Output depends on accumulated error over time
- Eliminates steady-state error
- Can cause overshoot and oscillation
- Responds to persistent errors

**Advantages:**
- Eliminates steady-state error
- Compensates for constant disturbances
- Improves long-term accuracy

**Disadvantages:**
- Can cause overshoot
- Integral windup problem
- Slower response

**Example:** Cruise control - adjusts throttle until speed error is zero

---

### 4.3 Derivative Control (D)

**Action:** u(t) = Kd·(de(t)/dt)

**Characteristics:**
- Output depends on rate of error change
- Anticipates future error
- Provides damping
- Never used alone

**Advantages:**
- Reduces overshoot
- Improves stability
- Faster settling time
- Anticipatory action

**Disadvantages:**
- Amplifies noise
- No action on steady-state error
- Requires filtering in practice

**Example:** Car suspension damper - responds to rate of motion change

---

## 5. Combined Control Actions

### P Control
- Fast response
- Steady-state error present
- Good for simple applications

### PI Control
- No steady-state error
- Moderate speed
- Most common in process control
- Good for slow processes

### PD Control
- Fast response with damping
- Steady-state error remains
- Good for servo systems
- Rarely used alone

### PID Control
- Fast response
- No steady-state error
- Well-damped
- Most versatile
- Requires careful tuning

---

## 6. Effect of Controller Parameters

### Increasing Kp:
- ↑ Speed of response
- ↓ Steady-state error (but doesn't eliminate)
- ↑ Overshoot
- ↓ Stability margin

### Increasing Ki:
- ↓ Steady-state error (eliminates it)
- ↑ Overshoot
- ↓ Stability
- ↑ Settling time

### Increasing Kd:
- ↓ Overshoot
- ↑ Stability
- ↓ Settling time
- ↑ Noise sensitivity

### Parameter Effects Summary Table

| Parameter | Rise Time | Overshoot | Settling Time | SS Error | Stability |
|-----------|-----------|-----------|---------------|----------|-----------|
| Kp ↑      | Decrease  | Increase  | Small change  | Decrease | Degrade   |
| Ki ↑      | Decrease  | Increase  | Increase      | Eliminate| Degrade   |
| Kd ↑      | Minor dec | Decrease  | Decrease      | No effect| Improve   |

---

## 7. PID Tuning Methods

### 7.1 Manual Tuning (Trial and Error)

**Step-by-step procedure:**

1. Set Ki = 0 and Kd = 0 (P control only)
2. Increase Kp until system oscillates steadily
3. Reduce Kp to 50% of oscillation value
4. Increase Ki until steady-state error is eliminated
5. Increase Kd to reduce overshoot if needed

---

### 7.2 Ziegler-Nichols Methods

**Method 1: Ultimate Gain Method (Closed-Loop)**

1. Set Ki = 0, Kd = 0
2. Increase Kp until sustained oscillation occurs
3. Note Ku (ultimate gain) and Pu (oscillation period)
4. Apply tuning formulas:

| Controller | Kp      | Ti      | Td      |
|------------|---------|---------|---------|
| P          | 0.5·Ku  | ∞       | 0       |
| PI         | 0.45·Ku | Pu/1.2  | 0       |
| PID        | 0.6·Ku  | Pu/2    | Pu/8    |

**Method 2: Reaction Curve Method (Open-Loop)**

1. Apply step input to system
2. Record S-shaped response curve
3. Draw tangent at inflection point
4. Determine L (lag time) and T (time constant)
5. Calculate R = L/T
6. Apply tuning formulas:

| Controller | Kp    | Ti  | Td    |
|------------|-------|-----|-------|
| P          | T/(L·R)| ∞   | 0     |
| PI         | 0.9·T/(L·R)| 3.33·L| 0|
| PID        | 1.2·T/(L·R)| 2·L   | 0.5·L|

---

### 7.3 Cohen-Coon Method

Better for processes with significant dead time:

For PID: 
- Kp = (T/L)·(1.35/R + 0.27)
- Ti = L·(2.5 + 0.46·R)/(1 + 0.61·R)
- Td = L·0.37/(1 + 0.19·R)

---

## 8. Practical Implementation Issues

### 8.1 Integral Windup

**Problem:** When actuator saturates, integral term continues accumulating, causing large overshoot.

**Solutions:**
- Anti-windup: Stop integration when output saturates
- Conditional integration: Only integrate when error is small
- Back-calculation: Adjust integral term based on saturation

### 8.2 Derivative Kick

**Problem:** Sudden setpoint changes cause large derivative spikes.

**Solution:** Take derivative of output, not error:

**u(t) = Kp·e(t) + Ki·∫e(τ)dτ - Kd·(dy(t)/dt)**

### 8.3 Noise in Derivative Action

**Problem:** High-frequency noise is amplified by derivative action.

**Solutions:**
- Low-pass filter on derivative term
- Filtered derivative: **D(s) = Kd·s/(1 + τ·s)**
- Use larger τ (e.g., τ = Td/10)

### 8.4 Sample Rate Selection

For digital implementation:
- Sample 10-20 times faster than system time constant
- Nyquist criterion: fs > 2·f_max
- Avoid aliasing

---

## 9. Digital PID Implementation

### Discrete-Time PID Equation

**Position form:**
```
u[k] = Kp·e[k] + Ki·Ts·Σe[i] + (Kd/Ts)·(e[k] - e[k-1])
```

**Velocity form:**
```
Δu[k] = Kp·(e[k] - e[k-1]) + Ki·Ts·e[k] + (Kd/Ts)·(e[k] - 2·e[k-1] + e[k-2])
```

Where:
- **Ts** = Sampling period
- **k** = Current sample
- **e[k]** = Error at sample k

### Numerical Integration Methods

**Rectangular (Forward Euler):**
```
I[k] = I[k-1] + Ts·e[k]
```

**Trapezoidal:**
```
I[k] = I[k-1] + (Ts/2)·(e[k] + e[k-1])
```

---

## 10. Performance Specifications

### Time-Domain Specifications:
- **Rise time (tr):** Time to reach 90% of final value
- **Peak time (tp):** Time to first peak
- **Settling time (ts):** Time to stay within ±2% of final value
- **Overshoot (Mp):** (Peak - Final)/Final × 100%
- **Steady-state error (ess):** lim(t→∞) [r(t) - y(t)]

### Frequency-Domain Specifications:
- **Gain margin (GM):** How much gain can increase before instability
- **Phase margin (PM):** Phase lag before instability
- **Bandwidth:** Frequency range of effective control

---

## 11. Applications of PID Control

### Industrial Applications:
- Temperature control in furnaces, reactors
- Pressure control in pipelines
- Flow control in chemical processes
- Level control in tanks
- Speed control in motors

### Consumer Applications:
- Cruise control in automobiles
- Thermostat in HVAC systems
- Oven temperature control
- Water heater control

### Robotics and Automation:
- Position control in servo systems
- Velocity control in drives
- Force control in manipulators
- Trajectory tracking

---

## 12. Advantages and Limitations

### Advantages:
✓ Simple structure and implementation
✓ No process model required
✓ Wide applicability
✓ Well-understood tuning methods
✓ Robust to parameter variations
✓ Industry standard

### Limitations:
✗ Not optimal for all processes
✗ Limited performance with large dead time
✗ Non-adaptive (fixed parameters)
✗ Cannot handle complex dynamics
✗ Manual tuning can be time-consuming
✗ May not handle nonlinear systems well

---

## 13. Advanced Topics (Brief Overview)

### Auto-Tuning PID
- Relay feedback methods
- Pattern recognition
- Adaptive algorithms

### Gain Scheduling
- Different PID parameters for different operating points
- Useful for nonlinear systems

### Cascade Control
- Multiple PID loops in series
- Inner loop for fast disturbances
- Outer loop for setpoint tracking

### Feedforward Control
- Combined with PID for measurable disturbances
- Anticipatory action

---

## 14. Summary and Key Takeaways

1. **PID combines three control actions:** proportional (present), integral (past), and derivative (future)

2. **Each term has specific purpose:**
   - P: Quick response
   - I: Eliminate steady-state error
   - D: Improve stability and reduce overshoot

3. **Tuning is critical:** Poor tuning leads to poor performance

4. **Practical implementation requires attention to:**
   - Integral windup
   - Derivative noise
   - Sample rate
   - Saturation limits

5. **PID is versatile but not universal:** Some processes need advanced control

---

## Practice Problems

### Problem 1
A P-only controller with Kp = 5 is applied to a first-order system. The steady-state error is 20%. What should Kp be to reduce error to 10%?

### Problem 2
Using Ziegler-Nichols ultimate gain method, Ku = 8 and Pu = 2 seconds. Calculate PID parameters.

### Problem 3
Explain why derivative control is never used alone. What would happen if you tried?

### Problem 4
A digital PID controller has Ts = 0.1 sec. The system time constant is 0.5 sec. Is this sampling rate adequate? Why?

---

## Further Reading

- Åström, K.J., & Hägglund, T. (2006). *Advanced PID Control*
- Franklin, G.F., Powell, J.D., & Emami-Naeini, A. *Feedback Control of Dynamic Systems*
- Ogata, K. *Modern Control Engineering*
- Seborg, D.E., et al. *Process Dynamics and Control*

---

## Questions?

**Review for next class:**
- State-space representation
- Root locus analysis
- Frequency response methods