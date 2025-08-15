---

**Lecture 4: 🖥️ PID Controller Implementation and Tuning in MATLAB**

**1️⃣ Introduction to PID Controllers**

* PID stands for Proportional-Integral-Derivative.
* Widely used in industrial control systems to achieve desired output.
* MATLAB provides tools to design, simulate, and tune PID controllers.

**2️⃣ PID Control Law**
$u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}$

* **Kp:** Proportional gain
* **Ki:** Integral gain
* **Kd:** Derivative gain
* **e(t):** Error signal (reference - output)

**3️⃣ Defining PID Controller in MATLAB**

```matlab
Kp = 2; Ki = 1; Kd = 0.5;
C = pid(Kp,Ki,Kd);
```

* `pid()` function creates a PID controller object.
* Can use `pidtune()` for automatic tuning.

**4️⃣ Connecting PID Controller to a Plant**

```matlab
s = tf('s');
G = 1/(s^2 + 3*s + 2); % Example plant
C = pid(2,1,0.5);
T = feedback(C*G,1); % Closed-loop system
step(T);
title('Closed-Loop Step Response with PID');
xlabel('Time (s)'); ylabel('Output');
```

* `feedback()` connects controller and plant in unity feedback configuration.

**5️⃣ Simulating Different PID Parameters**

* Vary **Kp**, **Ki**, **Kd** to observe effect on:

  * Rise time
  * Settling time
  * Overshoot
  * Steady-state error

```matlab
hold on;
Kp_values = [1 2 5];
for Kp = Kp_values
    C = pid(Kp,1,0.5);
    T = feedback(C*G,1);
    step(T);
end
legend('Kp=1','Kp=2','Kp=5');
```

**6️⃣ Using pidtune for Optimal Tuning**

```matlab
C = pidtune(G,'PID');
T = feedback(C*G,1);
step(T);
title('Step Response using pidtune');
```

* Automatically computes PID gains for a desired closed-loop response.

**7️⃣ Next Steps:**

* Apply PID control to first-order, second-order, and MIMO systems.
* Prepare for Lecture 5: Advanced Control Techniques (Lead-Lag, State Feedback) in MATLAB.
