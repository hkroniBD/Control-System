---

**Lecture 1: 🖥️ Introduction to MATLAB for Control Systems**

**1️⃣ What is MATLAB?**

* MATLAB (MATrix LABoratory) is a high-level programming language and interactive environment for numerical computation, visualization, and programming.
* Used extensively in engineering, science, and applied mathematics for simulation, modeling, and data analysis.

**2️⃣ History and Inventor**

* Developed in the late 1970s by **Cleve Moler**, a professor at the University of New Mexico.
* Initially created to provide students easy access to matrix software without needing Fortran.

**3️⃣ Why MATLAB is Used in Control Systems**

* Efficient for **matrix operations** and system modeling.
* Provides built-in **toolboxes** for control design, analysis, and simulation.
* Supports **visualization** of system responses (time-domain, frequency-domain).
* Integrates with **Simulink** for block-diagram-based modeling and simulation.

**4️⃣ Required MATLAB Packages for Control Systems**

* **Control System Toolbox:** Functions for transfer functions, state-space models, root locus, Bode and Nyquist plots.
* **Simulink:** Graphical environment for modeling, simulating, and analyzing dynamic systems.
* Optional: Signal Processing Toolbox for filtering and analysis.

**5️⃣ MATLAB Essentials for Beginners**

* Command Window basics: typing commands, executing code, clearing workspace using `clc`, `clear`, `close all`
* Defining variables: `x = 5; y = 10; z = x + y;`
* Displaying values: `disp(x)`, `fprintf('Value of y = %d
  ', y)`
* Creating vectors/matrices: `A = [1 2; 3 4]; v = 1:0.1:10;`
* Basic plotting: `plot(t, y)`, `xlabel('Time')`, `ylabel('Output')`, `title('Plot Title')`
* **What is a Transfer Function:** A transfer function represents the relationship between system output and input in the Laplace domain, assuming zero initial conditions.
* **Meaning:** It shows how input signals are transformed into output signals, useful for analyzing system dynamics.
* **Defining Transfer Functions in MATLAB:**

  * Using numerator and denominator vectors: `tf(num, den)`
  * Using symbolic 's' variable: `s = tf('s'); G = 1/(s+1)`
  * Using polynomials: `num = [1]; den = [1 1]; G = tf(num, den);`

**6️⃣ First-Order Control System Simulation with Examples**

* Transfer function: G(s) = 1 / (s + 1) (Example: RC circuit response)
* MATLAB code:

```matlab
s = tf('s');
G = 1/(s+1);
step(G);
title('Step Response of First-Order System (RC Circuit)');
xlabel('Time (s)'); ylabel('Voltage');
```

* Another example: First-order thermal system (room heater)

```matlab
s = tf('s');
K = 5; tau = 10;
G = K/(tau*s + 1);
step(G);
title('Step Response of Room Temperature System');
xlabel('Time (s)'); ylabel('Temperature Change');
```

* Observe rise time, settling time, and steady-state response for both electrical and thermal systems.

**7️⃣ Next Steps:**

* Experiment with different first-order systems by changing parameters.
* Prepare for multi-order system modeling and analysis in Lecture 2.
