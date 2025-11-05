🧭 Lecture: Transfer Function Models of Real-World Systems with Practical Parameters and MATLAB Simulation

(Mechanical, Electrical, Thermal, Fluid, and Power System Domains)


---

⚙️ 1. DC Motor – Armature Controlled

📘 Ref: Ogata, Modern Control Engineering (Ex. 2.8)

Parameter	Symbol	Typical Value	Unit

Armature Resistance	R	1.5	Ω
Armature Inductance	L	0.015	H
Moment of Inertia	J	0.02	kg·m²
Viscous Friction	B	0.002	N·m·s/rad
Back EMF Constant	Kₑ	0.05	V·s/rad
Torque Constant	Kₜ	0.05	N·m/A


Transfer Function

T(s) = \frac{Kₜ}{(J s + B)(L s + R) + KₑKₜ}

MATLAB Code

R = 1.5; L = 0.015;
J = 0.02; B = 0.002;
Ke = 0.05; Kt = 0.05;

num = [Kt];
den = [J*L, (J*R + B*L), (B*R + Ke*Kt)];
G_dc = tf(num, den)

step(G_dc)
stepinfo(G_dc)
pzmap(G_dc)

📈 Observation: Second-order overdamped; τ ≈ 0.15–0.2 s.


---

🧲 2. Mass–Spring–Damper System

📘 Ref: Nise, Control Systems Engineering Ch. 2

Parameter	Symbol	Typical Value	Unit

Mass	M	5	kg
Damping Coeff.	B	20	N·s/m
Spring Constant	K	1000	N/m


Transfer Function

T(s) = \frac{1}{M s^2 + B s + K}

MATLAB Code

M = 5; B = 20; K = 1000;
num = [1];
den = [M, B, K];
G_mech = tf(num, den)
step(G_mech)
damp(G_mech)

📈 Observation: Underdamped (ζ ≈ 0.45), oscillatory response.


---

🔌 3. Series RLC Circuit

📘 Ref: Dorf & Bishop, Modern Control Systems

Parameter	Symbol	Typical Value	Unit

Resistance	R	10	Ω
Inductance	L	0.5	H
Capacitance	C	100 µF	F


Transfer Function

T(s) = \frac{1}{L C s^2 + R C s + 1}

MATLAB Code

R = 10; L = 0.5; C = 100e-6;
num = [1];
den = [L*C, R*C, 1];
G_rlc = tf(num, den)
bode(G_rlc)
step(G_rlc)

📈 Observation: ωₙ ≈ 447 rad/s (≈ 71 Hz); damped oscillations.


---

🌡️ 4. Thermal System (Heater + Room)

📘 Ref: Ogata, Thermal Systems Ex. 2.6

Parameter	Symbol	Typical Value	Unit

Thermal Resistance	R	5	°C/W
Thermal Capacitance	C	200	J/°C


Transfer Function

T(s) = \frac{1}{R C s + 1}

MATLAB Code

R = 5; C = 200;
num = [1]; den = [R*C, 1];
G_thermal = tf(num, den)
step(G_thermal)

📈 Observation: 1st-order response; τ = 1000 s (~16.7 min).


---

💧 5. Liquid Level (Tank) System

📘 Ref: Ogata, Fluid Systems Example

Parameter	Symbol	Typical Value	Unit

Cross-sectional Area	A	0.5	m²
Resistance to Flow	R	200	s/m²


Transfer Function

T(s) = \frac{R}{A R s + 1}

MATLAB Code

A = 0.5; R = 200;
num = [R]; den = [A*R, 1];
G_tank = tf(num, den)
step(G_tank)

📈 Observation: τ = 100 s; sluggish response.


---

🚗 6. Vehicle Suspension (Quarter Car)

📘 Ref: Nise, Ex. 2.7

Parameter	Symbol	Typical Value	Unit

Sprung Mass	M	250	kg
Stiffness	K	15 000	N/m
Damping Coeff.	B	1000	N·s/m


Transfer Function

T(s) = \frac{B s + K}{M s^2 + B s + K}

MATLAB Code

M = 250; B = 1000; K = 15000;
num = [B, K];
den = [M, B, K];
G_susp = tf(num, den)
step(G_susp)

📈 Observation: ζ ≈ 0.4; damped oscillations.


---

⚙️ 7. Servo Motor with Tachometer Feedback

📘 Ref: Ogata, Servo Mechanisms

Parameter	Symbol	Typical Value	Unit

Amplifier Gain	Kₐ	10	—
Motor Gain	Kₘ	0.05	—
Tachometer Gain	Kₜ	0.1	—
Motor Time Constant	Tₘ	0.05	s


Transfer Function

T(s) = \frac{Kₐ Kₘ}{s (Tₘ s + 1) + Kₐ Kₘ Kₜ}

MATLAB Code

Ka = 10; Km = 0.05; Kt = 0.1; Tm = 0.05;
num = [Ka*Km];
den = [Tm, 1, Ka*Km*Kt];
G_servo = tf(num, den)
step(G_servo)

📈 Observation: Well-damped; fast (~0.05 s) response.


---

⚡ POWER SYSTEM MODELS


---

⚡ 8. Synchronous Machine – Swing Equation

📘 Ref: Kundur, Power System Stability and Control Ch. 3

Parameter	Symbol	Value	Unit

Inertia Constant	H	3.5	MJ/MVA
Damping Coeff.	D	0.5	pu torque/pu speed
Sync. Speed	ω₀	314	rad/s


\frac{Δω(s)}{ΔT_m(s)} = \frac{1}{2Hs + D}

MATLAB Code

H = 3.5; D = 0.5;
num = [1]; den = [2*H, D];
G_swing = tf(num, den)
step(G_swing)

📈 Observation: First-order lag; τ = 14 s.


---

⚙️ 9. Automatic Voltage Regulator (AVR)

📘 Ref: Anderson & Fouad, Power System Control and Stability

Parameter	Symbol	Value	Unit

Amplifier Gain	Kₐ	10	—
Amplifier Time	Tₐ	0.1	s
Exciter Gain	Kₑ	1	—
Exciter Time	Tₑ	0.4	s
Sensor Time	Tₛ	0.01	s


G_{AVR}(s) = \frac{KₐKₑ}{(Tₐs + 1)(Tₑs + 1)(Tₛs + 1)}

MATLAB Code

Ka = 10; Ta = 0.1; Ke = 1; Te = 0.4; Ts = 0.01;
num = [Ka*Ke];
den = conv(conv([Ta 1],[Te 1]),[Ts 1]);
G_avr = tf(num, den)
step(G_avr)

📈 Observation: Third-order; fast (0.1–0.4 s) voltage loop.


---

💨 10. Turbine–Governor System (Steam)

📘 Ref: Kundur, Ch. 8

Parameter	Symbol	Value	Unit

Governor Time	T_g	0.2	s
Turbine Time	T_t	0.5	s
Gain	K_g	1	—


G_{TG}(s) = \frac{K_g (T_t s + 1)}{(T_g s + 1)(T_t s + 1)}

MATLAB Code

Tg = 0.2; Tt = 0.5; Kg = 1;
num = [Kg*Tt, Kg];
den = conv([Tg 1],[Tt 1]);
G_tg = tf(num, den)
step(G_tg)

📈 Observation: Second-order overdamped; slow frequency control.


---

🌐 11. Transmission Line (π Model Approx.)

📘 Ref: Stevenson, Elements of Power System Analysis

Parameter	Symbol	Value	Unit

Inductance	L	0.001	H/km
Capacitance	C	12e-9	F/km
Resistance	R	0.05	Ω/km


T(s) = \frac{1}{L C s^2 + R C s + 1}

MATLAB Code

R = 0.05; L = 0.001; C = 12e-9;
num = [1]; den = [L*C, R*C, 1];
G_line = tf(num, den)
bode(G_line)

📈 Observation: High-frequency RLC behavior; resonance ~3–5 kHz/km.


---

🔋 12. Excitation System with Feedback Stabilizer

📘 Ref: IEEE Std 421.5 (Type ST1A)

Parameter	Symbol	Value	Unit

Gain	K	200	—
Time Constant	T₁	0.02	s
Stabilizer Gain	K_f	0.05	—
Stabilizer Time	T_f	1	s


G_{exc}(s) = \frac{K(1 + T₁s)}{(T₁s + 1)(1 + K_f T_f s)}

MATLAB Code

K = 200; T1 = 0.02; Kf = 0.05; Tf = 1;
num = K*[T1 1];
den = conv([T1 1],[Kf*Tf 1]);
G_exc = tf(num, den)
step(G_exc)

📈 Observation: Fast voltage response (~0.05 s); stabilizer reduces oscillation.


---

🧾 Comprehensive Comparative Summary

#	System	Order	Domain	τ (Dominant Time Constant)	Nature	Remarks

1	DC Motor	2	Electromechanical	0.15 s	Overdamped	Smooth rise
2	Mass–Spring–Damper	2	Mechanical	—	Underdamped	Oscillatory
3	RLC Circuit	2	Electrical	—	Resonant	Damped oscillation
4	Thermal	1	Thermal	1000 s	Slow	Large RC constant
5	Tank	1	Fluid	100 s	Slow	Sluggish fill
6	Suspension	2	Mechanical	0.1 s	Underdamped	Bouncy
7	Servo + Tachometer	2	Electromechanical	0.05 s	Well-damped	Fast servo loop
8	Synchronous Machine	1	Power	14 s	Slow	Rotor inertia effect
9	AVR	3	Power	0.1–0.4 s	Fast	Voltage regulation
10	Turbine–Governor	2	Power	0.2–0.5 s	Slow	Frequency response
11	Transmission Line	2	Electrical	μs range	Resonant	Distributed parameter
12	Excitation System	2	Power	0.02–1 s	Fast	Stabilized voltage loop



---

🧠 Follow-Up Quiz

1. Why does the thermal system respond much slower than an RLC circuit?


2. How does increasing viscous damping (B) affect poles in a mass–spring–damper system?


3. In a DC motor, which parameters influence overshoot and steady-state speed?


4. How to make an RLC circuit critically damped?


5. Two tanks in series yield what order of transfer function?


6. What happens to rotor speed response if inertia (H) is increased in a synchronous machine?


7. How does increasing Kₐ in an AVR affect system damping and stability?




---

✅ Answers

1. Thermal systems have large R × C → huge time constants.


2. Larger B moves poles left → more damping, less oscillation.


3.  and  affect overshoot;  and  affect steady-state speed.


4. Choose R so ζ = 1 (critical damping condition).


5. Two first-order tanks → second-order system.


6. Higher H → slower speed deviation response.


7. Larger Kₐ → faster AVR but reduced stability margin → possible oscillations.




---

Would you like me to add renewable-energy dynamic models next — e.g. PV inverter control, wind-turbine pitch control, and microgrid droop controllers — so the lecture fully covers smart-grid and renewable power dynamics?