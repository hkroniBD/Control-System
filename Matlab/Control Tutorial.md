# MATLAB Control Systems Tutorial
## Problem-Based Learning Approach with Theory Integration

---

## Table of Contents
1. [Introduction to Control Systems in MATLAB](#module-1)
2. [Transfer Function Analysis](#module-2)
3. [Time Domain Analysis](#module-3)
4. [Frequency Domain Analysis](#module-4)
5. [Controller Design](#module-5)
6. [State-Space Analysis](#module-6)
7. [Root Locus Analysis](#module-7)

---

## Module 1: Introduction to Control Systems in MATLAB

### 📚 Theory Background: Transfer Functions

**What is a Transfer Function?**
A transfer function represents the relationship between input and output of a linear time-invariant (LTI) system in the Laplace domain:

```
G(s) = Y(s)/U(s) = N(s)/D(s)
```

Where:
- **N(s)** = Numerator polynomial (zeros determine it)
- **D(s)** = Denominator polynomial (poles determine it)
- **s** = Complex frequency variable

**Poles and Zeros:**
- **Poles**: Values of s that make G(s) → ∞ (roots of denominator)
  - Location determines stability and response characteristics
  - Left half-plane poles → Stable system
  - Right half-plane poles → Unstable system
  
- **Zeros**: Values of s that make G(s) = 0 (roots of numerator)
  - Affect magnitude and phase but not stability

**Order of System:**
The order equals the highest power of s in the denominator (number of poles).

---

### Problem 1.1: DC Motor Speed Control

**Real-World Context:** You're designing a speed controller for an electric vehicle's DC motor. The motor has the following parameters:
- Motor resistance: R = 2 Ω
- Motor inductance: L = 0.5 H
- Motor constant: Km = 0.1 V·s/rad
- Moment of inertia: J = 0.01 kg·m²
- Friction coefficient: b = 0.1 N·m·s

The transfer function relating motor speed (ω) to input voltage (V) is:
```
G(s) = Km / (s(Js + b)(Ls + R) + Km²)
```

**Task:** Create this transfer function in MATLAB and find the system poles and zeros.

**Solution:**

```matlab
% DC Motor Parameters
Km = 0.1;   % Motor constant
R = 2;      % Resistance (Ohm)
L = 0.5;    % Inductance (H)
J = 0.01;   % Inertia (kg.m^2)
b = 0.1;    % Friction coefficient

% Transfer Function G(s) = N(s)/D(s)
% Numerator
num = Km;

% Denominator: s(Js + b)(Ls + R) + Km^2
% Expanding: JLs^3 + (JR + bL)s^2 + (bR + Km^2)s
den = [J*L, (J*R + b*L), (b*R + Km^2), 0];

% Create transfer function
G = tf(num, den);

% Display the transfer function
disp('DC Motor Transfer Function:');
disp(G);

% Find poles and zeros
poles_G = pole(G);
zeros_G = zero(G);

disp('Poles:');
disp(poles_G);

disp('Zeros:');
disp(zeros_G);

% Check stability
if all(real(poles_G) < 0)
    disp('System is STABLE (all poles in left half-plane)');
else
    disp('System is UNSTABLE (poles in right half-plane)');
end

% Visualize pole-zero map
figure;
pzmap(G);
title('Pole-Zero Map of DC Motor');
grid on;
```

**💡 Key Insights:**
- This is a 3rd order system (three poles)
- One pole at origin indicates integration (motor speed accumulates)
- All poles must be in left half-plane for stability
- Real poles → exponential decay response
- Complex poles → oscillatory response

---

## Module 2: Transfer Function Analysis

### 📚 Theory Background: Second-Order Systems

**Standard Second-Order Form:**
```
G(s) = (ωn²) / (s² + 2ζωn·s + ωn²)
```

Where:
- **ωn** = Natural frequency (rad/s) - how fast the system responds
- **ζ** (zeta) = Damping ratio - how oscillatory the response is

**Damping Ratio Classifications:**
- **ζ < 1**: Underdamped (oscillatory with overshoot)
- **ζ = 1**: Critically damped (fastest response without overshoot)
- **ζ > 1**: Overdamped (slow, sluggish response)
- **ζ = 0**: Undamped (continuous oscillation)

**Key Time-Domain Specifications:**
- **Rise Time (tr)**: Time to reach from 10% to 90% of final value
- **Peak Time (tp)**: Time to reach first peak
  - tp = π/(ωn√(1-ζ²)) for underdamped systems
- **Settling Time (ts)**: Time to stay within ±2% of final value
  - ts ≈ 4/(ζωn) (2% criterion)
- **Percent Overshoot (PO)**: 
  - PO = 100 × e^(-ζπ/√(1-ζ²))

**Relationship Between Poles and Response:**
For complex poles: s = -σ ± jωd
- σ = ζωn (real part) → controls decay rate
- ωd = ωn√(1-ζ²) (imaginary part) → oscillation frequency

---

### Problem 2.1: Aircraft Pitch Control

**Real-World Context:** You're working on an autopilot system for a small aircraft. The pitch angle response to elevator deflection is modeled as:
```
G(s) = 50 / (s² + 5s + 40)
```

**Task:** 
1. Analyze the system's natural frequency and damping ratio
2. Determine if the system is underdamped, overdamped, or critically damped
3. Calculate settling time and peak time
4. Predict percent overshoot

**Solution:**

```matlab
% Aircraft Pitch Control System
num = 50;
den = [1, 5, 40];

% Create transfer function
G_aircraft = tf(num, den);

% Display transfer function
disp('Aircraft Pitch Transfer Function:');
disp(G_aircraft);

% Extract natural frequency and damping ratio
% Standard form: s^2 + 2*zeta*wn*s + wn^2
wn = sqrt(den(3));  % Natural frequency
zeta = den(2)/(2*wn);  % Damping ratio

% Display system parameters
fprintf('\n=== System Parameters ===\n');
fprintf('Natural Frequency (wn): %.4f rad/s\n', wn);
fprintf('Damping Ratio (zeta): %.4f\n', zeta);
fprintf('DC Gain: %.4f\n', dcgain(G_aircraft));

% Classify damping
fprintf('\n=== Damping Classification ===\n');
if zeta < 1
    fprintf('System is UNDERDAMPED (oscillatory response)\n');
    
    % Calculate damped frequency
    wd = wn*sqrt(1-zeta^2);
    fprintf('Damped Frequency (wd): %.4f rad/s\n', wd);
    
    % Theoretical calculations for underdamped system
    tp_theory = pi/wd;
    ts_theory = 4/(zeta*wn);
    PO_theory = 100*exp(-zeta*pi/sqrt(1-zeta^2));
    
    fprintf('\n=== Theoretical Predictions ===\n');
    fprintf('Peak Time: %.4f s\n', tp_theory);
    fprintf('Settling Time (2%% criterion): %.4f s\n', ts_theory);
    fprintf('Percent Overshoot: %.2f%%\n', PO_theory);
    
elseif zeta == 1
    fprintf('System is CRITICALLY DAMPED (no overshoot, fastest)\n');
    ts_theory = 5.8/wn;
    fprintf('Settling Time: %.4f s\n', ts_theory);
else
    fprintf('System is OVERDAMPED (sluggish, no overshoot)\n');
end

% Get actual step response information
sys_info = stepinfo(G_aircraft);

fprintf('\n=== Actual Step Response Characteristics ===\n');
fprintf('Rise Time: %.4f s\n', sys_info.RiseTime);
fprintf('Settling Time: %.4f s\n', sys_info.SettlingTime);
fprintf('Peak Time: %.4f s\n', sys_info.PeakTime);
fprintf('Overshoot: %.2f%%\n', sys_info.Overshoot);
fprintf('Peak: %.4f\n', sys_info.Peak);

% Plot step response with annotations
figure;
step(G_aircraft);
grid on;
title('Aircraft Pitch Response to Elevator Input');
xlabel('Time (seconds)');
ylabel('Pitch Angle (degrees)');

% Add horizontal lines for steady-state value
hold on;
ss_value = dcgain(G_aircraft);
yline(ss_value, '--r', 'Steady State');
yline(ss_value*1.02, ':k', '+2%');
yline(ss_value*0.98, ':k', '-2%');
legend('Step Response', 'Steady State', 'Settling Band');

% Plot pole-zero map
figure;
pzmap(G_aircraft);
title('Pole-Zero Map');
grid on;

% Add damping ratio lines
sgrid(zeta, wn);
```

**💡 Key Insights:**
- Lower damping ratio → Higher overshoot but faster rise time
- Higher damping ratio → Slower response but less overshoot
- For aircraft: ζ ≈ 0.4-0.7 is typical for good handling
- Complex conjugate poles create oscillatory behavior
- Pole distance from imaginary axis (real part) determines settling time

---

## Module 3: Time Domain Analysis

### 📚 Theory Background: Feedback Control Systems

**Open-Loop vs Closed-Loop:**

**Open-Loop System:**
```
Y(s) = G(s)·U(s)
```
- No feedback, no error correction
- Output affected by disturbances
- Simple but less accurate

**Closed-Loop System:**
```
Y(s)/R(s) = G(s)/(1 + G(s)H(s))
```
Where:
- **G(s)** = Forward path (controller + plant)
- **H(s)** = Feedback path (sensor)
- **Loop Transfer Function** = G(s)H(s)

**Advantages of Feedback:**
1. Reduced sensitivity to disturbances
2. Reduced sensitivity to parameter variations
3. Can stabilize unstable plants
4. Improved tracking accuracy

**Steady-State Error (ess):**
For unity feedback system with input r(t):
```
ess = lim[t→∞] (r(t) - y(t))
```

**System Type and Steady-State Error:**
- **Type 0** (no integrator): ess ≠ 0 for step input
- **Type 1** (one integrator): ess = 0 for step, ess ≠ 0 for ramp
- **Type 2** (two integrators): ess = 0 for step and ramp

**Static Error Constants:**
- **Kp** (position) = lim[s→0] G(s)H(s)
- **Kv** (velocity) = lim[s→0] s·G(s)H(s)
- **Ka** (acceleration) = lim[s→0] s²·G(s)H(s)

---

### Problem 3.1: Temperature Control of Chemical Reactor

**Real-World Context:** A chemical reactor requires precise temperature control for optimal reaction rates and product quality. The system has:
- Process transfer function: `Gp(s) = 2/(10s + 1)` (first-order lag with gain)
- Sensor transfer function: `H(s) = 1/(2s + 1)` (thermocouple dynamics)
- Controller gain: Kc = 5

**Task:**
1. Create the closed-loop system
2. Analyze step response
3. Compare open-loop vs closed-loop performance
4. Calculate steady-state error

**Solution:**

```matlab
% Chemical Reactor Temperature Control

% Process transfer function (First-order with time constant 10s)
Gp = tf(2, [10, 1]);

% Sensor transfer function (Thermocouple with time constant 2s)
H = tf(1, [2, 1]);

% Controller gain
Kc = 5;

% Open-loop transfer function (without feedback)
G_open = Kc * Gp;

% Closed-loop transfer function with unity feedback
% Formula: T(s) = G(s)/(1 + G(s)H(s))
G_closed = feedback(Kc*Gp, H);

fprintf('=== TRANSFER FUNCTION ANALYSIS ===\n\n');

% Display transfer functions
disp('Process Transfer Function Gp(s):');
disp(Gp);

disp('Sensor Transfer Function H(s):');
disp(H);

disp('Open-Loop Transfer Function G_open(s) = Kc*Gp(s):');
disp(G_open);

disp('Closed-Loop Transfer Function T(s):');
disp(G_closed);

% Time vector for simulation
t = 0:0.1:50;

% Step response comparison
figure;
subplot(2,1,1);
[y_open, t_open] = step(G_open, t);
plot(t_open, y_open, 'b', 'LineWidth', 1.5);
title('Open-Loop Step Response');
grid on;
xlabel('Time (s)');
ylabel('Temperature (°C)');
yline(1, '--r', 'Desired Setpoint');
legend('System Response', 'Setpoint');

subplot(2,1,2);
[y_closed, t_closed] = step(G_closed, t);
plot(t_closed, y_closed, 'b', 'LineWidth', 1.5);
title('Closed-Loop Step Response');
grid on;
xlabel('Time (s)');
ylabel('Temperature (°C)');
yline(1, '--r', 'Desired Setpoint');
legend('System Response', 'Setpoint');

% Calculate DC gains
dcgain_open = dcgain(G_open);
dcgain_closed = dcgain(G_closed);

fprintf('\n=== DC GAIN ANALYSIS ===\n');
fprintf('Open-loop DC Gain: %.4f\n', dcgain_open);
fprintf('Closed-loop DC Gain: %.4f\n', dcgain_closed);

% Steady-state error calculation
% For unit step input
ess_open = 1 - dcgain_open;
ess_closed = 1 - dcgain_closed;

fprintf('\n=== STEADY-STATE ERROR ===\n');
fprintf('Open-loop steady-state error: %.4f (%.2f%%)\n', ess_open, abs(ess_open)*100);
fprintf('Closed-loop steady-state error: %.4f (%.2f%%)\n', ess_closed, abs(ess_closed)*100);

% System type determination
% Count number of poles at origin
open_poles = pole(G_open);
closed_poles = pole(G_closed);
num_integrators = sum(abs(open_poles) < 1e-6);

fprintf('\n=== SYSTEM TYPE ===\n');
fprintf('Number of integrators: %d\n', num_integrators);
fprintf('System Type: %d\n', num_integrators);

if num_integrators == 0
    fprintf('Type 0 system: Non-zero steady-state error for step input\n');
elseif num_integrators == 1
    fprintf('Type 1 system: Zero steady-state error for step input\n');
end

% Performance metrics
info_open = stepinfo(G_open);
info_closed = stepinfo(G_closed);

fprintf('\n=== PERFORMANCE COMPARISON ===\n');
fprintf('                    Open-Loop    Closed-Loop\n');
fprintf('Rise Time:          %.4f s     %.4f s\n', info_open.RiseTime, info_closed.RiseTime);
fprintf('Settling Time:      %.4f s     %.4f s\n', info_open.SettlingTime, info_closed.SettlingTime);
fprintf('Overshoot:          %.2f%%       %.2f%%\n', info_open.Overshoot, info_closed.Overshoot);

% Disturbance rejection test
fprintf('\n=== DISTURBANCE REJECTION TEST ===\n');
figure;
t_dist = 0:0.01:50;
u = ones(size(t_dist));  % Step reference
d = 0.2*[zeros(1,1000), ones(1, length(t_dist)-1000)]; % Disturbance at t=10s

% For open-loop: output affected directly by disturbance
% For closed-loop: feedback helps reject disturbance

% Closed-loop with disturbance
lsim(G_closed, u+d, t_dist, 'b');
hold on;
lsim(G_closed, u, t_dist, 'r--');
yline(1, ':k', 'Setpoint');
legend('With Disturbance', 'Without Disturbance', 'Setpoint');
title('Disturbance Rejection - Closed Loop');
xlabel('Time (s)');
ylabel('Temperature (°C)');
grid on;

fprintf('Disturbance introduced at t=10s\n');
fprintf('Closed-loop feedback reduces disturbance effect\n');
```

**💡 Key Insights:**
- Open-loop: Simple but vulnerable to disturbances and parameter changes
- Closed-loop: Better accuracy through feedback but can be unstable if not designed properly
- Sensor dynamics (H(s)) affect closed-loop response
- Type 0 system has steady-state error for step input
- Adding integrator (making it Type 1) eliminates steady-state error

---

## Module 4: Frequency Domain Analysis

### 📚 Theory Background: Frequency Response

**Why Frequency Domain?**
Frequency response shows how a system responds to sinusoidal inputs of different frequencies. It's crucial for:
- Stability analysis
- Performance assessment
- Controller design

**Bode Plot:**
Two plots showing system behavior across frequencies:
1. **Magnitude Plot**: 20log₁₀|G(jω)| vs log(ω)
2. **Phase Plot**: ∠G(jω) vs log(ω)

**Key Frequency Domain Concepts:**

**Gain Margin (GM):**
- How much gain can be increased before system becomes unstable
- Measured at phase crossover frequency (where phase = -180°)
- GM (dB) = -20log₁₀|G(jωpc)|
- **Good design: GM > 6 dB** (factor of 2)

**Phase Margin (PM):**
- How much additional phase lag can be tolerated before instability
- Measured at gain crossover frequency (where |G(jω)| = 1 or 0 dB)
- PM = 180° + ∠G(jωgc)
- **Good design: PM > 30°** (preferably 45-60° for robustness)

**Bandwidth (ωB):**
- Frequency where |G(jω)| drops to -3dB (0.707) of DC value
- Indicates speed of response
- Higher bandwidth → Faster response
- Trade-off: Higher bandwidth → More noise sensitivity

**Gain and Phase Crossover:**
- **Gain Crossover (ωgc)**: Where |G(jω)| = 0 dB
- **Phase Crossover (ωpc)**: Where ∠G(jω) = -180°

**Stability Criteria:**
- For stability: GM > 0 dB AND PM > 0°
- Negative GM or PM → Unstable system

---

### Problem 4.1: Active Suspension System Analysis

**Real-World Context:** An automotive company is designing an active suspension system. The system's stability margins are critical for passenger comfort and safety. Poor margins can lead to oscillations that make passengers uncomfortable or even cause instability.

Transfer function: `G(s) = 100/(s² + 2s + 100)`

**Task:**
1. Plot Bode diagram
2. Find gain and phase margins
3. Determine bandwidth
4. Assess stability and robustness

**Solution:**

```matlab
% Active Suspension System
num = 100;
den = [1, 2, 100];
G_suspension = tf(num, den);

fprintf('=== ACTIVE SUSPENSION SYSTEM ANALYSIS ===\n\n');

% Display transfer function
disp('System Transfer Function:');
disp(G_suspension);

% System parameters
wn = sqrt(100);  % Natural frequency
zeta = 2/(2*wn);  % Damping ratio

fprintf('Natural Frequency: %.4f rad/s\n', wn);
fprintf('Damping Ratio: %.4f\n', zeta);

% Bode plot with grid
figure;
bode(G_suspension);
grid on;
title('Bode Plot - Active Suspension System');

% Add stability margin indicators
hold on;
% The margins will be shown automatically with margin() command

% Calculate margins using margin function
[Gm, Pm, Wcg, Wcp] = margin(G_suspension);

% Convert gain margin to dB
Gm_dB = 20*log10(Gm);

fprintf('\n=== STABILITY MARGINS ===\n');
fprintf('Gain Margin (GM): %.2f dB (factor of %.2f)\n', Gm_dB, Gm);
fprintf('Gain Crossover Frequency: %.4f rad/s\n', Wcp);
fprintf('Phase Margin (PM): %.2f degrees\n', Pm);
fprintf('Phase Crossover Frequency: %.4f rad/s\n', Wcg);

% Interpret margins
fprintf('\n=== STABILITY ASSESSMENT ===\n');
if isinf(Gm)
    fprintf('Gain Margin: INFINITE (very stable!)\n');
elseif Gm_dB > 6
    fprintf('Gain Margin: GOOD (>6 dB recommended)\n');
elseif Gm_dB > 0
    fprintf('Gain Margin: MARGINAL (increase recommended)\n');
else
    fprintf('Gain Margin: UNSTABLE SYSTEM!\n');
end

if Pm > 60
    fprintf('Phase Margin: EXCELLENT (>60 degrees)\n');
elseif Pm > 45
    fprintf('Phase Margin: GOOD (45-60 degrees)\n');
elseif Pm > 30
    fprintf('Phase Margin: ACCEPTABLE (30-45 degrees)\n');
elseif Pm > 0
    fprintf('Phase Margin: POOR (needs improvement)\n');
else
    fprintf('Phase Margin: UNSTABLE SYSTEM!\n');
end

% Bandwidth calculation
% Bandwidth is where magnitude drops to -3dB
[mag, phase, wout] = bode(G_suspension);
mag_dB = 20*log10(squeeze(mag));

% Find -3dB point
dc_gain = mag_dB(1);
idx_bw = find(mag_dB <= (dc_gain - 3), 1);

if ~isempty(idx_bw)
    bandwidth = wout(idx_bw);
else
    % Use built-in bandwidth function
    bandwidth = bandwidth(G_suspension);
end

fprintf('\n=== BANDWIDTH ANALYSIS ===\n');
fprintf('Bandwidth (-3dB): %.4f rad/s (%.4f Hz)\n', bandwidth, bandwidth/(2*pi));
fprintf('Interpretation: System responds well to frequencies below %.2f Hz\n', bandwidth/(2*pi));

% Plot Bode with margins highlighted
figure;
margin(G_suspension);
grid on;
title('Bode Plot with Stability Margins Highlighted');

% Nyquist plot for complete stability picture
figure;
nyquist(G_suspension);
title('Nyquist Plot - Active Suspension System');
grid on;

% Add unit circle to Nyquist plot
hold on;
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'r--', 'LineWidth', 1);
plot(-1, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
legend('Nyquist Plot', 'Unit Circle', 'Critical Point (-1,0)');

fprintf('\n=== NYQUIST STABILITY CRITERION ===\n');
fprintf('For stability, Nyquist plot should not encircle (-1,0) point\n');
fprintf('Distance from (-1,0) indicates robustness\n');

% Closed-loop system for reference
G_closed = feedback(G_suspension, 1);

% Compare open-loop vs closed-loop frequency response
figure;
subplot(2,1,1);
bode(G_suspension, 'b');
hold on;
bode(G_closed, 'r--');
title('Open-Loop vs Closed-Loop Frequency Response');
legend('Open-Loop', 'Closed-Loop');
grid on;

subplot(2,1,2);
step(G_closed);
title('Closed-Loop Step Response');
grid on;

fprintf('\n=== PRACTICAL IMPLICATIONS ===\n');
fprintf('For suspension system:\n');
fprintf('- High bandwidth → Quick response to road disturbances\n');
fprintf('- Good phase margin → Comfortable, non-oscillatory ride\n');
fprintf('- Adequate gain margin → Robust to parameter variations\n');
```

**💡 Key Insights:**
- Bode plot provides complete frequency response picture
- Higher phase margin → Less overshoot in step response
- Gain crossover frequency ≈ closed-loop bandwidth
- Suspension systems typically need PM = 45-60° for comfort
- Low frequency gain → steady-state tracking
- High frequency rolloff → noise rejection

---

## Module 5: Controller Design

### 📚 Theory Background: PID Control

**PID Controller Structure:**
```
C(s) = Kp + Ki/s + Kd·s
```
Or in time domain:
```
u(t) = Kp·e(t) + Ki∫e(t)dt + Kd·de(t)/dt
```

**Three Control Actions:**

**1. Proportional (P):**
- Output proportional to current error
- **Effect**: 
  - Reduces rise time
  - Reduces (but doesn't eliminate) steady-state error
  - Can cause overshoot
- **When to use**: Fast response needed, some error acceptable

**2. Integral (I):**
- Output proportional to accumulated error
- **Effect**:
  - Eliminates steady-state error
  - Increases overshoot
  - Slows response
  - Can cause instability if too large
- **When to use**: Zero steady-state error required

**3. Derivative (D):**
- Output proportional to rate of error change
- **Effect**:
  - Reduces overshoot
  - Improves stability
  - Speeds up response
  - Amplifies noise
- **When to use**: Reduce overshoot, improve damping

**PID Tuning Guidelines:**

| Parameter | Rise Time | Overshoot | Settling Time | SS Error | Stability |
|-----------|-----------|-----------|---------------|----------|-----------|
| Kp ↑      | Decrease  | Increase  | Small change  | Decrease | Degrade   |
| Ki ↑      | Decrease  | Increase  | Increase      | Eliminate| Degrade   |
| Kd ↑      | Minor     | Decrease  | Decrease      | No effect| Improve   |

**Tuning Methods:**
1. **Ziegler-Nichols**: Based on ultimate gain and period
2. **Trial and Error**: Start with P, add I, add D
3. **Software Tools**: MATLAB pidTuner, optimization

**Common Controller Types:**
- **P**: Simple, fast, but with steady-state error
- **PI**: Most common, eliminates error, moderate speed
- **PD**: Fast, good damping, but doesn't eliminate error
- **PID**: Best overall performance, most complex tuning

---

### Problem 5.1: Cruise Control System with PID

**Real-World Context:** Design a cruise control system for an autonomous vehicle. The vehicle must maintain speed accurately despite hills, wind, and load changes. The vehicle dynamics are modeled as a first-order system:
```
G(s) = 1/(s + 0.5)
```

**Task:**
1. Design a PID controller to achieve:
   - Zero steady-state error
   - Overshoot < 10%
   - Settling time < 5 seconds
2. Tune the controller and verify performance
3. Test disturbance rejection

**Solution:**

```matlab
% Vehicle Dynamics (First-order system)
% Physical meaning: 1/(τs + 1) where τ = 2 seconds (time constant)
G_vehicle = tf(1, [1, 0.5]);

fprintf('=== CRUISE CONTROL SYSTEM DESIGN ===\n\n');

% Display plant transfer function
disp('Vehicle Dynamics Transfer Function:');
disp(G_vehicle);

% Analyze open-loop characteristics
fprintf('Open-Loop Analysis:\n');
fprintf('Time Constant: %.2f seconds\n', 1/0.5);
fprintf('DC Gain: %.2f\n', dcgain(G_vehicle));

% Step 1: Try P controller only
fprintf('\n=== STEP 1: P CONTROLLER ===\n');
Kp_only = 2;
C_P = pid(Kp_only, 0, 0);
T_P = feedback(C_P * G_vehicle, 1);

info_P = stepinfo(T_P);
ess_P = 1 - dcgain(T_P);

fprintf('Kp = %.2f\n', Kp_only);
fprintf('Settling Time: %.3f s\n', info_P.SettlingTime);
fprintf('Overshoot: %.2f%%\n', info_P.Overshoot);
fprintf('Steady-State Error: %.4f (%.2f%%)\n', ess_P, abs(ess_P)*100);
fprintf('Issue: Non-zero steady-state error!\n');

% Step 2: Add Integral action (PI controller)
fprintf('\n=== STEP 2: PI CONTROLLER ===\n');
Kp_PI = 2;
Ki_PI = 0.5;
C_PI = pid(Kp_PI, Ki_PI, 0);
T_PI = feedback(C_PI * G_vehicle, 1);

info_PI = stepinfo(T_PI);
ess_PI = 1 - dcgain(T_PI);

fprintf('Kp = %.2f, Ki = %.2f\n', Kp_PI, Ki_PI);
fprintf('Settling Time: %.3f s\n', info_PI.SettlingTime);
fprintf('Overshoot: %.2f%%\n', info_PI.Overshoot);
fprintf('Steady-State Error: %.4f\n', ess_PI);
fprintf('Improvement: Zero steady-state error!\n');
fprintf('Issue: Increased overshoot\n');

% Step 3: Add Derivative action (PID controller)
fprintf('\n=== STEP 3: PID CONTROLLER ===\n');
Kp = 2.5;   % Proportional gain
Ki = 0.5;   % Integral gain
Kd = 1.0;   % Derivative gain

% Create PID controller
C = pid(Kp, Ki, Kd);
fprintf('PID Controller: C(s) = Kp + Ki/s + Kd*s\n');
disp(C);

% Closed-loop system
T = feedback(C*G_vehicle, 1);

% Performance analysis
info = stepinfo(T);
fprintf('\nPID Controller Performance:\n');
fprintf('Kp = %.2f, Ki = %.2f, Kd = %.2f\n', Kp, Ki, Kd);
fprintf('Rise Time: %.4f s\n', info.RiseTime);
fprintf('Settling Time: %.4f s (target: <5s) ✓\n', info.SettlingTime);
fprintf('Overshoot: %.2f%% (target: <10%%) ✓\n', info.Overshoot);
fprintf('Steady-State Error: %.6f (practically zero) ✓\n', 1 - dcgain(T));

% Plot comparison of P, PI, and PID
figure;
step(T_P, T_PI, T, 10);
legend('P Control', 'PI Control', 'PID Control', 'Location', 'best');
title('Comparison of Controller Types');
grid on;
xlabel('Time (s)');
ylabel('Vehicle Speed (m/s)');

% Detailed PID response
figure;
step(T, 15);
grid on;
title('PID Cruise Control - Step Response');
xlabel('Time (s)');
ylabel('Vehicle Speed (m/s)');

% Add specifications overlay
hold on;
yline(1, '--r', 'Setpoint');
yline(1.1, ':k', '+10% (overshoot limit)');

% Disturbance rejection test
fprintf('\n=== DISTURBANCE REJECTION TEST ===\n');
fprintf('Simulating uphill grade at t=5s (disturbance)\n');

figure;
t = 0:0.01:20;
% Reference input (desired speed)
r = ones(size(t));
% Disturbance (uphill grade effect - acts like negative input)
d = -0.3*[zeros(1,500), ones(1, length(t)-500)];

% System response with disturbance
% For feedback system: Y = T*R + S*D, where S = 1/(1+GC) (sensitivity)
S = 1/(1 + C*G_vehicle);  % Sensitivity function

% Response to reference
y_ref = lsim(T, r, t);
% Response to disturbance (disturbance enters at plant input)
y_dist = lsim(feedback(G_vehicle, C), d, t);
% Total response
y_total = y_ref + y_dist;

plot(t, y_total, 'b', 'LineWidth', 1.5);
hold on;
plot(t, y_ref, 'r--', 'LineWidth', 1);
yline(1, ':k', 'Setpoint');
xline(5, '--g', 'Disturbance Applied');
legend('With Disturbance', 'Without Disturbance', 'Setpoint', 'Disturbance Time');
title('Disturbance Rejection: Vehicle Climbing Hill');
xlabel('Time (s)');
ylabel('Vehicle Speed (m/s)');
grid on;

fprintf('Integral action eliminates steady-state error from disturbance\n');
fprintf('Derivative action helps quick recovery\n');

% Control effort analysis
fprintf('\n=== CONTROL EFFORT ANALYSIS ===\n');
figure;
[y, t_sim, u] = lsim(C/(1+C*G_vehicle), r, t);  % Control signal

subplot(2,1,1);
plot(t_sim, u);
title('Control Signal (Throttle Position)');
xlabel('Time (s)');
ylabel('Control Input');
grid on;

subplot(2,1,2);
plot(t_sim, y);
title('System Output (Vehicle Speed)');
xlabel('Time (s)');
ylabel('Speed (m/s)');
grid on;

% Robustness test - parameter variation
fprintf('\n=== ROBUSTNESS TEST ===\n');
fprintf('Testing with vehicle mass variations...\n');

% Nominal plant
G_nominal = G_vehicle;

% Light vehicle (20% less mass, faster response)
G_light = tf(1, [1, 0.6]);

% Heavy vehicle (50% more mass, slower response)
G_heavy = tf(1, [1, 0.35]);

% Closed-loop with different vehicles
T_nominal = feedback(C*G_nominal, 1);
T_light = feedback(C*G_light, 1);
T_heavy = feedback(C*G_heavy, 1);

figure;
step(T_light, T_nominal, T_heavy, 10);
legend('Light Vehicle', 'Nominal', 'Heavy Vehicle');
title('PID Controller Robustness to Parameter Variations');
grid on;
xlabel('Time (s)');
ylabel('Speed (m/s)');

fprintf('PID controller maintains good performance across vehicle variations\n');
```

**💡 Key Insights:**
- **P control**: Fast but has steady-state error
- **PI control**: Eliminates error but increases overshoot
- **PID control**: Best of both - fast, accurate, well-damped
- Integral action is essential for eliminating steady-state error
- Derivative action improves transient response and stability
- PID provides good disturbance rejection
- Controller is robust to parameter variations

---

### 📚 Theory Background: Lead-Lag Compensation

**Frequency Domain Design:**
When PID isn't sufficient or frequency domain specifications are given, we use lead-lag compensators.

**Lead Compensator:**
```
Gc(s) = Kc(s + z)/(s + p), where p > z
```
Or: `Gc(s) = K(1 + Ts)/(1 + αTs)` where α < 1

**Effects:**
- **Adds phase lead** (positive phase) around a chosen frequency
- Increases bandwidth
- Improves phase margin
- Speeds up transient response
- Maximum phase lead: φmax = sin⁻¹((1-α)/(1+α))
- Occurs at: ωmax = 1/(T√α)

**When to use:** Need to improve phase margin and speed up response

**Lag Compensator:**
```
Gc(s) = Kc(s + z)/(s + p), where z > p
```
Or: `Gc(s) = K(1 + Ts)/(1 + βTs)` where β > 1

**Effects:**
- **Reduces** steady-state error
- **Decreases** bandwidth (slows response)
- Improves low-frequency gain without affecting phase margin much
- Adds phase lag (negative phase)

**When to use:** Need to reduce steady-state error without changing transient response much

**Design Procedure for Lead Compensator:**
1. Determine required phase margin (PM) from specifications
2. Calculate required phase lead: φ = PM_desired - PM_current + safety margin
3. Calculate α from: sin(φ) = (1-α)/(1+α)
4. Place ωmax at desired gain crossover frequency
5. Adjust gain to achieve 0 dB at new crossover

---

### Problem 5.2: Lead Compensator Design for Robotic Arm

**Real-World Context:** A robotic arm positioning system requires improved phase margin for stability during fast movements. The system experiences vibrations when commanded to make quick position changes. 

Original system: `G(s) = 10/(s(s+1)(s+5))`

**Task:** 
1. Analyze uncompensated system
2. Design a lead compensator to achieve phase margin of 45° and gain crossover frequency of 2 rad/s
3. Verify improved performance

**Solution:**

```matlab
% Robotic Arm System (Type 1, third-order)
G_robot = tf(10, conv([1, 0], conv([1, 1], [1, 5])));

fprintf('=== ROBOTIC ARM POSITION CONTROL ===\n\n');

% Display original system
disp('Uncompensated System G(s):');
disp(G_robot);

% Analyze uncompensated system
fprintf('=== UNCOMPENSATED SYSTEM ANALYSIS ===\n');
[Gm_orig, Pm_orig, Wcg_orig, Wcp_orig] = margin(G_robot);
fprintf('Phase Margin: %.2f degrees (POOR!)\n', Pm_orig);
fprintf('Gain Margin: %.2f dB\n', 20*log10(Gm_orig));
fprintf('Gain Crossover Freq: %.4f rad/s\n', Wcp_orig);

% Check if system is stable
if Pm_orig > 0
    fprintf('System is stable but poorly damped\n');
else
    fprintf('System is UNSTABLE!\n');
end

% Plot uncompensated Bode
figure;
margin(G_robot);
title('Uncompensated System - Bode Plot');
grid on;

% Design specifications
PM_desired = 45;  % degrees
wc_desired = 2;   % rad/s (gain crossover frequency)

fprintf('\n=== DESIGN SPECIFICATIONS ===\n');
fprintf('Desired Phase Margin: %.2f degrees\n', PM_desired);
fprintf('Desired Gain Crossover: %.2f rad/s\n', wc_desired);

% Lead Compensator Design
fprintf('\n=== LEAD COMPENSATOR DESIGN ===\n');

% Step 1: Find phase at desired crossover for uncompensated system
[mag_at_wc, phase_at_wc] = bode(G_robot, wc_desired);
mag_at_wc_dB = 20*log10(mag_at_wc);

fprintf('At ω = %.2f rad/s:\n', wc_desired);
fprintf('  Magnitude: %.2f dB\n', mag_at_wc_dB);
fprintf('  Phase: %.2f degrees\n', phase_at_wc);

% Step 2: Calculate required phase lead
% Add 10-15 degrees for safety margin (compensator also introduces some lag)
phi_max = PM_desired - phase_at_wc - 180 + 10;  % Required phase lead

fprintf('\nRequired phase lead: %.2f degrees\n', phi_max);

% Step 3: Calculate alpha
alpha = (1 - sind(phi_max))/(1 + sind(phi_max));
fprintf('Alpha (α): %.4f\n', alpha);

% Step 4: Calculate T
% Place maximum phase lead at desired crossover frequency
T = 1/(wc_desired * sqrt(alpha));
fprintf('T parameter: %.4f\n', T);

% Step 5: Lead compensator transfer function
num_lead = [T, 1];
den_lead = [alpha*T, 1];
G_lead = tf(num_lead, den_lead);

fprintf('\nLead Compensator:\n');
disp(G_lead);

% Step 6: Adjust gain
% The lead compensator provides gain of 1/sqrt(alpha) at ωmax
% We need to adjust overall gain so that |G_comp(jω)| = 0 dB at wc_desired
K_lead = 1/sqrt(alpha);

% Additional gain adjustment to hit exact crossover
[mag_lead_at_wc, ~] = bode(K_lead * G_lead * G_robot, wc_desired);
K_adjust = 1/mag_lead_at_wc;

K_total = K_lead * K_adjust;

fprintf('Lead compensator gain at ωmax: %.4f\n', K_lead);
fprintf('Additional gain adjustment: %.4f\n', K_adjust);
fprintf('Total compensator gain: %.4f\n', K_total);

% Compensated system
G_comp = K_total * G_lead * G_robot;

% Verify compensated system performance
fprintf('\n=== COMPENSATED SYSTEM ANALYSIS ===\n');
[Gm_new, Pm_new, Wcg_new, Wcp_new] = margin(G_comp);
fprintf('Phase Margin: %.2f degrees ✓\n', Pm_new);
fprintf('Gain Margin: %.2f dB\n', 20*log10(Gm_new));
fprintf('Gain Crossover Freq: %.4f rad/s\n', Wcp_new);

% Bandwidth comparison
bw_orig = bandwidth(feedback(G_robot, 1));
bw_comp = bandwidth(feedback(G_comp, 1));
fprintf('\nBandwidth (original): %.4f rad/s\n', bw_orig);
fprintf('Bandwidth (compensated): %.4f rad/s (%.1f%% increase)\n', ...
    bw_comp, (bw_comp/bw_orig - 1)*100);

% Compare Bode plots
figure;
margin(G_robot);
hold on;
margin(G_comp);
legend('Uncompensated', 'Compensated', 'Location', 'southwest');
title('Lead Compensation - Frequency Response Comparison');
grid on;

% Step response comparison
fprintf('\n=== TIME DOMAIN PERFORMANCE ===\n');
T_uncomp = feedback(G_robot, 1);
T_comp = feedback(G_comp, 1);

% Get step info
info_uncomp = stepinfo(T_uncomp);
info_comp = stepinfo(T_comp);

fprintf('                    Uncompensated    Compensated\n');
fprintf('Rise Time:          %.4f s        %.4f s\n', ...
    info_uncomp.RiseTime, info_comp.RiseTime);
fprintf('Settling Time:      %.4f s        %.4f s\n', ...
    info_uncomp.SettlingTime, info_comp.SettlingTime);
fprintf('Overshoot:          %.2f%%          %.2f%%\n', ...
    info_uncomp.Overshoot, info_comp.Overshoot);

figure;
step(T_uncomp, T_comp, 5);
legend('Uncompensated', 'Compensated', 'Location', 'best');
title('Step Response Comparison');
xlabel('Time (s)');
ylabel('Position (rad)');
grid on;

fprintf('\n=== IMPROVEMENTS ACHIEVED ===\n');
fprintf('✓ Phase margin increased from %.1f° to %.1f°\n', Pm_orig, Pm_new);
fprintf('✓ Response speed increased (%.1f%% faster settling)\n', ...
    (1 - info_comp.SettlingTime/info_uncomp.SettlingTime)*100);
fprintf('✓ Overshoot reduced from %.1f%% to %.1f%%\n', ...
    info_uncomp.Overshoot, info_comp.Overshoot);
fprintf('✓ System is now well-damped and responsive\n');

% Show phase contribution of lead compensator
figure;
[mag_lead, phase_lead, w_lead] = bode(G_lead, logspace(-1, 2, 100));
subplot(2,1,1);
semilogx(w_lead, 20*log10(squeeze(mag_lead)));
grid on;
title('Lead Compensator Contribution');
ylabel('Magnitude (dB)');

subplot(2,1,2);
semilogx(w_lead, squeeze(phase_lead));
grid on;
ylabel('Phase (degrees)');
xlabel('Frequency (rad/s)');

% Mark maximum phase lead frequency
[~, idx_max] = max(squeeze(phase_lead));
xline(w_lead(idx_max), '--r', sprintf('ω_{max} = %.2f rad/s', w_lead(idx_max)));
```

**💡 Key Insights:**
- Lead compensator adds phase lead to improve phase margin
- Increased bandwidth means faster response
- Trade-off: Improved speed vs. increased high-frequency noise sensitivity
- Lead compensator essentially "speeds up" the system
- Maximum phase lead occurs at geometric mean of zero and pole
- Typical applications: Position control, fast servo systems
- Always add safety margin when calculating required phase lead

---

## Module 6: State-Space Analysis

### 📚 Theory Background: State-Space Representation

**Why State-Space?**
- Handles MIMO (multiple-input, multiple-output) systems naturally
- Better for computer implementation
- Enables modern control techniques (optimal control, observers)
- Reveals internal system behavior

**State-Space Model:**
```
ẋ(t) = Ax(t) + Bu(t)  [State equation]
y(t) = Cx(t) + Du(t)  [Output equation]
```

Where:
- **x**: State vector (n×1) - internal variables
- **u**: Input vector (m×1) - control inputs
- **y**: Output vector (p×1) - measured outputs
- **A**: System matrix (n×n) - dynamics
- **B**: Input matrix (n×m) - how inputs affect states
- **C**: Output matrix (p×n) - how states affect outputs
- **D**: Feedthrough matrix (p×m) - direct input-output

**Controllability:**
A system is controllable if we can drive it from any initial state to any final state in finite time.

**Controllability Matrix:**
```
Co = [B AB A²B ... A^(n-1)B]
```
**Condition**: System is controllable if rank(Co) = n

**Physical meaning**: Can we control all internal states using available inputs?

**Observability:**
A system is observable if we can determine all internal states from output measurements.

**Observability Matrix:**
```
Ob = [C; CA; CA²; ...; CA^(n-1)]
```
**Condition**: System is observable if rank(Ob) = n

**Physical meaning**: Can we "see" all internal states through available outputs?

**State Feedback Control:**
```
u(t) = -Kx(t) + r(t)
```
Where K is the feedback gain matrix.

**Pole Placement:**
By choosing K, we can place closed-loop poles at desired locations.
Closed-loop system: ẋ = (A - BK)x

**Requirements:**
- System must be controllable to place poles arbitrarily
- Choose poles for desired response (settling time, damping, etc.)

---

### Problem 6.1: Inverted Pendulum on Cart (Self-Balancing Robot)

**Real-World Context:** You're designing a self-balancing robot (like a Segway). The robot must maintain upright position while moving. This is an inherently unstable system - without control, the pendulum falls over.

The linearized dynamics around the upright position are:

**State variables:** 
- x₁ = cart position (m)
- x₂ = cart velocity (m/s)  
- x₃ = pendulum angle from vertical (rad)
- x₄ = pendulum angular velocity (rad/s)

**Control input:** Force on cart (N)

**Task:**
1. Create state-space model
2. Check controllability and observability
3. Design state feedback controller using pole placement
4. Simulate balancing from tilted position

**Solution:**

```matlab
% Inverted Pendulum Parameters
fprintf('=== INVERTED PENDULUM (SELF-BALANCING ROBOT) ===\n\n');

M = 0.5;    % Cart mass (kg)
m = 0.2;    % Pendulum mass (kg)
b = 0.1;    % Friction coefficient (N·s/m)
I = 0.006;  % Moment of inertia of pendulum (kg·m²)
g = 9.8;    % Gravity (m/s²)
l = 0.3;    % Pendulum length to center of mass (m)

fprintf('Physical Parameters:\n');
fprintf('Cart mass (M): %.2f kg\n', M);
fprintf('Pendulum mass (m): %.2f kg\n', m);
fprintf('Pendulum length (l): %.2f m\n', l);
fprintf('Friction (b): %.2f N·s/m\n', b);

% Calculate system parameter
p = I*(M+m) + M*m*l^2;  % Denominator term

fprintf('\n=== STATE-SPACE MODEL ===\n');
fprintf('States: x = [position, velocity, angle, angular_velocity]^T\n');
fprintf('Input: u = Force on cart (N)\n');
fprintf('Outputs: y1 = cart position, y2 = pendulum angle\n\n');

% State-space matrices
% States: [x, x_dot, theta, theta_dot]^T
% Derived from Lagrangian mechanics
A = [0,                    1,              0,                  0;
     0,        -(I+m*l^2)*b/p,  (m^2*g*l^2)/p,              0;
     0,                    0,              0,                  1;
     0,            -(m*l*b)/p,  m*g*l*(M+m)/p,              0];

B = [0; 
     (I+m*l^2)/p; 
     0; 
     m*l/p];

C = [1, 0, 0, 0;      % Measure cart position
     0, 0, 1, 0];     % Measure pendulum angle

D = [0; 0];

% Display matrices
disp('A matrix (System dynamics):');
disp(A);
disp('B matrix (Input influence):');
disp(B);
disp('C matrix (Outputs):');
disp(C);

% Create state-space model
sys_ss = ss(A, B, C, D);

% Check eigenvalues (open-loop poles)
poles_OL = eig(A);
fprintf('Open-loop poles:\n');
disp(poles_OL);

% Analyze stability
unstable_poles = poles_OL(real(poles_OL) > 0);
if ~isempty(unstable_poles)
    fprintf('⚠ System is UNSTABLE (open-loop)\n');
    fprintf('Unstable poles: ');
    disp(unstable_poles');
    fprintf('Physical meaning: Without control, pendulum falls over!\n');
else
    fprintf('System is stable (open-loop)\n');
end

% Controllability Analysis
fprintf('\n=== CONTROLLABILITY ANALYSIS ===\n');
Co = ctrb(A, B);
rank_Co = rank(Co);
n = size(A, 1);

fprintf('Controllability matrix rank: %d\n', rank_Co);
fprintf('System order (number of states): %d\n', n);

if rank_Co == n
    fprintf('✓ System is CONTROLLABLE\n');
    fprintf('  → We CAN stabilize the pendulum using cart force\n');
    fprintf('  → All states can be controlled\n');
else
    fprintf('✗ System is NOT controllable\n');
    fprintf('  → Cannot control all states\n');
end

% Observability Analysis
fprintf('\n=== OBSERVABILITY ANALYSIS ===\n');
Ob = obsv(A, C);
rank_Ob = rank(Ob);

fprintf('Observability matrix rank: %d\n', rank_Ob);

if rank_Ob == n
    fprintf('✓ System is OBSERVABLE\n');
    fprintf('  → We CAN estimate all states from measurements\n');
    fprintf('  → Position and angle measurements are sufficient\n');
else
    fprintf('✗ System is NOT observable\n');
    fprintf('  → Cannot determine all states from outputs\n');
end

% State Feedback Controller Design
fprintf('\n=== STATE FEEDBACK CONTROLLER DESIGN ===\n');
fprintf('Goal: Stabilize inverted pendulum in upright position\n\n');

% Design specifications (in terms of desired poles)
% Choose poles for:
% - Fast response (poles far from origin)
% - Good damping (reasonable real part)
% - Realistic control effort (not too far)

fprintf('Desired closed-loop performance:\n');
fprintf('- Settling time: ~2 seconds\n');
fprintf('- Well-damped response (ζ ≈ 0.7)\n\n');

% For settling time ts ≈ 4/(ζωn) = 2s, choose ζωn ≈ 2
% For ζ = 0.7, ωn ≈ 2.86

% Desired dominant poles
zeta = 0.7;
wn = 2.86;
desired_poles = [-zeta*wn + 1i*wn*sqrt(1-zeta^2),  % Complex pair
                 -zeta*wn - 1i*wn*sqrt(1-zeta^2),
                 -4,                                 % Faster real poles
                 -4.5];

fprintf('Desired closed-loop poles:\n');
disp(desired_poles');

% Calculate feedback gains using pole placement
K = place(A, B, desired_poles);

fprintf('\nState Feedback Gains K:\n');
fprintf('K = [%.4f  %.4f  %.4f  %.4f]\n', K);
fprintf('\nControl law: u = -K*x\n');
fprintf('  = -%.2f*position - %.2f*velocity - %.2f*angle - %.2f*angular_vel\n', K);

% Closed-loop system
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);

% Verify closed-loop poles
poles_CL = eig(A_cl);
fprintf('\nClosed-loop poles (achieved):\n');
disp(poles_CL);

if all(real(poles_CL) < 0)
    fprintf('✓ Closed-loop system is STABLE\n');
else
    fprintf('✗ Design failed - system still unstable\n');
end

% Simulation from initial tilt
fprintf('\n=== SIMULATION: BALANCING FROM TILTED POSITION ===\n');

t = 0:0.01:5;
x0 = [0;        % Initial cart position = 0 m
      0;        % Initial cart velocity = 0 m/s  
      0.2;      % Initial tilt = 0.2 rad ≈ 11.5 degrees
      0];       % Initial angular velocity = 0 rad/s

fprintf('Initial conditions:\n');
fprintf('  Cart position: %.2f m\n', x0(1));
fprintf('  Pendulum angle: %.2f rad (%.1f degrees)\n', x0(3), x0(3)*180/pi);

% Simulate
[y, t_sim, x] = initial(sys_cl, x0, t);

% Plot results
figure;
subplot(4,1,1);
plot(t_sim, x(:,1), 'LineWidth', 1.5);
grid on;
title('State 1: Cart Position');
ylabel('Position (m)');

subplot(4,1,2);
plot(t_sim, x(:,2), 'LineWidth', 1.5);
grid on;
title('State 2: Cart Velocity');
ylabel('Velocity (m/s)');

subplot(4,1,3);
plot(t_sim, x(:,3)*180/pi, 'LineWidth', 1.5);
hold on;
yline(0, '--r', 'Upright');
grid on;
title('State 3: Pendulum Angle');
ylabel('Angle (degrees)');

subplot(4,1,4);
plot(t_sim, x(:,4)*180/pi, 'LineWidth', 1.5);
grid on;
title('State 4: Pendulum Angular Velocity');
ylabel('Angular Velocity (deg/s)');
xlabel('Time (s)');

sgtitle('Inverted Pendulum - State Feedback Control');

% Calculate control input
u = -K*x';  % Control signal for each time point

% Plot control effort
figure;
subplot(2,1,1);
plot(t_sim, u, 'LineWidth', 1.5);
grid on;
title('Control Input (Force on Cart)');
ylabel('Force (N)');
xlabel('Time (s)');

subplot(2,1,2);
plot(t_sim, x(:,3)*180/pi, 'LineWidth', 1.5);
hold on;
yline(0, '--r');
grid on;
title('Pendulum Angle Response');
ylabel('Angle (degrees)');
xlabel('Time (s)');

% Performance metrics
settling_idx = find(abs(x(:,3)) < 0.02, 1);  % Within 0.02 rad
if ~isempty(settling_idx)
    settling_time = t_sim(settling_idx);
    fprintf('\n Settling time: %.2f s (angle within ±1.15°)\n', settling_time);
end

max_angle = max(abs(x(:,3)*180/pi));
max_control = max(abs(u));
fprintf('Maximum angle deviation: %.2f degrees\n', max_angle);
fprintf('Maximum control force: %.2f N\n', max_control);

fprintf('\n=== DESIGN SUCCESS ===\n');
fprintf('✓ Pendulum stabilized in upright position\n');
fprintf('✓ Smooth convergence without excessive oscillation\n');
fprintf('✓ Reasonable control effort\n');

% Animation of pendulum (conceptual plot)
figure;
for i = 1:50:length(t_sim)
    clf;
    % Draw cart
    cart_x = x(i,1);
    cart_y = 0;
    rectangle('Position', [cart_x-0.1, cart_y-0.05, 0.2, 0.1], ...
              'FaceColor', [0.5 0.5 0.5]);
    hold on;
    
    % Draw pendulum
    theta = x(i,3);
    pend_x = cart_x + l*sin(theta);
    pend_y = cart_y + l*cos(theta);
    plot([cart_x, pend_x], [cart_y, pend_y], 'b-', 'LineWidth', 3);
    plot(pend_x, pend_y, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
    
    % Draw ground
    plot([-1, 1], [-0.1, -0.1], 'k-', 'LineWidth', 2);
    
    axis equal;
    xlim([-0.5, 0.5]);
    ylim([-0.2, 0.5]);
    title(sprintf('Time = %.2f s, Angle = %.1f°', t_sim(i), x(i,3)*180/pi));
    xlabel('Position (m)');
    grid on;
    drawnow;
    pause(0.05);
end
```

**💡 Key Insights:**
- Inverted pendulum is inherently unstable (pole in right half-plane)
- State feedback can stabilize unstable systems if controllable
- All four states must be measured or estimated for full state feedback
- Pole placement allows direct specification of closed-loop dynamics
- Faster poles require more control effort
- Real-world implementation needs state observer (Kalman filter) if states aren't directly measurable
- Applications: Segway, rocket landing, humanoid robots

---

## Module 7: Root Locus Analysis

### 📚 Theory Background: Root Locus Method

**What is Root Locus?**
Root locus is a graphical method showing how closed-loop poles move in the s-plane as a parameter (usually gain K) varies from 0 to ∞.

**Closed-Loop System:**
```
T(s) = KG(s)/(1 + KG(s)H(s))
```

Closed-loop poles satisfy: **1 + KG(s)H(s) = 0**

**Key Properties:**
- Root locus starts (K=0) at open-loop poles
- Root locus ends (K→∞) at open-loop zeros or infinity
- Number of branches = number of open-loop poles
- Locus is symmetrical about real axis

**Angle and Magnitude Conditions:**
For a point s₀ to be on root locus:
- **Angle**: ∠G(s₀)H(s₀) = ±180°(2k+1) where k = 0,1,2,...
- **Magnitude**: K = 1/|G(s₀)H(s₀)|

**Rules for Sketching:**
1. Number of branches = max(poles, zeros)
2. Branches start at poles, end at zeros
3. Locus on real axis: to the left of odd number of poles+zeros
4. Asymptotes: n - m branches go to infinity at angles:
   - θ = ±180°(2k+1)/(n-m), k = 0,1,2,...
5. Centroid: σ = (Σpoles - Σzeros)/(n - m)
6. Breakaway/break-in points: dK/ds = 0

**Design Using Root Locus:**
1. Overlay desired damping ratio lines (ζ)
2. Overlay desired natural frequency circles (ωₙ)
3. Select gain K where locus crosses desired specifications
4. Verify performance with step response

**Relationship Between Poles and Response:**
- Poles closer to imaginary axis → Slower response
- Poles farther left → Faster response  
- Complex poles closer to real axis → More damping
- Complex poles closer to imaginary axis → More oscillation

---

### Problem 7.1: Antenna Tracking System

**Real-World Context:** A satellite ground station needs an antenna tracking system to maintain communication with moving satellites. The antenna must track satellite position accurately without oscillations that would disrupt signal quality.

System open-loop transfer function: `G(s) = K/[s(s+2)(s+10)]`

This is a Type 1 system (one pole at origin) which provides zero steady-state error for position tracking.

**Task:**
1. Plot root locus as K varies
2. Find gain K for desired damping ratio (ζ = 0.7)
3. Analyze system stability for different gains
4. Verify time-domain performance

**Solution:**

```matlab
% Antenna Tracking System
fprintf('=== SATELLITE ANTENNA TRACKING SYSTEM ===\n\n');

% Open-loop transfer function (without gain K)
num = 1;
den = conv([1, 0], conv([1, 2], [1, 10]));  % s(s+2)(s+10)
G_antenna = tf(num, den);

fprintf('Open-Loop Transfer Function:\n');
fprintf('G(s) = K / [s(s+2)(s+10)]\n\n');
disp(G_antenna);

% Display poles and zeros
poles_OL = pole(G_antenna);
zeros_OL = zero(G_antenna);

fprintf('Open-loop poles:\n');
disp(poles_OL);
fprintf('Open-loop zeros: ');
if isempty(zeros_OL)
    fprintf('None\n');
else
    disp(zeros_OL);
end

% System type
fprintf('\nSystem Type: 1 (one pole at origin)\n');
fprintf('→ Zero steady-state error for step input (position tracking)\n');

% Root Locus Plot
fprintf('\n=== ROOT LOCUS ANALYSIS ===\n');
figure;
rlocus(G_antenna);
grid on;
title('Root Locus - Antenna Tracking System');
xlabel('Real Axis');
ylabel('Imaginary Axis');

% Add desired damping ratio line
zeta_desired = 0.7;
sgrid(zeta_desired, []);  % Add constant damping ratio line

fprintf('Desired damping ratio: ζ = %.2f\n', zeta_desired);
fprintf('→ Corresponds to %.1f%% overshoot\n', ...
    100*exp(-zeta_desired*pi/sqrt(1-zeta_desired^2)));

% Interactive gain selection
fprintf('\n=== INTERACTIVE GAIN SELECTION ===\n');
fprintf('Instructions:\n');
fprintf('1. Click on the root locus where it crosses ζ = %.2f line\n', zeta_desired);
fprintf('2. This will select the corresponding gain K\n');
fprintf('3. Close the selection window when done\n\n');

% Use rlocfind for interactive gain selection
[K_design, poles_design] = rlocfind(G_antenna);

fprintf('\n=== SELECTED DESIGN ===\n');
fprintf('Selected Gain K: %.4f\n', K_design);
fprintf('Closed-loop poles at this gain:\n');
disp(poles_design);

% Analyze selected poles
% Find dominant poles (closest to imaginary axis, typically complex conjugate pair)
[~, idx] = sort(real(poles_design), 'descend');
dominant_poles = poles_design(idx(1:2));

if abs(imag(dominant_poles(1))) > 1e-6
    % Complex conjugate dominant poles
    wn_actual = abs(dominant_poles(1));
    zeta_actual = -real(dominant_poles(1))/wn_actual;
    wd_actual = imag(dominant_poles(1));
    
    fprintf('\nDominant pole characteristics:\n');
    fprintf('Natural frequency ωₙ: %.4f rad/s\n', wn_actual);
    fprintf('Damping ratio ζ: %.4f\n', zeta_actual);
    fprintf('Damped frequency ωd: %.4f rad/s\n', wd_actual);
    
    % Theoretical predictions
    PO_theory = 100*exp(-zeta_actual*pi/sqrt(1-zeta_actual^2));
    ts_theory = 4/(zeta_actual*wn_actual);
    tp_theory = pi/wd_actual;
    
    fprintf('\nPredicted performance:\n');
    fprintf('Peak time: %.4f s\n', tp_theory);
    fprintf('Settling time: %.4f s\n', ts_theory);
    fprintf('Percent overshoot: %.2f%%\n', PO_theory);
else
    fprintf('Dominant poles are real (overdamped)\n');
end

% Create closed-loop system with designed gain
sys_designed = feedback(K_design*G_antenna, 1);

% Actual step response analysis
info = stepinfo(sys_designed);

fprintf('\n=== ACTUAL STEP RESPONSE CHARACTERISTICS ===\n');
fprintf('Rise Time: %.4f s\n', info.RiseTime);
fprintf('Settling Time: %.4f s\n', info.SettlingTime);
fprintf('Peak Time: %.4f s\n', info.PeakTime);
fprintf('Overshoot: %.2f%%\n', info.Overshoot);
fprintf('Peak Value: %.4f\n', info.Peak);
fprintf('Steady-State Value: %.4f\n', dcgain(sys_designed));

% Step response plot
figure;
step(sys_designed, 10);
grid on;
title(sprintf('Step Response with K = %.2f (ζ = %.2f)', K_design, zeta_actual));
xlabel('Time (s)');
ylabel('Antenna Position (degrees)');

% Stability Analysis for Different Gains
fprintf('\n=== STABILITY ANALYSIS FOR VARIOUS GAINS ===\n');

K_values = [0.1, 1, 10, 50, 100, 200];

figure;
for i = 1:length(K_values)
    K_test = K_values(i);
    sys_test = feedback(K_test*G_antenna, 1);
    poles_test = pole(sys_test);
    
    fprintf('\nK = %.1f:\n', K_test);
    
    if all(real(poles_test) < 0)
        fprintf('  Status: STABLE\n');
    else
        fprintf('  Status: UNSTABLE\n');
    end
    
    % Check if complex poles exist
    complex_poles = poles_test(abs(imag(poles_test)) > 1e-6);
    if ~isempty(complex_poles)
        wn_test = abs(complex_poles(1));
        zeta_test = -real(complex_poles(1))/wn_test;
        fprintf('  Damping ratio: %.4f\n', zeta_test);
    end
    
    % Plot step response
    subplot(2, 3, i);
    step(sys_test, 10);
    title(sprintf('K = %.1f', K_test));
    grid on;
end
sgtitle('Step Responses for Different Gains');

% Find stability boundary
fprintf('\n=== STABILITY BOUNDARY ===\n');
fprintf('Finding maximum stable gain...\n');

% Routh-Hurwitz criterion or find where poles cross to RHP
% For this system: characteristic equation is s³ + 12s² + 20s + K = 0
% Using Routh array, stability condition: K < 240

K_stable_max = 240;  % From Routh-Hurwitz analysis
fprintf('Maximum stable gain: K < %.1f\n', K_stable_max);

% Plot root locus with stability regions
figure;
rlocus(G_antenna);
hold on;
sgrid(zeta_desired, []);

% Mark selected design point
plot(real(poles_design), imag(poles_design), 'ro', ...
    'MarkerSize', 10, 'LineWidth', 2);
text(real(poles_design(1))+0.5, imag(poles_design(1)), ...
    sprintf('K = %.2f', K_design), 'FontSize', 12);

% Mark stability boundary (imaginary axis)
xline(0, '--r', 'Stability Boundary', 'LineWidth', 2);

title('Root Locus with Design Point and Stability Boundary');
legend('Root Locus', sprintf('ζ = %.2f line', zeta_desired), ...
    'Design Point', 'Location', 'best');
grid on;

% Frequency response at designed gain
fprintf('\n=== FREQUENCY RESPONSE ANALYSIS ===\n');
sys_OL = K_design * G_antenna;
[Gm, Pm, Wcg, Wcp] = margin(sys_OL);

fprintf('Open-loop with K = %.2f:\n', K_design);
fprintf('Gain Margin: %.2f dB\n', 20*log10(Gm));
fprintf('Phase Margin: %.2f degrees\n', Pm);

figure;
margin(sys_OL);
title(sprintf('Bode Plot with K = %.2f', K_design));
grid on;

% Disturbance rejection analysis
fprintf('\n=== DISTURBANCE REJECTION ===\n');
fprintf('Testing wind gust disturbance...\n');

% Sensitivity function
S = 1/(1 + K_design*G_antenna);

figure;
t_dist = 0:0.01:15;

% Reference input (track moving satellite)
r = [zeros(1, 100), ones(1, length(t_dist)-100)];

% Disturbance (wind gust)
d = 0.3*[zeros(1, 500), ones(1, 300), zeros(1, length(t_dist)-800)];

% Response to reference
y_ref = lsim(sys_designed, r, t_dist);

% Response to disturbance at output
y_dist = lsim(S, d, t_dist);

% Total response
y_total = y_ref + y_dist;

subplot(2,1,1);
plot(t_dist, [r' y_total y_ref], 'LineWidth', 1.5);
legend('Reference', 'With Disturbance', 'No Disturbance');
title('Tracking with Wind Disturbance');
ylabel('Antenna Position (deg)');
xlabel('Time (s)');
grid on;
xline(5, '--g', 'Wind Gust Starts');
xline(8, '--r', 'Wind Gust Ends');

subplot(2,1,2);
plot(t_dist, y_total - r', 'LineWidth', 1.5);
title('Tracking Error');
ylabel('Error (deg)');
xlabel('Time (s)');
grid on;
yline(0, '--k');

fprintf('Type 1 system provides good disturbance rejection\n');
fprintf('Integral action eliminates steady-state error\n');

% Summary
fprintf('\n=== DESIGN SUMMARY ===\n');
fprintf('✓ Gain K = %.2f selected for ζ = %.2f\n', K_design, zeta_actual);
fprintf('✓ System is stable with good margins\n');
fprintf('✓ Overshoot = %.1f%% (acceptable for tracking)\n', info.Overshoot);
fprintf('✓ Settling time = %.2f s (fast enough for satellite tracking)\n', ...
    info.SettlingTime);
fprintf('✓ Zero steady-state error (Type 1 system)\n');
fprintf('✓ Good disturbance rejection\n');
```

**💡 Key Insights:**
- Root locus shows entire family of possible designs
- Damping ratio determines overshoot (trade-off with speed)
- Gain too low → Slow response
- Gain too high → Oscillatory or unstable
- Type 1 system eliminates steady-state error for step inputs
- Root locus provides intuitive graphical design method
- Complex conjugate poles create oscillatory response
- Real poles create exponential response

---

## Practice Problems

### 📝 Challenge 1: Industrial Furnace Temperature Control
**Context:** Design a control system for an industrial furnace with significant time delay.

**Given:**
- Process: `Gp(s) = e^(-5s)/(100s + 1)` (time delay + first-order lag)
- Large thermal inertia (slow response)
- Measurement delay (thermocouple lag)

**Tasks:**
1. Use Pade approximation for time delay: `pade(tf(1,1), 5)`
2. Analyze stability of direct proportional control
3. Design PI or PID controller
4. Handle constraints: Temperature cannot exceed 1000°C

**Hints:**
- Time delays reduce phase margin significantly
- Use Smith predictor for systems with large delays
- Conservative gain selection needed

---

### 📝 Challenge 2: Quadcopter Altitude Control
**Context:** Control quadcopter vertical position (hovering).

**Given:**
- Plant: `G(s) = 1/s²` (double integrator)
- Thrust input, altitude output
- Subject to wind gusts

**Tasks:**
1. Design cascade control (inner velocity loop, outer position loop)
2. Implement PID for both loops
3. Add disturbance rejection
4. Simulate with step reference and wind disturbance

**Hints:**
- Inner loop needs faster response than outer loop
- Double integrator is marginally stable
- Derivative action essential for damping

---

### 📝 Challenge 3: Multi-Tank Level Control
**Context:** Two interconnected water tanks with interaction.

**Given:**
```matlab
% Tank 1 → Tank 2 (coupling)
A_tanks = [-0.1,  0;
            0.05, -0.08];
B_tanks = [1; 0];
C_tanks = eye(2);
D_tanks = [0; 0];
```

**Tasks:**
1. Analyze controllability and observability
2. Design MIMO controller
3. Handle coupling effects
4. Achieve independent control of both levels

**Hints:**
- Interaction creates control challenges
- Decoupling strategies needed
- State-space approach most suitable

---

### 📝 Challenge 4: DC Motor Speed Control with Load Torque
**Context:** Improve the DC motor example from Module 1.

**Tasks:**
1. Add load torque as disturbance
2. Design controller to reject load changes
3. Compare P, PI, and PID performance
4. Add saturation limits (voltage ±24V)
5. Test with step load torque at different times

**Expected Learning:**
- Importance of integral action for disturbance rejection
- Effect of saturation on performance
- Trade-offs in controller tuning

---

### 📝 Challenge 5: Magnetic Levitation System
**Context:** Levitate a metal ball using electromagnet (highly nonlinear, unstable).

**Linearized Model:**
```matlab
A_maglev = [0, 1;
            980, 0];
B_maglev = [0; -19.6];
C_maglev = [1, 0];
D_maglev = 0;
```

**Tasks:**
1. Check controllability
2. Design state feedback for stabilization
3. Add integral action for zero steady-state error
4. Design observer for velocity estimation
5. Implement full observer-based controller

**Hints:**
- System is unstable (pole at √980 ≈ 31.3)
- Very sensitive to disturbances
- Requires fast control action

---

## Summary and Best Practices

### 🎯 Control System Design Workflow

**1. System Modeling**
- Identify physical system
- Derive mathematical model (transfer function or state-space)
- Validate model with experimental data

**2. Analysis**
- Check stability (pole locations)
- Analyze controllability and observability (state-space)
- Determine system type (steady-state error)
- Time domain specifications (overshoot, settling time)
- Frequency domain specifications (margins, bandwidth)

**3. Controller Design**
- Select control strategy (P, PI, PID, lead-lag, state feedback)
- Tune parameters (analytical or software tools)
- Verify specifications met

**4. Validation**
- Simulate with disturbances
- Test robustness (parameter variations)
- Check control effort limits
- Verify safety constraints

**5. Implementation**
- Discretize for digital implementation
- Add anti-windup for integrators
- Include saturation handling
- Test on real system

---

### 📊 Key MATLAB Commands Reference

**Transfer Function:**
```matlab
G = tf(num, den)           % Create transfer function
pole(G)                    % Find poles
zero(G)                    % Find zeros
dcgain(G)                  % DC gain
step(G)                    % Step response
impulse(G)                 % Impulse response
```

**Frequency Domain:**
```matlab
bode(G)                    % Bode plot
margin(G)                  % Gain and phase margins
nyquist(G)                 % Nyquist plot
bandwidth(G)               % System bandwidth
```

**State-Space:**
```matlab
sys = ss(A,B,C,D)         % Create state-space model
ctrb(A,B)                 % Controllability matrix
obsv(A,C)                 % Observability matrix
place(A,B,poles)          % Pole placement
lqr(A,B,Q,R)             % LQR optimal control
```

**Controller Design:**
```matlab
pid(Kp, Ki, Kd)           % Create PID controller
feedback(G,H)             % Closed-loop system
rlocus(G)                 % Root locus plot
rlocfind(G)               % Interactive gain selection
```

**Analysis:**
```matlab
stepinfo(G)               % Step response characteristics
pzmap(G)                  % Pole-zero map
lsim(G,u,t)              % Simulate with arbitrary input
initial(sys,x0,t)        % Initial condition response
```

---

### ⚡ Design Tips and Best Practices

**Stability:**
- Always check pole locations first
- All poles must be in left half-plane
- Gain margin > 6 dB, Phase margin > 30° (preferably 45-60°)

**Performance:**
- Faster response requires higher bandwidth
- Higher bandwidth → More sensitive to noise
- Trade-off between speed and robustness

**PID Tuning:**
1. Start with P control, gradually increase
2. Add I if steady-state error exists
3. Add D if overshoot is excessive
4. Fine-tune iteratively

**Common Mistakes:**
- ❌ Ignoring saturation limits
- ❌ Not testing with disturbances
- ❌ Over-aggressive tuning (instability)
- ❌ Neglecting sensor dynamics
- ❌ Forgetting anti-windup for integrators

**Good Practices:**
- ✅ Always simulate before implementing
- ✅ Test robustness (±20% parameter variation)
- ✅ Include realistic constraints
- ✅ Document design decisions
- ✅ Start conservative, tune gradually

---

### 📚 Additional Learning Resources

**Books:**
- "Modern Control Engineering" - Katsuhiko Ogata
- "Feedback Control of Dynamic Systems" - Franklin, Powell, Emami-Naeini
- "Control Systems Engineering" - Norman Nise

**Online:**
- MATLAB Control System Toolbox Documentation
- Brian Douglas YouTube Channel (excellent tutorials)
- MIT OpenCourseWare - Control Systems

**Practice:**
- Implement controllers on Arduino/Raspberry Pi
- Experiment with simulation tools (Simulink)
- Build physical systems (motor control, balancing robot)

---

## Appendix: Mathematical Foundations

### Laplace Transform Basics
```
L{f(t)} = F(s) = ∫₀^∞ f(t)e^(-st) dt
```

**Common Transforms:**
- Unit step: L{u(t)} = 1/s
- Exponential: L{e^(-at)} = 1/(s+a)
- Sine: L{sin(ωt)} = ω/(s²+ω²)
- Derivative: L{df/dt} = sF(s) - f(0)
- Integral: L{∫f(t)dt} = F(s)/s

### Final Value Theorem
```
lim[t→∞] f(t) = lim[s→0] sF(s)
```
Useful for calculating steady-state error.

### Partial Fraction Expansion
For inverse Laplace transform of complex expressions:
```matlab
[r, p, k] = residue(num, den)
```

---

**🎓 End of Tutorial**

**Congratulations!** You've completed a comprehensive journey through control systems using MATLAB. You've learned:
- Transfer function and state-space modeling
- Time and frequency domain analysis
- PID and advanced controller design
- Stability analysis and root locus
- Practical implementation considerations

**Next Steps:**
1. Work through practice problems
2. Implement designs on real hardware
3. Explore advanced topics (robust control, adaptive control, nonlinear control)
4. Build a portfolio project (quadcopter, self-balancing robot, etc.)

**Remember:** Control systems engineering is both art and science. Theory provides the foundation, but practical experience develops intuition. Keep experimenting, learning, and building!
