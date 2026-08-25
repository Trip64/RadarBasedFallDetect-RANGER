# Physics & Kinematic Principles of the RANGER System

This document outlines the mathematical models, sensor physics, signal processing principles, and RF link budget calculations implemented across the RANGER ecosystem.

---

## Quick Navigation

- [1. Kinematic Mechanics of Human Falls](#1-kinematic-mechanics-of-human-falls)
  - [1.1 Gravitational Acceleration & Free-Fall Signature](#11-gravitational-acceleration--free-fall-signature)
  - [1.2 Impact Impulse Dynamics](#12-impact-impulse-dynamics)
  - [1.3 Post-Impact Stillness Criterion](#13-post-impact-stillness-criterion)
- [2. mmWave FMCW Doppler Radar Physics](#2-mmwave-fmcw-doppler-radar-physics)
  - [2.1 FMCW Chirp Frequency Modulation](#21-fmcw-chirp-frequency-modulation)
  - [2.2 Range Resolution](#22-range-resolution-delta-r)
  - [2.3 Doppler Velocity Shift](#23-doppler-velocity-shift)
- [3. Link Budget & RF Propagation](#3-link-budget--rf-propagation-24-ghz-esp-now--ble)
  - [3.1 Friis Free-Space Path Loss](#31-friis-free-space-path-loss-fspl)
  - [3.2 Total Indoor Link Margin](#32-total-indoor-link-budget)

---

## 1. Kinematic Mechanics of Human Falls

### 1.1 Gravitational Acceleration & Free-Fall Signature

In steady-state upright posture, a body-worn tri-axial accelerometer measures a static upward reaction force equal to standard earth gravity:

$$\|\vec{a}_{\text{static}}\| = \sqrt{a_x^2 + a_y^2 + a_z^2} \approx 1.0\text{ g}$$

During the initiation phase of a vertical fall, the body loses contact with supporting surfaces, entering a transient state of free fall where the total measured acceleration collapses toward zero:

$$\|\vec{a}_{\text{freefall}}\| < 0.3\text{ g} \quad \text{for } \Delta t \in [100\text{ ms}, 350\text{ ms}]$$

### 1.2 Impact Impulse Dynamics

Upon collision with the ground, deceleration generates an elastic-inelastic impulse:

$$\vec{J} = \int_{t_1}^{t_2} \vec{F}(t) \, dt = m \Delta \vec{v}$$

In the sampled acceleration signal, this creates an abrupt, high-amplitude spike:

$$\|\vec{a}_{\text{impact}}\| \ge 2.5\text{ g} \dots 6.0\text{ g}$$

Simultaneously, the Coriolis-rate gyroscopes record angular velocity excursions:

$$\|\vec{\omega}\| = \sqrt{\omega_x^2 + \omega_y^2 + \omega_z^2} > 120^\circ/\text{s}$$

### 1.3 Post-Impact Stillness Criterion

False positives (e.g. running, jumping, or sitting down rapidly) are differentiated from actual emergencies through post-impact equilibrium monitoring over a 3-second evaluation window:

$$|\|\vec{a}(t)\| - 1.0\text{ g}| \le 0.25\text{ g} \quad \forall t \in [t_{\text{impact}}, t_{\text{impact}} + 3000\text{ ms}]$$

---

## 2. mmWave FMCW Doppler Radar Physics

### 2.1 FMCW Chirp Frequency Modulation

The RD-03D and HLK-LD1125H radar modules operate within the **24.00 GHz – 24.25 GHz** ISM band. The transmitter synthesizes a linear Frequency Modulated Continuous Wave (FMCW):

$$f(t) = f_0 + \left(\frac{B}{T_c}\right) t$$

| Parameter | Symbol | Nominal Value | Description |
|---|---|---|---|
| Carrier Frequency | $f_0$ | $24.00\text{ GHz}$ | Center start frequency |
| Chirp Bandwidth | $B$ | $250\text{ MHz}$ | Total sweep frequency span |
| Chirp Duration | $T_c$ | $1.0\text{ ms}$ | Modulation sweep period |
| Propagation Speed | $c$ | $3 \times 10^8\text{ m/s}$ | Speed of electromagnetic wave |

### 2.2 Range Resolution ($\Delta R$)

The spatial range resolution is governed by the total sweep bandwidth $B$:

$$\Delta R = \frac{c}{2B} = \frac{3 \times 10^8\text{ m/s}}{2 \times 250 \times 10^6\text{ Hz}} = 0.60\text{ m} \quad (60\text{ cm})$$

Target distance $R$ is derived from the intermediate beat frequency $f_b$ produced in the homodyne receiver:

$$R = \frac{c \cdot T_c \cdot f_b}{2B}$$

### 2.3 Doppler Velocity Shift

Radial target movement relative to the antenna introduces a Doppler frequency shift $f_d$:

$$f_d = \frac{2 v_r}{\lambda} = \frac{2 v_r f_0}{c}$$

At $\lambda = 1.25\text{ cm}$ ($24\text{ GHz}$), a person falling with a radial velocity of $v_r = 1.5\text{ m/s}$ generates a characteristic Doppler shift:

$$f_d = \frac{2 \times 1.5}{0.0125} = 240\text{ Hz}$$

---

## 3. Link Budget & RF Propagation (2.4 GHz ESP-NOW / BLE)

### 3.1 Friis Free-Space Path Loss (FSPL)

$$\text{FSPL (dB)} = 20 \log_{10}(d) + 20 \log_{10}(f) - 147.56$$

At an indoor line-of-sight distance $d = 10\text{ m}$ and frequency $f = 2.44\text{ GHz}$:

$$\text{FSPL} \approx 60.1\text{ dB}$$

### 3.2 Total Indoor Link Budget

The received signal power $P_{rx}$ is given by:

$$P_{rx} = P_{tx} + G_{tx} + G_{rx} - \text{FSPL} - L_{\text{body}} - L_{\text{fade}}$$

| Variable | Description | Value |
|---|---|---|
| $P_{tx}$ | Transmitter Output Power (ESP32-S3 / nRF52840) | $+8.5\text{ dBm}$ |
| $G_{tx}$ | Transmitter Antenna Gain | $+1.5\text{ dBi}$ |
| $G_{rx}$ | Receiver Antenna Gain | $+2.0\text{ dBi}$ |
| $\text{FSPL}$ | Free Space Path Loss (10m @ 2.44 GHz) | $60.1\text{ dB}$ |
| $L_{\text{body}}$ | Human Body Dielectric Absorption Loss | $3.0\text{ dB}$ |
| $L_{\text{fade}}$ | Indoor Multipath Fading Allowance | $10.0\text{ dB}$ |
| **$P_{rx}$** | **Net Received Power at Receiver** | **$-61.1\text{ dBm}$** |

With an ESP32-S3 receiver sensitivity of $-97\text{ dBm}$, the communication link maintains a robust **$\approx 36\text{ dB}$ link margin**, ensuring reliable packet transmission through walls, doors, and human obstacles.
