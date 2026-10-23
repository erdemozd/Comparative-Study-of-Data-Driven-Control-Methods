# Comparative Study of Data-Driven Predictive Control Methods

![MATLAB](https://img.shields.io/badge/MATLAB-R2021b%2B-orange)
![Toolboxes](https://img.shields.io/badge/Toolboxes-Optimization%20%7C%20Control%20System-blue)
![Status](https://img.shields.io/badge/status-active%20development-yellow)

A MATLAB benchmark comparing **model-based**, **data-driven**, **hybrid** and **prior-informed** predictive controllers on a nonlinear three-mass spring–damper system.

The study investigates how the *amount* and *form* of prior system knowledge affect:

- closed-loop reference tracking
- prediction accuracy
- robustness to noisy and limited data
- optimization complexity and computational effort
- magnitude of the behavioral coefficients $g$
- excitation of the plant nonlinearity

| Controller | Prior knowledge used | Prediction model |
|---|---|---|
| **MPC** | Full nominal linear model | $X = S_x x_0 + S_u U$ |
| **DeePC** | None (data only) | $X = Y_f g$ |
| **SD-MPC** | Full nominal model + data-driven residual | $X = S_x x_M + S_u U_f g + R_f g$ |
| **HDeePC** | Model of the known subsystem ($m_1$) | $X = S_{\mathrm{full}} x_\kappa(0) + G_{\mathrm{full}} g$ |
| **PZ-DeePC** | Known pole locations (2 or 4 poles) | $X = G_x g + d_{\mathrm{past}}$ |

All controllers are tested on the same nonlinear plant, with the same reference trajectory, physical constraints, prediction horizon and cost structure, as far as their formulations permit.

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Repository Structure](#repository-structure)
3. [Three-Mass Benchmark System](#three-mass-benchmark-system)
4. [Nominal Linear Model](#nominal-linear-model)
5. [Training Data](#training-data)
6. [Common Tracking Formulation](#common-tracking-formulation)
7. [Controllers](#controllers)
    - [Model Predictive Control](#model-predictive-control-mpc)
    - [Standard DeePC](#standard-deepc)
    - [Semi-Data-Driven MPC](#semi-data-driven-mpc-sd-mpc)
    - [Hybrid DeePC](#hybrid-deepc-hdeepc)
    - [Pole-Informed DeePC](#pole-informed-deepc-pz-deepc)
8. [Regularization](#regularization)
9. [Noise Analysis of Pole Filtering](#noise-analysis-of-pole-filtering)
10. [Terminal Formulation Study](#terminal-formulation-study)
11. [Diagnostics](#diagnostics)
12. [Current Observations](#current-observations)
13. [Open Research Questions](#open-research-questions)
14. [Development Status](#development-status)
15. [References](#references)
16. [Author](#author)

---

## Quick Start

### Requirements

- MATLAB
- Optimization Toolbox (`quadprog`, interior-point-convex algorithm)
- Control System Toolbox (`ss`, `c2d`)

### Running

Start MATLAB in the repository root:

```matlab
startup_project          % add all folders to the path
generate_deepc_data      % generate the training data set
```

Then run any standalone controller script:

```matlab
run_mpc_closed_loop
run_deepc_closed_loop
run_sdmpc_closed_loop
run_hdeepc_closed_loop
```

PZ-DeePC is run with the corresponding pole-prior setup (`pzdeepc_setup_2pole` / `pzdeepc_setup_4pole`).

Each standalone script simulates only its own controller. Cross-controller experiments and sensitivity studies live in `Comparison/` and `tests/`.

---

## Repository Structure

```text
Comparative-Study-of-Data-Driven-Control-Methods/
│
├── common/                     # plant, parameters, shared utilities
│   ├── three_mass_params.m
│   ├── benchmark_settings.m
│   ├── build_reference.m
│   ├── block_hankel.m
│   ├── rk4_step.m
│   └── ...
│
├── data/
│   └── deepc_training_data.mat
│
├── MPC/
│   ├── mpc_setup.m
│   ├── mpc_control.m
│   └── run_mpc_closed_loop.m
│
├── DeePC/
│   ├── generate_deepc_data.m
│   ├── deepc_setup.m
│   ├── deepc_control.m
│   └── run_deepc_closed_loop.m
│
├── SD-MPC/
│   ├── sdmpc_setup.m
│   ├── sdmpc_control.m
│   └── ...
│
├── HDeePC/
│   ├── hdeepc_setup.m
│   ├── hdeepc_control.m
│   └── ...
│
├── PZ-DeePC/
│   ├── pzdeepc_setup_base.m
│   ├── pzdeepc_setup_2pole.m
│   ├── pzdeepc_setup_4pole.m
│   ├── pzdeepc_control.m
│   ├── pz_filter_matrix.m
│   ├── pz_filter_past_output.m
│   ├── pz_unfilter_map.m
│   └── ...
│
├── Comparison/                 # cross-controller comparisons
├── tests/                      # sensitivity and diagnostic studies
│   ├── deepc_terminal_study.m
│   ├── pzdeepc_noise_filter_study.m
│   └── ...
│
├── results/
├── MODEL_DERIVATION.md
├── startup_project.m
└── README.md
```

---

## Three-Mass Benchmark System

```text
wall ──(k1,c1)── m1 ──(k12,c12)── m2 ──(k23,c23,k3)── m3
          ▲
          u  (force on m1)
```

A single force input $u \in \mathbb{R}$ acts on the first mass. The main controlled output is the third-mass position $q_3$.

**State vector**

```math
x = \begin{bmatrix} q_1 & v_1 & q_2 & v_2 & q_3 & v_3 \end{bmatrix}^{\top}
```

**True nonlinear dynamics**

```math
\begin{aligned}
\dot q_1 &= v_1 \\
m_1 \dot v_1 &= u - k_1 q_1 - c_1 v_1 + k_{12}(q_2 - q_1) + c_{12}(v_2 - v_1) \\[4pt]
\dot q_2 &= v_2 \\
m_2 \dot v_2 &= -k_{12}(q_2 - q_1) - c_{12}(v_2 - v_1) + k_{23}(q_3 - q_2) + k_3 (q_3 - q_2)^3 + c_{23}(v_3 - v_2) \\[4pt]
\dot q_3 &= v_3 \\
m_3 \dot v_3 &= -k_{23}(q_3 - q_2) - k_3 (q_3 - q_2)^3 - c_{23}(v_3 - v_2)
\end{aligned}
```

### Nonlinearity

The cubic spring between $m_2$ and $m_3$ is the main nonlinearity:

```math
F_{\mathrm{cubic}} = k_3 (q_3 - q_2)^3,
\qquad
F_{\mathrm{linear}} = k_{23} (q_3 - q_2).
```

The **nonlinear-exposure metric** quantifies how far a controller drives the plant outside its approximately linear operating region:

```math
\eta_{\mathrm{NL}}
= 100\,\frac{|F_{\mathrm{cubic}}|}{|F_{\mathrm{linear}}|}
= 100\,\frac{k_3}{k_{23}}\,(q_3 - q_2)^2,
\qquad q_3 \neq q_2 .
```

### Parameters

| Symbol | Value | Symbol | Value |
|---|---|---|---|
| $m_1$ | 1.0 kg | $c_1$ | 1.0 N·s/m |
| $m_2$ | 1.5 kg | $c_{12}$ | 0.8 N·s/m |
| $m_3$ | 1.2 kg | $c_{23}$ | 0.7 N·s/m |
| $k_1$ | 20 N/m | $T_s$ | 0.05 s |
| $k_{12}$ | 15 N/m | $u_{\max}$ | 25 N |
| $k_{23}$ | 12 N/m | $q_{\max}$ | 0.9 m |
| $k_3$ | 150 N/m³ | | |

All values are set in [`common/three_mass_params.m`](common/three_mass_params.m).

---

## Nominal Linear Model

The nominal model neglects the cubic stiffness term:

```math
\dot x = A x + B u,
```

<details>
<summary><b>System matrices A and B</b> (click to expand)</summary>

```math
A =
\begin{bmatrix}
0 & 1 & 0 & 0 & 0 & 0 \\
-\frac{k_1 + k_{12}}{m_1} & -\frac{c_1 + c_{12}}{m_1} & \frac{k_{12}}{m_1} & \frac{c_{12}}{m_1} & 0 & 0 \\
0 & 0 & 0 & 1 & 0 & 0 \\
\frac{k_{12}}{m_2} & \frac{c_{12}}{m_2} & -\frac{k_{12} + k_{23}}{m_2} & -\frac{c_{12} + c_{23}}{m_2} & \frac{k_{23}}{m_2} & \frac{c_{23}}{m_2} \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & \frac{k_{23}}{m_3} & \frac{c_{23}}{m_3} & -\frac{k_{23}}{m_3} & -\frac{c_{23}}{m_3}
\end{bmatrix},
\qquad
B =
\begin{bmatrix} 0 \\ 1/m_1 \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}.
```

</details>

The discrete-time nominal model is obtained with a zero-order hold:

```math
x_{k+1} = A_d x_k + B_d u_k .
```

This model is used fully in MPC and partially in SD-MPC and HDeePC.

### Nominal modal frequencies

| Mode | Frequency |
|---|---|
| $f_1$ | $\approx 0.256$ Hz |
| $f_2$ | $\approx 0.678$ Hz |
| $f_3$ | $\approx 1.016$ Hz |

With $T_s = 0.05$ s, the sampling frequency is $f_s = 20$ Hz and the Nyquist frequency is $f_N = 10$ Hz. The relevant physical dynamics therefore lie mainly below approximately 1 Hz.

---

## Training Data

### Hankel matrices

The data-driven controllers use measured input–output trajectories

```math
u^d = \begin{bmatrix} u_0 & u_1 & \cdots & u_{T-1} \end{bmatrix},
\qquad
y^d = \begin{bmatrix} y_0 & y_1 & \cdots & y_{T-1} \end{bmatrix},
```

with full-state output measurement, $y_k = x_k$.

The Hankel depth is

```math
L = T_{\mathrm{ini}} + N,
```

where $`T_{\mathrm{ini}}`$ is the past horizon and $N$ the prediction horizon. The block Hankel matrix is

```math
H_L(u^d) =
\begin{bmatrix}
u_0 & u_1 & \cdots \\
u_1 & u_2 & \cdots \\
\vdots & \vdots & \ddots \\
u_{L-1} & u_L & \cdots
\end{bmatrix},
```

giving $n_g = T - L + 1$ behavioral coefficient variables.

For standard DeePC, the training input is checked for persistent excitation of approximately order $L + n$, with nominal system order $n = 6$.

### Measurement noise

Noisy output data are generated as

```math
y_k^{\mathrm{meas}} = y_k^{\mathrm{true}} + e_k,
\qquad
e_k \sim \mathcal{N}(0, \Sigma_y).
```

The standard deviation of each state channel is scaled by the RMS value of its clean signal:

```math
\sigma_{e_i} = \eta \sqrt{\frac{1}{T} \sum_{k=0}^{T-1} y_i(k)^2 } .
```

For example, $\eta = 0.05$ corresponds to a noise standard deviation equal to 5 % of the RMS magnitude of each channel.

> [!NOTE]
> The clean physical state is stored separately from the noisy measurement, so physical diagnostics such as nonlinear spring exposure are not contaminated by artificial measurement noise.

---

## Common Tracking Formulation

The desired coordinated state and its stacked reference are

```math
x_r = \begin{bmatrix} r & 0 & r & 0 & r & 0 \end{bmatrix}^{\top},
\qquad
X_r = \mathbf{1}_N \otimes x_r .
```

All controllers share the tracking cost

```math
J_{\mathrm{track}}
= \| X - X_r \|_{Q_N}^2 + \| U - \mathbf{1}_N u_s \|_{R_N}^2,
\qquad
Q_N = I_N \otimes Q,
\quad
R_N = I_N \otimes R .
```

The steady input $u_s$ is optimized internally.

For the coordinated static configuration $q_1 = q_2 = q_3 = r$, $v_1 = v_2 = v_3 = 0$, the physical equilibrium input is

```math
u_{\mathrm{eq}} = k_1 r .
```

> [!IMPORTANT]
> $u_{\mathrm{eq}}$ is **not** supplied to any controller. It is used only as a diagnostic.

---

## Controllers

### Model Predictive Control (MPC)

The model-based benchmark. Using the nominal discrete model, the stacked prediction is

```math
X = S_x x_0 + S_u U ,
```

and the optimization problem is

```math
\begin{aligned}
\min_{U,\,u_s}\quad
& \| X - X_r \|_{Q_N}^2 + \| U - \mathbf{1}_N u_s \|_{R_N}^2 \\
\text{s.t.}\quad
& X = S_x x_0 + S_u U, \\
& |u_k| \le u_{\max}, \\
& |q_i(k)| \le q_{\max}.
\end{aligned}
```

---

### Standard DeePC

The Hankel matrices are partitioned into past and future blocks:

```math
H_L(u^d) = \begin{bmatrix} U_p \\ U_f \end{bmatrix},
\qquad
H_L(y^d) = \begin{bmatrix} Y_p \\ Y_f \end{bmatrix}.
```

The behavioral equations are

```math
U_p g = u_{\mathrm{ini}},
\qquad
Y_p g - \sigma = y_{\mathrm{ini}},
\qquad
U = U_f g,
\qquad
X = Y_f g ,
```

with decision vector $z = [\,g^{\top}\ \ \sigma^{\top}\ \ u_s\,]^{\top}$. The DeePC problem is

```math
\begin{aligned}
\min_{g,\,\sigma,\,u_s}\quad
& \| Y_f g - X_r \|_{Q_N}^2 + \| U_f g - \mathbf{1}_N u_s \|_{R_N}^2
  + \lambda_g \| g \|_2^2 + \lambda_\sigma \| \sigma \|_2^2 \\
\text{s.t.}\quad
& U_p g = u_{\mathrm{ini}}, \\
& Y_p g - \sigma = y_{\mathrm{ini}}, \\
& |U_f g| \le u_{\max}, \\
& |S_q Y_f g| \le q_{\max}.
\end{aligned}
```

---

### Semi-Data-Driven MPC (SD-MPC)

SD-MPC splits the prediction into a nominal part and a data-driven residual:

```math
X = X_M + X_D,
\qquad
X_M = S_x x_M + S_u U .
```

The residual training signal and its Hankel matrix are

```math
r_d = y_d - x_{M,d},
\qquad
H_L(r_d) = \begin{bmatrix} R_p \\ R_f \end{bmatrix}.
```

The residual behavioral equations are

```math
U_p g = u_{\mathrm{ini}},
\qquad
R_p g - \sigma = r_{\mathrm{ini}},
\qquad
U = U_f g,
\qquad
X_D = R_f g ,
```

so the complete prediction becomes

```math
X = S_x x_M + S_u U_f g + R_f g .
```

The nominal model captures the main dynamics, while the data-driven component predicts the model mismatch.

---

### Hybrid DeePC (HDeePC)

HDeePC partitions the state into a known and an unknown subsystem:

```math
x = \begin{bmatrix} x_\kappa \\ x_u \end{bmatrix},
\qquad
x_\kappa = \begin{bmatrix} q_1 \\ v_1 \end{bmatrix},
\qquad
x_u = \begin{bmatrix} q_2 \\ v_2 \\ q_3 \\ v_3 \end{bmatrix}.
```

The jointly discretized nominal model is partitioned as

```math
\begin{bmatrix} x_\kappa^+ \\ x_u^+ \end{bmatrix}
=
\begin{bmatrix} A_\kappa & A_c \\ A_f & A_u \end{bmatrix}
\begin{bmatrix} x_\kappa \\ x_u \end{bmatrix}
+
\begin{bmatrix} B_\kappa \\ B_u \end{bmatrix} u .
```

The **unknown subsystem** is represented through data:

```math
U_p g = u_{\mathrm{ini}},
\qquad
Y_{u,p} g - \sigma = y_{u,\mathrm{ini}},
\qquad
U = U_f g,
\qquad
Y_u = Y_{u,f} g .
```

The **known subsystem** evolves according to

```math
x_\kappa(k+1) = A_\kappa x_\kappa(k) + A_y y_u(k) + B_\kappa u(k).
```

Condensing the known-state recursion gives

```math
X = S_{\mathrm{full}} \, x_\kappa(0) + G_{\mathrm{full}} \, g .
```

HDeePC therefore combines explicit subsystem physics with a behavioral representation of the unknown subsystem.

---

### Pole-Informed DeePC (PZ-DeePC)

PZ-DeePC embeds known pole locations directly into the behavioral predictor. For $s_p$ known poles,

```math
P(z) = \prod_{i=1}^{s_p} (z - \lambda_i)
     = z^{s_p} + a_1 z^{s_p - 1} + \cdots + a_{s_p}.
```

**Output filtering**

```math
y_k' = y_k + a_1 y_{k-1} + \cdots + a_{s_p} y_{k-s_p},
\qquad
y' = Q_y \, y,
\qquad
H_y' = Q_y H_y .
```

The filtered Hankel matrix is partitioned into a shortened filtered-past block $`Y'_{p-s_p}`$ and a future block $Y_f'$.

**Filtered behavioral equations**

```math
U_p g = u_{\mathrm{ini}},
\qquad
Y'_{p-s_p} g - \sigma = y_{\mathrm{ini}}',
\qquad
y_f' = Y_f' g .
```

**Reconstruction of the physical trajectory**

```math
X = M Y_f' g + d_{\mathrm{past}} = G_x g + d_{\mathrm{past}},
\qquad
G_x = M Y_f' .
```

**Optimization problem**

```math
\begin{aligned}
\min_{g,\,\sigma,\,u_s}\quad
& \| G_x g + d_{\mathrm{past}} - X_r \|_{Q_N}^2
  + \| U_f g - \mathbf{1}_N u_s \|_{R_N}^2 \\
& + \lambda_g \| g \|_2^2 + \lambda_\sigma \| \sigma \|_2^2 .
\end{aligned}
```

Both **2-pole** and **4-pole** variants are implemented.

---

## Regularization

### Behavioral coefficient regularization

With noisy data, the future Hankel block can be written as $Y_f = Y_f^0 + E_f$, where $Y_f^0$ is the noise-free trajectory matrix and $E_f$ the noise contribution. The prediction becomes

```math
Y_f g = Y_f^0 g + E_f g,
\qquad
\| E_f g \|_2 \le \| E_f \|_2 \, \| g \|_2 .
```

Large coefficient vectors therefore amplify noise contained in the Hankel data. The regularization term $\lambda_g \lVert  g \rVert _2^2$ limits excessive behavioral combinations, resulting in the trade-off

```math
\lambda_g \downarrow \;\Longrightarrow\; \| g \|_2 \uparrow \;\Longrightarrow\; \text{noise amplification} \uparrow,
\qquad
\lambda_g \uparrow \;\Longrightarrow\; \text{trajectory flexibility} \downarrow .
```

$\lambda_g$ is therefore treated as a key robustness parameter.

### Past-trajectory slack

Exact past matching is relaxed through $Y_p g - \sigma = y_{\mathrm{ini}}$ with penalty $\lambda_\sigma \lVert  \sigma \rVert _2^2$.

- **Large** $\lambda_\sigma$ forces the optimizer to explain the noisy past mainly through $g$.
- **Small** $\lambda_\sigma$ allows larger mismatch between the measured past and the behavioral trajectory.

The interaction between $\lambda_g$ and $\lambda_\sigma$ is important under noisy training data.

### Projection regularization

As an alternative to $J_g = \lambda_g \lVert  g \rVert _2^2$, projection-based regularization is investigated:

```math
J_g = \lambda_g \, \| (I - \Pi) \, g \|_2^2 ,
```

where $\Pi$ projects onto the relevant behavioral subspace.

| | Penalizes |
|---|---|
| Direct $L_2$ | the magnitude of every component of $g$ |
| Projection | mainly coefficient directions associated with ambiguity or weak behavioral constraints |

This may be particularly useful for PZ-DeePC, since pole filtering changes both the scale of the behavioral coordinates and the statistical properties of the noise.

---

## Noise Analysis of Pole Filtering

### Noise transformation

With measured output $y = y_0 + e$, pole filtering gives

```math
y' = Q_y y_0 + Q_y e,
\qquad
e' = Q_y e,
\qquad
\Sigma_{e'} = Q_y \Sigma_e Q_y^{\top}.
```

Even if $e$ is white Gaussian noise, $e'$ is generally **colored**.

For the 4-pole filter used in this project:

| Quantity | Measured amplification |
|---|---|
| Noise standard deviation | $\approx 7.6$ to $8.7\times$ |
| Total noise energy | $\approx 70\times$ |

This exposes a central PZ-DeePC trade-off:

```math
\text{effective-order reduction}
\quad\longleftrightarrow\quad
\text{measurement-noise amplification}.
```

### Power spectral density

The power spectral density describes how signal power is distributed over frequency. For ideal white noise $S_e(f) \approx \text{const}$, and after linear filtering

```math
S_{e'}(\omega) = \left| P\!\left(e^{j\omega}\right) \right|^2 S_e(\omega).
```

The 4-pole PZ filter strongly suppresses low-frequency content, while its gain increases toward the Nyquist frequency. The originally white measurement noise is therefore shaped strongly toward high frequencies, outside the band ($\lesssim 1$ Hz) where the relevant physical dynamics lie.

### Autocorrelation

The autocorrelation at lag $\ell$ is

```math
R_e(\ell) = \mathbb{E}\left[ e_k \, e_{k-\ell} \right],
```

and ideal white noise satisfies $R_e(\ell) = 0$ for $\ell \neq 0$. After pole filtering,

```math
e_k' = e_k + a_1 e_{k-1} + \cdots + a_{s_p} e_{k-s_p},
```

so neighboring filtered samples share several raw noise samples, and $R_{e'}(\ell) \neq 0$ for small nonzero lags.

Since PSD and autocorrelation form a Fourier-transform pair, $S_e(\omega) = \mathcal{F}\{R_e(\ell)\}$:

```math
\text{flat PSD} \;\Longleftrightarrow\; \text{white, uncorrelated noise},
\qquad
\text{shaped PSD} \;\Longleftrightarrow\; \text{colored, correlated noise}.
```

---

## Terminal Formulation Study

Three terminal formulations are investigated.

**1. Hard terminal equality** (original DeePC formulation)

```math
x_N = x_r,
\qquad
u_N = u_s .
```

**2. No terminal constraint**

The terminal equalities are removed completely. This enlarges the feasible trajectory set and lets the running objective alone determine the predicted trajectory.

**3. Noise-aware Frobenius terminal penalty**

For a terminal window of length $n_T$,

```math
E_T =
\begin{bmatrix}
x_{N-n_T+1} - x_r & \cdots & x_N - x_r
\end{bmatrix},
\qquad
J_T = \lambda_T \, \big\| W^{1/2} E_T \big\|_F^2,
\qquad
W = \Sigma_y^{-1}.
```

This penalizes terminal deviations relative to the uncertainty of each state channel instead of enforcing exact terminal equality.

> [!NOTE]
> In the tested noisy-data case, all three terminal formulations produced very similar trajectories. This suggests that the terminal constraint was **not** the dominant source of the observed noisy DeePC degradation.

---

## Diagnostics

| Quantity | Definition |
|---|---|
| Tracking RMSE | $`\mathrm{RMSE}_{q_3} = \sqrt{\frac{1}{N_s} \sum_k \left( q_3(k) - r(k) \right)^2}`$ |
| Behavioral coefficient norm | $\lVert g \rVert_2$ |
| Past-trajectory slack norm | $\lVert \sigma \rVert_2$ |
| One-step prediction error | $`\lVert e_1(k) \rVert_2,\ \ e_1(k) = x(k+1) - \hat x_{1 \mid k}`$ |
| Nonlinear spring exposure | $`100\,\lvert F_{\mathrm{cubic}} \rvert / \lvert F_{\mathrm{linear}} \rvert`$ |
| Optimized steady input | $u_s$, compared with the physical diagnostic $k_1 r$ |
| QP solve time | recorded at every closed-loop step |

---

## Current Observations

1. Large behavioral coefficient vectors can strongly amplify noisy Hankel data.
2. Increasing $\lambda_g$ can significantly improve the robustness of standard DeePC under noisy training data.
3. Excessive $\lambda_g$ can reduce useful trajectory freedom and make the controller overly conservative.
4. The same value of $\lambda_g$ is not necessarily appropriate for every controller, because the behavioral representations have different scaling.
5. PZ-DeePC requires separate regularization tuning because its filtered data have different statistical properties.
6. Pole filtering can strongly amplify white measurement noise.
7. The 4-pole filter amplified the noise standard deviation by approximately $7.6$–$8.7\times$.
8. The corresponding total noise-energy amplification was approximately $70\times$.
9. Pole filtering transforms initially white noise into colored, temporally correlated noise.
10. The tested terminal formulation was not the dominant source of noisy DeePC degradation.
11. Poor prediction can generate aggressive control actions that drive the nonlinear plant outside the region represented by the training data.
12. The interaction between data noise, $g$-regularization, slack regularization and prior-information filtering is a major focus of the current study.

---

## Open Research Questions

- How much prior knowledge is beneficial in data-driven predictive control?
- How do the controllers behave with limited training data?
- How should $\lambda_g$ be selected under noisy data?
- How should $\lambda_\sigma$ be selected relative to $\lambda_g$?
- Is projection regularization preferable to direct $L_2$ regularization?
- How does pole filtering alter measurement-noise statistics?
- How do 2-pole and 4-pole PZ-DeePC compare?
- Does more prior information reduce data requirements?
- Does more prior information also increase noise sensitivity?
- How does nonlinear plant exposure interact with prediction error?
- How does computational complexity scale with the different behavioral formulations?

---

## Development Status

🚧 **Under active development.** Current work includes:

- [ ] $\lambda_g$ sensitivity studies
- [ ] $\lambda_\sigma$ sensitivity studies
- [ ] limited-data experiments
- [ ] noisy-data experiments
- [ ] 2-pole vs. 4-pole PZ-DeePC
- [ ] projection regularization
- [ ] terminal-condition studies
- [ ] PZ-DeePC noise-filter analysis
- [ ] one-step prediction diagnostics
- [ ] nonlinear-exposure analysis
- [ ] computational-load comparison

---

## References

This project builds on literature on:

- Data-Enabled Predictive Control (DeePC)
- Hybrid Data-Enabled Predictive Control (HDeePC)
- Semi-Data-Driven Model Predictive Control (SD-MPC)
- predictive control with prior system knowledge
- pole-informed data-driven prediction
- model predictive tracking

Detailed academic references are documented in the accompanying research report.

---

## Author

**Erdem Özdemir**

Comparative study of model-based, data-driven, hybrid and prior-informed predictive control methods for nonlinear dynamical systems.

