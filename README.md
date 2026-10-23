# Comparative Study of Data-Driven Control Methods

This repository contains a comparative study of model-based, data-driven, hybrid, and prior-informed predictive control methods applied to a nonlinear three-mass spring-damper system.

The main objective is to investigate how different amounts and forms of prior system knowledge affect:

- closed-loop reference tracking,
- prediction accuracy,
- robustness to noisy and limited data,
- optimization complexity,
- behavioral coefficient magnitude,
- nonlinear plant excitation,
- and computational effort.

The implemented controllers are:

- **Model Predictive Control (MPC)**
- **Data-Enabled Predictive Control (DeePC)**
- **Semi-Data-Driven MPC (SD-MPC)**
- **Hybrid Data-Enabled Predictive Control (HDeePC)**
- **Pole-Informed DeePC (PZ-DeePC)**

All controllers are tested on the same nonlinear plant and use the same reference trajectory, physical constraints, prediction horizon, and cost structure as far as their formulations permit.

---

# 1. Three-Mass Benchmark System

The benchmark system is a three-mass spring-damper chain

```text
wall -- (k1,c1) -- m1 -- (k12,c12) -- m2 -- (k23,c23,k3) -- m3
```

with a single force input applied to the first mass.

The state vector is

```math
x =
\begin{bmatrix}
q_1 &
v_1 &
q_2 &
v_2 &
q_3 &
v_3
\end{bmatrix}^{\top}
```

and the input is

```math
u \in \mathbb{R}.
```

The main controlled output is the third-mass position $q_3$.

The true nonlinear dynamics are

```math
\dot q_1 = v_1
```

```math
m_1\dot v_1 =
u-k_1q_1-c_1v_1
+k_{12}(q_2-q_1)
+c_{12}(v_2-v_1)
```

```math
\dot q_2 = v_2
```

```math
m_2\dot v_2 =
-k_{12}(q_2-q_1)
-c_{12}(v_2-v_1)
+k_{23}(q_3-q_2)
+k_3(q_3-q_2)^3
+c_{23}(v_3-v_2)
```

```math
\dot q_3 = v_3
```

```math
m_3\dot v_3 =
-k_{23}(q_3-q_2)
-k_3(q_3-q_2)^3
-c_{23}(v_3-v_2).
```

The cubic spring term

```math
F_{\mathrm{cubic}}
=
k_3(q_3-q_2)^3
```

introduces the main nonlinearity.

The corresponding linear spring force is

```math
F_{\mathrm{linear}}
=
k_{23}(q_3-q_2).
```

A nonlinear-exposure metric is defined as

```math
\eta_{\mathrm{NL}}
=
100
\frac{|F_{\mathrm{cubic}}|}
{|F_{\mathrm{linear}}|}.
```

For $q_3 \neq q_2$,

```math
\eta_{\mathrm{NL}}
=
100
\frac{k_3}{k_{23}}
(q_3-q_2)^2.
```

This metric is used to quantify how strongly a controller drives the true nonlinear plant outside the approximately linear operating region.

---

# 2. Nominal Linear Model

The nominal model neglects the cubic stiffness term.

The continuous-time model is

```math
\dot x = Ax+Bu.
```

With the state ordering

```math
x=
\begin{bmatrix}
q_1 & v_1 & q_2 & v_2 & q_3 & v_3
\end{bmatrix}^{\top},
```

the system matrix is

```math
A=
\begin{bmatrix}
0&1&0&0&0&0\\
-\frac{k_1+k_{12}}{m_1}
&
-\frac{c_1+c_{12}}{m_1}
&
\frac{k_{12}}{m_1}
&
\frac{c_{12}}{m_1}
&
0&0\\
0&0&0&1&0&0\\
\frac{k_{12}}{m_2}
&
\frac{c_{12}}{m_2}
&
-\frac{k_{12}+k_{23}}{m_2}
&
-\frac{c_{12}+c_{23}}{m_2}
&
\frac{k_{23}}{m_2}
&
\frac{c_{23}}{m_2}\\
0&0&0&0&0&1\\
0&0&
\frac{k_{23}}{m_3}
&
\frac{c_{23}}{m_3}
&
-\frac{k_{23}}{m_3}
&
-\frac{c_{23}}{m_3}
\end{bmatrix},
```

and

```math
B=
\begin{bmatrix}
0\\
1/m_1\\
0\\
0\\
0\\
0
\end{bmatrix}.
```

The discrete nominal model is obtained using zero-order hold,

```math
x_{k+1}=A_dx_k+B_du_k.
```

This model is used directly in MPC and partially in SD-MPC and HDeePC.

---

# 3. Repository Structure

```text
Comparative-Study-of-Data-Driven-Control-Methods/
│
├── common/
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
├── Comparison/
│   └── ...
│
├── tests/
│   ├── deepc_terminal_study.m
│   ├── pzdeepc_noise_filter_study.m
│   └── ...
│
├── results/
├── MODEL_DERIVATION.md
├── startup_project.m
└── README.md
```

Standalone controller scripts simulate only the corresponding controller.

Cross-controller experiments are kept inside `Comparison/` and `tests/`.

---

# 4. DeePC Training Data

The data-driven controllers use measured input-output trajectories

```math
u^d=
\begin{bmatrix}
u_0 & u_1 & \cdots & u_{T-1}
\end{bmatrix}
```

and

```math
y^d=
\begin{bmatrix}
y_0 & y_1 & \cdots & y_{T-1}
\end{bmatrix}.
```

Full-state output measurement is assumed,

```math
y_k=x_k.
```

The Hankel depth is

```math
L=T_{\mathrm{ini}}+N,
```

where:

- $T_{\mathrm{ini}}$ is the past horizon,
- $N$ is the prediction horizon.

A block Hankel matrix is constructed as

```math
H_L(u^d)
=
\begin{bmatrix}
u_0 & u_1 & \cdots\\
u_1 & u_2 & \cdots\\
\vdots & \vdots & \ddots\\
u_{L-1} & u_L & \cdots
\end{bmatrix}.
```

The number of behavioral coefficient variables is

```math
n_g=T-L+1.
```

For standard DeePC, the training input is checked for persistent excitation of approximately order

```math
L+n,
```

where the nominal system order is

```math
n=6.
```

---

# 5. Measurement Noise

Noisy output data are generated as

```math
y_k^{\mathrm{meas}}
=
y_k^{\mathrm{true}}+e_k,
```

with

```math
e_k\sim\mathcal N(0,\Sigma_y).
```

The standard deviation of each state channel is scaled using the RMS value of its clean signal:

```math
\sigma_{e_i}
=
\eta
\sqrt{
\frac{1}{T}
\sum_{k=0}^{T-1}
y_i(k)^2
}.
```

For example, $\eta=0.05$ corresponds to a noise standard deviation equal to 5% of the RMS magnitude of each state channel.

The clean physical state is stored separately from the noisy measured state so that physical diagnostics such as nonlinear spring exposure are not contaminated by artificial measurement noise.

---

# 6. Common Tracking Formulation

The desired coordinated state is

```math
x_r=
\begin{bmatrix}
r&0&r&0&r&0
\end{bmatrix}^{\top}.
```

The stacked prediction reference is

```math
X_r=
\mathbf 1_N\otimes x_r.
```

The common tracking cost is

```math
J_{\mathrm{track}}
=
\|X-X_r\|_{Q_N}^2
+
\|U-\mathbf 1_Nu_s\|_{R_N}^2,
```

with

```math
Q_N=I_N\otimes Q
```

and

```math
R_N=I_N\otimes R.
```

The steady input $u_s$ is optimized internally.

For a coordinated static configuration,

```math
q_1=q_2=q_3=r,
```

```math
v_1=v_2=v_3=0,
```

the physical equilibrium input is

```math
u_{\mathrm{eq}}=k_1r.
```

This analytical equilibrium value is not supplied to the controllers and is used only as a diagnostic.

---

# 7. Model Predictive Control

MPC uses the nominal discrete model

```math
x_{k+1}=A_dx_k+B_du_k.
```

The stacked prediction can be written as

```math
X=S_xx_0+S_uU.
```

The MPC optimization problem is

```math
\begin{aligned}
\min_{U,u_s}\quad
&
\|X-X_r\|_{Q_N}^2
+
\|U-\mathbf 1_Nu_s\|_{R_N}^2
\\
\text{s.t.}\quad
&
X=S_xx_0+S_uU,
\\
&
|u_k|\le u_{\max},
\\
&
|q_i(k)|\le q_{\max}.
\end{aligned}
```

MPC is used as the model-based benchmark.

---

# 8. Standard DeePC

The Hankel matrices are partitioned into past and future blocks:

```math
H_L(u^d)
=
\begin{bmatrix}
U_p\\
U_f
\end{bmatrix},
```

```math
H_L(y^d)
=
\begin{bmatrix}
Y_p\\
Y_f
\end{bmatrix}.
```

The behavioral equations are

```math
U_pg=u_{\mathrm{ini}},
```

```math
Y_pg-\sigma=y_{\mathrm{ini}},
```

```math
U=U_fg,
```

```math
X=Y_fg.
```

The decision vector is

```math
z=
\begin{bmatrix}
g\\
\sigma\\
u_s
\end{bmatrix}.
```

The DeePC optimization problem is

```math
\begin{aligned}
\min_{g,\sigma,u_s}\quad
&
\|Y_fg-X_r\|_{Q_N}^2
+
\|U_fg-\mathbf 1_Nu_s\|_{R_N}^2
\\
&
+\lambda_g\|g\|_2^2
+\lambda_\sigma\|\sigma\|_2^2
\\
\text{s.t.}\quad
&
U_pg=u_{\mathrm{ini}},
\\
&
Y_pg-\sigma=y_{\mathrm{ini}},
\\
&
|U_fg|\le u_{\max},
\\
&
|S_qY_fg|\le q_{\max}.
\end{aligned}
```

---

# 9. Behavioral Coefficient Regularization

For noisy data, the future Hankel matrix can be written as

```math
Y_f=Y_f^0+E_f,
```

where $Y_f^0$ is the noise-free trajectory matrix and $E_f$ contains the measurement-noise contribution.

The DeePC prediction becomes

```math
Y_fg
=
Y_f^0g+E_fg.
```

The noise contribution is therefore

```math
E_fg.
```

Using the induced matrix norm,

```math
\|E_fg\|_2
\le
\|E_f\|_2\|g\|_2.
```

This shows why large behavioral coefficient vectors can amplify noise contained in the Hankel trajectory.

The regularization term

```math
\lambda_g\|g\|_2^2
```

is therefore used to limit excessive behavioral combinations.

The observed tradeoff is

```math
\lambda_g\downarrow
\quad\Longrightarrow\quad
\|g\|_2\uparrow
\quad\Longrightarrow\quad
\text{noise amplification}\uparrow.
```

However, excessive regularization can reduce useful trajectory freedom:

```math
\lambda_g\uparrow
\quad\Longrightarrow\quad
\text{trajectory flexibility}\downarrow.
```

Therefore $\lambda_g$ is treated as an important robustness parameter.

---

# 10. Past-Trajectory Slack

For noisy measurements, exact past matching is relaxed using

```math
Y_pg-\sigma=y_{\mathrm{ini}}.
```

The slack is penalized by

```math
\lambda_\sigma\|\sigma\|_2^2.
```

Large $\lambda_\sigma$ forces the optimizer to explain noisy past measurements mainly through $g$.

Smaller $\lambda_\sigma$ allows greater mismatch between the measured past and the behavioral trajectory.

The interaction between $\lambda_g$ and $\lambda_\sigma$ is therefore important under noisy training data.

---

# 11. Semi-Data-Driven MPC

SD-MPC decomposes the full state prediction into nominal and residual components:

```math
X=X_M+X_D.
```

The nominal prediction is

```math
X_M=S_xx_M+S_uU.
```

The residual training signal is

```math
r_d=y_d-x_{M,d}.
```

A residual Hankel matrix is constructed as

```math
H_L(r_d)
=
\begin{bmatrix}
R_p\\
R_f
\end{bmatrix}.
```

The residual behavioral equations are

```math
U_pg=u_{\mathrm{ini}},
```

```math
R_pg-\sigma=r_{\mathrm{ini}},
```

```math
U=U_fg,
```

```math
X_D=R_fg.
```

The complete prediction becomes

```math
X=
S_xx_M
+
S_uU_fg
+
R_fg.
```

Thus SD-MPC uses the nominal model for the main dynamics while the data-driven component predicts the model mismatch.

---

# 12. Hybrid DeePC

HDeePC partitions the state into known and unknown subsystems,

```math
x=
\begin{bmatrix}
x_\kappa\\
x_u
\end{bmatrix}.
```

For the three-mass benchmark,

```math
x_\kappa=
\begin{bmatrix}
q_1\\
v_1
\end{bmatrix},
```

and

```math
x_u=
\begin{bmatrix}
q_2\\
v_2\\
q_3\\
v_3
\end{bmatrix}.
```

The jointly discretized nominal model is partitioned as

```math
\begin{bmatrix}
x_\kappa^+\\
x_u^+
\end{bmatrix}
=
\begin{bmatrix}
A_\kappa&A_c\\
A_f&A_u
\end{bmatrix}
\begin{bmatrix}
x_\kappa\\
x_u
\end{bmatrix}
+
\begin{bmatrix}
B_\kappa\\
B_u
\end{bmatrix}u.
```

The unknown subsystem is represented through data:

```math
U_pg=u_{\mathrm{ini}},
```

```math
Y_{u,p}g-\sigma=y_{u,\mathrm{ini}},
```

```math
U=U_fg,
```

```math
Y_u=Y_{u,f}g.
```

The known subsystem evolves according to

```math
x_\kappa(k+1)
=
A_\kappa x_\kappa(k)
+
A_y y_u(k)
+
B_\kappa u(k).
```

After condensing the known-state recursion,

```math
X=
S_{\mathrm{full}}x_\kappa(0)
+
G_{\mathrm{full}}g.
```

HDeePC therefore combines explicit subsystem physics with a behavioral representation of the unknown subsystem.

---

# 13. Pole-Informed DeePC

PZ-DeePC incorporates known poles directly into the behavioral predictor.

For $s_p$ known poles,

```math
P(z)
=
\prod_{i=1}^{s_p}(z-\lambda_i).
```

Equivalently,

```math
P(z)
=
z^{s_p}
+a_1z^{s_p-1}
+\cdots
+a_{s_p}.
```

The output filtering operation is

```math
y_k'
=
y_k
+a_1y_{k-1}
+\cdots
+a_{s_p}y_{k-s_p}.
```

In matrix form,

```math
y'=Q_y y.
```

The filtered output Hankel matrix is constructed using

```math
H_y'=Q_yH_y.
```

It is partitioned into a shortened filtered-past block and a future block:

```math
Y'_{p-s_p},
\qquad
Y_f'.
```

The filtered behavioral equations are

```math
U_pg=u_{\mathrm{ini}},
```

```math
Y'_{p-s_p}g-\sigma
=
y_{\mathrm{ini}}'.
```

The future filtered prediction is

```math
y_f'=Y_f'g.
```

The physical trajectory is reconstructed using

```math
X=
MY_f'g+d_{\mathrm{past}}.
```

Defining

```math
G_x=MY_f',
```

gives

```math
X=G_xg+d_{\mathrm{past}}.
```

The PZ-DeePC optimization problem is

```math
\begin{aligned}
\min_{g,\sigma,u_s}\quad
&
\|G_xg+d_{\mathrm{past}}-X_r\|_{Q_N}^2
\\
&
+
\|U_fg-\mathbf 1_Nu_s\|_{R_N}^2
\\
&
+
\lambda_g\|g\|_2^2
+
\lambda_\sigma\|\sigma\|_2^2.
\end{aligned}
```

Both 2-pole and 4-pole PZ-DeePC variants are implemented.

---

# 14. PZ-DeePC Noise Transformation

Suppose the measured output is

```math
y=y_0+e.
```

After pole filtering,

```math
y'
=
Q_yy_0+Q_ye.
```

The filtered noise is therefore

```math
e'=Q_ye.
```

Its covariance becomes

```math
\Sigma_{e'}
=
Q_y\Sigma_eQ_y^\top.
```

Even if $e$ is white Gaussian noise, $e'$ is generally colored.

For the current 4-pole filter, the measured standard-deviation amplification was approximately

```math
7.6 \text{ to } 8.7.
```

The measured total noise-energy amplification was approximately

```math
70\times.
```

This reveals an important PZ-DeePC tradeoff:

```math
\text{effective-order reduction}
\quad\longleftrightarrow\quad
\text{measurement-noise amplification}.
```

---

# 15. Power Spectral Density

Power spectral density describes how signal variance or power is distributed over frequency.

For ideal white noise,

```math
S_e(f)\approx\text{constant}.
```

After linear filtering,

```math
S_{e'}(\omega)
=
|P(e^{j\omega})|^2S_e(\omega).
```

The main nominal mechanical frequencies of the benchmark are approximately

```math
f_1\approx0.256\ \mathrm{Hz},
```

```math
f_2\approx0.678\ \mathrm{Hz},
```

and

```math
f_3\approx1.016\ \mathrm{Hz}.
```

With

```math
T_s=0.05\ \mathrm{s},
```

the sampling frequency is

```math
f_s=20\ \mathrm{Hz},
```

and the Nyquist frequency is

```math
f_N=10\ \mathrm{Hz}.
```

The important physical dynamics therefore lie mainly below approximately $1$ Hz.

The 4-pole PZ filter strongly suppresses low-frequency content while its gain increases toward the Nyquist frequency.

As a result, the originally white measurement noise becomes strongly shaped toward higher frequencies.

---

# 16. Autocorrelation

Autocorrelation measures how strongly a signal is correlated with delayed versions of itself.

At lag $\ell$,

```math
R_e(\ell)
=
\mathbb E[e_k e_{k-\ell}].
```

Ideal white noise satisfies

```math
R_e(\ell)=0,
\qquad
\ell\neq0.
```

After pole filtering,

```math
e_k'
=
e_k
+a_1e_{k-1}
+\cdots
+a_{s_p}e_{k-s_p}.
```

Neighboring filtered samples therefore share several of the same raw noise samples.

Consequently,

```math
R_{e'}(\ell)\neq0
```

for small nonzero lags.

Power spectral density and autocorrelation are Fourier-transform pairs:

```math
S_e(\omega)
=
\mathcal F\{R_e(\ell)\}.
```

Therefore,

```math
\text{flat PSD}
\quad\Longleftrightarrow\quad
\text{white uncorrelated noise},
```

while

```math
\text{shaped PSD}
\quad\Longleftrightarrow\quad
\text{colored correlated noise}.
```

---

# 17. Terminal Formulation Study

Three terminal formulations are investigated.

## 17.1 Hard Terminal Equality

The original DeePC formulation uses

```math
x_N=x_r
```

and

```math
u_N=u_s.
```

## 17.2 No Terminal Constraint

The terminal equalities are removed completely.

This enlarges the feasible trajectory set and allows the running objective alone to determine the predicted trajectory.

## 17.3 Noise-Aware Frobenius Terminal Penalty

For a terminal window,

```math
E_T=
\begin{bmatrix}
x_{N-n_T+1}-x_r &
\cdots &
x_N-x_r
\end{bmatrix}.
```

A weighted Frobenius penalty is introduced:

```math
J_T=
\lambda_T
\|W^{1/2}E_T\|_F^2.
```

A noise-aware state weighting can be chosen as

```math
W=\Sigma_y^{-1}.
```

This penalizes terminal deviations relative to the uncertainty of each state channel rather than enforcing exact terminal equality.

In the tested noisy-data case, all three terminal formulations produced very similar trajectories.

This suggests that the terminal constraint was not the dominant source of the observed noisy DeePC degradation.

---

# 18. Projection Regularization

Standard DeePC regularization uses

```math
J_g=
\lambda_g\|g\|_2^2.
```

An alternative is projection-based regularization:

```math
J_g
=
\lambda_g
\|(I-\Pi)g\|_2^2.
```

Here, $\Pi$ projects onto the relevant behavioral subspace.

The conceptual distinction is:

- direct $L_2$ regularization penalizes the magnitude of every component of $g$,
- projection regularization penalizes mainly coefficient directions associated with ambiguity or weak behavioral constraints.

This may be particularly useful for PZ-DeePC because pole filtering changes both the scale of the behavioral coordinates and the statistical properties of the noise.

---

# 19. Main Diagnostic Quantities

## Tracking RMSE

```math
\mathrm{RMSE}_{q_3}
=
\sqrt{
\frac{1}{N_s}
\sum_k
(q_3(k)-r(k))^2
}.
```

## Behavioral coefficient norm

```math
\|g\|_2.
```

## Past-trajectory slack norm

```math
\|\sigma\|_2.
```

## One-step prediction error

```math
e_1(k)
=
x(k+1)-\hat x_{1|k},
```

with

```math
\|e_1(k)\|_2.
```

## Nonlinear spring exposure

```math
100
\frac{|F_{\mathrm{cubic}}|}
{|F_{\mathrm{linear}}|}.
```

## Optimized steady input

The internally optimized steady input $u_s$ is compared with the physical diagnostic

```math
k_1r.
```

## QP solve time

Solver time is recorded at every closed-loop step to compare computational complexity across the different methods.

---

# 20. Current Experimental Observations

The experiments performed so far indicate the following.

1. Large behavioral coefficient vectors can strongly amplify noisy Hankel data.

2. Increasing $\lambda_g$ can significantly improve the robustness of standard DeePC under noisy training data.

3. Excessive $\lambda_g$ can reduce useful trajectory freedom and make the controller overly conservative.

4. The same numerical value of $\lambda_g$ is not necessarily appropriate for every controller because the behavioral representations have different scaling.

5. PZ-DeePC requires separate regularization tuning because its filtered data have different statistical properties.

6. Pole filtering can strongly amplify white measurement noise.

7. The 4-pole filter tested in this project amplified noise standard deviation by approximately $7.6$ to $8.7$ times.

8. The corresponding total noise-energy amplification was approximately $70$ times.

9. Pole filtering transforms initially white noise into colored, temporally correlated noise.

10. The tested terminal formulation was not the dominant source of noisy DeePC degradation.

11. Poor prediction can generate aggressive control actions that drive the nonlinear plant outside the region represented by the training data.

12. The interaction between data noise, $g$-regularization, slack regularization, and prior-information filtering is a major focus of the current study.

---

# 21. Current Research Questions

The project currently investigates:

- How much prior knowledge is beneficial in predictive data-driven control?
- How do the controllers behave with limited training datasets?
- How should $\lambda_g$ be selected under noisy data?
- How should $\lambda_\sigma$ be selected relative to $\lambda_g$?
- Is projection regularization preferable to direct $L_2$ regularization?
- How does pole filtering alter measurement-noise statistics?
- How do 2-pole and 4-pole PZ-DeePC compare?
- Does increasing prior information reduce data requirements?
- Does increasing prior information also increase noise sensitivity?
- How does nonlinear plant exposure interact with prediction error?
- How does computational complexity scale with the different behavioral formulations?

---

# 22. Running the Project

Start MATLAB in the repository root and run:

```matlab
startup_project
```

Generate the training data using:

```matlab
generate_deepc_data
```

Individual controllers can then be run through their standalone scripts.

Examples:

```matlab
run_mpc_closed_loop
run_deepc_closed_loop
run_sdmpc_closed_loop
run_hdeepc_closed_loop
```

PZ-DeePC can be run using the corresponding pole-prior setup.

Cross-controller studies and sensitivity tests are located inside `Comparison/` and `tests/`.

---

# 23. MATLAB Requirements

The project uses MATLAB and mainly relies on:

- Optimization Toolbox
- Control System Toolbox

Quadratic programs are solved using:

```matlab
quadprog
```

with the convex interior-point algorithm.

---

# 24. Development Status

The project is under active development.

Current work includes:

- $\lambda_g$ sensitivity studies,
- $\lambda_\sigma$ sensitivity studies,
- limited-data experiments,
- noisy-data experiments,
- 2-pole versus 4-pole PZ-DeePC,
- projection regularization,
- terminal-condition studies,
- PZ-DeePC noise-filter analysis,
- one-step prediction diagnostics,
- nonlinear-exposure analysis,
- computational-load comparison.

---

# 25. References

The project is based on literature related to:

- Data-Enabled Predictive Control,
- Hybrid Data-Enabled Predictive Control,
- Semi-Data-Driven Model Predictive Control,
- predictive control with prior system knowledge,
- pole-informed data-driven prediction,
- model predictive tracking.

The detailed academic references are documented in the accompanying research report.

---

# 26. Author

**Erdem Özdemir**

Comparative study of model-based, data-driven, hybrid, and prior-informed predictive control methods for nonlinear dynamical systems.
