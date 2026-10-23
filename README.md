# Comparative Study of Data-Driven Control Methods

This repository contains a comparative study of model-based, data-driven, hybrid, and prior-informed predictive control methods applied to a nonlinear three-mass spring-damper system.

The objective is to compare how different amounts and forms of prior system knowledge affect:

- closed-loop reference tracking,
- prediction accuracy,
- robustness to noisy and limited data,
- optimization complexity,
- behavioral coefficient magnitude,
- sensitivity to nonlinear plant dynamics,
- and computational effort.

The implemented controllers are:

- **Model Predictive Control (MPC)**
- **Data-Enabled Predictive Control (DeePC)**
- **Semi-Data-Driven MPC (SD-MPC)**
- **Hybrid Data-Enabled Predictive Control (HDeePC)**
- **Pole-informed DeePC (PZ-DeePC)**

All controllers are tested on the same nonlinear plant and, where possible, use the same reference trajectory, constraints, prediction horizon, and cost structure.

---

# 1. Three-Mass Benchmark System

The benchmark system is a three-mass spring-damper chain

```text
wall -- (k1,c1) -- m1 -- (k12,c12) -- m2 -- (k23,c23,k3) -- m3
```

with a single force input applied to the first mass.

The state is

$$
x =
\begin{bmatrix}
q_1 &
v_1 &
q_2 &
v_2 &
q_3 &
v_3
\end{bmatrix}^{\top}.
$$

The input is

$$
u \in \mathbb{R}.
$$

The main controlled output is the third-mass position

$$
q_3.
$$

The true nonlinear dynamics are

$$
\dot q_1 = v_1,
$$

$$
m_1\dot v_1 =
u-k_1q_1-c_1v_1
+k_{12}(q_2-q_1)
+c_{12}(v_2-v_1),
$$

$$
\dot q_2 = v_2,
$$

$$
m_2\dot v_2 =
-k_{12}(q_2-q_1)
-c_{12}(v_2-v_1)
+k_{23}(q_3-q_2)
+k_3(q_3-q_2)^3
+c_{23}(v_3-v_2),
$$

$$
\dot q_3 = v_3,
$$

$$
m_3\dot v_3 =
-k_{23}(q_3-q_2)
-k_3(q_3-q_2)^3
-c_{23}(v_3-v_2).
$$

The cubic spring term

$$
F_{\mathrm{cubic}}
=
k_3(q_3-q_2)^3
$$

introduces the main plant nonlinearity.

A nonlinear-exposure diagnostic is defined as

$$
\eta_{\mathrm{NL}}
=
100
\frac{|F_{\mathrm{cubic}}|}
{|F_{\mathrm{linear}}|}.
$$

Since

$$
F_{\mathrm{linear}}
=
k_{23}(q_3-q_2),
$$

the ratio becomes

$$
\eta_{\mathrm{NL}}
=
100
\frac{k_3}{k_{23}}
(q_3-q_2)^2.
$$

This quantity is used to determine how strongly a controller drives the true nonlinear plant outside the approximately linear operating region.

---

# 2. Nominal Linear Model

The nominal model neglects the cubic spring term.

The continuous-time model is

$$
\dot x = Ax+Bu.
$$

With the state ordering

$$
x=
\begin{bmatrix}
q_1 & v_1 & q_2 & v_2 & q_3 & v_3
\end{bmatrix}^{\top},
$$

the state matrix is

$$
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
$$

and

$$
B=
\begin{bmatrix}
0\\
1/m_1\\
0\\
0\\
0\\
0
\end{bmatrix}.
$$

The discrete nominal model is obtained using zero-order hold,

$$
x_{k+1}=A_dx_k+B_du_k.
$$

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
│   └── sdmpc_control.m
│
├── HDeePC/
│   ├── hdeepc_setup.m
│   └── hdeepc_control.m
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
├── tests/
├── results/
├── MODEL_DERIVATION.md
├── startup_project.m
└── README.md
```

Standalone controller scripts simulate only the corresponding controller.

Cross-controller studies are kept in `Comparison/` and `tests/`.

---

# 4. DeePC Training Data

The data-driven controllers use a measured input-output trajectory

$$
u^d=
\begin{bmatrix}
u_0 & u_1 & \cdots & u_{T-1}
\end{bmatrix}
$$

and

$$
y^d=
\begin{bmatrix}
y_0 & y_1 & \cdots & y_{T-1}
\end{bmatrix}.
$$

Full-state output measurement is assumed,

$$
y_k=x_k.
$$

For a Hankel depth

$$
L=T_{\mathrm{ini}}+N,
$$

the block Hankel matrix is constructed as

$$
H_L(u^d)
=
\begin{bmatrix}
u_0 & u_1 & \cdots\\
u_1 & u_2 & \cdots\\
\vdots & \vdots & \ddots\\
u_{L-1} & u_L & \cdots
\end{bmatrix}.
$$

The number of behavioral coefficient variables is

$$
n_g=T-L+1.
$$

For standard DeePC, the training input is checked for persistent excitation of approximately order

$$
L+n,
$$

where

$$
n=6.
$$

---

# 5. Measurement Noise

Noisy output data are generated using

$$
y_k^{\mathrm{meas}}
=
y_k^{\mathrm{true}}+e_k,
$$

where

$$
e_k\sim\mathcal N(0,\Sigma_y).
$$

The standard deviation of each output channel is scaled relative to its clean RMS value:

$$
\sigma_{e_i}
=
\eta
\sqrt{
\frac{1}{T}
\sum_{k=0}^{T-1}
y_i(k)^2
}.
$$

For example,

$$
\eta=0.05
$$

means that the noise standard deviation is equal to 5% of the RMS value of each state channel.

The true clean state is stored separately so that physical diagnostics are not affected by artificial measurement noise.

---

# 6. Common Tracking Formulation

The desired coordinated state is

$$
x_r=
\begin{bmatrix}
r&0&r&0&r&0
\end{bmatrix}^{\top}.
$$

The stacked prediction reference is

$$
X_r=
\mathbf 1_N\otimes x_r.
$$

The general tracking cost is

$$
J_{\mathrm{track}}
=
\|X-X_r\|_{Q_N}^2
+
\|U-\mathbf 1_Nu_s\|_{R_N}^2,
$$

with

$$
Q_N=I_N\otimes Q,
$$

and

$$
R_N=I_N\otimes R.
$$

The steady input

$$
u_s
$$

is optimized internally.

For the physical coordinated equilibrium,

$$
q_1=q_2=q_3=r,
$$

$$
v_1=v_2=v_3=0,
$$

the analytical equilibrium input would be

$$
u_{\mathrm{eq}}=k_1r.
$$

This quantity is not supplied to the controllers and is used only as a diagnostic.

---

# 7. Model Predictive Control

MPC uses

$$
x_{k+1}=A_dx_k+B_du_k.
$$

The lifted prediction is

$$
X=S_xx_0+S_uU.
$$

The QP is

$$
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
$$

MPC is used as the model-based benchmark.

---

# 8. Standard DeePC

The Hankel matrices are partitioned into past and future components,

$$
H_L(u^d)
=
\begin{bmatrix}
U_p\\
U_f
\end{bmatrix},
$$

$$
H_L(y^d)
=
\begin{bmatrix}
Y_p\\
Y_f
\end{bmatrix}.
$$

The behavioral representation is

$$
U_pg=u_{\mathrm{ini}},
$$

$$
Y_pg-\sigma=y_{\mathrm{ini}},
$$

$$
U=U_fg,
$$

$$
X=Y_fg.
$$

The decision vector is

$$
z=
\begin{bmatrix}
g\\
\sigma\\
u_s
\end{bmatrix}.
$$

The DeePC optimization problem is

$$
\boxed{
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
}
$$

---

# 9. Behavioral Coefficient Regularization

For noisy data,

$$
Y_f=Y_f^0+E_f.
$$

The DeePC prediction becomes

$$
Y_fg
=
Y_f^0g+E_fg.
$$

The prediction-noise contribution is therefore

$$
E_fg.
$$

Using the induced norm,

$$
\|E_fg\|_2
\le
\|E_f\|_2
\|g\|_2.
$$

This explains why large behavioral coefficient vectors can strongly amplify noisy Hankel data.

The regularization term

$$
\boxed{
\lambda_g\|g\|_2^2
}
$$

restricts excessive trajectory combinations.

The observed behavior is:

$$
\lambda_g\downarrow
\quad\Rightarrow\quad
\|g\|\uparrow
\quad\Rightarrow\quad
\text{noise sensitivity}\uparrow.
$$

However, excessive regularization may cause

$$
\lambda_g\uparrow
\quad\Rightarrow\quad
\text{trajectory flexibility}\downarrow
\quad\Rightarrow\quad
\text{tracking bias}.
$$

Therefore $\lambda_g$ is tuned through dedicated sensitivity studies.

---

# 10. Past-Trajectory Slack

The noisy past-output equality is relaxed as

$$
Y_pg-\sigma=y_{\mathrm{ini}}.
$$

The slack is penalized using

$$
\boxed{
\lambda_\sigma\|\sigma\|_2^2.
}
$$

Large $\lambda_\sigma$ forces the optimizer to explain the noisy past primarily through $g$.

Smaller $\lambda_\sigma$ permits greater mismatch between the measured past and the behavioral trajectory.

The interaction between

$$
\lambda_g
$$

and

$$
\lambda_\sigma
$$

is therefore important in noisy DeePC.

---

# 11. Semi-Data-Driven MPC

SD-MPC decomposes the prediction into nominal and residual parts:

$$
X=X_M+X_D.
$$

The nominal prediction is

$$
X_M=S_xx_M+S_uU.
$$

The residual training signal is

$$
r_d=y_d-x_{M,d}.
$$

A residual Hankel matrix is formed,

$$
H_L(r_d)
=
\begin{bmatrix}
R_p\\
R_f
\end{bmatrix}.
$$

The behavioral residual equations are

$$
U_pg=u_{\mathrm{ini}},
$$

$$
R_pg-\sigma=r_{\mathrm{ini}},
$$

$$
U=U_fg,
$$

$$
X_D=R_fg.
$$

Therefore the complete prediction is

$$
\boxed{
X=
S_xx_M
+
S_uU_fg
+
R_fg.
}
$$

The data-driven component only models the discrepancy between the nominal model and the nonlinear plant.

---

# 12. Hybrid DeePC

HDeePC partitions the state into known and unknown subsystems,

$$
x=
\begin{bmatrix}
x_\kappa\\
x_u
\end{bmatrix}.
$$

For this project,

$$
x_\kappa=
\begin{bmatrix}
q_1\\
v_1
\end{bmatrix},
$$

and

$$
x_u=
\begin{bmatrix}
q_2\\
v_2\\
q_3\\
v_3
\end{bmatrix}.
$$

The jointly discretized nominal model is partitioned as

$$
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
$$

The unknown subsystem is represented through data,

$$
U_pg=u_{\mathrm{ini}},
$$

$$
Y_{u,p}g-\sigma=y_{u,\mathrm{ini}},
$$

$$
U=U_fg,
$$

$$
Y_u=Y_{u,f}g.
$$

The known subsystem evolves according to

$$
x_\kappa(k+1)
=
A_\kappa x_\kappa(k)
+
A_y y_u(k)
+
B_\kappa u(k).
$$

After condensing the recursion,

$$
\boxed{
X=
S_{\mathrm{full}}x_\kappa(0)
+
G_{\mathrm{full}}g.
}
$$

This allows model knowledge and data-driven trajectory prediction to be combined in the same controller.

---

# 13. Pole-Informed DeePC

PZ-DeePC incorporates known system poles into the behavioral predictor.

For $s_p$ known poles,

$$
P(z)
=
\prod_{i=1}^{s_p}(z-\lambda_i).
$$

Equivalently,

$$
P(z)
=
z^{s_p}
+a_1z^{s_p-1}
+\cdots
+a_{s_p}.
$$

The output filtering operation is

$$
y_k'
=
y_k
+a_1y_{k-1}
+\cdots
+a_{s_p}y_{k-s_p}.
$$

In matrix form,

$$
y'=Q_y y.
$$

The filtered Hankel matrix is

$$
H_y'=Q_yH_y.
$$

It is divided into

$$
Y'_{p-s_p}
$$

and

$$
Y_f'.
$$

The filtered behavioral equations are

$$
U_pg=u_{\mathrm{ini}},
$$

$$
Y'_{p-s_p}g-\sigma
=
y_{\mathrm{ini}}'.
$$

The future filtered prediction is

$$
y_f'=Y_f'g.
$$

The physical trajectory is reconstructed using

$$
\boxed{
X=
MY_f'g+d_{\mathrm{past}}.
}
$$

Defining

$$
G_x=MY_f',
$$

gives

$$
\boxed{
X=G_xg+d_{\mathrm{past}}.
}
$$

The PZ-DeePC optimization problem is

$$
\boxed{
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
}
$$

Both 2-pole and 4-pole variants are implemented.

---

# 14. PZ-DeePC Noise Transformation

If the measured output is

$$
y=y_0+e,
$$

then after pole filtering,

$$
y'
=
Q_yy_0+Q_ye.
$$

The filtered noise is therefore

$$
\boxed{
e'=Q_ye.
}
$$

Its covariance is

$$
\boxed{
\Sigma_{e'}
=
Q_y\Sigma_eQ_y^\top.
}
$$

Even when $e$ is white Gaussian noise, $e'$ is generally colored.

For the current 4-pole filter, the measured standard-deviation amplification was approximately

$$
7.6
\text{ to }
8.7.
$$

The measured total noise-energy amplification was approximately

$$
\boxed{
70\times.
}
$$

This creates an important PZ-DeePC tradeoff:

$$
\boxed{
\text{effective-order reduction}
\quad\leftrightarrow\quad
\text{measurement-noise amplification}.
}
$$

---

# 15. Power Spectral Density

The power spectral density $S_e(f)$ describes how signal variance is distributed over frequency.

White noise approximately satisfies

$$
S_e(f)\approx\text{constant}.
$$

After filtering,

$$
\boxed{
S_{e'}(\omega)
=
|P(e^{j\omega})|^2S_e(\omega).
}
$$

The main nominal mechanical modes of the benchmark are approximately

$$
f_1\approx0.256\ \mathrm{Hz},
$$

$$
f_2\approx0.678\ \mathrm{Hz},
$$

and

$$
f_3\approx1.016\ \mathrm{Hz}.
$$

With

$$
T_s=0.05\ \mathrm{s},
$$

the sampling frequency is

$$
f_s=20\ \mathrm{Hz},
$$

and the Nyquist frequency is

$$
f_N=10\ \mathrm{Hz}.
$$

Therefore most important physical dynamics lie approximately below

$$
1\ \mathrm{Hz}.
$$

The 4-pole filter strongly suppresses low-frequency components while increasing its magnitude toward the Nyquist frequency.

Consequently, the originally white measurement noise is transformed into high-frequency-amplified colored noise.

---

# 16. Autocorrelation

The autocorrelation at lag $\ell$ is

$$
R_e(\ell)
=
\mathbb E[e_k e_{k-\ell}].
$$

Ideal white noise satisfies

$$
R_e(\ell)=0,
\qquad
\ell\neq0.
$$

After filtering,

$$
e_k'
=
e_k
+a_1e_{k-1}
+\cdots
+a_{s_p}e_{k-s_p}.
$$

Adjacent filtered samples therefore share several raw noise samples, causing

$$
R_{e'}(\ell)\neq0
$$

for small nonzero lags.

Power spectral density and autocorrelation are Fourier-transform pairs,

$$
\boxed{
S_e(\omega)
=
\mathcal F\{R_e(\ell)\}.
}
$$

Thus,

$$
\text{flat PSD}
\Longleftrightarrow
\text{white uncorrelated noise},
$$

while

$$
\text{shaped PSD}
\Longleftrightarrow
\text{colored correlated noise}.
$$

---

# 17. Terminal Formulation Study

Three terminal formulations are investigated.

## Hard Terminal Equality

$$
x_N=x_r,
$$

$$
u_N=u_s.
$$

## No Terminal Constraint

The terminal equalities are completely removed.

## Noise-Aware Frobenius Terminal Penalty

For a terminal window,

$$
E_T=
\begin{bmatrix}
x_{N-n_T+1}-x_r &
\cdots &
x_N-x_r
\end{bmatrix}.
$$

The penalty is

$$
\boxed{
J_T=
\lambda_T
\|W^{1/2}E_T\|_F^2.
}
$$

A noise-scaled weighting can be chosen as

$$
W=\Sigma_y^{-1}.
$$

The tested formulations produced very similar trajectories under the investigated noisy-data case, indicating that the terminal constraint was not the dominant cause of the observed degradation.

---

# 18. Projection Regularization

Standard regularization uses

$$
J_g=
\lambda_g\|g\|_2^2.
$$

A projection-based alternative is

$$
\boxed{
J_g
=
\lambda_g
\|(I-\Pi)g\|_2^2.
}
$$

The matrix $\Pi$ projects onto the relevant behavioral subspace.

The conceptual difference is:

- $L_2$ regularization penalizes the magnitude of all components of $g$.
- Projection regularization penalizes mainly components associated with ambiguous or weakly constrained behavioral directions.

This may be especially useful for PZ-DeePC because pole filtering changes the coordinate scaling of the behavioral representation.

---

# 19. Main Diagnostic Quantities

Tracking RMSE:

$$
\mathrm{RMSE}_{q_3}
=
\sqrt{
\frac{1}{N_s}
\sum_k
(q_3(k)-r(k))^2
}.
$$

Behavioral coefficient norm:

$$
\|g\|_2.
$$

Past-trajectory slack norm:

$$
\|\sigma\|_2.
$$

One-step prediction error:

$$
e_{1}(k)
=
x(k+1)-\hat x_{1|k},
$$

with

$$
\|e_1(k)\|_2.
$$

Nonlinear spring exposure:

$$
100
\frac{|F_{\mathrm{cubic}}|}
{|F_{\mathrm{linear}}|}.
$$

The optimized steady input $u_s$ is also compared with

$$
k_1r.
$$

---

# 20. Main Observations So Far

Current experiments suggest:

1. Large behavioral coefficient vectors can strongly amplify noisy Hankel data.

2. Increasing $\lambda_g$ can substantially improve noisy standard-DeePC behavior.

3. Excessive $\lambda_g$ can restrict useful trajectory combinations.

4. The same numerical $\lambda_g$ is not necessarily appropriate for all behavioral representations.

5. PZ-DeePC requires separate regularization tuning because its filtered representation has different scaling and noise statistics.

6. Pole filtering can strongly amplify white measurement noise.

7. The filtered noise becomes colored and temporally correlated.

8. PZ-DeePC therefore exhibits a tradeoff between prior-information benefits and noise robustness.

9. The tested terminal formulation was not the primary source of the noisy DeePC degradation.

10. Poor prediction can cause aggressive control actions that drive the nonlinear plant further outside the region represented by the training data.

---

# 21. Current Research Questions

The project currently investigates:

- How much prior knowledge is beneficial in predictive data-driven control?
- How do DeePC-type controllers behave with limited datasets?
- How should $\lambda_g$ and $\lambda_\sigma$ be selected for noisy data?
- Is projection regularization preferable to direct $L_2$ regularization?
- How does pole filtering modify noise statistics?
- How do 2-pole and 4-pole PZ-DeePC compare?
- How does prior knowledge affect computational complexity?
- How does nonlinear plant exposure interact with prediction error?
- Can prior-informed methods reduce data requirements without sacrificing robustness?

---

# 22. Running the Project

Start MATLAB in the repository root and run:

```matlab
startup_project
```

Generate the training data:

```matlab
generate_deepc_data
```

Run individual controllers using their corresponding standalone scripts.

Examples:

```matlab
run_mpc_closed_loop
run_deepc_closed_loop
run_sdmpc_closed_loop
run_hdeepc_closed_loop
```

Cross-controller studies are located in `Comparison/` and `tests/`.

---

# 23. MATLAB Requirements

The project uses MATLAB and relies mainly on:

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
- PZ noise-filter analysis,
- prediction-error diagnostics,
- nonlinear-exposure analysis.

---

# 25. Author

**Erdem Özdemir**

Comparative study of model-based, data-driven, hybrid, and prior-informed predictive control methods for nonlinear dynamical systems.
