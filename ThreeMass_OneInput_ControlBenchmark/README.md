# Three-Mass / One-Input Predictive-Control Benchmark

This project replaces the previous two-mass benchmark with a three-mass chain

wall -- (k1,c1) -- m1 -- (k12,c12) -- m2 -- (k23 + cubic k3, c23) -- m3

with one force input applied to mass 1 and q3 as the main tracked position.

State:
    x = [q1; v1; q2; v2; q3; v3]

True nonlinear force between masses 2 and 3:
    F23 = k23*(q3-q2) + k3*(q3-q2)^3 + c23*(v3-v2)

The nominal model omits the cubic term only.

At coordinated equilibrium:
    q1 = q2 = q3 = r
    v1 = v2 = v3 = 0
    u_eq = k1*r

so the true and nominal plants share the same equilibrium. The nonlinearity is
therefore a transient/amplitude-dependent mismatch, not a steady-state one.

Folders
-------
common/      plant, parameters, utilities, model analysis
MPC/         nominal linear MPC
DeePC/       standard full-state DeePC
SD-MPC/      nominal model + data-driven residual
HDeePC/      QP HDeePC using known m1 dynamics and data-driven m2+m3 dynamics
PZ-DeePC/    two-pole and four-pole informed variants
Comparison/  common comparison script
data/        generated training data
results/     saved result MAT files

First run
---------
1. Run startup_project.m
2. Run common/analyze_three_mass_model.m
3. Run DeePC/generate_deepc_data.m
4. Run SD-MPC/generate_sdmpc_data.m
5. Run whichever controller runner you want.

Important HDeePC split
----------------------
Known states:
    x_kappa = [q1; v1]

Unknown/data-driven outputs:
    y_u = [q2; v2; q3; v3]

The known mass-1 model is linear and coupled to y_u through q2,v2. This matches
the QP HDeePC structure where known-state equations are explicit and only the
unknown output trajectory is represented by Hankel data.

PZ-DeePC
--------
Two variants are included:
    pzdeepc_setup_2pole.m
    pzdeepc_setup_4pole.m

The 2-pole version uses the slowest/least-damped nominal conjugate pair.
The 4-pole version uses the two slowest conjugate pairs.

Theory caveat
-------------
The HDeePC and pole-informed papers derive their strongest equivalence results
for linear systems. Here the true plant is nonlinear. The benchmark therefore
uses those methods as structured/prior-informed predictive controllers on a
nonlinear plant, with the prior derived from the nominal/local linear model.
