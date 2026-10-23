# Three-Mass Model Derivation

Topology:

wall -- (k1,c1) -- m1 -- (k12,c12) -- m2 -- (k23,k3,c23) -- m3

Input u acts on mass 1.

Define

dq12 = q2-q1
dv12 = v2-v1

dq23 = q3-q2
dv23 = v3-v2

Then

F12 = k12*dq12 + c12*dv12

F23 = k23*dq23 + k3*dq23^3 + c23*dv23

and

q1dot = v1
m1*v1dot = u - k1*q1 - c1*v1 + F12

q2dot = v2
m2*v2dot = -F12 + F23

q3dot = v3
m3*v3dot = -F23

At static coordinated equilibrium,

q1=q2=q3=r
v1=v2=v3=0

hence dq12=dq23=0, F12=F23=0, and

u_eq = k1*r.

This is the key reason for choosing the nonlinear spring between masses 2 and 3:
the cubic nonlinearity affects transient relative motion but does not change the
static equilibrium force.

Chosen parameters
-----------------
m1 = 1.0 kg
m2 = 1.5 kg
m3 = 1.2 kg

k1  = 20 N/m
k12 = 15 N/m
k23 = 12 N/m
k3  = 40 N/m^3

c1  = 1.0 N s/m
c12 = 0.8 N s/m
c23 = 0.7 N s/m

Ts = 0.05 s

The added third mass and second coupling were selected on the same physical scale
as the original two-mass benchmark rather than introducing a very light/heavy or
very stiff/soft extra subsystem.

The nonlinear-force ratio for the m2-m3 spring is

|F_cubic|/|F_linear|
    = (k3/k23)*(q3-q2)^2
    = 3.3333*(q3-q2)^2.

Examples:
|q3-q2| = 0.10 m -> 3.33 %
|q3-q2| = 0.20 m -> 13.33 %
|q3-q2| = 0.30 m -> 30.00 %
|q3-q2| = 0.40 m -> 53.33 %

Nominal/local linear model
--------------------------
The cubic derivative at dq23=0 is zero, so the nominal model obtained by dropping
the cubic term is exactly the Jacobian linearization at coordinated equilibrium.

Use common/analyze_three_mass_model.m to recompute poles/zeros directly in MATLAB.
