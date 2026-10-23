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

Noise experiment switches
-------------------------
The DeePC, HDeePC, SD-MPC and both PZ-DeePC closed-loop run scripts expose:

    offlineDataMode = 'noisy';  % 'clean' or 'noisy'
    measurementMode = 'noisy';  % 'clean' or 'noisy'
    measurementNoiseSeed = 42;

These are independent switches; all four combinations are supported. Both
modes default to noisy. The offline switch selects output data before Hankel
construction. Applied training inputs remain unchanged. The online switch
adds sensor noise before measurements enter the existing history buffers;
HDeePC also uses measured q1,v1 for its known-state initialization. SD-MPC
forms its online residual from measured output minus nominal model state.
Existing buffer timing and zero initial histories are retained.

Online noise uses deepcData.sigmaY in every controller, matching MPC's noisy
mode. Seed 42 gives the same sensor-noise sequence across controllers for a
shared simulation length. Offline and online draws are independent; changing
the offline mode does not change the online noise scale or sequence.

DeePC/HDeePC/PZ-DeePC select recorded y or yClean from the existing training
file. For legacy SD-MPC datasets, the runner reconstructs clean outputs from
the saved applied inputs and current plant parameters, starting at zero, then
rebuilds residuals in memory. Its noisy offline mode uses deepcData.sigmaY and
seed 1, replacing the older 1-percent noise convention for this experiment.
Use training datasets generated with the current plant parameters. Existing
data files are not overwritten by these selections.

Results include both modes, sensor scale, online seed, noise and measurements.
Tracking RMSE still uses the true simulated state. Each run overwrites its
usual result file, so copy results before running another mode if needed.
MPC only exposes measurementMode because it has no offline training data.

3D result playback and GIF export
--------------------------------
After running a controller simulation, run its separate animation script:

    MPC/animate_mpc.m
    DeePC/animate_deepc.m
    HDeePC/animate_hdeepc.m
    SD-MPC/animate_sdmpc.m
    PZ-DeePC/animate_pzdeepc_2pole.m
    PZ-DeePC/animate_pzdeepc_4pole.m

All use common/animate_three_mass.m and only read saved results. The scene
shows cubic masses, wall and inter-mass springs/dampers, and the applied
force on mass 1. The purple spring is the nonlinear mass-2/mass-3 coupling.
Right-hand plots show true q3 tracking, held input, and q3-q2 deformation.
The physical layout spacing is illustrative; actual displacements remain in
metres, with dashed footprints marking the zero-displacement positions.

Each script exposes FPS and PlaybackSpeed (default 60 FPS, 2x speed). GIF and
final-frame PNG files go to results/animations. Set both output paths to ''
for playback only. No controller is solved during animation. The shared
function also accepts a result struct and options.Visible = 'off' for export
without displaying the figure, and options.MaxFrames for a quick preview.
Full export may take longer than the resulting GIF's playback duration.

Smooth playback modes
---------------------
Animation scripts now default to playbackMode = 'preview' at 60 FPS. Preview
performs no GIF encoding or image capture per frame and follows elapsed time;
it skips overdue frames if graphics rendering cannot meet the requested FPS.
Set playbackMode = 'mp4' to render a video with the requested frame rate, then
watch the finished MP4 for smooth playback independent of rendering speed.
Set playbackMode = 'gif' for GitHub GIF export (capped at 50 FPS because GIF
uses centisecond delays). PlaybackSpeed controls time acceleration separately
from FPS. Export can take longer than the finished video's running time.
