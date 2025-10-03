# inverse_kinematics_UR5

> A toolbox and example scripts for collision-aware inverse kinematics (IK) on a UR5 robot (with rod/base/microphone attachments).
> Developed at DTU ACT.

---

## Overview

This repository provides:

* A collision-aware IK solver (`ikPositionCollisionAware`) for rigidBodyTree models, with joint-space minimization relative to a previous pose.
* Utility routines and example scripts for commanding a UR5 robot (real or simulated) to traverse 3D target grids while avoiding collisions.
* Scripts integrating RIR (room impulse response) measurement using sweeps, combining robotics motion and acoustic measurement.

The goal is to facilitate research combining robot motion planning, acoustics, and physical measurement in indoor environments.

---

## Repository Structure

```
/
├── README.md
├── src/
│   ├── ikPositionCollisionAware.m
│   ├── buildUR5WithRod.m
│   ├── get_dataRME.m
│   ├── UR5_Rod_RRT_IK.m
│   ├── UR5_RIR_Measurement.m
│   ├── UR5_RIR_Measurement_NI.m
│   └── UR5_Rod_Grid_IK.m
├── Data/
│   └── (grid definitions, recorded RIRs, etc.)
└── Environment/
    └── scenario/environment JSON files (e.g. kitchen.json, lab.json)
```

---

## Features

### `ikPositionCollisionAware`

* Solves position-only inverse kinematics for a given end-effector body name (e.g. “rodTip”) and target Cartesian position.
* Uses repeated random initial guesses plus the previous configuration as seeds.
* Rejects solutions that are in collision with environment obstacles (ignoring immediate parent–child self-collisions).
* Returns the feasible configuration closest (in Euclidean norm) to the `prevConfig` seed.
* Outputs diagnostic `info` struct (success flag, number of attempts, status, pose error).

### Motion Planning + Grid Scripts

* `UR5_Rod_RRT_IK.m` — connect to a UR5 (via RTDE), load a scenario, use RRT and IK to visit a 3D grid of points.
* `UR5_Rod_Grid_IK.m` — offline planning & visualization of UR5 robot + rod to traverse a 3D grid.
* `UR5_RIR_Measurement*.m` — measurement scripts combining robotic motion and acoustic sweep-based recording (Fireface / NI DAQ versions).

### Acoustic / Measurement Utilities

* Generation of exponential sweeps (`sweeptone`) with configurable silence padding.
* Wrappers for acquiring audio data robustly (e.g. `get_dataRME`).
* Estimation of impulse response from sweep + recording (`impzest`).
* File logging, plotting, spectrograms, configuration metadata, and optional email-based logging.

---

## Dependencies & Requirements

* MATLAB (tested with R2024b; Audio Toolbox, Robotics System Toolbox, Robotics System Toolbox Support Package for Universal Robots UR Series Manipulators)
* RTDE client support for UR5 (e.g. `urRTDEClient`)
* DAQ or audio interface support (RME Fireface, NI USB-4431)
* Collision geometry defined in robot model and scenario files (JSON)

Optional:

* Email setup (MATLAB `sendmail`) for logging
* Parallelization or retries if long measurement campaigns

---

## Installation & Setup

1. Clone the repository:

   ```sh
   git clone https://github.com/dtu-act/inverse_kinematics_UR5.git
   ```

2. To connect to the UR5 robotic arm:
    visit Matlab documentation [here](https://se.mathworks.com/help/robotics/urseries/ug/hardware-setup-for-ur-series-cobots-rtde.html) and [here](https://se.mathworks.com/help/robotics/urseries/ref/urrtdeclient.html?searchHighlight=urrtdeclient&s_tid=srchtitle_support_results_1_urrtdeclient).

3. Verify scenario JSON files in `Environment/`, and ensure `grid_cuboid.mat` or any other array configuration exists.

---

## Usage Examples

### Motion Planning & IK (Offline)

```matlab
% Visualise the robot moving through a 3D grid without real hardware
UR5_Rod_Grid_IK
```

### Robot + RRT + IK (Real Robot)

```matlab
% Command a UR5 to traverse a 3D grid via RRT planning and collision-aware IK
UR5_Rod_RRT_IK
```

### RIR Measurement (Single measurement, no robot)

```matlab
% Fireface (ASIO) based measurement
UR5_RIR_Measurement

% Or NI USB-4431 version
UR5_RIR_Measurement_NI
```

---

## Tips & Notes

* The IK routine only handles **position** (not orientation) — orientation is unconstrained.
* Collisions between immediate parent/child joints are skipped to allow the rod tip to extend outward.
* Because the solver is stochastic (random initial guesses), results may vary — set RNG seed for reproducibility.
* For measurement campaigns, measure atmospheric conditions (temperature, humidity) and calibrate gains.
* Use logging (diary) and email notifications for long unattended runs.

---

## Contributing & License

Contributions are welcome! Please submit pull requests or issue reports.

---

## Contact

Antonio Figueroa-Duran  
Acoustic Technology Group, Department of Electrical and Photonics Engineering, DTU  
Email: [anfig@dtu.dk](mailto:anfig@dtu.dk)

---
