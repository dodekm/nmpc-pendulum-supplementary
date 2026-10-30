# Self-commissioning NMPC reaction-wheel pendulum

Source-code supplement for a stepper-driven reaction-wheel pendulum: autonomous
embedded identification, validation, state estimation and nonlinear model
predictive control, with an optional native MATLAB experiment interface.

All feedback calculations run on the STM32. MATLAB observes the experiment,
records the serial stream and provides explicit operator commands; it is not
the real-time controller and is not needed for autonomous operation.

This repository contains the maintained implementation and the two curated
experimental recordings underlying the manuscript. It does not include the
manuscript, publication figures, tests, qualification reports, archives or
precompiled firmware. See [the research-data description](data/manuscript_two_rigs/README.md)
for the exact records, their limitations and the offline reproduction workflow.

## Contents

```text
arduino/pendulum_stepper_reaction_wheel_AutoID_NMPC/
  *.ino, *.cpp         Arduino compilation anchors and composition root
  AdaptiveApplication.h Public initialize/poll/status facade
  application/         Lifecycle contract, private orchestration and build identity
  platform/            Hardware policy, sensor/actuator boundary, timing and ISR tools
  telemetry/           Bounded diagnostics, serialization and acquisition streaming
  control/             Model, RK4, EKF, NMPC, terminal design and cycle policy
  identification/      Excitation, sensitivities, batch fitting and validation
  numerics/            Shared numerical utilities
  Eigen/               Bundled Eigen headers and license texts
  src/                 Bundled AS5600 and AccelStepper libraries
matlab/
  scr/                 MATLAB GUI launcher
  gui/                 Experiment application and bounded live-data model
  acquisition/         Native serial acquisition and session handling
  misc/                Hashing and safe record-publication helpers
  analysis/            Offline metrics and frozen-model validation replay
data/manuscript_two_rigs/
  A4988_20261010/       Original recording and exact-token CSV extracts
  HY_DIV268N_20261011/  Original recording and exact-token CSV extracts
```

The directory structure is significant. Keep the Arduino sketch name and its
folder name identical, and preserve the MATLAB folder nesting.

The sketch's `ARCHITECTURE.txt` describes ownership, interfaces and maintenance
contracts. Application services use fixed composition and compile-time templates;
no additional virtual hardware layer or heap-owned service was introduced.
Root `.cpp` files are intentional: Arduino discovers them and recursively builds
`src/`, but does not compile arbitrary nested header-package directories.

## Hardware and operating precautions

The intended target is **NUCLEO-F446ZE**, using an AS5600 pendulum-angle encoder
and either an A4988 shield or an HY-DIV268N external stepper driver. The present
motor convention is 200 full steps per revolution, with 1/8 microstepping.
This software is configured for these rigs, not arbitrary wiring or guaranteed
operation on an arbitrary pendulum.

**The firmware can start moving the apparatus autonomously after boot.** Before
power-up or reset, place the pendulum at its true original downward position,
physically stop the wheel, clear the moving assembly and cables, and remain
ready to use the physical user button. Do not adjust wiring while powered.

The physical user button is the primary stop input. Stopping MATLAB recording,
closing the GUI or disconnecting the PC does **not** stop the experiment.
A serial stop request or a zero speed command is not independent confirmation
that the mechanics have stopped. Wheel speed is a prescribed command; there
is no independent wheel-shaft encoder or direct missed-step detection.

## Select the physical driver before compiling

`platform/HardwareConfig.h` is the hardware-policy selection point. The source default
is **profile 1 (HY-DIV268N)**. Select profile 0 explicitly for A4988.

| Setting | A4988 | HY-DIV268N |
| --- | --- | --- |
| `ADAPTIVE_DRIVER_PROFILE` | `0` | `1` |
| STEP / DIR / EN, Arduino pin numbers | 11 / 12 / 7 | 9 / 10 / 8 |
| Microstep selection | MS1=8, MS2=9, MS3=10 | Set physical DIP switches |
| Default microstepping | 1/8 | 1/8 |
| Current control acceleration scale | 40 rad/s^2 | 100 rad/s^2 |
| Current wheel-command guard | 40 rad/s | 100 rad/s |

The selected limits are deployment policy, not measured universal actuator
capabilities. Identification excitation uses a separate 25 rad/s^2 slew limit
and 8 rad/s speed range. Select the physical current limit, supply and cooling
appropriately; the firmware does not measure available motor torque.

For HY-DIV268N, the default is a correctly buffered common-cathode interface.
`ADAPTIVE_COMMON_ANODE=1` is available only with matching physical wiring.
`ADAPTIVE_MICROSTEPS` must match the driver's actual switch configuration.
`ADAPTIVE_HOLD_AT_ZERO` must remain `0`: the demonstrated application releases
holding current at zero command. Powered stationary holding is not a retained
operating mode. The embedded runtime likewise requires
`NMPC_USE_SINGLE_PRECISION=1`; binary64 calculations are used only by local
numerical verification, not by an alternative deployed controller.

## Build the standalone firmware

The source uses C++17 and the STM32duino core **3.0.0**, whose installed toolchain
is xPack Arm GNU **14.2.1-1.1**. The intended deployment is single precision and
`-O3`, without `-ffast-math` or `-ffinite-math-only`. The source rejects those
unsafe floating-point modes because numerical failure checks require NaN and
infinity semantics.

Install Arduino IDE or Arduino CLI and the official STM32duino board package.
The board-manager index is:

```text
https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
```

For Arduino CLI, after configuring that index:

```text
arduino-cli core update-index
arduino-cli core install STMicroelectronics:stm32@3.0.0
```

From the repository root, build the A4988 configuration with:

```text
arduino-cli compile --fqbn "STMicroelectronics:stm32:Nucleo_144:pnum=NUCLEO_F446ZE,opt=o3std,rtlib=nano" --build-property "compiler.cpp.extra_flags=-DNMPC_USE_SINGLE_PRECISION=1 -DADAPTIVE_DRIVER_PROFILE=0" arduino/pendulum_stepper_reaction_wheel_AutoID_NMPC
```

For HY-DIV268N, change `-DADAPTIVE_DRIVER_PROFILE=0` to
`-DADAPTIVE_DRIVER_PROFILE=1`. This command compiles only; it does not upload.
Upload deliberately with your configured STM32/ST-LINK workflow after checking
the physical rig. In Arduino IDE, open the `.ino`, select Nucleo-144/F446ZE and
the `-O3` optimization option, and set the driver in `platform/HardwareConfig.h`.

Driver and precision overrides must apply to **all translation units**, not
only to the `.ino`. The firmware has linkage checks for inconsistent settings.
Eigen, AS5600 and AccelStepper are already bundled; installing duplicate
versions of those libraries is unnecessary. Arduino, Wire, HardwareTimer and
CMSIS/HAL come from the STM32duino platform.

## Autonomous sequence

With the apparatus prepared at downward rest, the firmware calibrates its
angle origin, acquires a coarse chirp, performs sensitivity-based batch
parameter fitting, acquires model-guided resonance/free-decay data, refines
the fit, and evaluates frozen coefficients on a separate validation record.
It then constructs model-dependent controller settings and performs a
motor-stopped nominal-cycle simulation before handing over to physical control.

An admitted model is used for swing-up, a 30-second upright hold with wheel
unloading, downward return and settling. Healthy control cycles repeat without
re-identifying. Startup/admission failures and recovery states are reported
over Serial. A valid nominal simulation is not a guarantee of physical capture;
actuation, mounting and model mismatch remain relevant.

## MATLAB GUI and acquisition

MATLAB **R2025b** with its JVM is the checked environment. No additional MATLAB
toolbox, UARTLink, MEX binary, PowerShell recorder or Python runtime is required.

From the repository root in MATLAB:

```matlab
addpath('matlab/scr');
app = pendulum_experiment_gui;
```

1. Close Arduino Serial Monitor and any other program owning the serial port.
2. Prepare and supervise the rig at original downward rest, with the wheel stopped.
3. Refresh the port list, select the correct port and rig annotation, and tick
   **Ready: supervised, down, wheel stopped** only when those conditions hold.
4. Click **Connect & Record**, then deliberately reset the board to capture boot
   and the complete identification-to-control sequence.
5. Let the firmware perform its autonomous cycle. Use notes to mark observations.
6. Stop the mechanics with the physical button when finished, then stop recording.

The rig label is recording metadata only: it does not configure firmware pins,
driver polarity or limits. Native Serial uses **250000 baud, 8N1**. The GUI is
an optional observer/operator with a bounded live preview, not an independent
verification of data completeness or shaft motion.

The GUI's timing trace (`compute_us`) includes EKF prediction/correction,
command-age preview and NMPC together; it is not a solver-only measurement.

The GUI is the single maintained user-facing experiment entrypoint and uses
the native serial acquisition engine. New sessions are written under
`matlab/scr/results/serial/`, which Git ignores. Keep each generated session's
`raw.txt`, `receive.jsonl`, `actions.jsonl`, `session.json` and `session.mat`
together. Newly acquired sessions remain ignored. Only the two explicitly
curated, byte-preserved manuscript recordings under `data/manuscript_two_rigs/`
are shipped in this repository.

## Configuration and reproduction scope

This is a source snapshot of the maintained implementation. No numerical plant
coefficients need to be supplied: they are estimated after boot. Tuning and
admission policies are explicit in `platform/HardwareConfig.h`, `identification/Config.h`
and `control/ControllerConfig.h`; model-normalized weights are in
`control/ModelTuning.h`.

For the manuscript-era deployment policies, set the following before compiling:

| Policy | A4988 deployment | HY-DIV268N deployment |
| --- | --- | --- |
| Control acceleration scale (rad/s^2) | 40 | 100 |
| Wheel-command guard (rad/s) | 40 | 70 |
| `maxTrainingRms` (rad) | 0.04 | 0.05 |
| `maxValidationRms` (rad) | 0.05 | 0.07 |
| `maxRelativeValidation` | 0.20 | 0.30 |

The current HY wheel-command guard is 100 rad/s, and the current shared admission
gates are 0.05/0.07/0.30. These later settings must not be retroactively assigned
to historical recordings. The deployment-policy table is configuration guidance,
not an assertion that this snapshot is byte-identical to every historical manual
build. Identification and physical trajectories depend on the apparatus and the
new observations; identical numerical estimates or motion are not guaranteed.

Keep the Git commit used to build an experiment with its recording. By default,
Serial metadata reports `source_fingerprint=unfingerprinted`; an optional
`ADAPTIVE_BUILD_FINGERPRINT` build definition can identify the supplied source
when applied consistently to all translation units. The manuscript recordings
do not establish a source-to-binary identity with this maintained repository.

## License

Project-owned software is licensed under **GPL-3.0-or-later**; see `LICENSE`.
Bundled third-party sources retain their own copyrights and license terms.
See `THIRD_PARTY_NOTICES.md` and the accompanying library license texts.
