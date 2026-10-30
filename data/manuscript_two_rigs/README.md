# Two independently commissioned reaction-wheel pendulums

These are the two hardware recordings used for the representative identification,
validation and control experiments in the manuscript by Martin Dodek and
Eva Miklovičová. The records were selected explicitly; the analysis does not
search for the best cycle or substitute samples from another experiment.

The original `raw.txt` files are byte-for-byte copies of the captured native
Serial streams. Each includes startup records, all three identification
acquisitions, separate prediction validation and control telemetry. Full files
are retained even where the paper uses only the first control cycle. No
private PC session configurations, local paths or acquisition-host port names
are included.

## Records and hardware

| Property | A4988_20261010 | HY_DIV268N_20261011 |
| --- | --- | --- |
| Acquisition session UTC identifier | 20261010T192243Z | 20261011T133748Z |
| Driver / supply | A4988 / 24 V | HY-DIV268N / 42 V |
| Current setting reported by operator | Potentiometer at maximum; not a calibrated current measurement | Selector labelled 2.5 A |
| MCU | NUCLEO-F446ZE, 180 MHz | NUCLEO-F446ZE, 180 MHz |
| Motor and wheel | NEMA17, 200 full steps/rev, 1/8 microstepping, 90 mm nut-loaded wheel | Equivalent motor/wheel; different pendulum counterweight |
| Control sample interval | 0.1 s | 0.1 s |
| Control acceleration scale | 40 rad/s² | 100 rad/s² |
| Recorded wheel-command guard | 40 rad/s | 70 rad/s |
| Training / validation RMS gates | 0.04 / 0.05 rad | 0.05 / 0.07 rad |
| Relative validation factor | 0.20 | 0.30 |
| First-cycle samples used | 1606 | 1467 |
| First-cycle recorded endpoint | 160.6 s; recording stops during downward return | 146.7 s; confirmed downward rest |

Both rigs use an AS5600 pendulum-angle encoder. Wheel speed and acceleration
are prescribed commands, not independently measured shaft quantities. The
operator reported no missed steps and no applied disturbance during either
experiment. The A4988 cycle completed after recording stopped according to
the operator; no missing endpoint or completion event is added to the data.
The HY record contains a partial subsequent cycle, which the paper excludes.
Pre-control settling telemetry may have cycle index 0; this is not the first
reported swing-up cycle.

The firmware reports `source_fingerprint=unfingerprinted`. These recordings
therefore do not establish binary identity with the present source checkout.
Current source defaults, notably the HY wheel guard of 100 rad/s and shared
admission gates, must not replace the recorded settings above.

## File contents and units

Each rig folder contains:

- `raw.txt`: the complete, authoritative received stream, with original line
  endings and decimal representations.
- `control_cycle_1.csv`: every `control_diag` row whose `cycle` field equals 1,
  in acquisition order. Column names come directly from `control_diag_header`.
- `identification_coarse.csv`, `identification_guided.csv` and
  `identification_validation.csv`: all 601 samples in each respective 600-interval
  acquisition. CSV values are copied from the original decimal tokens.
- `identified_model.csv`: the usable refined-fit candidate with the lowest
  logged RMS in the first attempt, matching the committed model in
  `control_config`. `bias` is the fitted constant acceleration and `coulomb`
  the smooth friction coefficient; they are not motor electrical parameters.
- `validation_metrics.csv`: the accepted independent validation's logged
  error metrics and initial velocity. RMS and peak errors are in radians.

In identification CSVs, `read_us` and `previous_command_ack_us` are unsigned
32-bit MCU microsecond timestamps. Differences use modulo 2³² arithmetic.
`angle_rad` is unwrapped measured pendulum angle from downward.
`held_speed_rad_s` at sample i>0 is the command held over the preceding
read-to-read interval. Row 0 has sentinel command and acknowledgement values;
it does not define an input interval. Use rows 1...600 for the 600 commands.
The command is applied as a speed jump followed by pendulum free flow.

Control `t_s` is elapsed nominal sample time within the cycle. Angles are
radians, wheel command `v_sent` is rad/s, interval-average command `u_sent` is
rad/s², and timing fields ending in `_us` are microseconds. `solve_us`
historically names the combined EKF, preview and controller computation,
not optimizer-only time. Control phase codes are 1 ascent, 2 upright hold,
3 downward return, 4 downward settling and 5 confirmed rest. The upright
branch can have either sign; wrap angular deviations to the nearest branch.

All three archive sample sequences and their firmware CRC-32 trailers were
checked during preparation. Each has 601 consecutive indices, 600 finite
approximately 0.1 s read intervals, and a matching binary32 transport checksum.
CRC-32 is an integrity check, not proof of physical model accuracy or shaft
tracking. `SHA256SUMS` fixes the exact published files.

## Offline reproduction in MATLAB

From the repository root, without connecting a rig:

```matlab
addpath('matlab/analysis', 'matlab/misc');
report = reproduce_reference_metrics;
```

The function checks the original raw-file hashes, reads the CSV extracts,
recomputes control and holding statistics, and performs a double-precision
frozen-model validation replay. Only the initial velocity is fitted to the
reserved first five intervals (0.5 s); all five shared coefficients remain
fixed. The remaining 595 endpoints are scored once. It uses the same speed-jump
order and five 0.02 s RK4 substeps as the reported numerical model. Small
roundoff differences from the MCU's binary32 validation are expected and
checked rather than hidden. The function neither writes figures nor modifies
the data, coefficients, tuning or hardware.

The GUI's bounded live preview is not an offline scientific validator; use
the raw records and this explicit reproduction function for the published
sample selections. The GUI remains the maintained interface for new native
Serial acquisitions.

## Integrity and reuse

The raw SHA-256 values are:

```text
7da5515c58043f72adc5583cd1d1e45485dd384fac71923247883910b0d7219e  A4988_20261010/raw.txt
193352aea6af7bbce30df9576ea90a7aadeab678e6432f731649ffc7ecc422e9  HY_DIV268N_20261011/raw.txt
```

Project software retains the repository's GPL-3.0-or-later terms. No separate
data-specific license has been added; public availability alone does not
imply a Creative Commons data license. Cite the manuscript and the exact
repository revision when using these records. The observations support the
two representative cycles, not a statistical claim of repeatability across
arbitrary plants or a worst-case computation-time bound.
