# VN-100 Calibration Procedure

## 0. Baseline premise: onboard HSI ships **DISABLED** on this hardware

The field sensor runs **FW 3.1.0.0**. In that ICD (§3.5.1, DEFAULT column), Reg 44's factory
default is **`0,1,5`** — i.e. **Mode=Off, ApplyCompensation=Disable**. Confirmed on hardware
across two separate `$VNRFS` resets.

**Calibration-strategy implication:** with onboard HSI off and unapplied by default, the "moving
target" problem — the solution shifting underneath you as you rotate — **doesn't occur on this
hardware**. An offline ellipsoid fit starts from a **clean, raw-magnetometer baseline** on a
factory-default sensor; all of §4 relies on this.

**Version differences (full list: `docs/protocol.md` §5.3; single source of truth in code:
`pyvn100/capabilities.py`):**

| | FW 3.1.0.0 (field) | FW 2.1 |
|---|---|---|
| Reg 44 factory default | **`0,1,5`** → Off / Disable | `1,3,5` → Run / Enable |
| Onboard HSI out of the box | **DISABLED** | ENABLED |
| Offline fit baseline | **clean** (raw mag) | dirty (onboard-corrected + moving target) |
| Reg 46 (progress/bins) | **ABSENT** (undefined in ICD) | present |
| Register written by `$VNSGB` | **43** | 74 |

> **Still, never ASSUME factory defaults.** The sensor may already have been configured
> (Reg 44 enabled + saved with `$VNWNV`). That's why the wizard, at the start of an offline
> session, **writes and reads back to verify** Reg 23 as identity and Reg 44 as `Off/Disable`
> (`calibration_dialog._start_session`) — a measurement, not an assumption.

**Requirements for raw data (unchanged):** `ApplyCompensation=Disable` (1) **and** Reg 23 identity.
Pipeline (ICD §4.5.1): `raw → factory cal. → Reg 23 → (if Enable) Reg 47 → output`. Reg 23 is
**always** active; Reg 44 only toggles the **onboard** stage.

> **Onboard convergence is now measured via Reg 47.** FW 3.1.0.0 has no progress register (46), so
> the wizard uses (a) **PC-side orientation coverage** (`tools/coverage.py`) for progress, and
> (b) **Reg 47 solution settling** (`registers.hsi_solution_converged`) plus its departure from
> identity as the convergence criterion. Both are ICD-supported; no decision depends on Reg 46.

Standard approach: **use Reg 44 to RESET and converge the onboard HSI for this environment, then
turn it OFF and save.** Two valid workflows follow below; the dashboard wizard implements both.

## 1. Why calibrate? (physics)

- **Hard-iron:** magnets/metal rigidly attached to the sensor shift the measurement sphere's center (offset).
- **Soft-iron:** iron/nickel/cobalt distort the sphere into an ellipsoid (scale/skew).

Ideal readings lie on a **sphere**. Calibration maps the ellipsoid back onto a sphere:
`calibrated = C·(raw − B)` (B: hard-iron offset, C: soft-iron matrix).

> **Limitation — misalignment (known, out of scope):** ellipsoid/sphere fitting corrects hard-iron
> (B) and soft-iron (C) but **cannot correct angular misalignment** between the magnetometer axes
> and the body frame — a rotated sphere is still a sphere, invisible to sphere-fitting (our `C`
> matrix is symmetric; it only scales/skews axes). VN-100 handles this separately via Reference
> Frame Rotation (Reg 26) if needed — out of scope here.

> **Environment matters (AN012):** the model assumes the external field is **fixed relative to the
> sensor body**. Rotating near **earth-fixed** fields that vary with position — laptops, monitors,
> metal desks — gets misread as body-fixed, corrupting the solution. Calibrate **in the final
> mounting position, powered up, away from magnetic anomalies.**

## 2. Register map (ICD FW v3.1.0.0 — verified line-by-line; differences in `protocol.md` §5.3)

| Reg | Name | Access | Fields |
|:--:|----|:--:|--------|
| 23 | Magnetometer Calibration | R/W | Row-major C[3×3] (9) + B[3]. Model: **`m_cal = C·(m_raw − B)`**. Identical across both ICDs |
| 35 | VPE Basic Control | R/W | Enable, **HeadingMode** {0 Absolute, 1 Relative, 2 Indoor}, FilteringMode, TuningMode |
| 43 | **Filter Startup Gyro Bias** | R/W | **`$VNSGB` writes HERE** (ICD §3.3.5). Was Reg 74 in FW 2.1 |
| 44 | **Real-Time HSI Control** | R/W | **Mode** {0 Off, 1 Run, 2 Reset}, **ApplyCompensation** {1 Disable, 3 Enable}, **ConvergeRate** 1–5. **Factory default: `0,1,5`** |
| 46 | ~~Magnetometer Calibration Status~~ | — | ⛔ **ABSENT in FW 3.1.0.0** (not listed in the ICD register index). Hardware returns an undocumented all-zero stub → **no decision depends on it** |
| 47 | **Real-Time HSI Results** | R/O | Onboard-computed C[3×3]+B[3]. Its input is mag data **after Reg 23 is applied** (ICD §4.5.1) — the stages chain together |
| 84 | Gyro Calibration | R/W | C[3×3]+B[3] (scale/alignment/**bias**) for mounting-induced errors. **Unrelated to `$VNSGB`** (which writes Reg 43) |

- **ConvergeRate:** 1 ≈ 60–90 s (slow/accurate), 5 ≈ 15–20 s (fast).
- **`ApplyCompensation`** (named `HSIOutput` in FW 2.1) has two values: **1 = Disable** (onboard
  not applied → output is Reg 23 only; identity means raw), **3 = Enable** (onboard HSI applied).
  **Numeric values match across both ICDs**, only the name changed.
- **Factory default for Reg 44 = `0,1,5`** (Off, Disable, 5) — per ICD §3.5.1 and confirmed on
  hardware. Version-dependent, so it lives in exactly one place in code: `pyvn100/capabilities.py`.
- **Convergence criterion (no Reg 46):** poll Reg 47 periodically; the solution has converged once
  it (a) has departed from identity and (b) has settled across consecutive reads
  (`registers.hsi_solution_converged`). Progress comes from PC-side coverage (`tools/coverage.py`).
- ✅ **Bring-up:** the identity triple (Reg 1 model + Reg 2 hardware + Reg 4 firmware) is read to
  determine which ICD applies (`pyvn100/selfcheck.py`).

## 3. Workflow A — the sensor's own HSI (wizard default)

1. Mount the sensor in its final position, powered up, away from magnetic anomalies.
2. **RESET:** `$VNWRG,44,2,<out>,<rate>` — `Mode=Reset` (clear the solution and start running).
   Since HSI ships **off** on FW 3.1.0.0, this step also **starts** it — it cannot be skipped.
3. **Rotate:** slowly through 360° on all axes to cover the sphere (the wizard's circular coverage
   view guides this).
4. **Watch:** `$VNRRG,47` — keep rotating until the solution departs identity and **settles**
   (the wizard measures this automatically). *On older v2.x hardware, `$VNRRG,46` bins/AvgResidual
   can also be monitored.*
5. **Freeze:** `$VNWRG,44,0,<out>,<rate>` — `HSIMode=OFF` (freeze the solution). Per UM001: *"once
   a valid solution is found in a stationary environment, turn HSI OFF."*
6. **Make it permanent:** most robust is reading Reg 47 and writing it into Reg 23, setting
   `HSIOutput=NO_ONBOARD`, then **`$VNWNV`**.

The dashboard wizard performs steps 2–6 in **two stages**, matching VN-100's RAM↔flash split:
- **"Apply ▸ Preview"** → steps 2–5 (Reg 47 → Reg 23, onboard OFF/NO_ONBOARD). Writes to **RAM
  only**; the user sees the result live. No `$VNWNV` — lost on power cycle.
- **"Save (persist)"** → step 6 (`$VNWNV`, flash). **"Discard"** → restores the Reg 23+44 snapshot
  taken before applying (reverts to the pre-preview state without touching flash).

## 4. Workflow B — offline ellipsoid fit (advanced)

First **stop the moving target:**
1. Set Reg 23 to identity + `$VNWRG,44,0,1,<rate>` — `HSIMode=OFF` **and** `HSIOutput=NO_ONBOARD`
   (stable raw data).
2. Rotate to collect raw mag data (coverage view guides this).
3. Offline fit → C (soft-iron), B (hard-iron). (`tools/calibration.py`: `mag_calibration` →
   `center=B`, `gain=C`.) `fit_ellipsoid` **pre-centers** the data before fitting (subtract the
   mean → fit → add the center back); when the mag center sits far from the origin (hard-iron),
   forcing `=1` normalization ill-conditions the design matrix, and pre-centering lowers the
   condition number and improves numerical stability (on measured sample data, sphericity improved
   from 3.55% to 2.06%, condition number ~45× better). The math is identical; only the computation
   is more stable.
4. Write `$VNWRG,23,<C…>,<B…>` (the fit output maps directly onto this form).
5. `HSIOutput=NO_ONBOARD` (1, onboard OFF → the Reg 23 solution is used) + **`$VNWNV`**.

`tools/coverage.py`: incoming mag direction vectors are binned into **equal-area** patches of the
sphere (12×6=72 cells); a motion gate (`|gyro|>~0.1 rad/s`) prevents a stationary sensor from
producing false coverage; a cell counts as "covered" at ≥6 samples.

> **Coverage metric — why it caps around ~89% in sim:** the wizard tracks two coverage measures.
> The **% bar gating "Fit"** (`cov_gate`, `calibration_dialog.py`) bins the **acceleration/gravity
> direction** — independent of hard-iron, whereas mag coverage alone can stay below 60% under strong
> hard-iron even after a perfect sweep. But gravity direction depends only on **pitch+roll**, not yaw
> (the yaw axis is the gravity axis), and the simulator spins yaw continuously while oscillating
> pitch/roll at incommensurate periods — gravity traces a **2D Lissajous curve** that never visits
> all 72 accel cells (≥6 samples each), so it **plateaus near 89%.** The on-screen **disk/cell**
> indicator bins **mag** direction instead (full 3D, yaw included), so it can reach 72/72. **Not a
> bug:** the fit threshold `MIN_COVERAGE_FIT=0.60` is well below 89%, so "Fit"/"Apply" is already
> enabled — don't wait for 100%. On **real hardware**, manually tilting through every orientation
> (including side/inverted faces) raises accel coverage too; a frozen bar while paused (e.g. sim
> idling after its 6/6 orientations) just means click "Resume".

## 4b. Hybrid mode — repeatable calibration from a "golden recording"

**Problem:** calibration gets retried often during development, and each attempt costs ~4 minutes of
**manual rotation**. Since no two hand motions match, no two runs share the same data — making it
impossible to tell whether a code change improved the fit or the operator just rotated better.

**Solution:** capture **one good session** once (the "golden recording"), then replay it for every
later attempt. Measurements come from the recording; commands still reach the **real sensor**:

```bash
python vn100_dashboard.py --replay logs/altin.csv --port auto --replay-speed 8
```

`Transport` (`pyvn100/transport.py`) exposes these as two independent axes:
`writable` (do commands reach the sensor?) and `data_is_recorded` (is data live or replayed?).

| mode | writable | data_is_recorded | transport |
|-----|----------|------------------|-----------|
| normal hardware | ✓ | ✗ | `SerialTransport` |
| pure playback | ✗ | ✓ | `ReplayTransport` |
| **hybrid** | ✓ | ✓ | `HybridTransport` |

### Two hard rules

1. **The recording must be captured in RAW mode** (Reg 23 = identity **and** onboard HSI off).
   Offline fit assumes `cal = C·(raw − B)` on **raw** mag data; fitting against already-corrected
   data (e.g. onboard HSI on) computes a correction on top of a now-disabled correction, producing
   a **wrong** result. The CSV carries no metadata to detect this, so it's guaranteed **by
   procedure** (below).
2. **Onboard HSI (Workflow A) doesn't work in hybrid mode.** It converges from the sensor's **own
   live** magnetometer; a replayed recording feeds it nothing, so a stationary sensor's Reg 47
   solution never leaves identity. The wizard **locks the method to offline fit** in hybrid mode
   (the onboard option becomes unselectable) so users don't wait on a dead end.

### Capturing a valid golden recording (one-time)

The cleanest way to guarantee raw data is to record **from inside the wizard's offline session**:
the wizard's `Start` already sets Reg 23 to identity and disables onboard HSI, so everything from
that point on is raw.

1. Connect the sensor **live**: `python vn100_dashboard.py --port auto`
2. `Mag Calibration…` → method **"Offline ellipsoid fit"** → **Start**
   (watch the console echo `$VNWRG,23,1,0,0,…` + `$VNWRG,44,0,1,…` — this is an **echo**, not a VNACK).
3. In the main window, **● START LOGGING** → rotate through all 6 orientations for ~4 min → **STOP
   LOGGING**.
4. Save the resulting `logs/vn100_*.csv` as `logs/altin.csv`. Done — no need to rotate again.

### Stream separation (why a dedicated transport was needed)

Even in hybrid mode, the desk-bound sensor keeps **broadcasting its own telemetry**. Left
unfiltered, that live stream would mix with the replayed (rotating) cloud and silently corrupt the
fit with stationary samples. `HybridTransport` therefore splits the sensor stream in two:

- telemetry (`$VNYMR` + binary frames) → **stripped**, never surfaces;
- command responses (`$VNRRG`/`$VNWRG`/`$VNERR`, plus the bridge's unprefixed `VNERR`/`VNMODE`) →
  **pass through** (Reg 23/44 snapshot for "Discard"; Reg 46/47; write-accepted echoes).

Stripped telemetry isn't discarded — it's exposed via `live_data`/`live_age`, so the **stillness
gate before `$VNWNV`** (UM001 §5.1.3, `still_reference`) checks the **real sensor's current
state**, not the recording. Without this split the gate would always see "motion" from the
replayed data, and **"Save" would never enable.**

### Playback speed

The wizard's collection timer runs at 30 ms and samples **only the latest** reading on each tick,
so faster playback drops samples. On a measured golden recording (265 s, 38.4 Hz, 10175 samples),
the hard-iron center stays stable **to the 4th decimal up to 8x**; at `--replay-speed 16`
orientation coverage drops below the 60% gate (to 56%) and **"Fit" won't enable**. Practical
ceiling: **8x** (~33 s/run).

## 5. Verification and quality

- Post-fit **sphericity** (norm std/mean): 0 = perfect sphere.
- VectorNav AN012 Hard/Soft Iron Calibration application note (2014 revision, p.10 — PDF not
  included in this repo): *"if the magnetic norm varies by less than 1%, the magnetometers are
  calibrated to better than 0.5°."* (The 2012 AN012 revision states 2°; the 2014 "Released"
  revision is treated as authoritative.)
- Onboard quality is measured by **Reg 47 solution settling**: `hsi_solution_converged()`
  (tolerance `0.002`, 3 consecutive reads) plus departure from identity. (`AvgResidual`/Reg 46
  exist only on FW v2.x hardware — not present here.)

## 6. Persistence — `$VNWNV`

`$VNWNV` (Write Settings) writes current register values to **flash** → survives power loss
(~500 ms). `$VNWRG` writes only to **RAM** (lost on power loss). `$VNRFS` resets to factory
defaults (erases). The wizard reflects this split in its UI: **Apply ▸ Preview** = RAM,
**Save** = `$VNWNV` (host command **VN SAVE**), **Discard** = restore RAM to its prior state.

> **"Clear Calibration From Sensor"** (red, requires confirmation): sets Reg 23 → identity and
> Reg 44 → that firmware's **factory default** (`capabilities.hsi_control_default`; `0,1,5` =
> onboard HSI **OFF** on FW 3.1.0.0), then `$VNWNV` + `$VNRST`. This permanently erases the
> sensor's **saved** calibration — it differs from `$VNRFS` only in touching calibration alone
> (other settings like baud rate are preserved). The wizard's **"Reset"** button, by contrast,
> writes Reg 23/44 back to sensor RAM from a pre-session snapshot if one exists (never touches
> flash); with no snapshot, it just clears the display.

## 7. Gyro bias (static) — drift while stationary

VN-100 continuously compensates gyro bias via its onboard VPE filter; even so, a small drift can
appear while the sensor is stationary — **at power-up or before it's fully warmed up.** Two
complementary approaches:

1. **Verification/characterization (no writes):** the dashboard's **"Gyro Bias (static)…"** tool
   holds the sensor still (~500 samples) and reports the gyro mean as bias, std as noise —
   confirming the bias is small and the sensor is ready. It shows a **live running bias ± σ** while
   collecting, then converts σ to **noise density (ARW = σ/√f_s, °/s/√Hz)**. `f_s` comes from the
   **true output rate (ODR)**: total packets decoded during the window ÷ elapsed time
   (`VN100.packet_count` delta) — not from arrival timestamps. **Why:** the tool's 30 ms sampling
   timer keeps only the latest packet, thinning anything above ~33 Hz; timestamp-based `f_s` would
   collapse to ~33 Hz and inflate ARW by `√(ODR/33)` (~2.5× at 200 Hz). `packet_count` isn't
   thinned, so it reflects the true ODR — σ itself is unaffected. (Timestamps are used only as a
   **fallback** when `packet_count` is unavailable.) *This isn't full **Allan variance** (hours of
   static data needed) — just an honest σ-to-noise-density conversion.* Try without hardware via
   `--sim-motion still`.
2. **Write to the sensor (SetGyroBias):** with the sensor **absolutely still**, `$VNSGB` copies the
   current estimated bias into an internal register (**FW 3.1.0.0: Reg 43**, Filter Startup Gyro
   Bias, ICD §3.3.5; Reg 74 in FW 2.1). The version mapping lives in one place,
   `capabilities.gyro_bias_reg`; the dialog targets it and **verifies by reading back** — a sensor
   `$VNERR` rejection skips `$VNWNV`. `$VNWNV` then makes it the persistent startup bias, reducing
   power-up drift. Separately, gyro compensation (scale/alignment/bias matrix) is read/written via
   **Reg 84**.
   - API: `vn100.set_gyro_bias()` (C: `vn100_set_gyro_bias`), host bridge:
     `hostlink.gyro_bias_capture()`. The simulator mimics `$VNSGB` by zeroing the stationary bias,
     verified without hardware by `tests/test_dualmode.py::test_sim_setgyrobias_reduces_bias`.

> **Warning:** SetGyroBias must only be used while the sensor is **stationary**; calling it during
> motion mistakes the motion for bias and writes the wrong compensation. The tool guards against
> this with `|gyro|` and `|accel|` stillness gates.

## 8. Sensor write convention (Reg 23)

`m_cal = C·(m_raw − B)` — B is subtracted **first**, then multiplied by C. Since our fit returns
`gain=C`, `center=B`, it maps directly: `$VNWRG,23, C00,C01,C02,C10,C11,C12,C20,C21,C22, B0,B1,B2`.

## 9. Further calibrations (roadmap)

- Accelerometer: **Reg 25 (Acceleration Compensation)**, 6-position static. (Confirm sign
  convention against firmware.)
- VPE HeadingMode (Reg 35): **Indoor/Relative** gives more stable yaw on a bench or indoors.
- Reference vectors, reference frame rotation (mounting orientation) — see UM001.

## References

- VectorNav UM001 **Rev 2.22 / FW v2.1** — Reg 23/35/44/46/47/84, HSI, Binary Output (reg 75),
  SetGyroBias. PDF not included in this repo (older FW v1.1 manual also absent).
- VectorNav AN012 Hard & Soft Iron Calibration — model assumptions, environment. PDF not included
  in this repo.
- ICD (single source of truth, included in this repo): `docs/protocol.md`,
  `STM32 Nucleo boards and VN 100 documents/VN100_ICD_fw3.1.0.0.pdf`.
- PX4/QGC, ArduPilot MagCal — 6-orientation workflow, live coverage; MicroStrain — bin color states.
