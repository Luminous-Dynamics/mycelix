# Fabrication Process Monitoring (FPM)

Fabrication Process Monitoring (FPM) is the canonical name for the Fabrication hApp's in-process manufacturing monitoring subsystem.

The historical Cincinnati name remains a compatibility/codename reference only. It is not the name of a proven detector algorithm.

## Evidence layers

FPM deliberately separates four layers:

1. Observation — measured process data such as temperature, motor current, vibration, and filament tension.
2. Detection — a deterministic inference that an observation differs from a declared baseline.
3. Intervention — a proposed or executed response to a detection. The first FPM engine does not perform interventions.
4. Assessment — a quality or defect conclusion about the produced part. This requires a separate, versioned assessment model and independent validation.

A detector result is therefore not, by itself, proof that a physical defect exists.

## Deterministic FPM core

The initial Rust implementation lives in the fabrication_common crate as the fpm module.

Its core pipeline is:

ProcessObservation -> BaselineProfile -> analyze() -> ProcessDetection

### Baseline

FPM uses robust median and median-absolute-deviation (MAD) statistics for the initial detector.

Required channels must have at least 5 baseline observations:

- hotend temperature;
- bed temperature;
- extruder current;
- vibration RMS.

Filament tension is optional and is only admitted into a baseline after the same minimum sample count is available.

The baseline records its schema version, detector version, source observation count, and feature statistics.

### Detection semantics

| Feature | Inference class |
|---|---|
| Hotend temperature | TemperatureDeviation |
| Bed temperature | TemperatureDeviation |
| Extruder current | ExtrusionInconsistency |
| Vibration RMS | VibrationAnomaly |
| Filament tension | FilamentSlip |

The detector emits an anomaly_score in [0, 1]. This value is a normalized decision score, not a probability, confidence, or physical severity measurement.

Zero-dispersion baselines are handled explicitly: an exact match is normal; any differing observation is anomalous, with no fabricated z-score.

## Provenance

Every ProcessDetection records:

- FPM schema version;
- detector identifier;
- detector version;
- SHA-256 digest of the baseline;
- SHA-256 digest of the observation;
- SHA-256 digest of the complete analysis input.

This means the same detector can be re-run against the exact committed inputs rather than relying on mutable runtime state.

The pure FPM engine does not read the DHT, call other zomes, read wall-clock time, or modify printer state.

## What is not claimed

FPM v1 does not claim:

- calibrated defect probabilities;
- general defect-detection accuracy;
- universal sensor thresholds;
- physical defect confirmation from a sensor anomaly alone;
- safe autonomous printer control.

Those require separate empirical qualification.

## Qualification direction

The first qualification corpus should contain at least:

- healthy traces;
- isolated sensor spikes;
- gradual sensor drift;
- correlated multi-sensor deviations;
- missing observations;
- duplicated observations;
- reordered observations;
- timestamp/sequence discontinuities;
- calibration/profile changes;
- detector-version changes.

Physical claims should additionally use independent reference measurements such as dimensional inspection, microscopy, CT/XCT, or other process-appropriate nondestructive/destructive tests.

## Standards context

ISO/ASTM 52953:2025 establishes general requirements for registration of process-monitoring and quality-control data in additive manufacturing and explicitly addresses multimodal data registration.

ISO/ASTM TR 52958:2026 provides a concrete in-situ monitoring and flaw-detection workflow using coaxial photodiode monitoring, statistical methods, clustering, seeded flaws, and CT-based validation.

ASTM E3353 covers in-process monitoring sensors, configurations, data analysis, and data uses for laser powder bed fusion.

The FPM architecture therefore treats data registration, inference, and physical validation as distinct concerns instead of treating a sensor anomaly as a finished quality verdict.

## Migration

Legacy Cincinnati types and APIs should not be renamed blindly. Once the FPM schemas have been qualified, the naming migration can introduce new canonical types while retaining explicit deserialization compatibility for existing records.