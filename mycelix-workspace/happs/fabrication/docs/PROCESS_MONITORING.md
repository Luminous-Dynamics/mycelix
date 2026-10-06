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

The baseline records its schema version, detector identifier/version, source observation count, source-observation SHA-256 commitment, and feature statistics.

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
## Exact contextual baseline boundary

FPM context binding is implemented separately from the raw detector so that a statistically unusual value is not confused with a context change.

A `ProcessContext` records, when available, the process phase, layer, move class, commanded speed, target temperatures, acquisition clock domain, sensor source identity, and calibration-profile commitment. Baseline construction requires the source observations to carry one exact common context.

The contextual selector is deliberately **exact-match only** in v1:

- exact context -> detector may run;
- context mismatch -> explicit `ContextMismatch` error;
- missing required registration/discriminator -> explicit `IncompleteContext` error;
- mixed-context baseline source window -> rejected rather than pooled.

There is no implicit nearest-context, global-baseline, or "best effort" fallback. Explicit compatibility rules can be added later as a separately versioned policy, but they must never be inferred from statistical similarity.

The contextual detection provenance records both the context commitment and a combined digest over the detector result and context. This keeps raw detector evidence distinguishable from the stronger claim that the baseline was selected under an exact registered process context.

## Registration and physical validation boundary

Context binding is necessary but not sufficient for multimodal registration or physical quality claims. ISO/ASTM 52953:2025 establishes requirements for registering process-monitoring and quality-control data, including multimodal data registration. ISO/ASTM TR 52958:2026 describes a flaw-detection workflow that uses process monitoring together with seeded flaws and CT validation. These support keeping data registration, inference, and independent physical validation as separate evidence layers. citeturn831155search0turn831155search1

The next research layer is therefore a separately versioned multimodal registration contract covering source-local clocks, alignment, calibration/profile identity, and explicit handling of missing or conflicting context. It should not silently convert alignment uncertainty into a normal or healthy result.
## Multimodal registration evidence

FPM now has a separate registration layer for heterogeneous sensor evidence. A `RegistrationEnvelope` binds source identity, modality, source-local sequence/timestamp, clock domain, an explicit correlation domain and producer-assigned correlation ID, calibration-profile commitment, process-context commitment, source-data digest, and an explicit alignment method.

The registration envelope also requires at least two distinct modalities and rejects duplicate `(source_id, modality)` participant identities. This prevents a single modality from manufacturing a superficially multimodal registration by repeating the same source record.

Registration state is derived from the envelope rather than accepted as a caller-supplied field. Correlation IDs are namespace-bound so an opaque ID reused by unrelated acquisition systems cannot by itself imply shared frame identity. The mechanically derived positive state is named `Consistent`, not `Registered`, because these inputs remain supplied by the acquisition/registration layer:

- **Consistent** — supplied registration metadata is internally coherent and passes the deterministic checks;
- **Unregistered** — required registration/alignment evidence is absent;
- **Conflicting** — participants disagree on context, calibration, clock/sequence, or timestamp alignment;
- **Invalid** — the envelope/schema/digest is malformed;
- **Unknown** — evidence is insufficient to establish the requested alignment.

The first exact methods are correlation-ID alignment and shared-clock exact source timestamp. A correlation ID is a producer-assigned identifier for the same acquisition frame within an explicit correlation domain; source-local sequence counters are not assumed comparable across modalities. Cross-clock registration requires an explicit committed transform or external registration-evidence digest. The presence of such a commitment records an explicit registration claim; it does not independently prove the transform, sensor calibration, or physical truth.

This distinction follows established industrial data semantics: OPC UA preserves a source timestamp assigned at the data source and distinguishes it from server receipt time; it also calls for synchronized source clocks where sources are redundant. IEEE 1588/PTP is a standard mechanism for synchronizing clocks in networked measurement and control systems. citeturn106905search0turn106905search3turn106905search6

A downstream detector should consume multimodal evidence only after the registration state and provenance requirements for that detector are satisfied. Registration status must never be silently converted into a Normal detection result.

## Next research boundary

The remaining work is empirical qualification of registration and detection against captured process traces, including clock skew, dropped/duplicated samples, calibration changes, context conflicts, and independent physical reference measurements. The current implementation remains a deterministic evidence contract, not a validated physical-defect detector.
## Structural qualification boundary

FPM now defines a narrowly scoped structural qualification profile. Its purpose is to let a verifier produce a deterministic, content-bound statement about the internal consistency of a registration envelope.

The profile binds the qualification schema, profile identifier/version, verifier identity, exact registration digest, assessed registration state, outcome, and a final qualification digest. The current implementation explicitly carries the registration and qualification digest profiles. They are Rust serde-json profiles for this version, not yet a cross-language canonical encoding contract; Issue #4313 tracks the separate canonicalization qualification. The qualification digest excludes its own digest field, so the resulting receipt can be deterministically recomputed and checked for tampering.

The positive outcome is deliberately named Qualified only in the sense of **qualified against the structural profile**. It does not establish source authenticity, sensor correctness, physical clock synchronization, calibration correctness, transform correctness, or physical part quality.

Outcome mapping is fail-closed:

- Consistent registration -> Qualified by the structural profile;
- Unregistered or Unknown registration -> InsufficientEvidence;
- Conflicting or Invalid registration -> Rejected.

This mirrors the broader Mycelix evidence pattern of keeping an authored statement, its transparency/registration, an independent verifier result, and physical truth as distinct evidence layers.

ISO/ASTM 52951:2026 strengthens the digital-thread rationale: its AM data-package model spans the workflow from design to acceptance and includes configuration management, production qualification, quality-control planning, and inspection requirements. FPM therefore treats the registration/qualification record as one evidence object within a larger part data package rather than as a standalone proof of manufacturing quality. citeturn832228search0turn832228search3

Issue #4306 tracks the future transition from structural qualification to independently verified qualification. That transition requires an actual verifier and exact evidence inputs; the structural profile does not silently confer that stronger status.
