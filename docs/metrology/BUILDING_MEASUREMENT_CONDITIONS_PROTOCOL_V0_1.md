# Measurement condition applicability v0.1

Parent: [MYC-BUILD-CONDITIONS-001 #6065](https://github.com/Luminous-Dynamics/mycelix/issues/6065).

## Purpose and ownership

This sidecar binds capability-condition references in the v0.1 instrument passport to actual condition observations spanning the sample's spatial and temporal support. It is not a second sensor registry, correction engine, calibration authority, stream-timebase implementation, uncertainty calculator or building model.

- Mycelix #6028/#6040 owns instrument/channel/sample/calibration passport semantics.
- Mycelix #6054 owns stream, clock-domain, boot-epoch and sequence qualification. This sidecar references but does not replace that owner.
- Mycelix #6048 owns numerical uncertainty propagation. This sidecar checks condition-observation uncertainty presence and profile bounds but does not calculate it.
- Mycelix #6042 owns dependency graph and affected-descendant invalidation.
- Mycelix #6064 owns derived-measurement graphs; derived outputs inherit condition dependencies through exact source samples.

## Paradigm shift: operating envelope as observed evidence

A calibration/capability statement is conditional on measurand, method, configuration, environment and installation. The system evaluates a specific sample against a versioned condition profile, not a device against a free-floating `valid=true` flag.

## Registry v0.1

Six synthetic profiles demonstrate condition contracts for thermal sensors, optical PM, gas sensing, water flow, electrical metering and acoustics. Their values are fixture values only; they are not physical limits, recommendations, local standards or accepted product tolerances. Each profile pins channel/measurand, firmware/configuration, required variables, spatial and temporal support and correction policy.

The synthetic corpus pins the profile registry by the Git blob SHA-1 returned for the exact registry file. This is repository-content identity for fixtures, not cryptographic authentication or a real-world trust anchor. Production source authenticity belongs to #6047.

## Condition observations

Each required variable binds its ID, quantity kind, unit and numeric/categorical semantics; source observation ID/digest; source instrument/channel and calibration record/state; source installation, geometry revision, local location context and spatial relation; uncertainty for numeric variables; rights/evidence state; and exact timebase/stream/clock-domain/boot-epoch support.

A different-zone or same-zone-only observation cannot silently stand in for a local condition. Spatial transfer is allowed only when the profile permits it and a method plus validation receipt are bound.

## Temporal coverage

Condition evidence must use the same timebase contract reference, stream, clock domain, boot epoch and tick unit as the passport sample; its interval must cover the complete sample support. Partial coverage, unresolved clock mapping or a different epoch produces abstention. Timebase fields in v0.1 fixtures are synthetic references only; real applicability waits for #6054 owner receipts.

## Correction policy

When a profile does not require correction, an unexpected correction is rejected. When one is required, exact method/revision/digest and validation receipt must match, the source-condition digest must exist, and application count must equal one. Missing, provider-asserted, mismatched or repeated corrections never pass.

## Dispositions

- `NOT_COMPARABLE`: profile, channel, unit, geometry, location or clock identity differs.
- `MORE_EVIDENCE_REQUIRED`: required observation, calibration, uncertainty, time mapping or correction evidence is incomplete or not independent.
- `OUTSIDE_VALIDATED_ENVELOPE`: an observed numeric condition exceeds the synthetic profile envelope.
- `EXPIRED_OR_INVALIDATED`: profile validity or firmware/install applicability is superseded.
- `REJECTED_FOR_DECLARED_PURPOSE`: rights are revoked or correction policy is violated.
- `ELIGIBLE_FOR_INDEPENDENT_CONDITIONS_REVIEW_ONLY`: all synthetic gates match; this is not passport admission or a physical claim.

## Claim ceiling

All records are synthetic. Passing this corpus proves only deterministic condition-applicability semantics. It does not prove any actual instrument operated within its capability envelope, validate a real correction algorithm, establish a thermal/IAQ/flow/acoustic/electrical measurement, authorize physical testing, or qualify comfort, safety, health, compliance or service performance.


## Evidence-state clarification

All registry profiles and all positive condition-observation fixtures use `synthetic_reference_only`. They do not represent independently characterized instruments, calibrated condition sensors, validated installation observations, or real rights grants. Provider-asserted, stale, revoked, out-of-envelope, non-comparable and incomplete states are negative-control fixtures. Only a future owner-verified condition record, resolved under the exact #6054 timebase and #6042 dependency contracts, may support a real sample-applicability review. This synthetic oracle never admits a physical measurement.


## Additional negative controls — 11 November 2026

The 43-case corpus separately checks the target passport sample's own timebase and rights state, expired condition calibration, invalidated condition installation, revoked and unknown condition-source states, and missing validation receipt for a required correction. Matching condition timestamps cannot rescue an unresolved target sample timebase; matching synthetic references cannot promote expired or revoked source evidence.
