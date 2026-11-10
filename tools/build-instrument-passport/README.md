# Building instrument passport v0.1

This is a synthetic typed-record and applicability-check slice for Mycelix #6034. It composes the frozen v0.1 measurand/channel meanings from #6028. Proposed v0.2 channel additions from #6029 are not admitted by this schema.

## Object boundaries

- Instrument identity/configuration is separate from vendor capability claims.
- Capability records preserve the operating range and conditions, resolution, accuracy basis/evidence class, response time, repeatability, drift and cross-sensitivity state.
- Calibration records preserve certificate identity/authentication, laboratory accreditation scope, method/revision, measurand/unit/range, reference standards, traceability, as-found/as-left results, prior-data impact, and uncertainty.
- Installation records preserve exact asset/building/room, geometry revision, local frame, placement, mounting/shielding and relocation state.
- Samples preserve raw value, QC state, event/acquisition time, time unit/timezone/clock source, synchronization uncertainty, sequence, processing method, source digest and exact calibration/installation/firmware/profile references.
- Uncertainty components preserve their correlation groups and sources; this slice does not silently combine independent components or calculate a new uncertainty.
- A purpose profile defines a synthetic decision-specific limit. It is not a universal comfort, safety or code threshold.

## Admission semantics

The evaluator rejects semantic mismatches, invalidates stale or out-of-envelope data, requires evidence for calibration/installation/uncertainty/processing, and only returns `ELIGIBLE_FOR_INDEPENDENT_ADMISSION_REVIEW_ONLY` when all declared synthetic requirements are met. It never grants final instrument admission or proves a building outcome.

## Claim ceiling

All passport data are synthetic. This does not qualify a real sensor, calibration laboratory, accreditation scope, room placement, measurement accuracy, uncertainty statement, indoor air/water quality, comfort, energy, safety or regulatory claim.
