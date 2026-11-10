# Building instrument passport v0.1

This is a synthetic typed-record and applicability-check slice for Mycelix #6034. It composes the frozen v0.1 measurand/channel meanings from #6028. Proposed v0.2 channel additions from #6029 are not admitted by this schema.

## Object boundaries

- Instrument identity/configuration is separate from vendor capability claims.
- Capability records preserve operating range and conditions, resolution, accuracy basis/evidence class, response time, repeatability, drift and cross-sensitivity state.
- Calibration records preserve certificate identity/authentication, laboratory accreditation scope, method/revision, measurand/unit/range, reference standards, traceability, as-found/as-left results, prior-data impact, and uncertainty.
- Installation records preserve exact asset/building/room, geometry revision, local frame, placement, mounting/shielding and relocation state.
- Samples preserve value, raw/derived processing state, QC state, event/acquisition time, time unit/timezone/clock source, synchronization uncertainty, sequence, processing method, source digest and exact calibration/installation/firmware/profile references.
- The independent-assay object is separate from the instrument sample. Channels that require an independent assay must bind the assay to the exact measured sample ID/digest, measurand, method/revision, laboratory scope, certificate and rights. An instrument reading cannot self-certify water quality.
- Uncertainty components preserve source records and correlation groups. This slice does not silently treat correlated inputs as independent, combine an uncertainty budget, or calculate a propagated uncertainty.
- A purpose profile defines synthetic decision-specific range, response-time, time-synchronization and uncertainty limits. It is not a universal comfort, safety or code threshold.

## Admission semantics

The oracle distinguishes:
- `NOT_COMPARABLE`: identity, measurand, unit, revision, geometry, method or time semantics do not match;
- `MORE_EVIDENCE_REQUIRED`: the required evidence is absent, incomplete or not independently supported;
- `EXPIRED_OR_INVALIDATED`: calibration/sample/installation is stale, out of envelope, faulted or superseded;
- `REJECTED_FOR_DECLARED_PURPOSE`: the evidence does not satisfy the exact declared purpose or rights constraints;
- `REJECTED_CLAIM_PROMOTION`: a reading is being used to assert a broader claim than it supports;
- `ELIGIBLE_FOR_INDEPENDENT_ADMISSION_REVIEW_ONLY`: synthetic gates are structurally satisfied, but a competent owner/reviewer still must decide physical admission.

An eligible record is not a PASS, measurement certification or device command. No downstream owner may convert it into an automatic building, health, safety, quality or regulatory claim.

## Synthetic corpus

45 frozen cases cover vendor-only capability, mismatched measurand/unit/range, calibration certificate and scope, traceability, as-found problems, relocation, serial reuse, firmware revisions, timebase/sequence, saturation and faults, derived-measure lineage, uncertainty/correlation, independent-assay binding, rights revocation and overbroad IAQ claims.

## Standards and method sources

- [ISO 7726:2025, Edition 3](https://www.iso.org/standard/78238.html): published thermal-environment instruments and measurement methods; verify applicability per thermal channel and method.
- [ISO/IEC 17025:2017](https://www.iso.org/standard/66912.html): current laboratory competence, impartiality and consistent operation requirements; accreditation does not cover every method/range by default.
- [SANAS accredited-facilities directory](https://www.sanas.co.za/pages/index.aspx?page=Accredited-Facilities): check the exact current scope schedule, measurand, range and method.
- [ILAC P10:07/2020](https://ilac.org/publications-and-resources/ilac-policy-series/): metrological traceability.
- [ILAC P14:09/2020](https://ilac.org/publications-and-resources/ilac-policy-series/): measurement uncertainty in calibration.
- [JCGM guides and GUM suite](https://www.bipm.org/en/web/guest/publications/guides): JCGM 100:2008 and current amendments/supplements for expressing measurement uncertainty.
- [IEC TR 63250:2021](https://webstore.iec.ch/en/publication/65090): repeatability, reproducibility, uncertainty and round-robin guidance for household-appliance performance methods; it does not cover production-unit variability or normal-household representativeness.

See [`BUILDING_INSTRUMENT_PASSPORT_METHOD_CROSSWALK_V0_1.md`](../../docs/metrology/BUILDING_INSTRUMENT_PASSPORT_METHOD_CROSSWALK_V0_1.md) for method-to-channel boundaries and [the benchmark protocol](../../docs/metrology/BUILDING_METROLOGY_BENCHMARKS_V0_1.md) for competitor baselines and target metrics.

## Claim ceiling

All passport data and results are synthetic. This does not qualify a real sensor, calibration laboratory, accreditation scope, room placement, measurement accuracy, uncertainty statement, indoor air/water quality, comfort, energy, safety or regulatory claim.
