# Building instrument passport v0.1

This is a synthetic typed-record and applicability-check slice for Mycelix #6034. It composes the frozen v0.1 measurand/channel meanings from #6028. Proposed v0.2 channel additions from #6029 are not admitted by this schema.

## Object boundaries

- Instrument identity/configuration is separate from vendor capability claims. The protocol profile has its own digest and independently verified status; local read-only capability is distinct from per-sample acquisition_mode and local_read_only evidence.
- Capability records preserve operating range and conditions, resolution, accuracy basis/evidence class, response time, repeatability, drift and cross-sensitivity state.
- Calibration records preserve certificate identity/authentication, laboratory accreditation scope, method/revision, measurand/unit/range, reference standards, traceability, as-found/as-left results, prior-data impact, and uncertainty.
- Installation records preserve exact asset/building/room, geometry revision, local frame, placement, mounting/shielding and relocation state.
- Samples preserve value, raw/derived processing state, QC state, observation date, event/acquisition time, time unit/timezone/clock source, synchronization uncertainty, sequence, processing method, source digest and exact calibration/installation/firmware/profile references. The passport rejects remote acquisition or a write-capable protocol for channels whose profile requires local read-only acquisition.
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

60 frozen cases cover vendor-only capability, mismatched measurand/unit/range, calibration certificate and scope, traceability, as-found problems, relocation, serial reuse, firmware revisions, timebase/sequence, saturation and faults, derived-measure lineage, uncertainty/correlation, independent-assay binding, rights revocation, overbroad IAQ claims, calibration-uncertainty lineage, calibration-range containment, retired-instrument invalidation, and a fully populated but still blocked derived-measurement record.

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

## Calibration-time applicability

The sample carries an explicit ISO-style observation date. The oracle checks that the calibration date is not after the sample date and that the sample date is not after the calibration due date. The `calibration_state=current` label cannot override an expired interval. Invalid date representations remain non-comparable rather than being guessed.

## Independent assay applicability

Channels whose v0.1 profile requires an independent assay (water pH and conductivity) cannot reach review eligibility without a separately identified assay record. The assay must bind the same observation/sample digest, exact channel measurand and unit, named method/revision, laboratory and accreditation scope, authenticated certificate and authorized rights. A valid assay record is evidence about that exact sample; it is not a general potability certificate or a sensor calibration.


## Local read-only acquisition boundary

The instrument passport requires:
- an exact protocol-profile ID and digest;
- independent verification of that protocol profile;
- an instrument capability declaration for local read-only operation;
- a per-sample declaration that acquisition was local and read-only.

This is a declared applicability check, not cryptographic verification of a protocol implementation. The protocol digest and verification receipt must be backed by a separate independent protocol/capability verification record before real instrument admission. Remote acquisition, command-capable write paths and provider-only protocol claims are negative cases.

## What v0.1 does not yet compute

The passport retains uncertainty inputs, calibration uncertainty, expanded uncertainty and correlation groups. It does not calculate covariance propagation, Monte Carlo distributions, coverage intervals, or conformity guard bands. Those are separate method-qualified tasks under #6034. Likewise, the certificate_state=authenticated field is a classified evidence state, not a built-in XML signature verifier; the DCC interoperability follow-on must verify the source certificate itself.

## 11 November 2026 passport hardening

- The frozen corpus contains **60** cases. The Rust oracle independently pins all 60 IDs and expected dispositions.
- A calibration certificate's uncertainty contribution must appear exactly once in the sample uncertainty budget and must match the exact certificate record, unit and value. Missing or duplicated calibration contributions require more evidence.
- The calibration interval must be internally valid and fully contained by the instrument's independently verified capability envelope. A certificate cannot expand device capability.
- Retired, replaced or moved instruments cannot support a current sample.
- **Derived measurands are deliberately not eligible under the single-instrument v0.1 passport.** A method ID and source digest alone are not a source graph. Mean-radiant temperature and useful thermal energy require exact input sample/calibration/installation/timebase references and a versioned method graph under [#6064](https://github.com/Luminous-Dynamics/mycelix/issues/6064).
- `conditions_id` remains an opaque pointer in v0.1. Actual condition observations, time/location applicability and validated correction revisions are a separate required contract under [#6065](https://github.com/Luminous-Dynamics/mycelix/issues/6065).
- The corpus-embedded passport schema no longer declares a nested `$id` or `$schema`; its local `$ref` values resolve from the corpus schema root.

Static verification at the 11 Nov 2026 cut: the independent recursive schema walk found zero errors across the complete corpus and 60 individual passports; independent disposition replay matched 60/60; all 60 IDs/dispositions matched the Rust-pinned table. This is static verification, not Rust execution. Hosted workflows must pass at the exact PR head before a software PASS is claimed.