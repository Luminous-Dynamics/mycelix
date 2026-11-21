# Mycelix Space Qualification Package v0.1

## Purpose

This is the first machine-readable contract slice for [SPACE-QUAL-BUNDLE-001](https://github.com/Luminous-Dynamics/mycelix/issues/6442). It defines a portable package envelope that can later compose exact source artifacts, request scope, object/time coverage, numerical assessments, exact-head execution receipts, authority assertions and layer-specific claim ceilings.

It is deliberately an adapter contract. It does not implement a propagator, conjunction probability method, covariance estimator, source-admission system or generic qualification-receipt framework.

## Files

- `contracts/space-qualification-package-v0.1.schema.json` — JSON Schema Draft 2020-12 structural contract.
- `contracts/fixtures/synthetic-source-state-set-v0.1.json` — exact synthetic source bytes referenced by the sample package.
- `contracts/fixtures/synthetic-package-indeterminate-v0.1.json` — a package fixture exercising the distinction between input coverage and risk conclusion.

## Semantic invariants

1. **Absence is not zero.** Field semantics distinguish `present`, `absent_optional`, `invalid`, `unsupported` and `transformed`. An absent optional covariance or collision-probability field is not a zero measurement.
2. **Coverage is scoped.** `complete` means every item in the declared request set and time-domain scope received a disposition. It does not prove that the request set includes every real object in orbit.
3. **Coverage is not risk.** A coverage-complete run may remain `indeterminate` when covariance, Pc, a numerical method or another required assessment is absent or inconclusive.
4. **Negative conclusions are gated.** The schema requires a negative conclusion to accompany complete package and coverage dispositions. A separate verifier must additionally reconcile counts, requested IDs, object dispositions, segment coverage and artifact references.
5. **Execution is exact-head.** A workflow receipt must name the exact repository, run ID and head SHA. `queued`, `in_progress`, `skipped`, `stale` and `blocked` are not passes.
6. **Authority is non-actuating.** The v0.1 package fixes `actuation_authority=false`; coordination records cannot grant spacecraft command authority.
7. **Claim layers remain separate.** Schema conformance, source integrity, input coverage, time-domain coverage, numerical assessment, covariance diagnostics, replay and operational safety must not be collapsed into one score.

## Standards/profile identity

A source profile must carry three separate values:

- ecosystem / standards owner;
- document or interface release;
- payload/message version and concrete encoding profile.

For example, the Office of Space Commerce's TraCSS CDM Release 2.1 document and the payload-level CCSDS CDM version/encoding are separate identifiers. CCSDS CDM v1.0 KVN, TraCSS CDM Release 2.1 KVN/XML, JSON-ST, CSV, TraCSS JSON and Mycelix's legacy flattened representation must not be treated as one parser target.

## Synthetic fixture interpretation

The fixture contains two manufactured objects and two synthetic time segments. It marks both objects as screened relative to this exact synthetic request set, so the coverage disposition is complete. Covariance is explicitly absent, collision probability is not calculated, and the package-level result is therefore `indeterminate` with `negative_conclusion=false`.

The fixture is not an ephemeris truth set, orbital simulation result, conjunction assessment, physical model validation or operational safety demonstration.

## Validation boundary and next increment

The JSON Schema provides structural validation only. It cannot by itself prove:

- integer count reconciliation across declared and repeated arrays;
- referential integrity between object IDs, source artifact IDs, time segments and assessment records;
- source bytes actually match their declared digest and byte length;
- time interval ordering or complete continuous-time coverage;
- parser conformance to any CCSDS / TraCSS profile;
- signature authenticity, credential scope or provider independence;
- numerical correctness, covariance calibration, physical truth or operational safety.

The next implementation increment should add a small dependency-light Rust verifier for those cross-field joins and exact-byte checks, plus frozen negative controls. The verifier must emit `PASS`, `FAIL` or `INCOMPLETE` with reason codes, and it must never upgrade synthetic evidence to a reference-replay or operational class.
