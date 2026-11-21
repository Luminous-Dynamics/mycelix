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


## Executable verifier contract

The standalone Rust verifier lives at `mycelix-workspace/mycelix-space/tools/space-qualification-verifier/`. It verifies cross-field invariants that JSON Schema cannot establish:

- requested object IDs exactly match one-per-object terminal dispositions;
- count fields reconcile (`eligible = assessed + unassessed`, `requested = eligible + excluded`);
- per-object time-segment references match the coverage ledger;
- source and assessment references resolve;
- retained artifact paths remain repository-relative and cannot escape the declared repository root;
- exact source byte lengths and SHA-256 digests match the retained files;
- `absent_optional` values remain null, never numeric zero;
- a negative event-screening conclusion requires complete object and time-segment coverage, no exclusions, completed conjunction-screen assessment, and a non-synthetic evidence class;
- synthetic packages and unexecuted replay remain `INCOMPLETE`, even when package integrity checks pass.

The executable emits a machine-readable `PASS`, `FAIL` or `INCOMPLETE` result. `--allow-incomplete` is intended for fixture CI: it permits a structurally valid synthetic fixture to exit successfully while retaining the `INCOMPLETE` result in its JSON output. It does not turn the fixture into an operational qualification.

The verifier is a consistency and evidence-integrity checker. It does not implement the full JSON Schema Draft 2020-12 evaluator, cryptographic signature/key validation, physical truth, orbit propagation, continuous-time event completeness, Pc correctness or covariance calibration.


## Pair-universe coverage and proof-carrying pruning

Object coverage and time-segment coverage are not sufficient to qualify a broad-phase conjunction sieve. A screening run may disposition every object while silently omitting one or more primary-secondary pairs. The v0.1 package therefore adds a distinct `pair_coverage` ledger for the declared primary-versus-each-requested-secondary universe.

Every requested pair receives exactly one disposition:

- `retained_candidate`: pass to the narrow-phase encounter search; any retained candidate must have a terminal pair-specific narrow-phase assessment before it can support a negative result.
- `proven_excluded`: a pruning decision with a retained witness artifact, exact witness SHA-256, named method/profile, and deterministic reason code.
- `unresolved`: the sieve cannot establish either safe exclusion or a complete candidate disposition.
- `unsupported`: the pair falls outside the declared model/profile envelope.

The verifier reconciles the full requested pair set and counts. It recomputes a domain-separated SHA-256 over the pair entries sorted by pair ID and canonical JSON object-key order. It binds a proven-excluded pair to the exact retained witness bytes. A retained pair's `searched_no_event` / `event_found` outcome must join to a completed pair-specific conjunction-screen assessment whose output artifact and digest are bound. An unresolved or unsupported pair, or a retained pair that has not completed narrow-phase search, blocks a complete negative conclusion.

This creates a **proof-carrying sieve**: candidate reduction is allowed, but every removed pair has an inspectable exclusion witness. If the required bound cannot be established, the pair remains retained, unresolved or unsupported; it is never silently pruned.

For adaptive uncertainty-aware broad phases, this is the important research distinction:
- uncertainty may widen candidate intervals when the source or propagated uncertainty grows;
- unknown, synthetic-only or uncalibrated covariance must not narrow exclusion bounds;
- a learned or data-derived threshold is a candidate-generation heuristic until its capture envelope is independently qualified;
- candidate capture, time-domain event search, covariance realism, Pc applicability and publication policy remain distinct claim layers.

The exact synthetic fixture contains one requested primary-secondary pair and a manufactured narrow-phase output artifact. It proves only pair-ledger shape and joins; it is not a physical screening run.


The pair manifest digest uses the domain prefix `mycelix-space-pair-manifest-v1`, an unsigned 64-bit big-endian length prefix, and the compact `serde_json` serialization of the pair array sorted by `pair_id`. JSON object keys are serialized in sorted order by the verifier's serde_json configuration. Any mutation to a pair disposition, source-artifact reference, witness, narrow-phase state or assessment link changes the digest.


## Whole-package content commitment

The v0.1 envelope carries `package_manifest_sha256`, a domain-separated digest of the entire package object with that one member removed. It commits the request and its source-configuration binding, every retained source artifact declaration, object/time/pair coverage ledgers, assessment input/output joins, execution and replay receipts, authority records, layer-specific claim matrix, result disposition and notices.

Canonicalization uses the verifier's sorted-object-key `serde_json` encoding, compact UTF-8 JSON bytes, domain prefix `mycelix-space-qualification-package-v0.1`, and an unsigned 64-bit big-endian byte-length prefix before hashing with SHA-256. The package digest does not authenticate the issuer or prove the evidence is true. It makes post-issuance mutation detectable when the digest is carried independently; signature/credential verification and physical validation remain separate qualification layers.
