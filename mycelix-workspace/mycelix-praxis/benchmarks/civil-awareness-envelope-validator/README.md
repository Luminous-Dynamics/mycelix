# Civil Awareness Envelope Validator

Offline, fail-closed structural and policy validator for the v0.1 civilian awareness evidence envelope used by UAS, maritime and civil-aviation research.

## Scope

This crate validates:
- the pinned JSON Schema Draft 2020-12 document and schema identity;
- the repository-pinned field-policy version and exact SHA-256 digest;
- the domain extension URI, revision and field allowlist;
- the `claim.value` allowlist for each `(domain, claim_type)`, rejecting nested objects and arrays in v0.1;
- RFC 8785 JCS payload and whole-envelope digests;
- strict JSON duplicate-key rejection and the 2 MiB input limit;
- event-time ordering, declared purpose vs allowed uses, retention expiry at ingest, rights state, source state, clock quality, freshness, calibration, missingness, abstention, dependency and spatial-purpose evidence.

It has **no network retrieval, device access, operational control integration, sensor ingestion, navigation/flight-control interface, or action authorization**. The JSON Schema is embedded with `include_str!`; remote `$ref` and `$dynamicRef` values are rejected. The compiler runs once and the reusable validator is applied to each input.

## Evidence dispositions

| Disposition | Meaning | CLI exit |
|---|---|---:|
| `REJECTED` | Schema, digest, allowlist, parser or structural invariant failed | 2 |
| `QUARANTINED` | Rights, authorization, privacy, purpose or retention gate failed | 3 |
| `UNRESOLVED` | Evidence quality / provenance is inadequate for replay comparison | 4 |
| `STRUCTURALLY_VALID_REQUIRES_AUTHORITY_RECEIPT` | Local structure and declared gates pass, but independent source attestation and signed authority receipt are still required | 0 |

Exit 0 does **not** mean accepted for deployment or field use. A self-declared `integrity_status: authenticated` is not cryptographic proof. This crate does not implement the shared evidence-plane receipt verifier, so it never emits an operationally accepted disposition.

The machine-readable report contains only stable reason codes, a schema error count and the pinned policy digest. It does not echo source values or raw validation messages.

## Run

From this directory:

```sh
cargo fmt --check
cargo test
cargo clippy --all-targets -- -D warnings
cargo run -- path/to/envelope.json
```

The binary accepts one JSON file path. It prints a JSON validation report to stdout and returns the disposition-specific exit code. Initialization and file/usage errors are reported to stderr with stable, value-free codes.

## Pinned artifacts

The crate embeds these sibling research artifacts from `mycelix-workspace/mycelix-praxis/docs/research/`:
- `civil-awareness-evidence-envelope-v0.1.schema.json`
- `civil-awareness-field-policy-v0.1.json`
- `civil-awareness-envelope-examples-v0.1.json`

Digest contract:
- `field_allowlist_digest`: SHA-256 over exact UTF-8 bytes of the field-policy file.
- `payload_digest`: SHA-256 over RFC 8785 canonical JSON bytes of `domain_extension.payload`.
- `envelope_content_digest`: SHA-256 over RFC 8785 canonical JSON bytes of the entire envelope after removing only `lineage.envelope_content_digest`; it therefore binds the other fields, including correction/supersession lineage.

The JCS library dependency range selects the RFC 8785-compatible `serde_json_canonicalizer` implementation. The implementation targets the `jsonschema 0.61` API, Draft 2020-12, with format validation enabled and default reference-resolution features disabled. A Cargo lockfile has not yet been generated in this authoring environment; exact transitive dependency resolution and lockfile review remain required before merge.

## Test strategy

The unit suite covers the three synthetic domain fixtures, unknown schema/policy identity, external-reference rejection, extra fields, wrong digests, nested claim values, unapproved rights, purpose mismatch, retention expiry, unverified sources, stale data, missing dependency groups, reversed event intervals, metadata/lineage digest tampering, duplicate JSON properties, over-size input and a canonicalization vector from RFC 8785.

The included examples are **synthetic and non-operational**. Unit tests prove only local parser/schema/policy/digest behavior. They do not establish source authenticity, independent truth, detector accuracy, vessel/aircraft safety, legal compliance, regulatory authorization or SOTA performance.

## Design constraints

- No weapon, interceptor, effector, mitigation, jamming, spoofing, takeover, targeting, hostile-intent or third-party-control functionality.
- No pilot, crew, controller or remote-operator identity / personal-location fields; no private communications, biometric fields or raw personal audio/video.
- No connections to vessel navigation control, aircraft flight controls, ATC/VTS commands, ACAS/TCAS or certified detect-and-avoid systems.
- Unknown schema revisions, policy mismatches and unapproved fields fail closed.
- Missing, stale, conflicted, withheld, unauthenticated and uncalibrated evidence remains unresolved or quarantined; it is never silently promoted to truth.

## Parent issue

[GCPN-CIVIL-ENVELOPE-VALIDATOR-001 — #5992](https://github.com/Luminous-Dynamics/mycelix/issues/5992)


## Coverage-aware observability evaluator

The `observability` module reuses this crate's strict duplicate-key parser and adds a pure, deterministic evaluator for the frozen UAS / maritime / civil-aviation observability DGP:

- DGP: `mycelix-workspace/mycelix-praxis/docs/research/civil-awareness-observability-dgp-v0.1.json`
- Schema: `mycelix-workspace/mycelix-praxis/docs/research/civil-awareness-observability-dgp-v0.1.schema.json`
- Profile: `mycelix-workspace/mycelix-praxis/docs/research/GCPN_CIVIL_OBSERVABILITY_DGP_V0_1.md`

Public API:

- `ObservabilityEvaluator::from_embedded()` compiles the pinned Draft 2020-12 schema without remote references.
- `evaluate_embedded()` validates and assesses the frozen corpus.
- `evaluate_dgp_bytes()` strictly parses exact bytes, checks the schema and reports one state/reason set per case.
- `assess_case_bytes()` assesses a single strict JSON case.

The evaluator deliberately does not read the DGP `expected` field; the unit-test harness compares its outputs with the oracle after evaluation. It binds the DGP/schema digests, preserves source-level claim references, and emits the same ten explicit states. It reports `NOT_OBSERVED_WITHIN_DECLARED_COVERAGE` only when source health, rights, freshness, completeness, coverage validity across the entire scope window, coverage-profile digest and dependency snapshot/frontier all pass.

A single fresh positive source observation can still be reported when the broader coverage envelope is incomplete; this is not a claim of complete monitoring. In a multi-source assessment, incomplete coverage or lineage remains `COVERAGE_UNKNOWN`, and qualified contradictory source-level outcomes remain `CONFLICT` with both claims preserved.

Run the existing commands from this package to execute the new tests as part of `cargo test --all-targets`. No new graph, source registry, coverage GIS, evidence ledger, trust root or runtime sensor integration is added.
## Dependency lock and validation status

This package currently has no committed `Cargo.lock`. The path-scoped workflow resolves dependencies and runs formatting, all-target tests and Clippy, but a lockfile must be generated, reviewed and committed before merge. CI success without a committed lockfile does not establish reproducible transitive dependency resolution.

The observability evaluator's 19-case suite includes all ten states, missing/stale dependency-frontier controls, a coverage-proof-removal mutation, a qualified cross-source conflict, a positive source observation under partial broader coverage, and a direct AIS non-carriage precedence assertion. Tests establish only the declared synthetic state-transition behavior; they do not qualify a field coverage model or real detector.

Latest development note: 44 package library tests passed on the prior exact head; Clippy's final `collapsible_if` warning was corrected in the current branch and awaits a new exact-head workflow result.
## Probability calibration and exposure-qualified metrics

The `calibration` module provides small pure Rust primitives for the #6008 calibration benchmark:

- `score_calibration`: Brier score, log loss, prevalence-baseline Brier score, Brier Skill Score, equal-width reliability bins and descriptive ECE. Each record retains a steward-defined source-group ID; group count is reported but independence is not inferred.
- `probability_of_detection`: Pd only among independently confirmed events with valid observation exposure, with a 95% Wilson interval. Zero eligible events/exposure returns `None`, not Pd = 0.
- `false_alarm_rates`: raw false alerts and already-deduplicated false-event clusters per eligible source-hour; zero exposure is an error rather than a zero rate.

These primitives do not implement cluster-bootstrap uncertainty, source/event clustering, a DGP generator, field calibration or deployment thresholds. ECE is secondary and bin-dependent. Use #5973 for clustered uncertainty; retain every source group and report denominators/strata. Do not compare results unless eligible exposure and source/dependency profiles are matched.

Benchmark specification: `mycelix-workspace/mycelix-praxis/docs/research/GCPN_COVERAGE_CALIBRATION_BENCHMARK_V0_1.md`; DGP and schema are pinned alongside it. The DGP is a specification only and its generator is not yet implemented.