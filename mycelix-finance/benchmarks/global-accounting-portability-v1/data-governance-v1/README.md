# Finance Cross-Border Data-Flow Governance Context (development draft)

This is the first synthetic context package for [FIN-DATA-GOV-001 / #5847](https://github.com/Luminous-Dynamics/mycelix/issues/5847).

## Why this is separate from accounting portability

Accounting, tax, reporting and data-governance applicability are different profile dimensions. A qualified accounting or tax profile does not authorise a data transfer. A cloud storage location does not prove where data are processed, where support staff can access them, or where backups and onward recipients reside.

The package models data flows and actors explicitly. It does not make a legal transfer determination.

## Files

- `finance_data_flow_context_v1.0.0.schema.json`: closed Draft 2020-12 context schema.
- `fixtures/finance_data_flow_candidates_v1.0.0.json`: ten synthetic candidate data-flow contexts.
- `finance_data_flow_oracle_v1.0.0.schema.json` and `fixtures/finance_data_flow_oracle_v1.0.0.json`: public reference oracle and schema.
- `finance_data_flow_review_report_v1.0.0.schema.json`: closed report schema for deterministic review triage.
- `../rust-resolver/src/bin/validate_finance_data_flow_context.rs`: Rust Draft 2020-12 and cross-reference validator.
- `../rust-resolver/src/bin/evaluate_finance_data_flow_review.rs`: deterministic review-disposition evaluator, knowledge-frontier enforcement, report fingerprinting, oracle conformance and negative controls.

## Semantics

- Responsible party/controller, operator/processor, subprocessor, recipient, onward recipient and support provider are distinct actor roles.
- Actor identity, role list and home-scope metadata are effective-dated and carry a canonical whole-second UTC `recorded_at` timestamp (`YYYY-MM-DDTHH:MM:SSZ`); offsets and fractional seconds are rejected. Actor scope/role metadata recorded after `knowledge_at` is excluded; metadata not valid for the context period cannot silently supply a controller scope.
- Collection, primary storage, active processing, backup/DR, support access, recipient and onward-recipient locations are separate observations.
- A data asset has a synthetic subject class, data categories and field-class references, never real personal records.
- A purpose is separately identified and effective-dated; a later purpose must not be presumed covered by an earlier purpose.
- Legal profiles and transfer mechanisms are versioned references with their own status, scope, evidence and reviewer metadata. Each context carries only the relevant jurisdiction regime references rather than treating a universal list as selected law.
- Actor-role claims are flow-bound and may be disputed; conflicting claims are retained, never overwritten.
- Retention/deletion state is distinct from transfer permission. An active legal hold cannot be silently overridden by a scheduled deletion.
- The UK profile is deliberately distinct from the EU profile and marks the post-5-February-2026 regime as a separate synthetic version.
- Unknown location is explicit. Missing support-access or onward-recipient facts are not assumed local.

## Important legal boundary

The POPIA section 72 reference is a cross-border transfer provision with defined transfer conditions. This package does not equate it with an unconditional data-residency mandate. The EU GDPR and UK GDPR transfer chapters are represented as distinct reference profiles. The candidate profile data are synthetic, not executable law.

Primary references:
- South Africa POPIA section 72: https://www.justice.gov.za/legislation/acts/2013-004.pdf
- EU GDPR Chapter V: https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX%3A32016R0679
- UK Data (Use and Access) Act 2025 consolidated text at 5 February 2026: https://www.legislation.gov.uk/ukpga/2025/18/2026-02-05

## Local Rust validation

From the `rust-resolver` directory:

```sh
cargo test --locked --bin validate_finance_data_flow_context
cargo test --locked --bin evaluate_finance_data_flow_review
cargo run --locked --bin validate_finance_data_flow_context
cargo run --locked --bin evaluate_finance_data_flow_review
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

The validator checks Draft 2020-12 schemas, exact actor/asset/purpose/profile/mechanism/flow references, location-scope references, actor-role conflict links, retention/legal-hold invariants and the synthetic claim ceiling.

The evaluator uses a fixed reason precedence to derive review dispositions, emits every triggered reason code, pins the exact candidate-bundle bytes with SHA-256, validates its report schema, and compares primary dispositions/reasons against every public oracle row. It enforces both valid-time overlap and `recorded_at <= knowledge_at` for actor metadata, role claims, purposes, profiles, mechanisms, flows, locations and retention assertions; future actor metadata cannot determine controller/responsible-party scope. Out-of-period actor scope metadata produces a separate review reason. It emits only `needs_review`, `indeterminate` or `no_trigger_detected`. The last disposition is not a compliance or transfer qualification.

## Decision semantics and adversarial controls

Reason precedence is stable: actor/authority conflict; active legal hold; used unreviewed/disputed purpose; unknown support-access location; assertion recorded after the knowledge frontier; actor scope not valid for the context period; unqualified UK regime profile; incomplete onward-destination mechanism coverage; unqualified onward destination; accounting-only profile; foreign support access; foreign processor location. The priority selects the primary reason; all detected reason codes remain in the report.

Unknown support location, a future-recorded material fact, actor metadata recorded after the frontier, and an accounting-only profile are indeterminate, not assumed local or treated as legal violations. Actor metadata that is not valid for the assessed period requires review. The evaluator tests include a local-only negative control, an unassessed mechanism case, multiple onward destinations, EU/UK regime separation, active legal hold, disputed actor roles, a future-recorded support-location mutation, future actor-scope metadata, and an out-of-period actor scope.

## Synthetic claim ceiling

All actors, jurisdiction scopes, data assets, fields, purposes, mechanism references, evidence identifiers and profiles are synthetic. No legal basis, adequacy finding, transfer permission, POPIA/GDPR compliance, anonymity, data residency, production security or deletion authority is established. The evaluator has no network or external side effects; it never transfers, deletes, blocks, suspends or changes records.

## Validation status

The schema, fixtures, Rust validator and review evaluator are authored draft artifacts. Ten validator tests and fourteen evaluator tests are authored, but have not yet been executed. No independent full Draft 2020-12 runtime pass, Cargo compile, `cargo test`, rustfmt, Clippy, CI or professional legal/privacy review is claimed. A prior semantic simulation of the original ten public oracle dispositions was performed outside Rust; it is not Rust execution, an independent implementation or schema validation.
