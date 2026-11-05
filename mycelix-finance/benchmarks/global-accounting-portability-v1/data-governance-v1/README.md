# Finance Cross-Border Data-Flow Governance Context (development draft)

This is the first synthetic context package for [FIN-DATA-GOV-001 / #5847](https://github.com/Luminous-Dynamics/mycelix/issues/5847).

## Why this is separate from accounting portability

Accounting, tax, reporting and data-governance applicability are different profile dimensions. A qualified accounting or tax profile does not authorise a data transfer. A cloud storage location does not prove where the data is processed, where support staff can access it, or where backups and onward recipients reside.

The package therefore models the data flow and its actors explicitly; it does not make a legal transfer determination.

## Files

- `finance_data_flow_context_v1.0.0.schema.json`: closed Draft 2020-12 context schema.
- `fixtures/finance_data_flow_candidates_v1.0.0.json`: ten synthetic candidate data-flow contexts.
- `finance_data_flow_oracle_v1.0.0.schema.json`: public reference-oracle schema.
- `fixtures/finance_data_flow_oracle_v1.0.0.json`: separate public expected-review outcomes.
- `../rust-resolver/src/bin/validate_finance_data_flow_context.rs`: standalone Rust Draft 2020-12 and cross-reference validator.

## Semantics

- Responsible party/controller, operator/processor, subprocessor, recipient, onward recipient and support provider are distinct actor roles.
- Collection, primary storage, active processing, backup/DR, support access, recipient and onward-recipient locations are separate observations.
- A data asset has a synthetic subject class, data categories and field-class references, never real personal records.
- A purpose is separately identified and effective-dated; a later purpose must not be presumed covered by an earlier purpose.
- Legal profiles and transfer mechanisms are versioned references with their own status, scope, evidence and reviewer metadata. Each context carries only the relevant jurisdiction regime references rather than treating a universal list as selected law.
- Actor-role claims are flow-bound and may be disputed; the same service provider can be an operator for one flow and make a different capacity claim for another. Conflicting claims are retained, never overwritten.
- Retention/deletion state is distinct from transfer permission. An active legal hold cannot be silently overridden by a scheduled deletion.
- The UK profile is deliberately distinct from the EU profile and marks the post-5-February-2026 regime as a separate synthetic version.
- Unknown location is explicit. Missing support-access or onward-recipient facts are not assumed to be local.

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
cargo run --locked --bin validate_finance_data_flow_context
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

The validator checks Draft 2020-12 schemas, exact actor/asset/purpose/profile/mechanism/flow references, location-scope references, actor-role conflict links, retention/legal-hold invariants and the declared synthetic claim ceiling. It does not decide whether a transfer is legally permitted.

## Synthetic claim ceiling

All actors, jurisdiction scopes, data assets, fields, purposes, mechanism references, evidence identifiers and profiles are synthetic. No legal basis, adequacy finding, transfer permission, POPIA/GDPR compliance, anonymity, data residency, production security or deletion authority is established. Passing the package would prove only handling of these declared synthetic context structures.

## Validation status

The schema, fixtures and Rust validator are authored draft artifacts. They have not yet been run through an independent full Draft 2020-12 runtime, `cargo test`, rustfmt, Clippy, CI or professional legal/privacy review. No pass is claimed.