# Home Compliance Core (prototype)

A small deterministic Rust core for one narrow part of a future homeowner-controlled design system: deciding whether a scoped evidence package is ready for professional review.

This crate is intentionally separate from Holochain WASM zomes. It does not run devices, generate architecture, parse IFC, interpret statutes, calculate structural safety, or issue approvals.

## What it enforces

- Rule-pack metadata is present, its declared status is active, and its effective-date window includes the assessment date.
- Each declared mandatory check has exactly one finding.
- Findings bind to the exact rule-pack ID and edition used by the assessment.
- Missing findings, duplicate or unexpected findings, mandatory Fail/Unknown results, invalid evidence-digest syntax, and unjustified NotApplicable results block readiness.
- The only positive status is ReadyForProfessionalReview; there is no Compliant, Certified, Approved, or ReadyToBuild enum. The evolution module validates artifact revision lineage and plans bounded migrations only across edges declared verified-lossless.

## Status

Early prototype and design slice. Unit tests are included but were not run in the authoring environment because Rust tooling is unavailable there. The crate includes an evolution module for revision lineage and migration planning, but is not yet integrated with an IFC parser, a real rule-pack registry, identity verification, cryptographic artifact storage, migration executors, or a professional/municipal workflow.

The test rule pack uses example.invalid and synthetic digest strings. It is not a real South African or other jurisdiction pack.

## Run

From this directory, with a Rust 2024-compatible toolchain:

    cargo test --offline
    cargo clippy --all-targets -- -D warnings

No third-party Rust dependencies are required.

## Non-goals and legal boundary

A report produced by this crate is not legal advice or evidence of building-code compliance. In production, the scoped mandatory checks must be authored and reviewed against legally applicable sources, their version/effective dates must be verified, professional identities and authority must be validated, and the relevant architect, engineer, installer, or municipality remains responsible for approvals and certificates within its remit. A syntactically valid SHA-256 string is not proof that the referenced artifact actually hashes to that digest; artifact loading and digest recomputation belong to a future evidence-store adapter.
