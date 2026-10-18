# Home Compliance Core (prototype)

A small deterministic Rust core for one narrow part of a future homeowner-controlled design system: deciding whether a scoped evidence package is ready for professional review.

This crate is intentionally separate from Holochain WASM zomes. It does not run devices, generate architecture, parse IFC, interpret statutes, calculate structural safety, or issue approvals.

## What it enforces

- Rule-pack metadata is present, its declared status is active, and its effective-date window includes the assessment date.
- Each declared mandatory check has exactly one finding.
- Findings bind to the exact rule-pack ID and edition used by the assessment.
- Missing findings, duplicate or unexpected findings, mandatory Fail/Unknown results, invalid evidence-digest syntax, and unjustified NotApplicable results block readiness.
- The only positive status is ReadyForProfessionalReview; there is no Compliant, Certified, Approved, or ReadyToBuild enum. The evolution module validates artifact revision lineage, verifies declared critical invariants before minting an opaque migration type accepted by the bounded planner, and computes deterministic downstream invalidation closure after a changed input. The regulatory module distinguishes in-force law, adopted-but-not-yet-effective instruments, proposals, guidance, withdrawn sources and unknown status, and refuses to silently resolve conflicting normalized requirements unless an explicit jurisdiction-specific precedence policy uniquely settles them.

## Status

Early prototype and design slice. Unit tests are included but were not run in the authoring environment because Rust tooling is unavailable there. The crate includes revision-lineage, invariant-gated migration planning, dependency invalidation and time-aware regulatory-source modules, but is not yet integrated with an IFC parser, a live source registry, identity verification, cryptographic artifact storage, trusted semantic extraction, migration executors, or a professional/municipal workflow.

The test rule pack uses example.invalid and synthetic digest strings. It is not a real South African or other jurisdiction pack.

## Reference home scenario

The file `fixtures/johannesburg-adaptable-home-v1.json` defines a technology-neutral, synthetic reference brief: several style families, several construction families, stable artifact identifiers, and change scenarios for an adaptable single-storey home. It is **not** a real site, municipal rule pack, architectural plan, IFC model, engineering calculation, or compliance claim. In particular, the fixture marks stabilized-earth construction as requiring additional local validation rather than treating every construction family as automatically admissible.

The Rust integration tests in `tests/home_evolution_scenario.rs` exercise seven lifecycle properties:

- a changed energy-rule revision invalidates the energy finding and submission package without invalidating unrelated structural and drawing outputs;
- a floor-plan change invalidates its downstream energy, structure, drawing, export and submission artifacts;
- missing mandatory evidence blocks readiness for professional review;
- a proposed rule cannot silently replace an applicable current-rule assertion; and
- key fixture identities are checked against the machine-readable brief; and
- a 10,000-edge dependency chain returns the complete deterministic closure. This is a scale/correctness test, not yet a latency benchmark.

The core currently mirrors the relevant dependency edges in Rust and checks identity fragments in the JSON fixture; it does **not** parse the entire JSON document. Keep this limitation explicit until a proper, versioned schema validator is added. The only positive outcome remains `ReadyForProfessionalReview`, not compliance, certification, approval, or permission to build.

## Run

From this directory, with a Rust 2024-compatible toolchain:

    cargo test --offline
    cargo clippy --all-targets -- -D warnings

No third-party Rust dependencies are required.

## Non-goals and legal boundary

A report produced by this crate is not legal advice or evidence of building-code compliance. In production, the scoped mandatory checks must be authored and reviewed against legally applicable sources, their version/effective dates must be verified, professional identities and authority must be validated, and the relevant architect, engineer, installer, or municipality remains responsible for approvals and certificates within its remit. A syntactically valid SHA-256 string is not proof that the referenced artifact actually hashes to that digest; artifact loading and digest recomputation belong to a future evidence-store adapter.
