# Home Compliance Core (prototype)

A small deterministic Rust core for one narrow part of a future homeowner-controlled design system: deciding whether a scoped evidence package is ready for professional review.

This crate is intentionally separate from Holochain WASM zomes. It does not run devices, generate architecture, parse IFC, interpret statutes, calculate structural safety, or issue approvals.

## What it enforces

- Rule-pack metadata is present, its declared status is active, and its effective-date window includes the assessment date.
- Each declared mandatory check has exactly one finding.
- Findings bind to the exact rule-pack ID and edition used by the assessment.
- The assessment check list must exactly match the check manifest declared inside the supplied rule pack, including mandatory flags; omitted, unexpected, duplicate or flag-changed checks block readiness. This is a consistency guard, not proof that the rule pack is complete, authentic or legally authoritative.
- Missing findings, duplicate or unexpected findings, mandatory Fail/Unknown results, invalid evidence-digest syntax, and unjustified NotApplicable results block readiness.
- `evaluate_review_gate` performs metadata/scope checks only and can return `MetadataCompleteAwaitingEvidenceVerification`, never review-ready. `evaluate_verified_review_gate` promotes to `ReadyForProfessionalReview` only after a trusted `EvidenceVerifier` adapter confirms each referenced artifact exists and its actual bytes match the declared SHA-256; missing, mismatched, or unverifiable evidence blocks. The crate defines the adapter boundary but does not yet provide a production artifact-store implementation. Neither status means Compliant, Certified, Approved, or ReadyToBuild. The evolution module validates artifact revision lineage, verifies declared critical invariants before minting an opaque migration type accepted by the bounded planner, and computes deterministic downstream invalidation closure after a changed input. The `affected_cause_tree` API records a stable parent pointer for each impacted artifact so a reviewer can reconstruct the shortest change-to-output chain in linear space. The regulatory module distinguishes in-force law, adopted-but-not-yet-effective instruments, proposals, guidance, withdrawn sources and unknown status, and refuses to silently resolve conflicting normalized requirements unless an explicit jurisdiction-specific precedence policy uniquely settles them.

## Status

Early prototype and design slice. Unit tests are included but were not run in the authoring environment because Rust tooling is unavailable there. The crate includes revision-lineage, invariant-gated migration planning, dependency invalidation and time-aware regulatory-source modules, but is not yet integrated with an IFC parser, a live source registry, identity verification, cryptographic artifact storage, trusted semantic extraction, migration executors, or a professional/municipal workflow.

The test rule pack uses example.invalid and synthetic digest strings. It is not a real South African or other jurisdiction pack.

## Reference home scenario

The file `fixtures/johannesburg-adaptable-home-v1.json` defines a technology-neutral, synthetic reference brief: several style families, several construction families, stable artifact identifiers, and change scenarios for an adaptable single-storey home. It is **not** a real site, municipal rule pack, architectural plan, IFC model, engineering calculation, or compliance claim. In particular, the fixture marks stabilized-earth construction as requiring additional local validation rather than treating every construction family as automatically admissible.

The Rust integration tests in `tests/home_evolution_scenario.rs` exercise seven lifecycle properties:

- a changed energy-rule revision invalidates its dependent finding and submission package without unnecessarily invalidating unrelated structural evidence or drawings;
- a floor-plan change invalidates downstream structure, energy, drawing, export, assessment and submission artifacts, but not the source rule pack;
- a complete synthetic evidence package reaches only `ReadyForProfessionalReview`, never a compliance or approval status;
- an unknown mandatory energy result blocks review readiness;
- a proposed rule cannot silently replace an applicable current-rule assertion;
- the key identifiers used by the tests are checked against the machine-readable brief; and
- a 10,000-edge dependency chain returns the complete, deterministic closure. This is a scale/correctness test, not yet a latency benchmark.

The Rust integration tests read dependency-edge pairs from the shared JSON fixture instead of maintaining a second hand-coded graph. A standard-library-only CI verifier checks the fixture against the supported keyword subset of the published JSON Schema, validates that mutation expectations agree with dependency reachability, and runs seven negative controls. This verifier is a test tool, not a generic JSON Schema implementation or a production runtime dependency. The production crate still does not load the fixture or generate building geometry. The metadata-only review gate cannot report readiness; the verified gate's sole positive status remains `ReadyForProfessionalReview`, not compliance, certification, approval, or permission to build.

## Run

From this directory, with a Rust 2024-compatible toolchain:

    cargo test --offline
    cargo clippy --all-targets -- -D warnings
    cargo bench --offline --bench dependency_impact

The dependency-impact benchmark measures sorted closure and compact cause-tree construction for 1,000, 10,000 and 100,000-edge chains. It checks the produced result counts and final cause link before printing timings. Results depend on hardware and toolchain; record both when comparing runs. CI compiles the benchmark but intentionally does not enforce a timing threshold. No third-party Rust dependencies are required.

## Non-goals and legal boundary

A report produced by this crate is not legal advice or evidence of building-code compliance. In production, the scoped mandatory checks must be authored and reviewed against legally applicable sources, their version/effective dates must be verified, professional identities and authority must be validated, and the relevant architect, engineer, installer, or municipality remains responsible for approvals and certificates within its remit. A syntactically valid SHA-256 string is not proof that the referenced artifact actually hashes to that digest. The crate now requires an `EvidenceVerifier` adapter for review-ready status, but a production adapter that securely loads immutable artifact bytes and recomputes their SHA-256 has not yet been implemented.
