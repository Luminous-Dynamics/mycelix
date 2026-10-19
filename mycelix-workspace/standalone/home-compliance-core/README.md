# Home Compliance Core (prototype)

A small deterministic Rust core for one narrow part of a future homeowner-controlled design system: deciding whether a scoped evidence package is ready for professional review.

This crate is intentionally separate from Holochain WASM zomes. It does not run devices, generate architecture, parse IFC, interpret statutes, calculate structural safety, or issue approvals.

## What it enforces

- Rule-pack metadata is present, its declared status is active, and its effective-date window includes the assessment date.
- Each declared mandatory check has exactly one finding.
- Findings bind to the exact rule-pack ID and edition used by the assessment.
- The assessment check list must exactly match the check manifest declared inside the supplied rule pack, including mandatory flags; omitted, unexpected, duplicate or flag-changed checks block readiness. This is a consistency guard, not proof that the rule pack is complete, authentic or legally authoritative.
- Missing findings, duplicate or unexpected findings, mandatory Fail/Unknown results, invalid evidence-digest syntax, and unjustified NotApplicable results block readiness.
- `evaluate_review_gate` performs metadata/scope checks only and can return `MetadataCompleteAwaitingEvidenceVerification`, never review-ready. `evaluate_verified_review_gate` promotes to `ReadyForProfessionalReview` only after a trusted `EvidenceVerifier` adapter confirms each referenced artifact exists and its actual bytes match the declared SHA-256; missing, mismatched, or unverifiable evidence blocks. The crate defines the adapter boundary but does not yet provide a production artifact-store implementation. Neither status means Compliant, Certified, Approved, or ReadyToBuild. The evolution module validates artifact revision lineage, verifies declared critical invariants before minting an opaque migration type accepted by the bounded planner, and computes deterministic downstream invalidation closure after a changed input. The `affected_cause_tree` API records a stable parent pointer for each impacted artifact so a reviewer can reconstruct the shortest change-to-output chain in linear space. The regulatory module distinguishes in-force law, adopted-but-not-yet-effective instruments, proposals, guidance, withdrawn sources and unknown status, and refuses to silently resolve conflicting normalized requirements unless an explicit jurisdiction-specific precedence policy uniquely settles them. The applicability evaluator now defaults to exact jurisdiction matching; descendant reach or an enumerated jurisdiction set must be explicitly configured by a reviewed jurisdiction profile. It never assumes that an ancestor-shaped ID alone makes a source apply to a city.

## Global jurisdiction registry and explicit legal reach

The `jurisdiction_registry` module requires an auditable capability card before a profile may advertise its declared `Supported` status. The three registry entries (Johannesburg, San Francisco, Berlin) are synthetic research-only profiles, not supported jurisdictions. Each explicitly classifies all 11 regulatory domains; every domain is currently marked `missing` until reviewed source modules and independent tests exist. Any future supported claim requires module/source references, source-registry and test-corpus digests, a nonempty independent test corpus, a configured freshness window, an update policy, an accountable maintainer, and two independent reviews of the exact card subject. Missing coverage, stale registry metadata, dissent/inconclusive review, or a mismatched snapshot blocks the supported-claim metadata gate.

“Metadata complete” is not legal certification: the prototype does not authenticate reviewers, recompute source bytes, verify legal authority, or prove that test cases are adequate. The card lets us scale country-by-country without pretending every module or country is implemented.

## Global rule composition and evidence-led reform

The standalone crate now includes a composable ruleset engine and a code-reform dossier contract:

- Global reference modules remain non-binding references; jurisdictional modules compose by explicit parent links, but geographic scope never silently creates legal precedence.
- Conflicting binding requirements remain explicit conflicts. The metadata-only composer rejects every override, even one with a plausible reviewer string and digest. `compose_verified_rule_modules` accepts a reviewed override only when an external `OverrideVerifier` adapter authenticates the decision, verifies evidence bytes, and confirms authority to override that exact inherited rule. The crate does not implement that production verifier.
- Owner preferences are kept distinct from law, guidance, engineering baselines, and proposals.
- Evidence-backed reform proposals are tied to a reform dossier that records supporting and conflicting evidence, methods, uncertainty, counter-evidence searches, affected-group impacts, conflicts of interest, and independent reviews pinned to a specific dossier snapshot.
- Preparing a dossier for public consultation is not a finding that science is conclusive and does not amend a live building code. A separate consultation closeout records the public notice, feedback archive, response record, and dates. An adoption-decision record must cite the exact dossier snapshot and consultation-closeout digest, the authority and its decision basis, the public decision source, disposition, and effective date. These are metadata-consistency checks only; they do not authenticate the authority or change active rules automatically.

See [Global Building-Code Composition and Evidence Reform](docs/GLOBAL_BUILDING_CODE_COMPOSITION_AND_REFORM.md) for the design, workflow, and acceptance targets. The types are prototypes; they are not a globally authoritative code database or a legal interpreter.

## Global composition and evidence-backed reform

The crate now includes a prototype composable ruleset engine in `src/composition.rs` and a reform dossier workflow in `src/reform.rs`. Global reference modules do not become law by being imported; jurisdiction module ancestry is validated without inferring precedence. Conflicting applicable binding statements remain explicit conflicts until a reviewed override record names the inherited statement and preserves its provenance. Owner preferences and non-binding baselines/proposals remain separate.

Code-reform dossiers record methods, evidence that supports or challenges a proposal, limitations, counter-evidence searches, uncertainty, affected-group impacts, conflicts of interest, and independent reviews pinned to a specific snapshot. A dossier can be prepared for public consultation, but this does not mean the science is conclusive or that a code has changed. A legally authorized authority must still adopt any amendment.

See [Global Building-Code Composition and Evidence Reform](docs/GLOBAL_BUILDING_CODE_COMPOSITION_AND_REFORM.md) for the global architecture, design-freedom principles, public reform lifecycle, and acceptance benchmarks. The `jurisdiction-source-watch-za-johannesburg-2026-v1.json` fixture also indexes four official South African Government source locators: the existing Act publication, the 2026 Amendment Bill consultation notice, its deadline-extension notice, and the Cabinet statement describing publication for consultation. These are locator-only records: source bytes remain unverified, SHA-256 fields are blank, legal applicability is unevaluated, and no binding rule module is emitted. The CI verifier is intended to keep those boundaries fail-closed. The implementation is prototype metadata and workflow logic—not an authoritative global code database or legal interpreter.

## Status

Early prototype and design slice. The dedicated GitHub Actions Home Compliance Core workflow passed on exact head `16e7c5e22c9dd684d9dabd73cee2dea727fa9a3a`: 113 unit tests, 8 integration tests, fixture/schema validation with 24 negative controls, Rust formatting, offline benchmark compilation, and strict Clippy. The benchmark harness was compiled but not timed. Rust tooling is unavailable in the authoring environment, so these results come from the linked hosted run, not a local execution. The crate is not yet integrated with a real IFC parser, cryptographically authenticated live source registry, production artifact storage, trusted semantic extraction, migration executor, or a professional/municipal workflow.

The test rule pack uses example.invalid and synthetic digest strings. It is not a real South African or other jurisdiction pack.

## Reference home scenario

The file `fixtures/johannesburg-adaptable-home-v1.json` defines a technology-neutral, synthetic reference brief: several style families, several construction families, stable artifact identifiers, and change scenarios for an adaptable single-storey home. It is **not** a real site, municipal rule pack, architectural plan, IFC model, engineering calculation, or compliance claim. In particular, the fixture marks stabilized-earth construction as requiring additional local validation rather than treating every construction family as automatically admissible.

The Rust integration tests in `tests/home_evolution_scenario.rs` exercise eight lifecycle properties:

- a changed energy-rule revision invalidates its dependent finding and submission package without unnecessarily invalidating unrelated structural evidence or drawings;
- a floor-plan change invalidates downstream structure, energy, drawing, export, assessment and submission artifacts, but not the source rule pack;
- simultaneous floor-plan and energy-rule revisions produce the union of downstream invalidations once, with a deterministic shortest cause for shared outputs;
- a complete synthetic evidence package reaches only `ReadyForProfessionalReview`, never a compliance or approval status;
- an unknown mandatory energy result blocks review readiness;
- a proposed rule cannot silently replace an applicable current-rule assertion;
- the key identifiers used by the tests are checked against the machine-readable brief; and
- a 10,000-edge dependency chain returns the complete, deterministic closure. This is a scale/correctness test, not yet a latency benchmark.

The Rust integration tests read dependency-edge pairs from the shared JSON fixture instead of maintaining a second hand-coded graph. A standard-library-only CI verifier checks the fixture against the supported keyword subset of the published JSON Schema, validates that mutation expectations agree with dependency reachability, and runs fourteen negative controls, including unsupported or malformed schema keywords, ineffective array constraints, cyclic graphs, incomplete multi-root invalidation, and ambiguous or duplicate changed-root declarations. This verifier is a test tool, not a generic JSON Schema implementation or a production runtime dependency. The production crate still does not load the fixture or generate building geometry. The metadata-only review gate cannot report readiness; the verified gate's sole positive status remains `ReadyForProfessionalReview`, not compliance, certification, approval, or permission to build.

## Run

From this directory, with a Rust 2024-compatible toolchain:

    cargo test --offline
    cargo clippy --all-targets -- -D warnings
    cargo bench --offline --bench dependency_impact

The dependency-impact benchmark measures sorted closure and compact cause-tree construction for chain, binary-tree, and fan-out graphs at 1,000, 10,000, and 100,000 edges, with five samples and median/min/max timings. It checks the output count, ordering, and selected cause link. Results depend on hardware and toolchain; record both when comparing runs. CI compiles the benchmark but intentionally does not enforce a timing threshold. No third-party Rust dependencies are required.

## Non-goals and legal boundary

A report produced by this crate is not legal advice or evidence of building-code compliance. In production, the scoped mandatory checks must be authored and reviewed against legally applicable sources, their version/effective dates must be verified, professional identities and authority must be validated, and the relevant architect, engineer, installer, or municipality remains responsible for approvals and certificates within its remit. A syntactically valid SHA-256 string is not proof that the referenced artifact actually hashes to that digest. The crate now requires an `EvidenceVerifier` adapter for review-ready status, but a production adapter that securely loads immutable artifact bytes and recomputes their SHA-256 has not yet been implemented.
