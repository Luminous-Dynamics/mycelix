# Evidence Plane RFC v0

Status: research RFC and design proposal. This does not claim a production ingestion service, artifact store, verifier, signer, dashboard or cross-repository adapter exists.

## Thesis

Make evidence—not confidence or presentation—the unit that moves through the system. Every result should link to inputs, transforms, execution conditions, checks, limitations and a maximum claim ceiling. A polished result must never outrun its evidence.

Local-first and Rust-first: Git stores small schemas, manifests, protocols, code and synthetic fixtures. Large or restricted data belongs in a content-addressed store with explicit access and retention policy, not Git by default. Verification should work offline after permitted source acquisition.

## Architecture

1. Source registry: canonical URL/DOI, publisher, source version, license/terms, attribution, steward-contact requirements, temporal/spatial coverage, units and unresolved metadata conflicts.
2. Explicit acquisition: separate network-enabled command saves to cache outside Git; records retrieval time, media type, byte count and SHA-256; refuses silent overwrite; rejects empty/unexpected content. Acknowledging a contact request is not proof contact occurred.
3. Content-addressed storage: immutable bytes keyed by digest with classification, ACL, retention and provenance. A digest establishes byte identity, not truth, authenticity, legality or completeness.
4. Typed normalization: named/versioned transforms record input hashes, code commit, toolchain, configuration digest, units, CRS, time basis, missing-data policy and output digest. No implicit unit/CRS conversions.
5. Provenance DAG: source, transform, intermediate dataset, model, run, result and publication are linked. Missing parents, changed inputs, cycles and unpinned tools fail closed.
6. Benchmark registry: lock primary endpoints, data versions, site/time holdouts, baselines, tuning budget, hardware, stop/exclusion rules, uncertainty method and thresholds before results are inspected.
7. Matched-budget executor: record code/model/data digests, backend/precision, seed policy, environment, resource budgets, retries and gate outcomes. Queued CI is not a passing run.
8. Independent verifier: a small Rust CLI validates schema, computes digests, checks exact commit identity, replays permitted transforms and runs mutation controls. Schema validation alone cannot verify hashes or scientific truth.
9. Consumers: Symthaea for scientific/model evaluation; Symtropy for simulation and resilience; Sol Atlas for spatiotemporal presentation; Mycelix may anchor receipt hashes later. Offline verification must not depend on the DHT.

## Industry benchmark alignment

See [INDUSTRY_BENCHMARKS.md](INDUSTRY_BENCHMARKS.md) for research-backed scorecards connecting Antarctic controller testing to IEEE 2030.8/NREL-NLR practice, and mapping shared benchmark principles to Sol Atlas and Symthaea.

## Polar infrastructure, habits and resource evidence

See [POLAR_SYSTEMS_RESEARCH.md](POLAR_SYSTEMS_RESEARCH.md) for a cross-project research plan covering location-specific Arctic/Antarctic design envelopes, infrastructure stress tests, voluntary polar living routines, and an evidence-audited Arctic hydrocarbon resource/trade-off atlas.

## Evidence maturity ladder

| Tier | Minimum evidence | Maximum generic claim |
|---|---|---|
| S0 | Synthetic or planned fixture, explicitly labelled | synthetic_only |
| S1 | Citation-level transcription; source bytes not end-to-end verified | transcription_only |
| S2 | Source bytes/artifact hashed; stewardship, parser and lineage recorded | source_reproducible |
| S3 | Independent replay with locked inputs/protocol and effective negative controls | independently_reproduced |
| S4 | Independent, domain-appropriate external, field or operational validation | domain_validated |

These are evidence classes, not a universal measure of truth or risk. Stronger claims only apply to the declared scope, environment and boundary conditions; the ladder does not certify a medical device, infrastructure design, reserve estimate or safety-critical controller.

## Non-negotiable rules

- Unknown, missing, contradictory or stale material facts stay unknown; no silent defaults that improve a score.
- Synthetic fixtures test software behavior, not physical performance.
- A URL is not evidence that data bytes were downloaded. A hash is not proof of correctness or permission.
- Annual programme aggregates cannot be assigned to station points unless a source supports that allocation.
- Model output is not measurement; forecast is not observation; resource assessment is not discovery, reserve, commercial viability or development permission.
- Queued, pending, skipped, missing or empty CI status is not pass. Report checks actually executed at the exact commit.
- Unmatched information, compute, precision, tuning or failure-handling cannot support a fair SOTA claim.
- Safety/service hard gates cannot be traded against a better aggregate score.
- Public receipts must not expose secrets, personal data, precise private locations or restricted content. Hashes may leak equality for guessable inputs.

## Standards to reuse

- [W3C PROV-O](https://www.w3.org/TR/prov-o/) offers portable Entity/Activity/Agent provenance relations; adapt to it rather than requiring RDF in every local workflow.
- [RO-Crate](https://www.researchobject.org/ro-crate/specification.html) is a research-object packaging/metadata pattern; evaluate a profile for datasets and runs.
- [SLSA provenance](https://slsa.dev/spec/v1.0/provenance) and [in-toto](https://in-toto.io/) inform software build attestations; build provenance is not scientific validation.
- [MLCommons/MLPerf](https://mlcommons.org/benchmarks/) are precedents for fair, reproducible and affordable comparisons; domain metrics still need local definitions.

## Stages and acceptance targets

V0 contract: schema, examples, threat model, benchmark protocol and integration map. The initial validator checks structure only; artifact verification is a separate explicit mode. Require a real Draft 2020-12 validator, a passing positive fixture, at least ten expected failing single-field mutations, and review of claim ceilings.

V1 Rust CLI: receipt validation, hash checking, lineage checks and mutation suite; pinned dependencies and offline normal mode. Test wrong hash/commit, missing parent, path traversal, units/CRS drift, forbidden reuse, missing gates, result tampering and synthetic-to-physical escalation.

V2 acquisition/store: content-addressed cache, quarantine, atomic writes, safe permissions, size limits, archive traversal/bomb defenses, terms checks, retention/deletion receipts and backup/restore tests.

V3 independent execution: separate candidate from evaluator; frozen protocol, site/time holdouts, matched budgets, raw trials, uncertainty and failed-trial reporting.

V4 signing and adapters: signing/key revocation, privacy, offline verification and recovery; consider Mycelix anchoring only after review. Ledger consensus is not scientific truth.

Proposed targets, not achieved results: 100% quantitative claims link to receipts/inputs or are marked unverified; zero silent source-conflict erasure; 100% rejection of known material mutations; two independent implementations agree on receipt hashes before signing; zero safety-critical regressions accepted in exchange for aggregate-score gains; P95 validation below 1 second for small manifests (excluding large-file hashing), measured on published hardware; all benchmark reports disclose baselines, budgets, splits, uncertainty and failures.
