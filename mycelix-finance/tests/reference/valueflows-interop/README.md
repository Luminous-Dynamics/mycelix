# Mycelix / ValueFlows Read-Only Mapping Contract

**Status:** experimental reference fixture; not a production adapter and not an authoritative ledger implementation.

## Purpose

This corpus freezes candidate export mappings for SAP, TEND and MYCEL. It is deliberately isolated from production balance mutation paths. Its checker validates the local contract and registered negative controls; passing it does not establish full ValueFlows conformance, Holochain runtime compatibility, distributed settlement, or security. The checker, test expectations and corpus are all in the same PR-controlled trust domain, so this is not an independent verifier and cannot defend against a malicious change that weakens both code and fixtures.

## Run

From the repository root:

```sh
cargo fmt --manifest-path mycelix-finance/tools/valueflows-interop-checker/Cargo.toml -- --check
cargo test --manifest-path mycelix-finance/tools/valueflows-interop-checker/Cargo.toml
cargo run --manifest-path mycelix-finance/tools/valueflows-interop-checker/Cargo.toml
```

The dedicated GitHub Actions workflow runs the same checks when the contract, checker, or workflow changes. It also downloads `hrea.dna` and `hrea.happ` from the pinned `happ-0.5.0-beta.1` release URL and checks their bytes against the recorded SHA-256 digests; this verifies artifact identity only, not compatibility, safety, or license reuse approval. Third-party Actions are pinned by immutable commit SHA rather than mutable version tags.

For pull requests, the workflow explicitly checks out `github.event.pull_request.head.sha` rather than GitHub's synthetic merge ref. The reported result should therefore name the exact proposed source head. A queued/pending/skipped check is not a pass.

## Contract decisions

- **Export-only:** an imported external event never authorizes a Mycelix SAP/TEND/MYCEL balance mutation or sets settlement final.
- **SAP:** exact integer micro-unit quantity, canonical base-10 string and named unit profile. Do not pass authoritative quantity through binary floating point.
- **TEND:** model as a community-scoped mutual-credit position/commitment, not as a freely transferable bearer asset detached from its service obligation and counterparties. Current `TendExchange.hours` and cumulative hour fields are `f32` while balances are integer-valued. Lossless numeric export is therefore **blocked** until a versioned quantization/precision profile, error bound and round-trip tests are approved. Preserve TEND identity, unit meaning, counterparties, community scope and zero-sum obligation context; no implicit TEND-to-SAP conversion.
- **MYCEL:** export an attributable, scoped claim/attestation only. It is not a transferable currency or a spendable balance.
- **Identity and provenance:** source system, record ID, action/event ID, revision/sequence, scope, asset identity and claim ceiling are part of the envelope contract.
- **Corrections:** append a correction referencing the original event; never rewrite the original event's history.
- **Uncertainty:** stale, unavailable, unsupported or conflicting evidence stays indeterminate unless an explicit policy supplies a narrower disposition.
- **Version pins:** ValueFlows' status page describes the documentation/specification release as 1.0.0, while its generated RDF ontology metadata reports version info 0.17 and the version IRI `https://w3id.org/valueflows/ont/0.17/vf`. The pinned hREA `sprout` commit describes an implementation profile of “ValueFlows 1.0 + Holochain 0.7”, and release `happ-0.5.0-beta.1` claims ValueFlows 1.0 + Holochain 0.7.x. These are different versioning layers, **not by themselves evidence of incompatibility**; the actual vocabulary/schema and adapter tuple need qualification. The corpus records the hREA release commit and SHA-256 digests for both artifacts as evidence, not as a Mycelix dependency selection. The release/source refs remain distinct, and neither constitutes runtime qualification with Mycelix.

The checker currently declares 19 Rust unit-test functions. It exact-checks the nine-field v1 envelope, mapping policy strings, per-instrument ValueFlows entity sets, hREA source/release commits, DNA/hApp asset digests, license identity, and upgrade boundary. Negative tests exercise weakened policies, extra fields/entities, valid-shaped but wrong hashes, license-reuse promotion, and unsupported in-place migration claims. These are source-level contract checks, not a claim that the latest exact-head workflow has passed. Inspect the result and logs for the exact SHA before treating any contract as qualified.

## Verification trust boundary

This is a self-consistency/conformance check for accidental or reviewed changes, not an independent anti-tampering oracle. The checker source, workflow and corpus can all be changed in the same pull request. For high-assurance qualification, the required checker policy must be protected on the base branch or run from a separately pinned verifier whose identity is not controlled by the candidate change; bind results to the exact PR head SHA. Reviewers should not interpret the workflow's existence as independent security review.

## Non-goals

This is not JSON-LD serialization, a normative ValueFlows conformance suite, an hREA DNA adapter, a data migration tool, a balance reducer, or proof that any economic effect has settled. The next phase, if justified by this contract, is a typed exporter and round-trip fixture only. External imports remain non-authoritative until separate domain authorization and exact-head multi-agent qualification exist.
