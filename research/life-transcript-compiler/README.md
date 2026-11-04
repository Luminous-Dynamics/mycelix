# LIFE-TRANSCRIPT-COMPILER-001 — Compile privacy across a transcript

Related work: [issue #5489](https://github.com/Luminous-Dynamics/mycelix/issues/5489), [single-presentation privacy compiler #5487 / PR #5488](https://github.com/Luminous-Dynamics/mycelix/pull/5488), [structural leakage oracle #5485 / PR #5486](https://github.com/Luminous-Dynamics/mycelix/pull/5486), and [credential interoperability #5447](https://github.com/Luminous-Dynamics/mycelix/issues/5447).

## Question

Can the compiler choose presentation options across multiple rounds so that required task answers stay identical while the attacker's *final transcript-level candidate set* remains above a declared floor?

This dependency-free Rust program is a synthetic exact-search oracle. It neither implements credentials nor proves a real-world anonymity property.

## Paradigm shift

A privacy-preserving single presentation does not imply a privacy-preserving sequence. An observer can intersect candidate sets across sessions and use a sequence of otherwise modest side channels to narrow the subject. This benchmark searches a complete transcript and applies one global candidate-support constraint. It does not score rounds independently and add the scores.

The W3C BBS Cryptosuites v1.0 Candidate Recommendation Draft describes structural correlation from disclosed index patterns and document structure; it is a work-in-progress specification, not a final Recommendation: https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/

RFC 9901 SD-JWT defines selectively disclosable claims using salted digests and requires independently generated high-entropy salts for each disclosure. The synthetic observer channels here are metadata-level controls and do not test the RFC cryptography: https://datatracker.ietf.org/doc/rfc9901/

## Frozen synthetic model

- 512 subjects, four rounds, and eight balanced task-answer transcripts (64 subjects per transcript).
- Required predicates follow [predicate_0, predicate_1, predicate_2, predicate_0]. Each plan emits exactly the same four answers for every subject.
- Each subject receives an independently generated statement count (8–31) and three-of-eight optional-index mask in each round.
- Each round chooses one of 9 options: exact / two-bucket / common padded count, crossed with exact / balanced coarse parity / omitted index pattern.
- The complete plan space is 9^4 = 6,561. Each plan is evaluated against the intersection of all four rounds' observer-visible fields.
- The coarse index field is the parity of how many of the selected indices fall in the lower four positions. Among three-of-eight subsets this partitions the 56 patterns into two balanced classes.
- stable_identifier is retained only for a deliberately unsafe positive control; it is not a selectable plan.

## Objective and costs

The global policy requires p05 transcript-level candidate support >=16 out of 512. The compiler minimizes the sum of four-round abstract costs, then maximizes p05 support at equal cost, then applies a lexical plan-label tie-break.

Costs are synthetic ranking units only. They are not byte counts, latency, CPU time, memory, or measured implementation overhead. The selected plan reports candidate support from the final transcript, not per-round means.

## Run

~~~sh
cargo +1.96.0 fmt --manifest-path research/life-transcript-compiler/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-transcript-compiler/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-transcript-compiler/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-transcript-compiler/Cargo.toml > report.csv
~~~

The dedicated workflow pins Rust 1.96.0 and action revisions, asserts the exact subject commit, runs tests/format/strict Clippy, compares report bytes across two executions, and uploads the full 6,561-plan report with a SHA-256-bound receipt.

## Golden expectations (not qualification claims)

For seed 0x5EEDC0DE, an independent enumeration predicts a minimum feasible total cost of 128 abstract units and a best p05 support of 27. The selected plan is expected to use padded counts in rounds 1, 2 and 4; a two-bucket count in round 3; and omit index patterns in all rounds. The Rust test freezes this outcome.

These are finite-fixture regression expectations, not a risk estimate, protocol result, or production cost.

## Next protocol-backed step

Use pinned SD-JWT and BBS-capable implementations, capture actual presentations, and map real observer-visible fields into the transcript evaluator. Gate performance/privacy comparisons on official vectors and independent verification. OpenID4VP/OpenID4VCI + HAIP 1.0 has an official conformance program: https://openid.net/openid4vp-and-openid4vci-conformance-tests-are-complete-and-open-for-self-certification/

## Claim ceiling

This oracle excludes network/timing channels, external resolution, issuer/verifier collusion beyond modeled fields, out-of-family population shifts, cryptographic flaws, and real proof-artifact transformations. It is not differential privacy, anonymity, unlinkability, standards conformance, a production compiler, or evidence of SOTA superiority.
