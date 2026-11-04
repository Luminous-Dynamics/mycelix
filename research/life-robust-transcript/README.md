# LIFE-ROBUST-TRANSCRIPT-001 — Compile against population shift

Related work: [issue #5491](https://github.com/Luminous-Dynamics/mycelix/issues/5491), [transcript compiler #5490](https://github.com/Luminous-Dynamics/mycelix/pull/5490), and [robust query-guard issue #5459](https://github.com/Luminous-Dynamics/mycelix/issues/5459).

## Objective

Test whether the lowest-cost transcript plan selected against one reference population remains privacy-policy feasible under a deliberately shifted auxiliary population. The robust compiler optimizes the worst p05 candidate support across every admitted scenario, not the reference-only result.

This is a finite synthetic adversarial evaluation. It is not a guarantee that its two scenarios contain the true population.

## Frozen scenarios

Both scenarios contain the same 512 task transcripts, generated using the schedule [predicate_0, predicate_1, predicate_2, predicate_0]. This yields eight balanced observable answer transcripts with 64 subjects each.

- Reference scenario: independently generated statement-count and three-of-eight index patterns for each round.
- Shifted scenario: preserves the answer transcripts, but within every 64-subject task cohort it assigns 12 subjects to the lower count bucket (8–19) and coarse-index parity class 0, while 52 subjects fall in the upper count bucket (20–31) and class 1. Exact index masks vary within each parity class.

The shift is intentionally adversarial and explicitly constructed. It is a counterexample to generalizing a reference-cohort pass, not a representative demographic model.

## Complete plan search

Four rounds, nine profiles per round, and 6,561 complete plans. Every candidate set is derived from the intersection of the four observer-visible views. The compiler only accepts a plan if p05 candidate support is at least 16 in **both** scenarios.

Counts: exact, two-bucket, or common padding. Index patterns: exact, balanced one-bit parity class, or omitted. Abstract costs reproduce the single-population transcript benchmark and are only synthetic ranking units.

## Independent preflight expectation

A separate Python implementation of the frozen generator and signature-grouping logic found:

- The single-population optimum from #5490 has reference p05=27 but shifted p05=12, leaving 96/512 shifted subjects below k=16.
- Exactly one of the 6,561 plans satisfies p05 >=16 in both scenarios: padding counts and omitting index metadata in all four rounds.
- This robust plan has 144 abstract cost units and p05=64 in both scenarios, because only the required eight-way task transcript remains visible.

These values are known-answer targets, not Rust test results or privacy guarantees. The dedicated Rust workflow must pass on the exact head before qualification.

## Run

~~~sh
cargo +1.96.0 fmt --manifest-path research/life-robust-transcript/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-robust-transcript/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-robust-transcript/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-robust-transcript/Cargo.toml > report.csv
~~~

The exact-head workflow pins Rust 1.96 and action SHAs, tests formatting / unit invariants / strict Clippy, byte-compares two reports, and retains SHA-256 evidence.

## Industry path and claim ceiling

The BBS Cryptosuites v1.0 document is a W3C Candidate Recommendation Draft, not a final Recommendation, and discusses residual structural correlations: https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/

OpenID4VP and OpenID4VCI + HAIP 1.0 have official conformance suites; those suites are the protocol-correctness gate before an actual implementation's outputs enter this benchmark: https://openid.net/openid4vp-and-openid4vci-conformance-tests-are-complete-and-open-for-self-certification/

The current harness does not implement credential cryptography or demonstrate standards conformance. It does not prove anonymity, unlinkability, differential privacy, universal robust privacy or SOTA superiority. It excludes out-of-family populations, timing/network channels, external resolution, real attacker adaptation and production costs.
