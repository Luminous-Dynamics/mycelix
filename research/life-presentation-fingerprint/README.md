# LIFE-PRESENTATION-FINGERPRINT-001 — Structural leakage despite fresh proofs

Related work: [issue #5485](https://github.com/Luminous-Dynamics/mycelix/issues/5485) and [credential interoperability issue #5447](https://github.com/Luminous-Dynamics/mycelix/issues/5447).

## Question

If a holder creates fresh, uncorrelated proof bytes for each presentation, can two colluding verifiers still narrow the holder's candidate set through message counts, disclosed statement positions, or stable identifiers?

This dependency-free Rust 2024 program is a synthetic channel oracle. It does not implement BBS, SD-JWT, a credential issuer, or a verifier. It deliberately separates the effect of individual visible channels from cryptographic proof bytes.

The W3C [Data Integrity BBS Cryptosuites v1.0 Candidate Recommendation Draft](https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/) discusses residual correlation from total message count, revealed statement-index patterns, unique identifiers and proof metadata. This benchmark turns several of those observations into fixed regression cases; it is not a test of the W3C algorithms.

## Frozen synthetic model

- 512 synthetic subjects; no real or person-identifying information.
- Each has a generated statement count in [8, 31], an optional disclosed-index pattern drawn from 8 index positions, and a unique synthetic stable identifier.
- Two verifier observations represent the same task and underlying credential profile. Verifier-specific required indices are treated as public profile metadata; the tested index pattern is the optional index subset that remains comparable across presentations.
- The `fresh_proof_only` control intentionally models proof bytes as fresh and unmatchable, without claiming to generate or verify cryptographic proofs.
- Candidate sets are computed across all subjects using only the fields allowed by each observable profile. Ground-truth subject identity is used only to score the resulting candidate counts.

## Observable profiles

| Profile | Observable matching features | Purpose |
|---|---|---|
| `fresh_proof_only` | No stable cross-presentation features | Negative control: fresh bytes alone are not a match key |
| `exact_statement_count` | Exact statement count | Exposes count fingerprints |
| `bucketed_statement_count` | Count bucket (8–15, 16–23, 24–31) | Measures partial count coarsening |
| `fixed_padded_statement_count` | A single common padded count | Models removal of count as a distinguishing feature |
| `revealed_index_pattern` | Optional revealed-index mask | Isolates structural position fingerprinting |
| `fixed_padded_count_plus_revealed_indices` | Common padded count + optional index mask | Tests whether count padding alone removes index leakage |
| `exact_count_plus_revealed_indices` | Exact count + optional index mask | Combined structural signal |
| `stable_disclosed_identifier` | Unique stable identifier | Deliberately unsafe positive control |

The 8-position mask is a simplified, deterministic stand-in for index patterns. It is not a serialization of real BBS proof artifacts. Fixed padding sets the modeled visible count to the same common length for every synthetic subject. It removes count information from this particular channel but does not suppress independently modeled index or identifier fields.

## Metrics

The CSV reports raw subject denominator, unique-candidate count and ppm, subjects with at most ten candidates and ppm, candidate-set min/p05/p50/p95/max, and mean candidate count in milli-units. Raw counts accompany ppm so low-frequency events do not disappear through rounding.

No single privacy score is formed: candidate-set leakage from each observer channel remains separate. This benchmark does not estimate real-world anonymity or attack success rates.

## Run

```sh
cargo +1.96.0 fmt --manifest-path research/life-presentation-fingerprint/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-presentation-fingerprint/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-presentation-fingerprint/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-presentation-fingerprint/Cargo.toml
```

The dedicated exact-head workflow checks out the PR head by SHA, asserts the checked-out commit, pins Rust 1.96.0, runs formatting/tests/strict Clippy, compares two report outputs byte-for-byte, and retains the manifest, report digests and qualification receipt.

## Frozen acceptance criteria (not results)

- Proof-only control: every subject retains the full 512-person candidate set.
- Unique-identifier positive control: every subject has exactly one candidate.
- Fixed padding removes exact statement count as a matching feature.
- A profile with fixed count padding plus an index mask can still narrow the candidate set relative to the proof-only / fixed-count-only control.
- Coarsening count to buckets never narrows a candidate set compared with exact count alone.
- The report is rectangular, deterministic, and has explicit denominators.

## Industry comparison and claim boundary

The current CSV is a synthetic reference model only. It does not show that Mycelix outperforms SD-JWT or BBS, and it cannot establish cryptographic unlinkability. The follow-on in #5447 must use version-pinned official SD-JWT vectors and the W3C BBS test suite, actual issuer/holder/verifier flows, an independent verifier, and matched hardware before standards-conformance or performance claims are made.

No DP, anonymity, production-security, real-user risk, standards-conformance or SOTA-superiority claim is made.
