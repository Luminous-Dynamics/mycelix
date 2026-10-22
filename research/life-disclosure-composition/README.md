# Life disclosure composition benchmark

This dependency-free Rust reference benchmark complements [LIFE-DISCLOSURE-LINKABILITY-001 (#5401)](https://github.com/Luminous-Dynamics/mycelix/issues/5401) and its pairwise channel harness in research/life-disclosure-linkability/.

## Research question

What happens when an observer composes several individually purpose-bounded disclosures about the same subject?

The fixture contains 4,096 synthetic subjects, each with six independent four-valued categorical attributes. Each of six hypothetical purposes asks for one Boolean predicate. The benchmark compares:

- exact_attribute: reveal the full categorical value, from which the required predicate can be evaluated;
- predicate_only: reveal only the Boolean result needed for the purpose.

Both profiles must return identical answers to all six required task predicates. Candidate sets are calculated only from observable responses over the synthetic population. Subject indices and truth labels are not emitted.

## Known-answer result for this constructed universe

| Cumulative disclosures | Exact attribute candidate set | Predicate-only candidate set |
|---:|---:|---:|
| 1 | 1,024 | 2,048 |
| 2 | 256 | 1,024 |
| 3 | 64 | 512 |
| 4 | 16 | 256 |
| 5 | 4 | 128 |
| 6 | 1 | 64 |

These are mathematical properties of this constructed fixture, not empirical privacy estimates, a representative population model, or a guarantee that predicate-only disclosure is anonymous. The toy predicates split every field into two categories; real claims can be correlated or intrinsically identifying.

The structural result is that task-equivalent answers can reveal different amounts of auxiliary information. Across six disclosures, exposing exact attribute values uniquely identifies each record in the fixture, while the six required Boolean answers leave 64 compatible records.

## Composition assumption

This is an upper-bound composition experiment conditional on the adversary being able to associate the sequence with one subject. Association might arise from a separate side channel or collusion; this program does not prove that pairwise presentations can be linked. Cross-recipient identifier, proof and metadata matching is the distinct concern in the companion linkability benchmark.

This harness is not a trained adaptive attacker, browser/network timing test, cryptographic implementation, differential-privacy mechanism, or independent privacy audit.

## Run

```sh
cargo +1.96.0 fmt --manifest-path research/life-disclosure-composition/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-disclosure-composition/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-disclosure-composition/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-disclosure-composition/Cargo.toml
```

The executable emits a deterministic CSV report with min/p50/p90/p95/max candidate counts, unique-candidate rate, share of targets with at most ten candidates, candidate-set retention per step and a task-utility correctness guard. The printed FNV-1a fingerprint is non-cryptographic; CI separately computes SHA-256 over the report and qualification receipt.

## Next experiments

1. Extend the corpus with correlated attributes and skewed prevalence, freezing the generator and independent known-answer cases first.
2. Add composition with missing, unavailable, stale and conflicting source states; missing evidence must not silently become a negative predicate.
3. Add an independently implemented verifier that reconstructs candidate sets from public synthetic observations only.
4. Test threshold disclosure, randomized response and privacy-budget accounting only with a precisely specified mechanism and utility trade-off. Do not claim differential privacy from this categorical benchmark.
5. Add multi-observer sequence association, auxiliary public information and calibrated probabilistic attackers after this reference oracle qualifies.

## Standards context

The W3C Verifiable Credentials Data Model 2.0 Recommendation discusses identifier-, signature-, metadata- and validation-based correlation: https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/

The W3C Data Integrity BBS Cryptosuites v1.0 is a Candidate Recommendation Draft defining selective disclosure and unlinkable derived proofs; that primitive does not by itself prevent correlation through identifying claims or surrounding metadata: https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/

## Claim ceiling

A successful exact-head workflow establishes only that this deterministic synthetic candidate-set model, utility-equivalence invariant and output are reproducible for the bound commit. It establishes no anonymity guarantee, cryptographic unlinkability, differential privacy, production performance, legal compliance, user usability or superiority over existing solutions.
