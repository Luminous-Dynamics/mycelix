# LIFE-RELATIONAL-001 — Cross-subject privacy spillover

## Question

Can an observer infer a target's protected attribute from other subjects' public proxy disclosures when the target's protected value and direct protected-claim fields are withheld?

This is a synthetic red-team benchmark. It does not implement a credential protocol, a privacy mechanism, or a calibrated risk model for real people.

## Paradigm

The unit of analysis is a target inside a relationship structure, not an isolated presentation. Each synthetic cohort has a latent binary property. A subject's protected label and public peer proxy are independently correlated with that latent property at a frozen probability. The target's own row is excluded from the peer evidence for each attack. Exact-cohort membership or coarse-bucket membership is deliberately available as side information in the corresponding attacker arm; this relationship metadata is not counted as disclosure of the target's protected value.

This deliberately tests a privacy externality: a target may reveal nothing and still be inferable because peers disclosed correlated data. It is not enough to assert that each person's own disclosure is minimized.

## Attack profiles

- population_prior: predict using a majority label learned from separate training cohorts; inspect no target-linked peers.
- exact_cohort_peer_majority: use only the public proxy observations of other subjects in the target's exact synthetic cohort.
- coarse_neighborhood_peer_majority: use proxy observations from a coarse bucket that merges four latent cohorts, hiding exact cohort identity but retaining neighborhood linkage.
- randomized_peer_negative_control: use randomly assigned peer proxy labels; the attacker should fall back toward its prior.
- uncorrelated_target_negative_control: score an independent target secret against cohort-correlated peer proxies; the peers should not improve inference materially.

Training and test cohort identifiers occupy disjoint ranges. The report uses a deterministic xorshift64* generator with a fixed seed derivation and no third-party dependencies. The experiment sweeps 4, 8, 16 and 32 subjects per cohort and peer-signal probabilities from 0.55 to 0.95.

## Metrics

Accuracy and balanced accuracy are reported in parts per million (ppm), along with prior accuracy and lift above the prior. The report also records the number of peer observations consumed, signed peer-observation delta relative to exact-cohort access (positive means more peer records exposed; negative means fewer), and direct protected-attribute/claim disclosures.

Evidence access and inference accuracy remain separate. The signed peer-observation delta is relative to exposing the exact cohort (positive means more peer records are visible, negative means fewer); it is not a user-utility metric, calibrated privacy loss, or a monotone guarantee that coarsening improves privacy. A real utility / fairness evaluation requires a separate task-specific benchmark.

A post-hoc audit recomputes target count, correct predictions and protected-claim disclosure count from per-target outcome records rather than trusting the evaluator's summary counters. Both values appear in the CSV for comparison. The benchmark also emits selective risk–coverage rows at fixed confidence thresholds of 0.50, 0.625, 0.75, 0.875 and 0.95. Each row reports coverage, accuracy among non-abstained targets and abstention count. Confidence is the maximum peer vote share (or the training prior's majority share when no peer evidence is available); it is **uncalibrated**, and must not be interpreted as a posterior probability.

## Run

~~~sh
cargo +1.96.0 fmt --manifest-path research/life-relational-privacy/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-relational-privacy/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-relational-privacy/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-relational-privacy/Cargo.toml
~~~

The dedicated workflow checks out and asserts the exact PR-head commit, runs format/test/Clippy, runs the deterministic report twice, compares byte-for-byte output, and retains the report, manifest, SHA-256 checksums and qualification receipt.

## Proposed acceptance criteria

- Zero direct protected-attribute/claim disclosures in all profiles.
- Identical report bytes for identical source and manifest.
- Randomized-peer and uncorrelated-target controls remain within the frozen 100,000 ppm absolute accuracy-vs-prior tolerance.
- At the frozen large-cohort positive control, strong peer correlation exceeds weak peer correlation by at least 200,000 ppm accuracy.
- The independent audit agrees with outcome-record-derived counts.
- Higher confidence thresholds cannot increase accepted coverage; abstention counts are recomputed from individual outcomes.
- CSV width remains rectangular and includes explicit privacy, evidence-access and selective-risk/coverage columns.

These are proposed synthetic-test gates, not measured benchmark results yet.

## Research grounding

The W3C Verifiable Credentials Data Model 2.0 cautions that unlinkable signatures are not a complete solution: disclosed attributes, identifiers, metadata and validation behavior can still enable correlation. NIST SP 800-226 explains that differential privacy's guarantee depends on its unit of privacy and identifies group privacy as a separate concern. This harness is motivated by those concerns but does **not** implement differential privacy or prove group privacy.

References:
- https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/
- https://csrc.nist.gov/pubs/sp/800/226/final
- https://csrc.nist.gov/glossary/term/group_privacy

## How this complements industry baselines

This benchmark is not a replacement for standards conformance and does not claim to beat credential cryptography. It adds a different attack axis those protocol tests alone cannot establish.

| Baseline | What it provides | What this synthetic experiment adds |
|---|---|---|
| W3C VC Data Model 2.0 and presentation profiles | Credential semantics and explicit privacy warnings about identifiers, signatures, metadata and validation correlation | An executable test where the target's protected value is withheld but inference can arise from other subjects' disclosures |
| OpenID4VP 1.0 | A standardized presentation-request/response protocol, including verifier/client and transaction nonce binding to mitigate replay | Measures whether peer disclosures still expose a target even if a presentation is correctly bound to its intended verifier/session |
| SD-JWT / BBS implementations | Concrete selective-disclosure and proof-format behavior that must be evaluated in real, version-pinned implementations | A protocol-independent attack corpus to run alongside those real implementations; the synthetic model does not imitate their cryptography |
| NIST SP 800-226 | Guidance for evaluating systems that claim differential privacy and for identifying common DP hazards | Relational inference measurements that must not be mislabeled as differential privacy or group privacy |

Standards status matters for reproducibility: OpenID4VP 1.0 is a Final Specification published in July 2025. The W3C publication-history page lists Data Integrity BBS Cryptosuites v1.0 as a Candidate Recommendation Draft on 10 September 2026. A future protocol comparison must pin exact documents, code revisions, suites and profile settings, and report conformance separately from privacy attack outcomes.

Sources:
- W3C VC Data Model 2.0: https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/
- OpenID4VP 1.0 Final: https://openid.net/specs/openid-4-verifiable-presentations-1_0-final.html
- W3C BBS publication history: https://www.w3.org/standards/history/vc-di-bbs/
- NIST SP 800-226: https://csrc.nist.gov/pubs/sp/800/226/final

## Claim ceiling

A passing run covers only this synthetic model, source commit, manifest and deterministic seed. It does not establish real-world group privacy, anonymity, causal inference, regulatory compliance, or production safety. Do not interpret abstract signal probabilities as estimates for health, finance, employment, housing or any live Mycelix service.
