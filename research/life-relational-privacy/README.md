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

Accuracy and balanced accuracy are reported in parts per million (ppm), along with prior accuracy and lift above the prior. The report also records the number of peer observations consumed, missing peer rows relative to exact-cohort access, and direct protected-attribute/claim disclosures.

Evidence access and inference accuracy remain separate. Evidence-row counts are not a user-utility metric; they are only an auditable proxy for how much target-linked evidence was available in this synthetic model. A real utility / fairness evaluation requires a separate task-specific benchmark.

A post-hoc audit recomputes target count, correct predictions and direct-disclosure count from per-target outcome records rather than trusting the evaluator's summary counters. Both values appear in the CSV for comparison.

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
- CSV width remains rectangular and includes explicit privacy and evidence-access columns.

These are proposed synthetic-test gates, not measured benchmark results yet.

## Research grounding

The W3C Verifiable Credentials Data Model 2.0 cautions that unlinkable signatures are not a complete solution: disclosed attributes, identifiers, metadata and validation behavior can still enable correlation. NIST SP 800-226 explains that differential privacy's guarantee depends on its unit of privacy and identifies group privacy as a separate concern. This harness is motivated by those concerns but does **not** implement differential privacy or prove group privacy.

References:
- https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/
- https://csrc.nist.gov/pubs/sp/800/226/final
- https://csrc.nist.gov/glossary/term/group_privacy

## Claim ceiling

A passing run covers only this synthetic model, source commit, manifest and deterministic seed. It does not establish real-world group privacy, anonymity, causal inference, regulatory compliance, or production safety. Do not interpret abstract signal probabilities as estimates for health, finance, employment, housing or any live Mycelix service.
