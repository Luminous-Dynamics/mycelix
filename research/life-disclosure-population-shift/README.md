# LIFE-DISCLOSURE-SHIFT-001

## Purpose

Test whether an adaptive disclosure guard's promise generalizes when the population used by the gate differs from the population an attacker can use as auxiliary information.

The companion adaptive query guard benchmark (https://github.com/Luminous-Dynamics/mycelix/pull/5456) lets the policy evaluator and attacker share the same synthetic population. This crate separates:

- **Reference population**: the cohort the query gate uses to decide whether a question is admissible.
- **Attacker/target population**: the population used to evaluate the answers, candidate sets, and sensitive-attribute inference after queries are released.

The primary research claim being tested is deliberately narrow: a guard that meets a threshold under one synthetic reference population may not meet it under a shifted population.

## Frozen profiles

The generator is deterministic and has no external dependencies. Every population contains 512 synthetic records.

- matched_control: reference population reused as the attacker population, an optimistic control.
- independent_seed: a second independently generated population with the same generator family.
- debt_correlation_reversed: target-population debt-risk is changed to income_band >= 4, explicitly breaking the reference relationship.
- income_marginal_shift: income bands are shifted modulo six without changing age or jurisdiction, so eligibility answers are preserved.
- rare_tail: every 29th record enters a synthetic long-tail feature combination.

Compare three query policies: unrestricted; a branch-size guard requiring both answer branches to contain at least k=32 reference records; and a dual guard that also caps each protected label's posterior-confidence uplift at 100,000 ppm (10 percentage points) for either possible answer branch.

The gate's admission function receives only the reference population, current reference cohort, candidate predicate, and policy. It receives neither the target ID nor its next answer. It checks both possible branches before the caller uses the target's answer to update the cohorts. This is important because a guard whose allow/deny result depends on the answer it is trying to protect creates a side channel of its own.

## Metrics

The report includes task-answer utility agreement; candidate-set average and p05/p50/p95; raw count and ppm rate below the configured k; empty candidate count; accepted-query volume; stopped-by-guard rate; majority-rule inference accuracy for high-income and debt-risk with evaluable denominators; and the maximum observed uplift from the starting candidate cohort in each protected label's majority posterior confidence.

Raw violation counts are reported in addition to ppm. With 512 targets, one event is about 1,953 ppm; raw counts remain the audit reference so normalized rates cannot conceal the event denominator.

The below_k_subject_count metric is measured against the attacker's actual candidate set after the disclosure sequence. This can expose a mismatch even though each branch was at least k under the policy's reference population.

## Known-answer test intent

The Rust unit suite freezes deterministic results for the independently seeded and correlation-reversed stress profiles. The expected counts are intended as synthetic regression answers, not general population estimates. A green test means the checked implementation matches this exact synthetic generator and evaluator; it does not prove that its assumptions match real populations.

## Reproducibility

    cargo fmt --manifest-path research/life-disclosure-population-shift/Cargo.toml --all -- --check
    cargo test --locked --manifest-path research/life-disclosure-population-shift/Cargo.toml
    cargo clippy --locked --manifest-path research/life-disclosure-population-shift/Cargo.toml --all-targets -- -D warnings
    cargo run --locked --manifest-path research/life-disclosure-population-shift/Cargo.toml

CI runs the report twice and compares it byte-for-byte, then computes a SHA-256 receipt bound to the checked-out commit.

## Claim ceiling

This is a deterministic, synthetic, finite-population evaluator. It is not a real-world re-identification estimate, anonymity proof, differential-privacy guarantee, production privacy policy, cryptographic unlinkability result, protocol conformance result, or SOTA comparison. Its greedy attacker sees only modeled predicate answers, and its policy has privileged reference-population access. The test specifically demonstrates why even those limited assumptions must be documented.

The threshold values and 10-percentage-point uplift are experimental parameters, not universal recommendations. Deployment choices require explicit threat modeling, independent review, lawful purpose, subgroup/fairness analysis, and real target-system evidence.

## Next gates

1. Exact-head formatting, tests, strict Clippy, repeated report and SHA-256 receipt pass.
2. Review whether the policy can be given a current, authenticated, relevant reference population at all.
3. Add missing/stale reference snapshot and sample-size/confidence-bound cases.
4. Add worst-subgroup and multiple auxiliary-dataset attackers, plus false-answer and refusal-side-channel tests.
5. Compare the same task under version-pinned SD-JWT/BBS profiles and independent verifiers; measure conformance, task utility, privacy risk, proof bytes, p50/p95 latency, CPU and peak memory separately.
6. Do not claim superiority until the same protocol versions, task, population regimes, hardware, and independently checked vectors have been compared.

## Standards context

The W3C Verifiable Credentials Data Model 2.0 privacy guidance and its Data Integrity threat-model work motivate testing content and metadata correlation even when proof values are unlinkable. This benchmark does not implement VC, SD-JWT, BBS, or OpenID4VP and does not claim conformance to any of them. Standards and implementation comparison is tracked separately in LIFE-CREDENTIAL-INTEROP-001 (https://github.com/Luminous-Dynamics/mycelix/issues/5447).
