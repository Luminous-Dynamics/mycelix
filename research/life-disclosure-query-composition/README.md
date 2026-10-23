# LIFE-DISCLOSURE-QUERY-COMPOSITION-001

## Question

How much privacy is lost when a verifier receives the correct task decision, receives the task's underlying predicates, or can adaptively ask more yes/no claims over repeated interactions?

The intended paradigm shift is to measure a **privacy–utility frontier under an explicit query policy**, not to treat selective disclosure, fresh proof values, or a correct task result as privacy guarantees by themselves.

## Executable research slice

This dependency-free Rust harness creates 1,024 synthetic people and compares:

- `task_result_only`: disclose only the exact eligibility decision;
- `task_inputs_only`: disclose the two binary inputs used to calculate that same decision;
- `adaptive_q0` through `adaptive_q8`: disclose the eligibility decision, then let an attacker select up to eight additional binary claims adaptively;
- `risk_guard_k32` and `risk_guard_k64`: apply a minimum-branch-size admission guard requiring both possible answers to leave at least 32 or 64 synthetic candidates;
- `dual_guard_k32_uplift100k`: require at least 32 candidates on both answer branches and reject a query if either branch would increase the modeled confidence of **either** protected attribute (high-income or debt-risk) by more than 100,000 ppm (10 percentage points);
- `full_attribute_disclosure`: expose the full modeled predicate vector as an upper-bound comparison.

The eligibility task is `adult AND in_jurisdiction`. All tested profiles are expected to preserve the same task answer. The adaptive attacker starts with the task-result cohort, uses the known synthetic population as auxiliary information, selects the unanswered predicate that most evenly splits the current candidate cohort, observes that target's answer, and intersects the cohort with the answer. Ties are resolved deterministically by feature order.

This is a *query-composition attack model*, not a credential cryptography implementation. The attacker is deliberately strong about having auxiliary population data but deliberately narrow about observing only the modeled predicate answers.

The policy experiments are intentionally not equivalent. The **branch-size-only guard** prevents a response from reducing the modeled cohort below (k), but can still permit a query that discloses a protected attribute with certainty. The **dual guard** additionally measures the maximum-class posterior confidence for high-income and debt-risk in the current candidate set and in each of the two possible next-answer branches. It denies the next query if either protected attribute's confidence would rise by more than the configured absolute tolerance. Crucially, the gate evaluates *both possible branches before reading the target's answer*, so the admit/deny decision itself is not selected according to that next answer in this toy model.

The 100,000-ppm / 10-percentage-point uplift is a benchmark parameter, **not** differential-privacy epsilon, a general privacy threshold, or a recommended universal value. The candidate population is fully known and synthetic; a live policy could not safely copy this oracle without establishing an appropriate data, threat, fairness, and governance model.

## Metrics

Every output row reports:

- exact task utility agreement in parts per million (ppm);
- average candidate-set size in milli-persons and p05 / p50 / p95 candidate counts;
- rate at which the modeled candidate set is a singleton, in ppm;
- majority-rule inference accuracy for high-income and debt-risk attributes, in ppm;
- average number of answered adaptive queries (milli-queries per person) and the share of people whose remaining query sequence was stopped by a guard, in ppm.

Inference-accuracy columns measure global majority-guess correctness under the finite synthetic cohort model; the dual guard's admission threshold instead bounds per-attribute **posterior confidence uplift** between the current cohort and each candidate branch. They are distinct metrics and should be reported together.

The population-prior inference accuracies are emitted as comment lines for comparison. Singleton candidate sets and inference accuracy are **synthetic attack outcomes under this generator**, not real-world re-identification rates or calibrated risk estimates. Candidate-set size is not, by itself, a formal anonymity guarantee.

The test suite asserts deterministic population generation, full task-answer utility for the compared disclosure profiles, monotonic candidate-set shrinkage as queries accumulate, full-disclosure cohort containment, deterministic report output, minimum-cohort preservation under the guards, explicit query stopping, and per-step bounds on the dual guard's modeled sensitive-confidence uplift. These are invariants of this synthetic evaluator, not proofs that a production policy engine is correct.

## Reproducibility

    cargo fmt --manifest-path research/life-disclosure-query-composition/Cargo.toml --all -- --check
    cargo test --locked --manifest-path research/life-disclosure-query-composition/Cargo.toml
    cargo clippy --locked --manifest-path research/life-disclosure-query-composition/Cargo.toml --all-targets -- -D warnings
    cargo run --locked --manifest-path research/life-disclosure-query-composition/Cargo.toml

CI repeats the report and compares the output byte-for-byte, then records a SHA-256 digest of that report against the checked-out commit.

## What this does not establish

- No SD-JWT, BBS, OpenID4VP, Holochain or production protocol is implemented or tested.
- This is not a population-representative dataset, a real-world attack success rate, differential privacy, a proof of anonymity, or a cryptographic unlinkability result.
- The fixed predicate vocabulary, single binary task, synthetic distribution, and known auxiliary population omit real-world side channels, network metadata, issuer/verifier collusion, false claims, and user behavior.
- The balanced-split greedy policy is one explicit adaptive attacker, not an optimal or universal attacker.
- A branch-size guard only bounds the modeled candidate set. It does not prevent attribute inference, and the benchmark intentionally demonstrates that limitation.
- The dual guard only bounds the specified posterior-confidence change for two modeled sensitive labels in the known synthetic population. It is not differential privacy, does not account for unmodeled labels or side channels, and is not robust against population misspecification or auxiliary data mismatch.
- Denial itself can be a side channel in general systems. This toy gate evaluates both possible answer branches before deciding; deployment would still need tests for repeated refusals, timing, policy changes and cross-session state.
- Equal task accuracy in this benchmark shows only that these toy disclosure profiles compute this toy decision correctly; it does not establish equal utility for other transactions.

## Next qualification gates

1. Exact-head format, tests, strict Clippy and deterministic report check pass.
2. Add explicit task diversity and false-accept / false-reject tests.
3. Evaluate skewed, correlated, shifted and adversarially rare subpopulations.
4. Add budget enforcement and policy denial outcomes, not only observation.
5. Repeat against pinned SD-JWT and BBS implementations with identical task utility, independent verifier agreement, matched hardware and versioned conformance vectors.
6. Obtain independent review before claiming practical superiority or deployment readiness.

## Standards motivation

The W3C Verifiable Credentials Data Model 2.0 privacy guidance says selective disclosure and unlinkable proofs do not eliminate correlation via disclosed content, identifiers, metadata, validation behavior, or repeated issuance. The current W3C Data Integrity Threat Model v1.1 is a Group Note Draft; its unlinkable-derived-proof section identifies statement counts, revealed-index patterns, and globally unique disclosed IDs as residual correlation channels. These are motivation for measuring observable outcomes, not evidence that this synthetic harness conforms to those specifications.
