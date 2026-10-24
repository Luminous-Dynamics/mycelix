# PRAXIS-LEARNING-OUTCOME-ANALYSIS-001

**Status:** deterministic source, input/output schemas, synthetic data fixture, and 13-control matrix authored. Rust execution and full JSON Schema meta-validation remain outstanding.

## Purpose

The protocol in PRAXIS-LEARNING-OUTCOME-PROTOCOL-001 declares what a study intends to measure. This increment adds a small deterministic analysis engine and a typed participant-outcome dataset so that a declared protocol can lead to a reproducible, version-bound result record.

The first implementation deliberately supports only an individual-randomized, two-arm, unadjusted difference-in-means analysis of the preregistered primary endpoint. It does **not** support cluster-randomized inference, covariate adjustment, or multiple imputation. Unsupported configurations fail closed instead of quietly switching estimators.

## What the engine does

- Accepts exact protocol JSON bytes, exact outcome-dataset JSON bytes, and an immutable 40-hex analysis source revision.
- Validates the protocol through the protocol validator and rejects draft-only protocols for inferential analysis.
- Requires each pseudonymous participant to appear once, use a registered arm, and carry one primary outcome or an explicit missingness reason.
- Validates the outcome endpoint ID, scale bounds, study identity, dataset schema version, and analysis configuration.
- Emits arm-level randomized/observed/missing counts, means, and sample standard deviations. Numeric results use fixed-point integers: input outcome values are scaled by 1,000; report values are scaled by a further 1,000,000.
- Binds the exact input bytes with BLAKE3 and records the analysis source revision, endpoint, estimator, and estimand.
- Sorts numeric values before variance computation, making result estimates invariant to participant-row permutation. Raw input digests still change when the byte sequence changes.
- Uses a Welch two-sample interval with a Cornish–Fisher approximation to the Student-t critical value only when both arms have at least 30 observed outcomes, each arm meets the registered randomized sample target, all primary outcomes are observed, and the standard error/degrees of freedom are usable.

## Fail-closed behavior

| Condition | Behavior |
|---|---|
| Draft protocol | Rejects inferential analysis |
| Duplicate participant, unknown arm, wrong endpoint, out-of-range value, malformed JSON, or invalid missingness encoding | Rejects the input |
| Any missing primary outcome | Withholds treatment-effect point estimate and confidence interval; reports arm counts and worst-case bounds using the registered endpoint scale |
| Fewer than 30 observed outcomes in an arm or randomized N below the registered target | May report a descriptive difference, but suppresses the inferential interval |
| Both arm variances are zero or the Welch calculation is unusable | Reports no inferential interval |
| Cluster design, covariate-adjusted estimator, or multiple imputation | Rejects as unsupported by analysis v1 |

The missing-outcome bounds are sensitivity bounds, **not confidence intervals**. They are computed by assigning every missing outcome in each arm the endpoint scale's lowest or highest allowed value to form worst- and best-case treatment-minus-comparator differences. The interval can be wide; that is an honest statement about missing information, not a model failure to conceal.

V1 does not impute missing data. This is intentional: a missing-data plan name is not an implementation. A later version can add a predeclared imputation method with pinned version, inputs, assumptions, and sensitivity analyses.

## Statistical limitations

This is a reproducibility and guardrail tranche, not a validated statistical package or a causal certificate.

- The Welch interval's critical value uses an explicit Cornish–Fisher approximation, and the implementation imposes a 30-per-arm minimum. It does not calculate a p-value, correct for multiple primary endpoints, or independently verify that the sample-size/power digest points to an authentic calculation.
- A protocol marked preregistered is checked for a URI and digest shape, but this function does not contact or authenticate the registry. The caller must verify registration independently.
- The dataset's arm labels are accepted as declared data; this code does not verify random assignment, recruitment, consent, or the provenance of assessment scoring.
- ITT is reported as the estimand only when the supplied dataset represents the randomized participants and all primary outcomes are present; when outcomes are missing, effect inference is suppressed. The presence of randomized participant rows does not independently prove that the dataset is complete.
- This analysis only covers the primary endpoint. The secondary endpoint vector (engagement, time, cost, displacement, and adverse events) requires an expanded multi-endpoint pipeline and separate reporting. None is folded into the primary learning effect.

These boundaries must remain visible in any downstream dashboard.

## Synthetic fixture

`participant_outcomes.synthetic.json` contains 240 explicitly synthetic rows, with 120 synthetic tokens per arm and no missing values. The values are deterministic test data designed to exercise calculations; they are not learner records, and any resulting difference is **not evidence** of tutoring efficacy. Its matching study protocol remains a draft fixture. The Rust unit tests mutate a copy to preregistered-shaped metadata solely to exercise the analysis path; its example registry URL/digest are placeholders and are not a valid registration receipt.

## Adversarial and metamorphic tests

The matrix covers a complete-data result, row-order invariance with byte-digest change, missing-outcome suppression/bounds, draft rejection, duplicate participants, out-of-range outcomes, unsupported imputation, missing-reason encoding, wrong endpoint, insufficient sample, malformed input, and unknown arms. Every matrix item points to a named Rust test.

A static source audit can check JSON parsing, schema/DTO field parity, fixture uniqueness/ranges, and test-name linkage. It cannot establish that the Rust tests compile or pass; those require execution on the exact head. The next qualification step is to run `cargo fmt --check`, `cargo test -p praxis-core`, the actual JSON Schema Draft 2020-12 validator, and independent statistical review before promoting the implementation beyond draft.

## Research rationale and comparison

- [StudentBench (2026 preprint)](https://arxiv.org/abs/2609.28470) demonstrates why tutoring capability, observed learning gains, engagement, and cost should be measured as separate dimensions within a defined GRE study.
- [NBER Working Paper 35621](https://www.nber.org/papers/w35621) reports improved recovery after mistakes alongside slower progression and nuanced delayed-test results in a 6,000-plus-student mastery-based study.
- [NBER Working Paper 35620](https://www.nber.org/papers/w35620) reports that access to a tutor does not guarantee substantive use or gains beyond practice alone.

The internal target is not a fabricated SOTA score. It is: (1) exact reproducibility; (2) no effect claim when the primary outcome is missing; (3) explicit, conservative missing-data sensitivity bounds; (4) no unsupported estimator substitutions; and (5) a result receipt that can be independently reconstructed. External superiority or efficacy claims must wait for a matching benchmark run or appropriately governed study.
