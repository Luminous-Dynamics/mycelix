# PRAXIS-LEARNING-OUTCOME-PROTOCOL-001

**Status:** versioned protocol contract, Rust validator, Draft 2020-12 schemas, synthetic fixture, and adversarial control matrix authored. Executable Rust/schema validation and independent review remain outstanding.

## Why this is a separate evaluation track

Tutor response quality is not the same thing as learning. A model can produce plausible explanations and still fail to improve retention, transfer, learner autonomy, or achievement per unit of time and cost. This protocol therefore makes *independently assessed delayed learning* the primary endpoint class while keeping capability scores, operational mechanisms, cost, and safety as separate dimensions.

The protocol is deliberately stricter than a generic logging format:

- Exactly one primary endpoint is named before outcomes are observed; v1 permits delayed retention or near/far transfer, not response quality, engagement, or an opaque weighted tutor score.
- Transfer items must be declared unpracticed before enrollment, measured at least 24 hours later, and assessed independently by a blinded assessor.
- The analysis plan records the intention-to-treat estimand, a compatible confidence/alpha pair, multiplicity correction, missing-data sensitivity plan, target power (at least 80%), a minimum detectable effect, and the digest of the retained power/sample-size justification.
- Every protocol must retain engagement, time-on-task, cost-per-gain, displaced independent practice, and adverse-event dimensions. They are reported separately, not buried in a single aggregate.
- Cluster-randomized studies require classroom/school assignment and a cluster-robust estimator.
- A protocol marked preregistered or complete must carry a preregistration URI and a BLAKE3 digest. A draft must not claim a preregistration receipt.

The Rust validator checks these protocol-level rules. It does **not** verify the external URI, recompute the sample-size calculation digest, validate actual participant outcomes, run statistical inference, authenticate assessors, or establish causal effects.

## Evidence this design responds to

- [LongTutor (ACL 2026)](https://aclanthology.org/2026.acl-long.1371/) separates historical evidence acquisition, knowledge-state diagnosis, and adaptive teaching action, and reports that strength on history retrieval does not guarantee strong diagnosis/action.
- [TutorBench](https://labs.scale.com/leaderboard/tutorbench) measures rubric-based tutoring capabilities such as adaptive explanations, assessment/feedback, and active-learning support. It is useful for capability comparison, not a substitute for measured learning gains.
- [StudentBench (September 2026 preprint)](https://arxiv.org/abs/2609.28470) reports a GRE learning-outcome study with 2,383 participants and AI, human-tutor, and no-tutor conditions. Its effects and cost estimates are specific to its tested tasks and study design; they are a replication target, not a universal guarantee.
- [NBER Working Paper 35621 (August 2026)](https://www.nber.org/papers/w35621) reports a field experiment with more than 6,000 middle-school students: AI support improved next-attempt correctness after mistakes but slowed progress and did not produce uniform delayed-learning gains.
- [NBER Working Paper 35620 (August 2026)](https://www.nber.org/papers/w35620) reports a two-year cluster-randomized study in 18 middle schools where achievement effects were modest and engagement was a binding constraint.

These studies do not share a common outcome scale or population. Their results must not be combined into a cross-study leaderboard without compatible estimands and measurement.

## Endpoint definitions

| Dimension | What it measures | Interpretation boundary |
|---|---|---|
| Delayed retention | Performance on previously practiced material after a predeclared delay | Retention is not transfer |
| Near transfer | Performance on unpracticed but structurally close items | Item novelty and relation must be predeclared |
| Far transfer | Performance on unpracticed items requiring generalization | Primary candidate in the synthetic contract only; not universally best for every domain |
| Immediate gain | Baseline-to-immediate post-assessment change | Secondary; vulnerable to short-term cueing |
| Error recovery | Attempts/time to return to an independently correct answer after a mistake | Mechanism metric, not sufficient proof of learning |
| Engagement / time | Actual use, attempts, and time-on-task | Usage is not itself learning |
| Cost per gain | Cost divided by an explicitly defined learning-gain unit | Report formula, uncertainty, and denominator; avoid zero/negative-gain division artefacts |
| Displacement | Whether assisted use replaces independent retrieval/practice or teacher contact | Must be measured, not presumed beneficial |
| Adverse events | Predefined safety, dependency, distress, or other domain-relevant harms | No observed events is not proof of no risk |

The schema stores scale values as integers scaled by 1,000 to avoid float serialization drift. Every endpoint also declares a unit and favorable direction: learning-score endpoints must improve upward, error-recovery attempt counts, cost, displaced practice, and adverse-event counts must improve downward, while engagement and time-on-task are interpretation-sensitive rather than mechanically rewarded. Instrument and scoring/calculation procedures have separate digests.

## Study and analysis discipline

1. Freeze the protocol, primary endpoint, item-family rules, sample-size justification, missing-data strategy, and subgroup plan before observing outcomes.
2. For randomized designs, use intention-to-treat as the primary estimand. A cluster assignment requires inference that accounts for clustering.
3. Report each arm's planned and observed sample count, attrition, outcome distribution, effect estimate, and confidence interval. The first validator increment defines the protocol but not the participant-row/result schema or analysis engine.
4. Do not treat a nonsignificant difference as proof of safety or noninferiority. Those claims require an explicit margin, a design powered for it, and confidence intervals that exclude unacceptable harm.
5. Keep immediate performance, delayed retention, near transfer, far transfer, mechanism metrics, cost, displacement, and harms separate. No single weighted "AI tutor quality" score can compensate for a negative result on a primary learning outcome or a safety gate.
6. For the internal matched comparison, B0 is the unchanged deterministic Praxis route and B1 adds pinned Symthaea advice without relaxing policy gates. Matched data, evaluator, system/model configuration, sampling, and execution details remain mandatory under the run-integrity contract. External comparisons must follow each benchmark/study's published protocol instead.

## Synthetic fixture limitations

`study_protocol.synthetic.json` is a shape/conformance fixture with illustrative arm counts (120 per arm), placeholder digest strings, a draft status, and no participant records or study results. **The counts are not a power calculation or sample-size recommendation; the digest-shaped placeholders are not cryptographic receipts.** The fixture must never be cited as evidence that tutoring improves learning.

The mutation matrix lists 18 positive/negative protocol controls and links each to a named Rust test. Static source checks can verify schema shape, field parity, fixture consistency, unique control IDs, and test-name linkage. They cannot substitute for compiling/running the Rust tests, validating the schema with a full Draft 2020-12 implementation, independent educator adjudication, a real model benchmark, or a governed learning study.

## Qualification targets

- 100% of protocol controls produce the declared acceptance/rejection finding when executed.
- Zero unknown or duplicate endpoint IDs, and zero unpracticed-transfer violations in accepted protocols.
- 100% of efficacy protocols carry a pinned pre-enrollment power/sample-size plan, a missingness strategy, and separately reported harm/displacement dimensions.
- For actual studies, publish arm-level estimates and uncertainty; report outcomes by predeclared diagnostic/learner strata only when privacy and sample size permit.
- Claim superiority over another tutoring system only on a matching external benchmark run under its published rules. Claim learning efficacy only from independently assessed learner outcomes and an appropriate study design.
