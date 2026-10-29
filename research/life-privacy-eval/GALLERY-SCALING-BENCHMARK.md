# Gallery-scaling benchmark contract (design-only / not implemented)

**Status:** design-only / not implemented. This document is a preregistration contract for a future evaluator mode. The current `retrieval` and `open-set` commands use a fixed candidate gallery and do not implement this experiment. Do not describe this document as an executed benchmark.

## Why this is a separate problem

Open-set FPIR depends on gallery/enrollment size, score threshold, and the candidate-list policy. A single fixed-gallery FPIR or Top-k number therefore cannot justify a deployment-scale claim. NIST IREX 10 publishes FNIR at fixed FPIR, rank-based measures, computation timing, and results at different enrollment sizes. In the distinct 1:N biometric identity-proofing scope of NIST SP 800-63A-4, NIST requires an FPIR of 1:1,000 or better (0.1%) and a test gallery no smaller than 90% of intended operational size. These are benchmarks for that defined scope, not universal requirements for credential privacy.

References:
- NIST IREX 10 identification track: https://pages.nist.gov/IREX10/
- NIST SP 800-63A-4, Digital Identity Guidelines — Identity Proofing and Enrollment: https://pages.nist.gov/800-63-4/sp800-63a.html
- NIST IR 8034, Fingerprint Vendor Technology Evaluation: https://nvlpubs.nist.gov/nistpubs/ir/2014/nist.ir.8034.pdf
- Exact binomial confidence limits (NIST Dataplot): https://www.itl.nist.gov/div898/software/dataplot/refman2/auxillar/exacbici.htm

## Preregistered inputs

Each benchmark run must freeze and hash:
1. An operational gallery size (N_{op}), data cut, identity sampling frame, and eligibility rules.
2. A maximum test gallery of at least (0.9N_{op}) when claiming relevance to the NIST SP 800-63A-4 1:N identity-proofing requirement; use the intended operational size where feasible.
3. A preregistered gallery-size grid, including the largest available size and logarithmically spaced smaller sizes. Record actual eligible identities, templates per identity, and exclusions at every size.
4. Separate training, validation, and test query identities where the threat model permits; no attack training/tuning identity may appear in the test query set. Mated queries must have a known candidate in that gallery draw; non-mated queries must be absent from the draw.
5. Opaque query/candidate handles, oracle-only identity membership, exact score/truth joins, and immutable per-draw identity membership lists. The attacker must never receive oracle labels or subject identifiers.
6. Independent, preregistered gallery draws from the declared candidate sampling frame. Save RNG algorithm, seed, draw ID, and selected candidate IDs in an oracle-only artifact. Do not re-use one convenient gallery draw for every reported size and call the resulting uncertainty population-wide.

## Required paired experiments

For each gallery size (N), report two distinct threshold policies:

- **Validation-locked operational policy:** select threshold using validation non-mated queries and a preregistered FPIR target for that gallery size; freeze the threshold before test. Report observed test FPIR and its confidence bound, FNIR at ranks 1/3/5/10, and the exact threshold.
- **Frozen-threshold scaling policy:** lock a threshold from the preregistered reference operating point, then reuse it unchanged across gallery sizes. This exposes how FPIR and FNIR change as the candidate pool grows rather than hiding the effect by recalibrating every size.

Do not select thresholds on test data. Do not pool results from different gallery sizes into one metric. The threshold target is part of the experiment definition, and comparisons must use the same target and calibration protocol.

For a baseline-versus-mitigation study, use the same query subjects, gallery draw IDs, score joins, thresholds policy, bootstrap seed schedule, and strata for both arms. Report paired treatment-minus-baseline metrics and intervals; retain both per-draw outcomes and aggregate summaries.

## Uncertainty design — crossed query/gallery sampling

The future evaluator must not use an independent-row bootstrap. It must estimate uncertainty from both:
- query-subject clusters (including all sessions belonging to the same subject), and
- independently sampled gallery draws from the declared identity frame.

Use a preregistered crossed or hierarchical resampling procedure that resamples query-subject clusters and gallery-draw units, accounts for repeated use of a query in different draws, and keeps the target-in-gallery rule consistent within each draw. Validate interval coverage with simulations under known synthetic data-generating processes before using this estimator for real claims. Include degenerate, sparse-event, single-stratum, repeated-session, and target-missing negative controls. Publish the resampling algorithm, code version, seed, valid replicate count, and coverage results.

Gallery draws alone do not guarantee inference over an unobserved population. If the gallery sampling frame is convenience sampled, the claim must remain conditional on that frame and its collection process. Population-level claims require a defensible sampling frame and transportability analysis.

## Required output at each gallery size

- Gallery size, unique identities, templates per identity, number of gallery draws and query-subject counts by mated/non-mated class.
- Locked threshold and target FPIR; test FPIR, one-sided exact-binomial upper confidence bound for subject-level any-session false-identification events, query-level FPIR interval, FNIR at ranks 1/3/5/10, rank-1 hit rate, and per-stratum results.
- Candidate-list length policy, ties, score semantics, missing/failure-to-process rates, and explicit treatment of mated queries whose identity is absent from a draw.
- Query time and total search time at p50/p95/p99, throughput, memory, stored template/payload bytes, and measured energy on specified hardware. Keep these dimensions separate; do not hide trade-offs in one weighted score.
- Baseline and mitigation privacy-utility-cost frontier, including utility retained and any excluded/abstained queries.
- Input, score, membership, configuration, report, code and environment digests.

## Provisional acceptance rules

These rules are intended to prevent unsupported claims, not to declare compliance automatically:

1. All exact score/oracle joins, identity-disjointness rules, gallery-membership semantics and report schemas pass fail-closed negative controls.
2. The largest benchmark gallery meets the preregistered operational-size criterion; otherwise, explicitly label the result as a smaller-scale study, not deployment-scale evidence.
3. Thresholds are selected on validation only. A privacy mitigation must reduce the strongest applicable preregistered attacker's ability under paired test conditions and retain declared minimum task utility.
4. Report the confidence interval and the exact-binomial upper bound; zero events is not a standalone pass. Subject-level binomial bounds require independent, representative subject clusters and remain conditional on the scorer and the gallery sampling frame.
5. Every mandatory attack, stratum and gallery size is reported. Missing evidence is inconclusive, not a pass. Official protocol conformance, formal privacy guarantees, and deployment certification remain separate gates.


## Operational scale and cost evidence

The `gallery-sweep` command accepts an optional declared intended operational gallery size and reports the evaluated-largest-gallery ratio. An omitted denominator is reported as `NOT_DECLARED`; a ratio below 90% is flagged as below that declared scale. This ratio is descriptive and does not imply standards conformance. NIST SP 800-63A-4 does require a gallery of at least 90% of intended operational size and FPIR of 1:1,000 or better in its narrowly defined 1:N biometric identity-proofing scope; those values are not universal requirements for credential-linkability evaluation. NIST IREX 10 is a useful *evaluation design* reference because it reports FNIR at fixed FPIR, rank-based miss rates, search time and the effect of enrollment size on an operational dataset of 500,000 people, but its iris-matching scores are not comparable to credential linkability scores. See https://pages.nist.gov/IREX10/ and https://pages.nist.gov/800-63-4/sp800-63a.html.

The v0.9 evaluator consumes precomputed attacker scores, so it explicitly reports `NOT_MEASURED_PRECOMPUTED_SCORE_INPUTS` rather than inventing latency, throughput, memory, payload and energy numbers. A future adapter must supply a separate scorer-run receipt with hardware/software identity, workload counts, p50/p95/p99 latency, throughput, peak memory, stored/transmitted bytes, measured energy per query, exclusions/failures, and score/config digests. These cost measurements should remain a separate vector beside privacy and task utility, not collapse into a weighted single score.

## First finite-grid coverage diagnostic — 29 October 2026

The v0.9 gallery-sweep implementation now exists, and the first diagnostic is committed at coverage/gallery_crossed_bootstrap.py. It uses a deterministic synthetic crossed-subject / gallery-draw Bernoulli model and compares nominal 95% crossed percentile-bootstrap intervals with the known marginal event probability. This is deliberately a finite diagnostic grid, not a universal proof.

With 250 outer synthetic studies and 120 bootstrap replicates per study (seed 990117), the two-way percentile bootstrap covered a true 0.1% event probability in only 48.4% of studies for the IID sparse-event scenario; Monte Carlo Wilson 95% interval for the coverage estimate was 42.28–54.57%. A small crossed-cluster scenario (20 query subjects, 10 gallery draws, target 1%, subject and gallery log-odds effects of 0.7) achieved 84.4% observed coverage (Monte Carlo Wilson 95% interval 79.39–88.37%). Several moderate/dense scenarios were closer to nominal, but performance is not uniform across conditions.

A separate conditional bound is now reported per realised gallery draw: the one-sided exact Clopper–Pearson upper endpoint for the fraction of non-mated query subjects with any false identification in that draw. Its alpha is allocated as 0.05 / (gallery_size_count × draw_count × 2 threshold policies), applying Bonferroni across every size/draw/policy cell in the report rather than treating each curve as a separate family. A Rust exact-distribution regression checks this Clopper–Pearson endpoint at alpha values 0.05, 0.01, 0.00625, 0.00078125 and 0.001 over a finite trial/probability grid. This verifies numerical behavior on that grid, not the independence/representativeness assumptions or population transportability. It assumes independent representative query-subject clusters conditional on that gallery and scorer and does not account for candidate-frame sampling uncertainty. It is a different estimand from the crossed percentile interval and must not be substituted for it in a way that hides gallery-draw uncertainty. The overall release gate stays closed.

This finding is an explicit **negative result**: crossed percentile-bootstrap intervals are not qualified for rare-event or certification use. The executable reports KNOWN_SPARSE_EVENT_UNDERCOVERAGE_DO_NOT_USE_FOR_CERTIFICATION and CI must preserve this failure control. Follow-up research should compare at least: exact binomial bounds on independently sampled subject-level any-session events, a conservative cluster-level method, BCa/studentized alternatives where estimable, and a carefully specified two-way cluster method. Any replacement must run a broader preregistered coverage grid including zero-event, low-event, imbalance, subject heterogeneity, draw heterogeneity, and distribution shift. Do not tune and report coverage on the same DGP grid as if that were independent validation.

## Implementation gate

Do not implement this by silently changing the interpretation of the existing fixed-gallery commands. Introduce a separately versioned `gallery-sweep` mode with explicit oracle-only gallery-draw membership, deterministic seeded sampling, per-size validation locking, crossed/hierarchical uncertainty, and static known-answer fixtures. The command now exists, but the first finite-grid coverage diagnostic found sparse-event undercoverage. This file remains a research contract; gallery-sweep intervals are exploratory and must not qualify a privacy claim until an improved uncertainty method independently clears a preregistered coverage study.