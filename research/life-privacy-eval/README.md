# LIFE Privacy Evaluation: held-out linkability and retrieval risk evidence v0.4

This crate is a **metric evaluator**, not a credential verifier and not a linkability attacker. It evaluates scores emitted by an independently implemented attacker using only verifier-observable presentation artifacts. It never invents attack features or hand-selects feature weights.

## Paradigm shift

Separate the **attacker's view**, the **ground-truth oracle**, and the **evaluator** into three trust domains. The attacker receives sanitized presentation features and an opaque \`pair_id\`; it emits only \`pair_id\` and a finite numeric score. The evaluator separately joins that score file to ground-truth subject pairs and labels. Ground-truth identifiers/labels must never be supplied to the scoring process. This prevents a common benchmark failure: accidentally using evaluation labels or stable subject IDs as predictive features.

The evaluator then reports discrimination (ROC-AUC), imbalanced-ranking quality (average precision), recall at low false-positive rates, per-stratum results, and a deterministic **subject-clustered bootstrap** confidence interval for AUC. Pair-row bootstrap is intentionally avoided because pairs sharing people are not independent observations.

## Input contracts

\`truth.tsv\` (oracle-only; never pass this file to the attack scorer):

\`\`\`text
pair_id<TAB>subject_a<TAB>subject_b<TAB>label<TAB>stratum
pair_0001<TAB>test_subject_0001<TAB>test_subject_0001<TAB>1<TAB>stable_artifact
pair_0002<TAB>test_subject_0001<TAB>test_subject_0002<TAB>0<TAB>stable_artifact
\`\`\`

\`label=1\` means both presentations came from the same synthetic subject; \`label=0\` means different subjects. This crate enforces the equality against \`subject_a == subject_b\`. Each pair ID must be unique.

\`scores.tsv\` (the only evaluator input that originates from the attacker):

\`\`\`text
pair_id<TAB>score
pair_0001<TAB>0.91
pair_0002<TAB>0.14
\`\`\`

The score is an ordering score, **not necessarily a calibrated probability**. Pair IDs must be random opaque IDs and must not encode subject identity. The schema intentionally forbids subject and label columns in this file.

\`training-subjects.txt\` lists all subject IDs exposed during attack training or tuning. For a non-learning attacker, provide an empty file. The evaluator rejects any intersection between training and test subjects. This is a mechanical guardrail, not proof that an external model has no other leakage.

## Run

\`\`\`sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked --release -- \
  fixtures/truth.tsv \
  fixtures/scores.tsv \
  fixtures/training-subjects.txt \
  report.json 1000 9901
\`\`\`

Use \`-\` as the report path to write JSON to stdout. Repeated invocations with the same bytes, bootstrap count, and seed produce deterministic metric output.


## Paired mitigation comparison (v0.2)

The compare command evaluates baseline and treatment scores on the same exact truth rows. Example:

    cargo run --locked --release --manifest-path research/life-privacy-eval/Cargo.toml -- compare \
      research/life-privacy-eval/fixtures/truth.tsv \
      research/life-privacy-eval/fixtures/paired-baseline-positive-control.tsv \
      research/life-privacy-eval/fixtures/paired-treatment-no-signal-control.tsv \
      research/life-privacy-eval/fixtures/training-subjects.txt \
      paired-report.json 1000 9901 0.55

The two score files must have identical unique pair IDs, and each independently passes the same truth join and train/test subject-disjointness checks. The evaluator uses the existing canonical ROC-AUC implementation and resamples the same subject multiplicities for baseline and treatment, reporting a percentile 95% interval for treatment-minus-baseline AUC. A negative upper bound means the supplied treatment ranked linkage pairs less effectively on this cohort; intervals crossing zero are inconclusive.

The proposed AUC upper-bound target defaults to 0.55 and can be overridden per preregistered profile. Smoke-sized inputs cannot qualify a privacy profile: the evaluator first requires the 10,000-pair / 2,000-subject screening floor. Even when this floor is met, the output is only a provisional profile result requiring review. Overall release status remains not evaluated until task utility and applicable official conformance evidence are separately supplied.

The checked-in baseline/treatment files are a deliberately synthetic evaluator known-answer control (perfect-ranking baseline versus tied-score treatment), not an attack model or a real mitigation result. Task-utility and conformance flags remain false by design.


## Validation-locked low-FPR operating point (v0.3)

The original metrics command reports TPR at empirical FPR caps by reading the complete test ROC curve. Those are exploratory ranking summaries, not deployable thresholds, because they are selected from the same labels being evaluated. The default report schema flags this limitation.

The `calibrated` subcommand selects a threshold on an independent validation cohort, then applies the frozen threshold to test scores. Example:

    cargo run --locked --release --manifest-path research/life-privacy-eval/Cargo.toml -- calibrated research/life-privacy-eval/fixtures/calibration-truth.tsv research/life-privacy-eval/fixtures/calibration-scores.tsv research/life-privacy-eval/fixtures/truth.tsv research/life-privacy-eval/fixtures/scores.tsv research/life-privacy-eval/fixtures/training-subjects.txt locked-operating-point-report.json 0.01 1000 9901

Calibration and test subject sets and pair handles must be disjoint; both partitions must contain positive and negative pairs and must be disjoint from declared training subjects. The threshold maximizes validation TPR subject to the configured empirical validation FPR cap, with tied scores kept together. It is never re-tuned on test labels.

The report includes test TP/FP counts, fixed-threshold TPR/FPR, subject-cluster bootstrap intervals and empirical FPR resolution (one false positive divided by negative test pairs). **Zero observed false positives remain inconclusive for rare-event risk**: a zero-width bootstrap interval cannot bound unobserved tail events. The checked-in controls are synthetic; task utility and official conformance are intentionally absent, so overall release status remains not evaluated.

## Metrics

- **ROC-AUC**: probability that a random positive pair ranks above a random negative pair, with half-credit for ties.
- **Average precision (AP)**: non-interpolated ranking metric calculated at score-tie boundaries; useful when positive pairs are rare.
- **TPR at FPR ≤ 1% / 5%**: maximum recall at a threshold whose empirical false-positive rate does not exceed that cap. Equal scores enter as a group; the evaluator does not split ties to manufacture an operating point.
- **95% AUC interval**: percentile interval from deterministic subject-cluster bootstrap. It resamples subjects, then assigns same-subject pair weight \`m(s)\` and cross-subject pair weight \`m(a) × m(b)\`, where \`m\` is the subject's bootstrap multiplicity. This handles dependence among edges more honestly than independent-pair bootstrap, but it is still a percentile interval—not a proof of exchangeability or a universal guarantee.
- **Per-stratum results**: each stratum is reported separately. Thin or single-class strata produce \`null\` metrics instead of a fake pass.

## Pilot fixture vs release evidence

The checked-in fixtures are small, synthetic smoke tests. The report labels them \`SMOKE_ONLY_BELOW_MINIMUM\`. They do **not** support a privacy claim. Proposed release-evidence minimums are ≥10,000 held-out pairs and ≥2,000 distinct test subjects, subject-disjoint train/test, a preregistered threat model, multiple documented attacker baselines, and independent audit of observable features. Minimum sample size is necessary but not sufficient.

For each format/profile and adversary, report the whole privacy–utility–cost frontier: ROC-AUC and 95% CI, AP, recall at low FPR, top-1 re-identification, subgroup/worst-stratum results, claims disclosed beyond necessity, payload bytes, p50/p95/p99 latency, memory and measured energy. Do not combine these into one weighted score.

## Scope and non-claims

- Does not parse, issue, or verify SD-JWT, BBS, mdoc, or OpenID4VP presentations.
- Does not prove a dataset is representative, that an attacker is strong, or that the scoring process only used allowed observations.
- Does not establish differential privacy or formal unlinkability.
- Does not claim standards conformance, production security, SOTA performance, or a privacy pass.
- Inputs must use synthetic opaque IDs. Do not put real names, identity numbers, credential IDs, raw claims, public stable hashes of real identifiers, or secrets into fixtures or reports.

## Standards baseline

LIFE evaluation complements protocol conformance rather than replacing it. Run OpenID4VP/OpenID4VCI + HAIP conformance where applicable; maintain a separate privacy track. RFC 9901 describes different unlinkability goals and SD-JWT's limits. The W3C Threat Model for Decentralized Credentials (Group Note Draft, 8 September 2026) covers the ecosystem's actors and data flows and records linking/surveillance risks from credentials, status/registry data, metadata and patterns of use. The W3C Data Integrity BBS Cryptosuites v1.0 Candidate Recommendation Draft (10 September 2026) describes unlinkable derived proofs while noting residual linkability from disclosed-message indexes, total message count and issuer-key choices.

References:
- RFC 9901: https://datatracker.ietf.org/doc/rfc9901/
- W3C Threat Model for Decentralized Credentials: https://www.w3.org/TR/2026/DNOTE-threat-model-decentralized-credentials-20260908/
- W3C Data Integrity BBS Cryptosuites v1.0 CRD: https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/
- OpenID4VP/OpenID4VCI + HAIP conformance self-certification: https://openid.net/openid4vp-and-openid4vci-conformance-tests-are-complete-and-open-for-self-certification/


## Threat coverage and claim ceilings — 29 October 2026

The machine-readable `threat-coverage-matrix.json` accounts for all 25 threat IDs in the W3C Verifiable Credentials Data Model Threat Model v2.1 draft exactly once: 15 are mapped to evaluation or privacy-utility lanes, and 10 are explicitly routed to separate security/governance gates. It maps those threats to observer capabilities, existing scenario IDs, minimum evidence, and explicit claim ceilings. The workflow checks its threat/profile counts, uniqueness, status vocabulary, and scenario references, then binds its SHA-256 digest into the qualification receipt. This protects the **coverage contract** from silently drifting away from the scenario catalogue; it does not establish that every mapped channel is implemented or tested.

The matrix separates six attacker profiles: no-signal control, deterministic channel matching, adaptive tabular attacker, longitudinal attacker, colluding parties, and network/device telemetry. The last three remain proposed or deployment-dependent unless their exact artifacts and receipts are present. It also explicitly routes replay/context misuse, wallet trust, and post-presentation retention to separate security or governance evidence instead of mislabeling them as linkability metrics.

Standards context: the W3C threat model is a Group Note Draft and must be treated as an evolving threat taxonomy; RFC 9901 is the IETF Standards Track SD-JWT specification; BBS derived-proof properties do not remove every metadata or issuer-key correlation channel. OpenID4VP/OpenID4VCI + HAIP conformance remains a separate official test lane. The evaluator's synthetic smoke controls cannot be presented as credential-format measurements, deployment privacy proof, or a SOTA result.

Release-scale priorities remain: matched real-format implementations and task utility; a preregistered attacker ladder with subject-disjoint splits; top-k retrieval and effective-anonymity-set reporting where retrieval is defined; worst-stratum analysis; longitudinal/collusion traces; validation-locked low-FPR evaluation with rare-event uncertainty; and payload, p50/p95/p99 latency, memory and measured-energy frontier. The 10,000-pair / 2,000-subject floors are screening floors, not sufficient evidence or an automatic pass.


## Closed-set retrieval risk — v0.4

Pairwise ROC-AUC does not answer a different operational question: if a verifier has a cohort of candidate identities, how often can it rank the correct person first or among its top few candidates? The `retrieval` subcommand evaluates that question without trying to derive retrieval ranks from pairwise classifier scores.

### Inputs

Oracle-only `retrieval-truth.tsv` has exactly these columns:

```text
query_id<TAB>query_subject<TAB>candidate_id<TAB>candidate_subject<TAB>true_match<TAB>stratum
```

Attacker output `retrieval-scores.tsv` contains only `query_id`, `candidate_id`, and finite `score`. Do not expose query/candidate subject identities or `true_match` labels to attack scoring. Pair rows are joined exactly; missing, extra or duplicate query/candidate handles fail closed. Training subjects must be disjoint from every query and candidate subject.

This first implementation deliberately uses a **closed-set** design: each query has exactly one true candidate, the query subject appears in the candidate pool, and every query uses the same candidate pool. Open-set detection (including the case where no candidate matches) is a separate extension and must not be silently mixed into these metrics.

### Metrics and tie semantics

The report includes Top-1/3/5/10 re-identification rate, mean reciprocal rank, expected target rank and median/p95 rank, a uniform-random ranking baseline, per-stratum outcomes, and deterministic query-subject-cluster bootstrap intervals for Top-1, Top-5 and MRR. Equal scores are handled by uniform expected rank within the true target's tie block; this gives fractional Top-k credit and avoids lexical candidate IDs acting as an unstated tie-breaker.

Intervals are explicitly **conditional on the fixed candidate pool**. Query-subject resampling does not measure uncertainty from sampling a different candidate population, and shared candidate pools can induce dependence. Future work should add a preregistered candidate-population resampling design or crossed query/candidate cluster method before making broader population claims.

The evaluator does **not** convert arbitrary scores into posterior probabilities and does not claim an effective anonymity set. Anonymity set size alone ignores uneven attacker probabilities, while entropy-based estimates can also misrepresent identification correctness when the population distribution is non-uniform or heavy-tailed. See [Andersson & Lundin, *On the Fundamentals of Anonymity Metrics*](https://dl.ifip.org/db/conf/ifip9-6/fidis2007/AnderssonL07.pdf) and the 2024 [scaling-law analysis of identification techniques](https://www.nature.com/articles/s41467-024-55296-6). A defensible probability-based effective anonymity metric requires calibrated candidate posterior probabilities and explicit calibration/shift checks; it is not inferred from ranks here.

Run it with:

```sh
cargo run --locked --manifest-path research/life-privacy-eval/Cargo.toml -- retrieval \
  research/life-privacy-eval/fixtures/retrieval-truth.tsv \
  research/life-privacy-eval/fixtures/retrieval-scores.tsv \
  research/life-privacy-eval/fixtures/retrieval-training-subjects.txt \
  /tmp/retrieval-report.json 300 9901
```

The committed four-query fixture is a synthetic known-answer check only. It is expected to produce Top-1 = 0.375, Top-3 = 0.75, mean reciprocal rank = 0.58333333, and mean expected rank = 2.375. These are fixture semantics, not real-world measurements. Release remains closed without separate utility and official conformance evidence.


## Open-set identification — v0.5

Closed-set ranking assumes every query has a true candidate in the gallery. Real identification deployments also encounter queries whose subject is **not** represented in that gallery. The `open-set` subcommand evaluates those non-mated queries explicitly and follows the established open-set identification idea of false-negative identification rate at a specified false-positive identification rate (FNIR@FPIR). See the [NIST open-set fingerprint identification evaluation](https://www.nist.gov/publications/performance-evaluation-fingerprint-open-set-identification-algorithms) and the [ECCV 2024 Open-Set Biometrics benchmark paper](https://www.ecva.net/papers/eccv_2024/papers_ECCV/html/7925_ECCV_2024_paper.php).

### Validation-locked operating point

The command accepts validation and test truth/scores. Every query has zero or one true match; all queries use exactly the same candidate handle/subject pool. The candidate pool itself must match across validation and test. Validation and test query handles and query subject identities are disjoint; training subjects are disjoint from candidate and query subjects. The truth file remains oracle-only; score files contain only opaque query/candidate handles and finite scores.

The operating threshold is selected on validation queries only by maximizing rank-1 correct-identification rate subject to the declared empirical false-positive identification rate (FPIR) cap; ties are not split, and the threshold is frozen before test evaluation. The test report includes FNIR at ranks 1/3/5/10, FPIR, precision among accepted queries, FPIR resolution, per-stratum outcomes and query-subject cluster-bootstrap intervals for FNIR@1, FNIR@5 and FPIR. Zero observed false identifications do not certify rare-event safety.

Example:

```sh
cargo run --locked --manifest-path research/life-privacy-eval/Cargo.toml -- open-set \\
  research/life-privacy-eval/fixtures/open-set-validation-truth.tsv \\
  research/life-privacy-eval/fixtures/open-set-validation-scores.tsv \\
  research/life-privacy-eval/fixtures/open-set-test-truth.tsv \\
  research/life-privacy-eval/fixtures/open-set-test-scores.tsv \\
  research/life-privacy-eval/fixtures/open-set-training-subjects.txt \\
  /tmp/open-set-report.json 0.01 300 9901
```

The report's bootstrap intervals are conditional on the fixed candidate pool. The screening minimum is 2,000 mated and 2,000 non-mated queries per partition, plus at least 2,000 distinct query subjects in each class and a candidate pool of at least 10; repeated sessions do not satisfy the distinct-subject floors. These are screening floors, not enough by themselves to bound rare false-identification risk. The four-query-per-partition fixture is smoke-only; its expected locked threshold is 0.95 at a zero-FPIR validation cap, with validation FNIR@1 = 0.5. These are synthetic known-answer semantics only. The overall privacy/release gate remains closed without task-utility and official conformance evidence. Open-set identification is distinct from pairwise ROC-AUC and from protocol conformance; do not infer posterior probabilities or an effective anonymity set from arbitrary ranking scores.


## Subject-cluster rare-event bound — v0.7

The open-set report keeps query-level FPIR/FNIR and query-subject cluster-bootstrap intervals, and separately aggregates each non-mated subject's repeated queries into one event: did **any** session produce any false identification? A one-sided **exact Clopper-Pearson binomial upper confidence bound** is applied to those subject-level Bernoulli outcomes. For zero observed events among (n) independent subject clusters, the exact endpoint is (1 - 0.05^{1/n}). For nonzero counts, the dependency-free Rust implementation solves the binomial-tail equation by deterministic bisection and a log-sum-exp probability calculation.

At 95% confidence, zero events require **299 independent non-mated subject clusters for a 1% upper target** and **2,995 for a 0.1% upper target**. This is much less conservative than the earlier Hoeffding bound. It is not a free pass: the interpretation still assumes independent, representative subject clusters, conditions on the fixed gallery and frozen scorer, and does not cover gallery-population uncertainty, distribution shift, dependent subjects or a formal privacy guarantee. The query-level FPIR bootstrap endpoint and subject-level exact-binomial endpoint must both meet the target before the operating point can become provisional. Utility and official conformance remain separate gates.

## Gallery scaling and operational-size evidence — design contract

The next benchmark specification is `GALLERY-SCALING-BENCHMARK.md` (**design-only; not implemented**). It requires preregistering gallery sizes, a deployment-size target, reproducible nested identity samples, size-specific validation-only thresholds, paired candidate draws for mitigation comparisons, and a crossed query/gallery uncertainty analysis. Each gallery draw is an experimental unit; repeated queries from one identity are not independent people. Do not derive a deployment claim from the current fixed-gallery retrieval or open-set modes alone.

For context, NIST's current IREX 10 identification evaluation reports FNIR at fixed FPIR, candidate ranking, computation timing and performance by enrollment size; its public report cards use a 500,000-person enrolled population for the operational dataset. See [NIST IREX 10](https://pages.nist.gov/IREX10/) and the [IREX 10 identification track](https://pages.nist.gov/IREX10/). Separately, NIST SP 800-63A-4 requires, within its 1:N biometric identity-proofing scope, FPIR of 1 in 1,000 (0.1%) or better and testing with a gallery no smaller than 90% of intended operational size. That is a specific identity-proofing requirement, not a universal credential unlinkability threshold. See [NIST SP 800-63A-4](https://pages.nist.gov/800-63-4/sp800-63a.html).

The exact-head workflow checks the statistical endpoint, nonzero-event behavior, report schema and synthetic fixture determinism. It does not make synthetic data evidence of real credential unlinkability.


## Exact interval coverage regression checks — v0.8

The unit suite now validates more than a few known answers for the one-sided Clopper-Pearson subject-level upper bound. For each preregistered pair of sample size (n in {5,10,20,50,100}) and true event probability (p in {0.005,0.01,0.025,0.05,0.1,0.25,0.5,0.75,0.9,0.99}), it sums the exact binomial PMF over all observed event counts whose reported upper bound contains (p). The coverage must be at least 95% (within a stated floating-point tolerance). A separate test checks that the bound is monotone in the observed event count and that the zero-event bound decreases with sample size.

These are deterministic exact-distribution numerical regression checks over a finite grid—not Monte Carlo and not a proof for all possible probabilities or for dependent/nonrepresentative subject clusters. The confidence guarantee still depends on the binomial sampling assumptions and remains conditional on the fixed gallery/scorer.


## Gallery-scaling open-set experiment — v0.9.0

The new **gallery-sweep** command is a distinct experiment mode. It samples identity-level galleries with a deterministic seed, builds nested galleries for each draw, selects a threshold on validation queries separately at each gallery size under a declared empirical FPIR cap, and evaluates those locked thresholds on disjoint test query subjects. A second curve locks the threshold selected at a declared reference size (the default is the largest gallery) and reuses it unchanged at every size, making the cost of gallery growth visible.

Example from this directory (the arguments use the fixture paths shown):

    cargo run --locked --release --manifest-path Cargo.toml -- gallery-sweep \
      fixtures/gallery-validation-query-truth.tsv \
      fixtures/gallery-validation-scores.tsv \
      fixtures/gallery-test-query-truth.tsv \
      fixtures/gallery-test-scores.tsv \
      fixtures/candidate-frame.tsv \
      fixtures/gallery-training-subjects.txt \
      gallery-report.json 0.01 8 300 9901 2,4,8,12 12

Input contracts:

- Query truth contains only query_id, query_subject, and stratum. It is oracle-only and must not be supplied to the attacker.
- The candidate frame contains candidate_id and candidate_subject; it is also oracle-only. This v0.9 mode permits one candidate per identity, not multiple templates per identity.
- Attacker scores contain only query_id, candidate_id, and a finite score. The score grid must cover every query against every identity in the declared candidate frame. Query handles and candidate handles must be opaque.
- Training identities are disjoint from every query and candidate identity. Validation and test query handles and subjects are disjoint.
- Gallery sizes are strictly increasing. The largest size must equal the candidate-frame size. Each draw shuffles the same sorted candidate frame with the declared seed schedule; smaller galleries are prefixes of that draw's permutation and are therefore nested. Each draw has the same gallery membership across validation and test.
- Scores are assumed invariant to gallery composition. This is appropriate for pairwise candidate scorers but not for rankers whose score changes with the candidate pool; that is a separate input contract, not silently assumed to work.
- The report includes per-size FNIR at ranks 1/3/5/10, FPIR, per-draw outcomes, per-size validation-locked thresholds, a frozen-reference-threshold scaling curve, and crossed query-subject/gallery-draw percentile-bootstrap intervals.

**Important qualification boundary:** a finite-grid coverage diagnostic is now committed at `coverage/gallery_crossed_bootstrap.py`. It reproduced severe sparse-event undercoverage in the crossed percentile bootstrap: 48.4% observed 95% interval coverage at a true 0.1% event rate (250 synthetic studies; Monte Carlo Wilson 95% interval 42.28–54.57%), and 84.4% coverage in a smaller crossed clustered case. Moderate/dense scenarios did better, but this does not override the sparse-tail failure. Report status is `KNOWN_SPARSE_EVENT_UNDERCOVERAGE_DO_NOT_USE_FOR_CERTIFICATION`; these intervals are exploratory and blocked from certification use. Separately, each realized gallery draw now reports a one-sided exact Clopper–Pearson upper bound on the proportion of non-mated test subjects with any false identification in that draw. Its alpha is Bonferroni-adjusted across the whole report as 0.05 / (gallery-size count × gallery-draw count × two threshold policies), so the per-draw bounds target at least 95% simultaneous coverage under their shared assumptions. That bound is conditional on the realized gallery and scorer, assumes independent representative query-subject clusters, and does not cover uncertainty in the candidate sampling frame; it does not rehabilitate the crossed-bootstrap interval. They are conditional on the submitted candidate frame and do not establish transportability to an unobserved population. The included fixture is a deliberately easy synthetic positive control: it is a test of joins, sampling, locking and reporting, not evidence of a real attack or mitigation.

The evaluator remains fail-closed about the overall release claim: task utility and official conformance evidence are absent, and the overall gate remains NOT_EVALUATED_CROSSED_GALLERY_EVIDENCE_IS_NOT_RELEASE_CERTIFICATION. The gallery protocol preregistration remains the governing research contract; see GALLERY-SCALING-BENCHMARK.md.
