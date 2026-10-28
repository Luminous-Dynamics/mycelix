# LIFE Privacy Evaluation: held-out linkability metrics and paired mitigation evidence v0.3

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
cargo run --locked --release -- \\
  fixtures/truth.tsv \\
  fixtures/scores.tsv \\
  fixtures/training-subjects.txt \\
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

The machine-readable `threat-coverage-matrix.json` maps the W3C Verifiable Credentials Data Model Threat Model v2.1 draft to observer capabilities, existing scenario IDs, minimum evidence, and explicit claim ceilings. The workflow checks its threat/profile counts, uniqueness, status vocabulary, and scenario references, then binds its SHA-256 digest into the qualification receipt. This protects the **coverage contract** from silently drifting away from the scenario catalogue; it does not establish that every mapped channel is implemented or tested.

The matrix separates six attacker profiles: no-signal control, deterministic channel matching, adaptive tabular attacker, longitudinal attacker, colluding parties, and network/device telemetry. The last three remain proposed or deployment-dependent unless their exact artifacts and receipts are present. It also explicitly routes replay/context misuse, wallet trust, and post-presentation retention to separate security or governance evidence instead of mislabeling them as linkability metrics.

Standards context: the W3C threat model is a Group Note Draft and must be treated as an evolving threat taxonomy; RFC 9901 is the IETF Standards Track SD-JWT specification; BBS derived-proof properties do not remove every metadata or issuer-key correlation channel. OpenID4VP/OpenID4VCI + HAIP conformance remains a separate official test lane. The evaluator's synthetic smoke controls cannot be presented as credential-format measurements, deployment privacy proof, or a SOTA result.

Release-scale priorities remain: matched real-format implementations and task utility; a preregistered attacker ladder with subject-disjoint splits; top-k retrieval and effective-anonymity-set reporting where retrieval is defined; worst-stratum analysis; longitudinal/collusion traces; validation-locked low-FPR evaluation with rare-event uncertainty; and payload, p50/p95/p99 latency, memory and measured-energy frontier. The 10,000-pair / 2,000-subject floors are screening floors, not sufficient evidence or an automatic pass.
