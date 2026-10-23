# LIFE-RELATIONAL-003 — Stronger calibrated relational attackers

## Question

Is a relational privacy mitigation robust against more than a majority-vote attacker? This research slice compares simple relational rules with two model families while keeping target-protected values outside attacker-visible input.

This is a synthetic benchmark. It evaluates empirical attack performance and probability calibration; it does not establish differential privacy, anonymity, real-world group privacy or production safety.

## Train / calibration / test protocol

Each frozen configuration generates independent communities for three disjoint splits:
- **Train:** fit learned attacker parameters.
- **Calibration:** fit a two-parameter Platt map from raw attacker scores to the target protected label. Calibration labels are used only on this disjoint calibration split.
- **Test:** compute all final metrics; protected labels are joined only after raw predictions and calibrated probabilities are frozen.

The 24 train / 24 calibration / 24 test community counts, cohort sizes, peer-signal levels and five seeds are pinned in `manifest.json`. No test community may appear in train or calibration. The report retains all seed/configuration rows.

## Attacker profiles

- `population_prior`: smoothed prevalence prior.
- `exact_cohort_majority`: exact-cohort proxy fraction.
- `coarse_neighborhood_majority`: proxy fraction for a neighborhood that merges four primary cohorts.
- `overlapping_group_union_majority`: union of exact-cohort and secondary-community peers, deduplicated.
- `randomized_peer_control`: uses independent/randomized peer proxies rather than the correlated public proxy.
- `logistic_ridge`: fixed-schedule, L2-regularized logistic regression using all seven attacker-visible summary features.
- `gaussian_naive_bayes`: independent Gaussian naive Bayes model family over those same features.
- `uncorrelated_target_negative_control`: logistic model trained on an independent synthetic target secret instead of the protected label; its final attack score is evaluated against the protected label and should not beat the prior materially.

The `AttackerView` Rust type includes only IDs/group metadata and public proxy values. It excludes both the protected label and the independent evaluation secret. Features are built only from `AttackerView` values; evaluator truth is attached only after prediction. A regression test flips both hidden labels with visible views held fixed and requires identical view/features.

## Metrics

Report accuracy, balanced accuracy, class-conditional false-positive/false-negative rate, Brier score, log loss and expected calibration error (ECE, fixed 10-bin definition) for both raw and calibrated probabilities. Each row also reports selective coverage, selective accuracy among non-abstained targets and abstention count at five fixed confidence thresholds: 0.50, 0.625, 0.75, 0.875, and 0.95.

Probability columns are empirical scores on a held-out synthetic distribution. Platt scaling is fit only on the disjoint calibration communities; it does not prove probabilities are calibrated on a different real-world population. ECE depends on the declared bins and sample. Runtime and memory are not yet measured by this first slice and are not inferred from test duration.

## Why this is stronger than one attacker

A privacy guard that defeats a majority rule may remain vulnerable to another classifier or a relational partition that gives access to more peer records. The benchmark tests two supervised model families and four transparent non-learned baselines, compares the randomized-proxy control, and measures calibration / selective risk rather than reporting raw accuracy alone.

This is not an SOTA claim. A fair industry comparison still needs matched, version-pinned real SD-JWT/BBS/OpenID4VP implementations and identical task, transport, verifier/audience, nonce, cohort split and hardware conditions. Protocol conformance, attack accuracy, calibration, latency/resource cost and utility must be reported as separate axes.

## Run

~~~sh
cargo +1.96.0 fmt --manifest-path research/life-relational-attackers/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-relational-attackers/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-relational-attackers/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-relational-attackers/Cargo.toml
~~~

The dedicated workflow asserts the exact PR-head SHA, runs format/tests/strict Clippy, compares two reports byte-for-byte and uploads the report, manifest, checksums and qualification receipt.

## Proposed acceptance criteria (not results)

- Training, calibration and final test communities are pairwise disjoint.
- Protected labels and independent test secrets are absent from attacker feature types.
- Calibration is fit only on the calibration split; no test threshold/seed tuning.
- All fixed seeds and signal levels remain in the report.
- Raw/calibrated Brier, log loss, ECE, FPR/FNR and selective coverage are reported separately.
- Randomized/uncorrelated negative controls stay near the frozen prior baseline within a preregistered tolerance.
- Identical source, manifest and seed schedule produces byte-identical output.

## Research grounding

This builds on [LIFE-RELATIONAL-001 #5474 / PR #5475](https://github.com/Luminous-Dynamics/mycelix/pull/5475) and [LIFE-COMPOSITE-BUDGET-001 #5479 / PR #5480](https://github.com/Luminous-Dynamics/mycelix/pull/5480). The W3C VC Data Model 2.1 Working Draft discusses correlation through status/revocation, metadata, patterns of use and aggregation; NIST SP 800-226 is a reference for evaluating genuine DP claims, which this empirical attacker benchmark does not make.

References:
- https://www.w3.org/TR/vc-data-model-2.1/
- https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/
- https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/
- https://csrc.nist.gov/pubs/sp/800/226/final

## Claim ceiling

A passing run qualifies only the pinned synthetic distribution, feature view, model families, seed schedule and metrics. It does not establish privacy of any live Mycelix zome, differential privacy, anonymity, causal inference, or production-protocol safety.
