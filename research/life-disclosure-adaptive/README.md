# Life disclosure adaptive linkability benchmark

Dependency-free Rust research harness for [LIFE-DISCLOSURE-ADAPTIVE-001 (#5416)](https://github.com/Luminous-Dynamics/mycelix/issues/5416).

## Paradigm

The original linkability baseline uses fixed exact-match rules. This crate adds a small interpretable attacker trained on feature equality channels and evaluated only on identities excluded from its training split. The model never receives subject IDs, presentation IDs, row order, or labels as input features.

## Frozen split and corpus

- 120 synthetic subjects, each presented to 3 recipients in 2 sessions per recipient.
- 80 subjects assigned to train and 40 to held-out test by deterministic hash ranking. Split occurs **before pairs are constructed**.
- Exactly 76,800 cross-recipient train pairs (960 positive same-subject pairs) and 19,200 cross-recipient test pairs (480 positive same-subject pairs).
- Each strategy has a separate model and identical train/test subject membership.
- Five profiles: stable global ID; pairwise ID with reusable proof; pairwise ID with fresh proof but stable lineage/behavior; pairwise ID with fresh proof and structural metadata removed; predicate-only claims with structural metadata removed.
- In the `metadata_minimized` and `predicate_only` profiles, statement count, disclosed-index pattern, and schema profile are explicitly absent (not merely assumed to be hidden); the feature extractor does not award an equality match for two absent values. A test locks this boundary.
- Training negatives are deterministically downsampled to 5 negatives per positive for optimization. Test metrics use the complete held-out pair set.
- The pair-feature vector includes per-claim equality, whole-claim-vector equality, identifier/proof/lineage/behavior equality, statement-count equality, disclosed-index equality, schema-profile equality, and the count-plus-index interaction.
- Features are all observable equality indicators. The internal subject labels only define pair labels and bootstrap clusters; they do not enter feature extraction or model fitting as input values.

## Attacker and evaluation

A full-batch logistic regression is fit deterministically with fixed optimizer steps, learning rate, class-balanced weights and L2 penalty. The same held-out pairs are evaluated using a declared exact-channel-union rule (predict a link if any observable equality channel matches). Because training uses class-balanced weights and negative downsampling, the logistic score is a ranking score for this experiment, **not a calibrated real-world posterior probability**. This is a small transparent reference adversary, not a general-purpose machine-learning search.

The subject-cluster bootstrap sorts each test score vector once and reuses that ordering across bootstrap replicates; each replicate recomputes only cluster weights and tie-group totals. This preserves the metric while keeping repeated report generation practical in CI.

For each strategy, the report compares three ranking/evaluation baselines:
- `balanced_logistic`: learned only from subject-disjoint training examples;
- `exact_channel_union`: deterministic rule that predicts a link when any observable equality channel matches;
- `seeded_random_control`: deterministic pseudo-random score derived from an opaque pair reference, independent of pair labels by construction.

For each baseline the report includes positive/negative held-out pair counts and TP/FP/FN/TN at score threshold 0.5; precision/recall/FPR at 0.5; rank-based ROC-AUC with ties handled as half credit; maximum TPR reachable at FPR <= 1%; and minimum FPR to reach TPR >= 95%. The random control is a chance-reference sanity check, not a privacy defense or adversary.

A 200-replicate **subject-cluster bootstrap** provides percentile intervals for both ROC-AUC and average precision. It resamples held-out subjects and weights within-subject positive pairs by one cluster multiplicity and between-subject negative pairs by the product of cluster multiplicities. This avoids pretending that overlapping pairs are independent observations. Threshold-0.5 precision/recall/FPR are descriptive only: the class-balanced fit does not produce deployment-calibrated probabilities. Prefer the fixed-FPR/FPR-at-recall operating points for policy discussion, and do not tune them on the test labels.

## Export safety

Feature and label files are separate. Files are sorted by opaque pair reference rather than generation order; the feature vectors exclude labels and all identifier/ordinal input features. Test labels are available only in a separate offline-evaluation file and must never be included in model fitting. These are synthetic fixtures only.

## Run

```sh
cargo +1.96.0 fmt --manifest-path research/life-disclosure-adaptive/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-disclosure-adaptive/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-disclosure-adaptive/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-disclosure-adaptive/Cargo.toml -- --write-corpus-dir target/life-disclosure-adaptive
sha256sum target/life-disclosure-adaptive/*
```

## Claim ceiling

No cryptographic credentials are implemented. This is not a claim of real-world attack success, anonymity failure rates, differential privacy, cryptographic unlinkability, legal compliance, standards conformance or SOTA superiority. Passing qualifies only the pinned synthetic feature model, split, logistic attacker, exact-rule comparator and cluster bootstrap implementation.
