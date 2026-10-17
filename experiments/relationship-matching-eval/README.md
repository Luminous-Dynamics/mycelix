# Relationship Matching Evaluation Harness

**Status:** exploratory research tool, not product or production matching code.

This isolated, dependency-free Rust crate implements an initial offline slice proposed in the public Mycelix RFC at PR #5127. The current source has not yet been compiled or tested on its exact branch head. It generates seeded synthetic adult-only participants, applies hard eligibility constraints before ranking, compares four policies—feasibility-only pair ordering, one-sided utility, pairwise mutual utility, and an exposure-balanced mutual-utility allocator—and emits a deterministic JSON report.

## Run

Use a Rust 2021 toolchain. From this directory:

- **Tests:** cargo test
- **Baseline report across a 30-seed suite:** cargo run -- 20261017 200 5 30
- **Eight-scenario robustness report:** cargo run --bin robustness_sweep -- 20261017 200 5 30
- **Eight-scenario temporal exposure report (10 rounds by default):** cargo run --bin temporal_sweep -- 20261017 200 5 30 10
- **Quality/fairness frontier (70%, 80%, 90% mutual-utility floor):** cargo run --bin quality_frontier -- 20261017 200 5 30 10

Arguments are seed-start, even participant count (4..=2000), top-k recommendations per participant, and seed count (1..=1000; default 30). The harness evaluates consecutive seeds, emits per-seed corpus fingerprints and policy metrics, then reports means, sample standard deviations, ranges, normal-approximation 95% intervals where at least two seeds are present, and per-seed relative exposure-Gini reduction. Defaults are seed-start 20261017, 200 participants, top-k 5, and 30 seeds. A one-seed run is possible with a seed count of 1 but emits no confidence interval.

## Multi-scenario robustness sweep

The added `robustness_sweep` binary evaluates the same four policies against eight seeded synthetic scenario families. From this directory:

- `cargo run --bin robustness_sweep -- 20261017 200 5 30`
- Arguments are seed-start, even participant count (4..=2000), top-k, and seed count (1..=1000).
- Each scenario/seed combination receives a corpus fingerprint and per-policy metrics. The report includes scenario-level means, sample standard deviations, ranges, normal-approximation intervals, and paired per-seed Gini reductions versus one-sided utility.
- The binary aborts if any policy emits an invalid recommendation row. This is a structural gate only; it does not establish that a fairness target is met.

Scenario families:

| Scenario | Stress dimension |
|---|---|
| `baseline` | Original generator, retained for comparability |
| `sparse_market` | Few interests drawn from a larger tag universe; tests low candidate degree and disconnected pockets |
| `dense_market` | Many interests and lower utility floors; tests high candidate density |
| `opt_out_heavy` | Higher inactivity and discovery opt-out rates |
| `skewed_goals` | Different connection-goal distributions across the two abstract partitions |
| `concentrated_preferences` | Deliberately homogeneous fully feasible market; ties expose identifier-order popularity concentration |
| `adversarial_blocking` | Coordinated blocking against two candidate profiles; blocks are respected, never relaxed |
| `cold_start` | A subset of participants has only a rare, one-interest profile |

All families are synthetic stress fixtures, not representative estimates of people or dating markets. The concentration fixture is an intentional algorithmic negative control: a deterministic ranker that creates extreme exposure concentration should be caught by metrics, not allowed to "pass" because the data generator is too convenient.

The same seed value is used across scenario families, but each scenario applies a deterministic transformation and therefore has its own corpus fingerprint. Fingerprints remain non-cryptographic and are not provenance/security receipts. Bind any published result to an exact source SHA and compiler version.

## Multi-round exposure and historical allocation

The `temporal_sweep` binary evaluates whether a fairness improvement survives repeated recommendation batches rather than resetting exposure counts each time:

- `one_sided_repeated`: re-run one-sided utility every round.
- `batch_balanced_repeated`: re-run exposure balancing with an empty exposure ledger each round.
- `historical_exposure_balanced`: seed each new batch with the accumulated incoming exposure counts from previous rounds.

Example:
```sh
cargo run --bin temporal_sweep -- 20261017 200 5 30 10
```

Arguments are seed-start, even participant count, top-k, seed count, and round count. It reports cumulative coverage, raw and opportunity-normalized exposure Gini, paired raw-Gini reductions, per-seed fingerprints, and optional source/toolchain metadata. The history API rejects unknown participant IDs and counter overflow. The accumulator updates atomically on overflow failure.

**Interpretation limit:** participants and preferences remain static across rounds in this first temporal experiment. It demonstrates the difference between per-batch and cumulative allocation on a fixed market; it does not model join/leave events, preference drift, profile edits, changing opt-in, or a distributed durable ledger. Long-horizon production qualification must add those events and test history expiry, purpose limitation, deletion, and recovery.

## Quality–fairness Pareto frontier

The `quality_frontier` binary compares a small, explicit quality constraint with exposure allocation across the same eight market families and multiple repeated rounds:

- One-sided declared-utility ranking.
- Pairwise mutual-utility ranking.
- Batch-reset exposure balancing.
- Historical exposure balancing normalized by current feasible-counterpart count.
- Historical exposure balancing restricted to candidates whose mutual-utility floor is at least 70%, 80%, or 90% of the viewer's best feasible mutual-utility floor.

Command:
```sh
cargo run --bin quality_frontier -- 20261017 200 5 30 10
```

The report pairs cumulative raw exposure Gini on a common hard-feasibility population with a separate policy-qualified opportunity report. The qualified denominator counts only directed viewer-to-candidate edges that satisfy the same quality floor as the policy; candidate coverage and opportunity-normalized Gini are then computed over that qualified opportunity set. These two denominators answer different questions and must not be substituted for each other. The report also includes a `relative_mutual_floor_retention` proxy: the selected pair's minimum directed synthetic utility divided by the viewer's best feasible pair's minimum directed utility. It is a synthetic ranking proxy, **not** a human acceptance rate, compatibility diagnosis, or relationship-quality measure. The quality thresholds are research controls, not recommended product settings. Review the scenario-level Pareto frontier: lower exposure inequality is only useful if the resulting trade-off in mutual utility is acceptable.

The exposure-balancing allocator now compares cumulative exposure per currently feasible counterpart using exact rational comparison rather than raw exposure counts alone. This reduces a specific distortion when two candidates have materially different opportunity degrees. It is still a greedy allocator, not a globally optimal matching algorithm, and the current-history normalization assumes a static feasible graph within a run. A changing population requires carefully defined historical opportunity accounting, privacy scope and retention rules.

## What the harness is designed to test

- All policies use the same eligibility predicate before ranking or allocation.
- `discovery_opt_in` is modeled separately from `active`. Opt-in authorizes consideration for discovery only; it is not consent to a specific introduction.
- The feasibility-only policy deliberately ignores interest utility after constraints pass; its stable pair-priority function is pseudo-random-looking but not cryptographic randomness.
- The exposure-balanced policy allocates one candidate at a time to each opted-in viewer and uses incoming exposure counts as its primary selection objective; it is a simple benchmark baseline, not a globally optimal matching algorithm.
- Inactive users, mismatched connection goals, incompatible age ranges, mutual blocks, insufficient shared interests, and insufficient declared utility are filtered before ranking.
- Output ordering is deterministic for fixed input and policy.
- Coverage and raw exposure Gini are measured over participants who have at least one feasible counterpart; opportunity-normalized exposure Gini divides a candidate's incoming exposure by their number of feasible counterparts before measuring inequality. The two metrics answer different questions and are reported together.
- The report includes a canonical synthetic-corpus FNV-1a 64-bit fingerprint per seed so same-input regressions are visible. The fingerprint is non-cryptographic and cannot serve as a security or provenance receipt. To bind a report to source and toolchain metadata, run with `GIT_COMMIT_SHA` set to the exact commit SHA and `RUSTC_VERSION` set to the toolchain version; absent values are emitted as `null`.

## What it does not prove

- It does not run a stable-marriage, maximum-cardinality, or globally optimal exposure-fair matching algorithm.
- It does not model pair-specific consent. A later product protocol must record an explicit two-sided acceptance/decline/withdraw state before revealing mutual interest or enabling communication.
- A mutual shortlist is not a human acceptance, date, successful relationship, or compatibility verdict.
- Synthetic interest weights are not psychological traits and are not suitable for production profiles.
- The FNV-1a fingerprint is not cryptographic evidence or a provenance/security receipt.
- It does not test real-user fairness, safety classifier quality, privacy controls, Holochain integration, messaging, or UI behavior. Normal-approximation confidence intervals are exploratory diagnostics and do not replace a preregistered statistical analysis plan.

This harness deliberately includes no personal data, network access, model dependency, image ranking, public reputation score, or external crate dependency. Do not connect it to live profile or message data. A later qualification step should replace the non-cryptographic fingerprint with the repository's approved content-digest format and bind all results to exact code/environment evidence.
