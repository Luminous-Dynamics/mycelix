# Proof-Carrying Reconciliation Hypotheses v1.0.0 — S0

This package starts [FIN-RECON-HYP-001 / #5880](https://github.com/Luminous-Dynamics/mycelix/issues/5880). It defines a portable contract for matching candidate groups, their evidence, allocations, alternatives, ambiguity and residuals. It does not confirm/reconcile accounting records, create journal entries, write to an ERP or post to a ledger.

## Why this is not just another fuzzy matcher

Oracle Fusion already supports one-to-one, one-to-many, many-to-one, many-to-many and zero-amount matching, with grouping/matching attributes such as amount, date, currency, transaction type, reference, counterparty and bank codes. Microsoft Dynamics 365 documents one-to-one, one-to-many and many-to-many matching, prioritized rule sets, reversal handling and configurable penny tolerances. BlackLine documents high-volume transaction matching, one-to-many/many-to-one payment application, exception routing and reconciliation audit trails.

Mycelix should not claim novelty for rule-based matching, cardinality support, fuzzy matching, weighting or exception queues.

The target paradigm is **reconciliation as a proof-carrying hypothesis, not a boolean**:
- matching candidates are distinct from allocations;
- allocations are distinct from human approval and ledger confirmation;
- each candidate is tied to exact input bytes, a versioned rule set and matcher provenance;
- every score is decomposed into criterion-level outcomes and observed values;
- one-to-many / many-to-one / many-to-many groups carry explicit pairwise allocations;
- totals are conserved in integer minor units and over-allocation is forbidden;
- alternate candidates remain present and ambiguous ties are never broken by row order;
- partial payments retain an explicit residual for review;
- currency/entity/account/direction conflicts cannot be papered over by an aggregate score;
- reversals have explicit original-transaction lineage;
- no proposal posts or writes back to a ledger.

## Artifacts

- `reconciliation_input_v1.0.0.schema.json`: strict Draft 2020-12 input bundle.
- `reconciliation_hypotheses_v1.0.0.schema.json`: output candidate/hypothesis contract.
- `reconciliation_validation_report_v1.0.0.schema.json`: structural and allocation validation report.
- `fixtures/reconciliation_inputs_s0_v1.0.0.json`: eight synthetic input runs.
- `fixtures/reconciliation_hypotheses_s0_v1.0.0.json`: eight synthetic gold-control outputs. These were authored as test fixtures; the matcher engine state is explicitly `not_executed_synthetic_fixture`, its engine digest is null, and it is not benchmark model performance.
- `../rust-resolver/src/bin/validate_reconciliation_hypotheses_s0.rs`: standalone Rust validator and report generator.

## S0 coverage

Eight runs, 21 source items, one pinned rule set and nine match hypotheses:

| Run | Shape | Expected disposition |
|---|---|---|
| RECON-001 | 1:1 exact | eligible candidate |
| RECON-002 | 1:N split settlement | eligible candidate |
| RECON-003 | N:1 grouped statement lines | eligible candidate |
| RECON-004 | N:N allocation matrix | eligible candidate |
| RECON-005 | partial settlement | review required; 3,000 minor-unit residual |
| RECON-006 | two equal-score alternatives | both ambiguous; zero allocations |
| RECON-007 | USD/EUR conflict without FX evidence | blocked; zero allocations |
| RECON-008 | reversal pair | eligible candidate with matched reversal lineage |

Static fixture audit:
- 8 runs and 21 source items;
- 9 hypotheses and 14 candidate pairs;
- 10 pairwise allocations;
- five eligible exact groups, one review-required group, two ambiguous alternatives and one blocked group;
- six explicit unmatched/residual item records;
- 53,000 minor units remain explicitly unresolved across partial, ambiguous and blocked groups;
- 100% exact source-bundle digest binding and 100% canonical rule-set digest binding;
- zero allocation overages, zero allocated source reuse, no cross-currency allocations and no silently dropped residuals.

These values describe synthetic control fixtures. No matching engine actually ran and no real ERP or bank data was processed.

## Reconciliation input contract

Each run pins:
- tenant, entity, account, period and knowledge frontier;
- source items with source side, source system, document reference and SHA-256, external identifier, amount in integer minor units, currency, direction, transaction date, received-at, reference, counterparty and transaction type;
- optional reversal-of identifier;
- rule-set reference/version, criterion weights, required-for-auto criteria, mismatch/missing actions, amount/date tolerances, auto/review thresholds, ambiguity margin and allowed cardinalities.

Every criterion's weight is in basis points and the rule-set weights sum to exactly 10,000. Rule-set digests are SHA-256 over canonical sorted-key JSON serialized without whitespace. The output also pins the exact raw input bundle bytes with SHA-256.

## Hypothesis and allocation semantics

A hypothesis contains left statement item refs, right ledger item refs, declared cardinality, full Cartesian candidate-pair set, criterion evaluations, allocations, weighted score, decision state and alternative refs.

Criterion evaluations preserve the exact observed values from source items. S0 scoring is deterministic:
- amount compares group totals in minor units against the pinned amount tolerance;
- date compares each left/right date pair against the pinned date tolerance;
- currency, entity, account, direction, reference, counterparty and transaction type use exact normalized equality;
- match criteria contribute their pinned weight to the score;
- mismatch/missing actions independently determine whether a candidate is review-required or blocked.

This weighted score is only a ranking signal. It is not a probability and has no calibration claim.

An allocation is a positive integer amount assigned between one statement item and one ledger item. The sum allocated from each left item and into each right item cannot exceed its amount. An eligible-for-auto-match candidate must allocate every item in its group exactly, satisfy every required-for-auto criterion, exceed the auto threshold, have no review/block condition and have positive source amounts.

If an item has an unallocated balance, that residual must appear in `unmatched_items`. If two hypotheses overlap on a source item and lie within the pinned ambiguity margin, both must remain `ambiguous`, reference each other and allocate nothing. Blocked and ambiguous candidates allocate nothing. A human approval, reconciliation confirmation or journal-posting state is deliberately absent from S0.

A reversal candidate can be eligible only when all its items are reversal transactions and all share the same non-empty `reversal_of_external_id`.

## Local Rust validation

From `mycelix-finance/benchmarks/global-accounting-portability-v1/rust-resolver`:

```sh
cargo test --locked --bin validate_reconciliation_hypotheses_s0
cargo run --locked --bin validate_reconciliation_hypotheses_s0
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

Optional arguments:
- `--data-dir PATH`: reconciliation package folder (default is the sibling `reconciliation-hypotheses-v1`);
- `--output PATH`: write the deterministic validation report;
- `--help`: show usage.

The validator checks Draft 2020-12 schemas, exact raw input digest, canonical rule-set digests, source scope and knowledge/period boundaries, unique source identities, rule weights and thresholds, candidate cardinality, complete candidate-pair sets, source-derived criterion values, deterministic scores, blocked/review/auto eligibility, per-item allocation limits, source-item reuse, overlapping alternatives, reversal lineage, unmatched residual completeness and engine execution-provenance state.

Twenty-two Rust tests are authored. They cover all eight fixtures; each cardinality; partial residuals; ambiguity; currency blocking; reversal lineage; over-allocation; source-item reuse; arbitrary ambiguity resolution; missing/invalid alternative links; criterion tampering; rule weights; input hash binding; missing unmatched residuals; review-action priority over high scores; and synthetic non-execution provenance. **Tests are authored but not executed.** No Cargo compile/test, rustfmt, Clippy, independent Draft 2020-12 runtime validation or CI pass is claimed.

## Industry comparison and research baselines

Official documentation:
- Oracle Fusion reconciliation matching rules: https://docs.oracle.com/en/cloud/saas/financials/26d/fairp/reconciliation-matching-rules.html
- Oracle Account Reconciliation Transaction Matching: https://docs.oracle.com/en/cloud/saas/account-reconcile-cloud/raarc/transaction_matching_about.html
- Microsoft Dynamics 365 advanced bank reconciliation: https://learn.microsoft.com/en-us/dynamics365/finance/cash-bank-management/reconcile-bank-statements-advanced-bank-reconciliation
- BlackLine transaction reconciliation: https://www.blackline.com/resources/glossaries/transaction-reconciliation/
- BlackLine account reconciliation: https://www.blackline.com/resources/glossaries/account-reconciliation/

Matched benchmark arms should compare:
1. exact reference / one-to-one rule engine;
2. prioritized date+amount+counterparty rules;
3. configurable tolerance and fuzzy weighted matching;
4. deterministic bipartite / b-matching / min-cost-flow candidate generation;
5. probabilistic candidate ranking with calibrated scores;
6. candidate generation plus independent allocation/provenance verifier;
7. Mycelix evidence graph plus candidate solver;
8. expert reconciler on a blinded subset.

Use identical, rights-cleared input snapshots and matching scope. Freeze rule-set bytes, thresholds, source revision, matcher/solver version, tie-break policy, candidate output and reviewer state. Benchmark all baselines under the same tolerances and cardinality permissions.

## Proposed S1/S2 target gates (not achieved)

Build 50 adversarial synthetic scenarios for S0.1, at least 100 dual-reviewed professional cases for S1, and at least 250 cases plus a hidden holdout for S2.

Proposed gates:
- zero false auto-eligible candidates on seeded critical controls;
- 100% allocation conservation, source bounds and entity/currency/scope integrity;
- 100% explicit residual accounting for partial allocations;
- >=99% precision for auto-eligible candidate groups before human approval;
- >=95% recall at a separately reported review threshold;
- >=95% recall of plausible alternatives;
- zero silent selections when a near-tie exists;
- >=30% reviewer-time reduction without increased material errors.

Report group-level and item-pair precision/recall/F1, allocation exactness, false eligible/blocked rates, ambiguity recall, unmatched calibration, selective risk/coverage, reviewer overrides, p50/p95 latency, fully loaded cost and human time. Keep false auto-eligibility as a hard failure, not a metric that can be averaged away.

## Claim ceiling

The current S0 validates a deterministic, synthetic hypothesis contract and example allocations. It does not establish matching accuracy, calibrated AI confidence, auditor acceptance, ERP interoperability, ledger posting safety, accounting-policy correctness or superiority over Oracle, BlackLine, SAP or Dynamics 365. Those require expert-labelled S1/S2 data and matched, independently executed systems tests.