# FIN-JOURNAL-SET-001 — Period journal-set completeness and roll-forward v1

**Tracking:** [#5881](https://github.com/Luminous-Dynamics/mycelix/issues/5881)  
**Per-entry authority:** [#1123](https://github.com/Luminous-Dynamics/mycelix/issues/1123) / [PR #5820](https://github.com/Luminous-Dynamics/mycelix/pull/5820)  
**Population authority:** [#5790](https://github.com/Luminous-Dynamics/mycelix/issues/5790)  
**Close DAG owner:** [#5803](https://github.com/Luminous-Dynamics/mycelix/issues/5803)

## 1. Research thesis

An individually balanced journal is not a complete set of books. A trial balance can balance after:
- a complete source event and its journal are omitted together;
- an extra balanced journal is added;
- two balanced entries duplicate the same economic event;
- an unauthorized entry is posted to a second entity;
- a correction removes an original entry instead of appending a new one;
- opening balances do not match the prior period close;
- a source/policy/account frontier changes after a close was frozen.

The set theorem is therefore:

**A period journal set is complete only relative to an independently qualified expected-event population, an exact entity/ledger/profile scope, and one authorized terminal treatment for every expected event.**

The verifier does not decide whether the upstream population is really complete. It consumes the explicit qualification state, commitment and event list from #5790. It does not recreate per-entry qualification from #1123, source authenticity, account ownership, legal validity or accounting-policy judgments.

## 2. Owner composition

| Layer | Owner | Input to this verifier |
|---|---|---|
| Source event inventory and expected population | #5790 / #5801 | Qualified population commitment and event references |
| Individual journal qualification | #1123 / #5820 | Canonical journal identity, source event, entity, policy and postings |
| Currency/unit profiles | #5814 / #5829 | Exact denomination and unit profile references |
| Bank statement continuity | #5827 / #5828 | Source-specific events, not general ledger authority |
| Procedure/evidence stand-back | #5798 / #5838 / #5841 | Evidence/procedure dispositions outside this ledger theorem |
| Close DAG and stale propagation | #5803 / #5805 | Orchestration and downstream workpaper invalidation |
| Benchmark/evaluator isolation | #5782 / #5799 | Task and private-evaluator composition |

## 3. Candidate input contract

The v1 manifest binds:
- reporting entity, ledger, accounting profile and denomination;
- period boundaries, close cutoff and knowledge frontier;
- source-population state, frontier ID, expected/closed commitments and expected event references;
- exactly one event treatment per expected event;
- journal IDs, synthetic payload commitments, event bindings, entity/ledger/policy/profile/period, effective/booking/value dates and correction/reversal links;
- pinned account normal-side/unit profiles;
- predecessor close, opening balances and closing balances.

Money remains exact canonical integer minor-unit strings. There are no floats or implicit scale conversions. Fixture digest tokens are synthetic commitments, not SHA-256 hashes of real journal bytes.

## 4. Terminal dispositions

- `complete_exact_population_and_rollforward`
- `complete_with_authorized_nonposting`
- `incomplete_expected_population`
- `missing_journal_projection`
- `orphan_or_unauthorized_journal`
- `duplicate_economic_posting`
- `conflicting_journal_identity`
- `account_rollforward_conflict`
- `period_boundary_conflict`
- `unsupported_profile_or_unit`
- `source_frontier_unqualified`
- `stale_close_frontier`
- `indeterminate_requires_review`

A balanced trial balance is a subordinate invariant. It never overrides missing source events, orphan journals, wrong entity/unit, source-frontier uncertainty, duplicates or stale close state.

## 5. v1.0.0 public synthetic corpus

- Candidate input schema: `FIN_JOURNAL_SET_INPUT_V1.schema.json`
- 25 candidate-visible cases: `fin_journal_set_candidates_v1.0.0.json`
- Public oracle: `public-journal-set-oracle/fin_journal_set_oracle_v1.0.0.json`
- Oracle schema: `public-journal-set-oracle/fin_journal_set_oracle_v1.schema.json`
- Rust verifier: `journal-set-verifier/`

The cases cover expected-event omission, balanced missing/extra entry, duplicate event projection, orphan posting, exact replay, changed identity, population not complete, authorized non-posting, unresolved events, cross-entity and unit substitution, minor-unit mismatch, predecessor/opening conflict, missing predecessor close, late booking and separate booking/value dates, append-only reversals, over-reversal, missing correction original, post-close staleness, account-map mismatch, changed source frontier and order invariance.

The public oracle is for repeatable regression testing. It is **not** a blind holdout and must not be mounted in candidate environments for model benchmarking.

## 6. Semantic controls in the Rust verifier

1. Exact journal ID plus identical JSON semantics is idempotent replay.
2. Same journal ID with changed digest or any changed semantic field is a conflict.
3. Each economic event has one terminal treatment. Required posting events need exactly one journal.
4. Authorized non-posting requires both authorization and reason references.
5. Every journal event maps to the expected source event population.
6. Journal entity, ledger, accounting profile, denomination and period must match the close scope.
7. Every journal balances exactly in minor units; every posting uses an admitted account and unit profile.
8. Reversals must target an existing original and use opposite-side postings bounded by original posting amounts. Corrections cannot erase original records.
9. An effective time or booking/receipt time after the frozen cutoff needs an explicit post-close authorization; otherwise the close frontier is stale.
10. Prior close account balances must exactly equal current opening balances.
11. The verifier independently reconstructs account balances using the declared normal side and checks the closing trial balance.
12. Equivalent input set permutations must preserve semantic disposition.

## 7. Qualification and hard gates

- zero false-complete results;
- all expected events have one explicit terminal treatment;
- zero accepted orphan/unauthorized journals;
- zero accepted duplicate economic postings;
- exact debit/credit balance for every journal;
- exact account roll-forward and predecessor/opening continuity;
- zero unqualified or changed source frontiers promoted to complete;
- zero historical journal overwrite through correction/reversal;
- deterministic set permutation behavior.

The current verifier consumes upstream completeness qualification; it does not independently establish real bank/accounting feed completeness. A future composed qualification needs a live #5790 receipt and the #1123 canonical journal objects, each bound to an exact frontier.

## 8. Benchmark and industry research snapshot — 6 November 2026

FinancialAuditBench (FAB), announced 30 September 2026, is an adjacent long-horizon audit benchmark. Its repository is MIT-licensed and its paper contains 90 tasks (workpaper completion and review) across six synthetic engagement binders, with average 179 files per engagement. The owner reports that no model strictly passed more than 69% of procedures; strict pass rates were 8% on revenue and 4% on receivables tasks. Strict pass requires every applicable rubric check to be met; this is not an exact journal-set score.

- Owner overview: https://modusalliance.com/insights/introducing-financial-audit-bench/
- Code and MIT license: https://github.com/modus-audit/financial-audit-bench
- Paper: https://arxiv.org/abs/2609.32835
- Dataset: https://huggingface.co/datasets/modusaudit/FinancialAuditBench

AgenticFinBench (3 November 2026) contains 230 real-world financial tasks across eight categories, two languages and four complexity levels. Its paper reports hallucination auditing reduced the baseline leader's score by 7.58 points and moved it to fourth place; it also attributes more variation in evidence grounding to harness differences (28.6%) than model differences (8.8%). It evaluates broad financial agent responses, not period journal completeness.

- Paper: https://arxiv.org/abs/2611.04813
- Project: https://nashnovaresearch.github.io/AgenticFinBench/

Do not combine FAB, AgenticFinBench, FINBALANCE, DualBench100 or APEX into one composite. They test different tasks, graders and claim types.

### Proposed targets (ambitions, not measured results)

- 100% disposition parity on this public synthetic corpus.
- Zero critical false-complete outcomes on a hidden, professionally adjudicated period-journal set.
- 100% detection of seeded omitted expected events, extra journals, duplicate events, cross-entity/profile substitution and one-minor-unit roll-forward errors.
- A paired gain of at least 15 percentage points on the preregistered hard population-integrity subset over the strongest matched model/tool baseline.
- Measure cost per correctly qualified entity-period, reviewer minutes, exception leakage, p50/p95/p99 latency and repeated-run consistency separately.

## 9. Claim ceiling

A pass establishes only the declared public synthetic period-population, journal-set and roll-forward behavior. It does not prove real-world source completeness, valuation truth, authorization, legal validity, tax/IFRS/GAAP compliance, audit sufficiency, complete real books, production readiness or an authorized close signoff.


### Post-close authorization and timestamp ordering

A post-close journal is admitted only when both the authorization boolean and the explicit post_close_authorization_ref are present. All ordered timestamp fields use fixed-width UTC whole-second form (YYYY-MM-DDTHH:MM:SSZ); offset-bearing or fractional-second forms are excluded from this v1 contract. This keeps timestamp comparison deterministic and prevents lexical misordering.
