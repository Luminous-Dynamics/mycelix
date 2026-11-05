# Evidence-Qualified Accounting and Clarification Benchmark v1.0.0 — S0

This package starts [FIN-TRUST-BENCH-001 / #5857](https://github.com/Luminous-Dynamics/mycelix/issues/5857). It is a public synthetic benchmark for source-bound accounting reconstruction, clarification, abstention and escalation. It is not professional accounting advice, tax advice, a ledger, or authority to post entries.

## Why a separate benchmark is needed

Two contemporary external benchmarks expose complementary weaknesses:

- The current APEX-Accounting leaderboard page describes 100 tasks across ten worlds and 1,475 criteria. At the 2026-11-05 19:39 SAST snapshot, it showed a leading display of 71.5% ±4.8. Its methodology page lists 100 tasks and k=4; the live page explains eight attempts for Pass@1, Pass@8 and Pass^8, while canonical Mean Criteria uses k=4.
- The published APEX-Accounting paper (29 July 2026) describes a separate 160-task held-out cohort across ten worlds, reports 56.4% Mean Criteria@3 for its leading evaluated model, best Pass@8 of 21.5%, and no model above 2.6% Pass^8. The paper explicitly excludes tasks requiring human-in-the-loop clarification and identifies asking a clarifying question rather than assuming as a missing professional skill. It also excludes tax, audit, consolidation, multi-entity, multi-currency and external reporting.
- FINBALANCE (Findings of EMNLP 2026) reports 46% maximum exact final balance-sheet accuracy on its 710-record evaluation split. Reconstructing statements from journal entries improved four models by 26–41 percentage points; the paper notes that plausible entries can fail supporting-document linkage and aggregation.

The live APEX 100-task page and published 160-task paper cohort must not be treated as one comparable denominator or score series. Full timestamped observations are in [external benchmark evidence PR #5853](https://github.com/Luminous-Dynamics/mycelix/pull/5853).

Sources:
- APEX current leaderboard: https://www.mercor.com/apex/apex-accounting-leaderboard/
- APEX methodology: https://www.mercor.com/apex/methodology/
- APEX paper: https://arxiv.org/abs/2607.27189
- FINBALANCE paper: https://aclanthology.org/2026.findings-emnlp.675/

## Paradigm: ask before guessing; prove before delivering

The unit of success is not a plausible answer. It is a **source-bound, period-correct, entity-correct and reproducible accounting disposition**.

A case can end in only one of four states:

- `deliver`: evidence needed for the requested output is present and internally consistent;
- `clarify`: a bounded, decision-relevant fact is missing or contradictory and can be obtained;
- `abstain`: source integrity or knowledge-frontier integrity is broken, so downstream work must stop;
- `escalate`: the evidence conflict requires a responsible professional or system owner rather than an automated guess.

Clarification is a capability, not a failure. A question is useful only when it targets a fact that changes recognition, measurement, classification, period, entity, currency, account mapping or authority. Redundant or vague questions count against the agent.

A delivered journal line must be proof-carrying:
- stable source document identity, namespace and revision;
- precise source locator (page, table, row, sheet or cell);
- typed source fact and exact value;
- entity, currency, period and accounting policy/profile version;
- source evidence recorded no later than the requested knowledge frontier;
- explicit transformation lineage and uncertainty;
- double-entry equality and statement reconstruction checks.

A correct amount with no source binding is not a correct journal. A balanced journal with the wrong entity, currency or period is not correct either. The oracle is separate from the task bundle; candidates do not receive expected dispositions, questions, journal lines or statement values.

## S0 case coverage

The expanded public S0 contains **17 synthetic tasks and 17 oracle rows**:

1. Complete USD invoice and supported service period — deliver.
2. Unknown service start date — clarify.
3. Conflicting reporting-entity evidence — clarify.
4. Duplicate invoice source identity/revision — deduplicate and deliver once.
5. EUR invoice without approved FX rate — clarify.
6. Credit note whose original invoice is absent from the source set — escalate.
7. Acceptance fact recorded after the requested knowledge frontier — exclude and clarify.
8. Service spanning October and November — allocate 170,000 minor units to October and 140,000 to November from a 310,000 total, with separate entries and statements.
9. Synthetic source-integrity mismatch — abstain.
10. Invoice and purchase-order amount conflict — escalate.
11. Unknown chart-of-accounts mapping — clarify.
12. Invoice received after the knowledge frontier — exclude, do not backdate, and clarify.
13. Disputed supplier identity — clarify using the authoritative supplier record.
14. Invalidated service end date — clarify rather than infer.
15. Same external document ID in two namespaces — preserve both source identities and deliver once.
16. Credit-note parent invoice received after the frontier — escalate; later receipt cannot retroactively resolve the earlier assessment.
17. Service start after service end — escalate instead of guessing which source date is wrong.

Expected dispositions: three deliver, seven clarify, one abstain and six escalate. The 17 cases contain 25 source documents, 58 facts, five expected journal entries and five expected statement lines. The values are synthetic; no real client or book is represented. Document integrity flags are fixture controls, not cryptographic measurements of live source bytes.

## Deterministic validator

From the `rust-resolver` directory:

```sh
cargo test --locked --bin validate_evidence_qualified_accounting_s0
cargo run --locked --bin validate_evidence_qualified_accounting_s0
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

The validator:
- validates candidate and oracle Draft 2020-12 schemas;
- checks unique document, fact, journal-entry, line and statement identities;
- checks tenant/source document identity and duplicate-link rules while preserving same IDs in different namespaces;
- resolves every fact to a source document;
- enforces valid-time overlap with the accounting period and excludes facts/documents recorded or received after `knowledge_at`;
- derives deterministic disposition/reason and compares every oracle row;
- refuses to use a future-received parent invoice to resolve a credit note;
- verifies each journal entry independently, so unbalanced entries cannot cancel one another in aggregate;
- reconstructs statement lines using each account's declared normal balance and linked journal entries;
- checks statement/journal/invoice totals and exact source-fact binding;
- requires non-deliver cases to have no expected journal or statement output;
- fingerprints the exact candidate and oracle bytes with SHA-256;
- guarantees that the benchmark never posts to a live ledger or performs another external side effect.

Twenty unit tests are authored, including future-received invoices, future FX facts, future parent invoices, disputed supplier identity, invalidated service dates, namespace collisions, impossible service-period intervals, per-entry balance mutation and period allocation. They are **not yet executed**.

## Benchmark plan and target gates

S0 is a synthetic smoke corpus, not an expert-adjudicated performance result.

- S0: expand to 50 synthetic tasks with deterministic generators, oracle isolation and negative controls.
- S1: at least 100 professionally reviewed cases, dual annotation, adjudication and frozen development/holdout split.
- S2: at least 250 cases plus a hidden holdout covering multiple entities, currencies, accounting frameworks, policy versions, periods and source systems.
- S3: authorized external benchmark evaluation only where access and dataset terms permit.

Hard gates remain separate from aggregate score:
- 100% source-locator binding for material journal lines;
- zero unbalanced journal entries;
- zero silent entity, currency, sign or period coercions;
- 100% clarify/abstain/escalate on seeded decision-critical missing, disputed, future-recorded, invalidated or integrity-failed facts;
- 100% deterministic replay for exact source revisions, policy versions, code and knowledge frontiers;
- zero duplicate booking from duplicate source documents and zero namespace identity collapse;
- zero statement mismatch against the journal-derived control;
- zero live ledger posting or external side effect by the benchmark validator.

Proposed S1/S2 quality goals (not achieved results):
- >90% exact statement accuracy on S1 and >95% on S2 after freezing the rubric;
- >=95% of clarification questions decision-relevant;
- <=5% redundant clarification questions;
- >=90% of solvable ambiguity resolved with the minimal adjudicated question set;
- >=30% reviewer-time reduction without increased material errors.

Report Pass@1, Pass@k and Pass^k only with explicit attempt semantics; exact statement accuracy; source-binding completeness; journal balance; clarification precision/recall; abstention quality; escalation rate; per-case failure causes; p50/p95 latency; tool calls; retries; fully loaded cost; reviewer time; and confidence intervals. Hard false-qualification/false-assurance failures must never be averaged away.

## Matched comparison design

On the same rights-cleared cases and evidence snapshots, compare:
1. model-only;
2. model with document/spreadsheet tools;
3. deterministic accounting rules only;
4. model + tools + ledger-reconstruction feedback;
5. typed source-bound economic-event graph;
6. typed graph + independent verifier + temporal replay;
7. Symthaea candidate analyst + Mycelix verifier;
8. qualified accounting professional on a blinded, stratified subset.

Hold model/version, prompt, tools, retry count, budgets, source revisions and evaluator version fixed. Expert adjudication is required for material accounting conclusions; an LLM judge may assist but must not be the sole authority for exact balances, source binding or legal sufficiency.

APEX's paper-reported 56.4% Mean Criteria@3 and the current live page's 71.5% display are separate cohort snapshots. Mycelix must reproduce an authorized matching cohort/harness or report only its own benchmark track. FINBALANCE's 46% exact statement accuracy is a separate task family. Do not combine these numbers into one SOTA percentage.

## Claim ceiling and next milestone

This S0 package tests whether a small deterministic evaluator can preserve source refs, enforce bitemporal evidence, balance synthetic journals, reconstruct synthetic statements and select safe clarification/abstention states against a public oracle. It does not show that a model can perform these tasks, that expert reviewers agree, that accounting policy is correct or that any legal/tax conclusion is valid.

Next: expand to 50 adversarial tasks, dual-annotate at S1, and implement an agent-output evaluator that scores questions and deliverables independently of the public oracle. Keep task inputs and oracle data physically separated in every future runner.
