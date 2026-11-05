# Evidence-Qualified Accounting and Clarification Benchmark v1.0.0 — S0

This package starts [FIN-TRUST-BENCH-001 / #5857](https://github.com/Luminous-Dynamics/mycelix/issues/5857). It is a synthetic, public benchmark for source-bound accounting reconstruction, clarification, abstention and escalation. It is not professional accounting advice, tax advice, a ledger, or authority to post entries.

## Why a separate benchmark is needed

Two contemporary external benchmarks expose complementary weaknesses:

- The current APEX-Accounting leaderboard page describes 100 tasks across ten worlds and 1,475 criteria. When inspected on 2026-11-05 at 19:39 SAST, it showed a highest displayed result of 71.5% ±4.8. Its methodology page also lists 100 tasks and k=4, while its live evaluation notes say agents are run eight times for Pass@1, Pass@8 and Pass^8, with Mean Criteria as the canonical k=4 score.
- The published APEX-Accounting paper (29 July 2026) describes a separate 160-task held-out cohort across ten worlds, reports 56.4% Mean Criteria@3 for its leading evaluated model, no model above 2.6% Pass^8, and 21.5% best Pass@8. The paper explicitly excludes tasks requiring human-in-the-loop clarification, and says this omits the real professional skill of asking a clarifying question rather than assuming. **The 100-task live page and 160-task paper cohort must not be treated as one comparable denominator or score series.**
- FINBALANCE (Findings of EMNLP 2026) reports 46% maximum exact final balance-sheet accuracy on its 710-record evaluation split. Reconstructing statements from journal entries improved four models by 26–41 percentage points; the paper notes that plausible entries can still fail to link supporting documents and aggregate consistently.

Sources:
- APEX current leaderboard: https://www.mercor.com/apex/apex-accounting-leaderboard/
- APEX methodology: https://www.mercor.com/apex/methodology/
- APEX paper: https://arxiv.org/abs/2607.27189
- FINBALANCE paper: https://aclanthology.org/2026.findings-emnlp.675/

## Paradigm: ask before guessing; prove before delivering

The unit of success is not a plausible answer. It is a **source-bound, period-correct, entity-correct and reproducible accounting disposition**.

A case can end in only one of four states:

- `deliver`: the evidence needed for the requested output is present and internally consistent;
- `clarify`: a bounded, decision-relevant fact is missing or contradictory and the user can resolve it;
- `abstain`: source integrity or knowledge-frontier integrity is broken, so downstream work must stop;
- `escalate`: the evidence conflict requires a responsible professional or system owner rather than an automated guess.

This makes clarification a capability, not a failure. A question is useful only when it targets a fact that changes recognition, measurement, entity, currency, classification, period or authority. Redundant or vague questions count against the agent.

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

The public S0 contains 11 synthetic tasks and 11 oracle rows:

1. Complete USD invoice with a supported entity and service period — deliver.
2. Unknown service start date — clarify.
3. Conflicting reporting-entity evidence — clarify.
4. Duplicate invoice source identity/revision — deduplicate and deliver once.
5. EUR invoice without an approved EUR/USD rate — clarify.
6. Credit note whose original invoice is absent from the source set — escalate.
7. Acceptance fact recorded after the requested knowledge frontier — exclude it and clarify.
8. Service spanning October and November — allocate 170,000 minor units to October and 140,000 to November from a 310,000 total, with separate entries and statements.
9. Synthetic source-integrity mismatch — abstain.
10. Invoice and purchase-order amount conflict — escalate.
11. Unknown chart-of-accounts mapping — clarify.

All entities, documents, account references, values, currencies, periods, source locators and evidence states are synthetic. They do not represent real clients or books. Document integrity flags are synthetic fixture controls; no claim is made that source bytes were fetched or cryptographically checked.

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
- checks tenant/source document identity and duplicate-link rules;
- resolves every fact to a source document;
- excludes facts whose `recorded_at` or source document `received_at` is later than `knowledge_at`;
- derives a deterministic disposition and primary reason from the evidence;
- compares those results with the public oracle;
- rejects oracle source refs that point to unknown, disputed, invalidated or future evidence;
- verifies debit/credit equality by entity/currency/period, exact invoice-total reconstruction, and statement-line reconstruction from linked journal entries;
- requires non-deliver cases to have no expected journal or statement output;
- guarantees that the benchmark oracle never posts to a live ledger.

The validator contains nine authored tests. They are **not yet executed**. JSON parsing and static authoring checks do not equal a Draft 2020-12 runtime pass, Rust compile, Cargo test, rustfmt, Clippy or CI pass. No such pass is claimed.

## Benchmark plan and target gates

S0 is only the first synthetic smoke corpus. It is not expert-adjudicated and cannot establish superiority.

- S0: 50 synthetic tasks, public oracles, seeded contradictions and deterministic negative controls.
- S1: at least 100 expert-reviewed cases, two independent annotations, adjudication and a frozen development/holdout split.
- S2: at least 250 cases plus a hidden holdout covering multiple entities, currencies, accounting frameworks, policy versions, periods and source systems.
- S3: authorized external benchmark evaluation only where access and dataset terms permit.

Hard gates remain separate from any aggregate score:
- 100% source-locator binding for material journal lines;
- zero unbalanced journal entries;
- zero silent entity, currency, sign or period coercions;
- 100% clarify/abstain/escalate on seeded decision-critical missing, disputed, future-recorded or integrity-failed facts;
- 100% deterministic replay for identical source revisions, policy versions, code and knowledge frontiers;
- zero duplicate booking from duplicate source documents;
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

APEX's paper-reported 56.4% Mean Criteria@3 and its current live page's 71.5% score are separate cohort snapshots, not a valid direct comparison. Mycelix must reproduce a permitted matching cohort/harness or report only its own benchmark track. FINBALANCE's 46% exact statement accuracy is a separate task family. Do not combine these numbers into one SOTA percentage.

## Claim ceiling and next milestone

This S0 package tests whether a small deterministic evaluator can preserve source references, enforce a knowledge frontier, balance synthetic journals, reconstruct synthetic statement lines and select safe clarification/abstention states against a public oracle. It does not show that a model can perform these tasks, that expert reviewers agree, that accounting policy is correct, or that any legal/tax conclusion is valid.

Next: expand to 50 adversarial tasks, dual-annotate at S1, and implement an agent-output evaluator that scores the questions and deliverables independently of the public oracle. Keep task inputs and oracle data physically separated in every future runner.
