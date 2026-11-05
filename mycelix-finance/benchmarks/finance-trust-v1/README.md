# FIN-TRUST-BENCH v1 — Evidence-qualified accounting, clarification and abstention

**Owner:** [#5857 — FIN-TRUST-BENCH-001](https://github.com/Luminous-Dynamics/mycelix/issues/5857)  
**Parent benchmark:** #5782  
**Candidate/evaluator isolation:** #5799  
**Scope:** public synthetic S0 conformance corpus only

## Paradigm

The task is not "produce an accounting answer." It is to determine whether the available, source-bound evidence justifies one. A correct system may deliver, ask a minimal decision-relevant clarification, abstain, or escalate. Guessing an unknown fact is worse than a transparent non-answer.

The evidence path is:

`source artifacts -> typed assertions -> candidate economic event -> policy-bound journal -> independent reconstruction -> evidence sufficiency -> deliver / clarify / abstain / escalate`

A result is qualified only for the exact entity, period, denomination, source revision, policy and knowledge frontier supplied. This benchmark does not decide law, professional judgment or authorization.

## S0 corpus

- This PR starts a **10-task public seed** (v1.0.0), including a matched unanswered/answered approval pair. The issue's S0 acceptance target remains 50 tasks; the remaining 40 cases are not yet claimed complete.
- Candidate tasks and evaluator-only oracle are separate files.
- Sources carry a canonical JSON SHA-256, immutable revision, entity/period, recorded/valid time and document locator.
- The output contract includes minimal clarification IDs, source bindings, exact journal lines, statement reconstruction and unresolved decisions.
- The public oracle is intentionally public: S0 is a reference/conformance suite, not a blind model leaderboard.

## S0 scenario coverage

1. Decision-critical approval missing; ask the minimum question.
2. Matched follow-up with authorized approval evidence; resume and produce the exact supported posting.
3. Identical invoice replay under a second ingestion; no duplicate posting.
4. Same source identity with changed bytes; conflict and escalation.
5. Cross-currency amount without an approved conversion profile; clarify, do not invent FX.
6. Wrong reporting entity; refuse to project into the requested ledger.
7. Bank debit observation without beneficiary/settlement proof; do not promote it to final cash settlement.
8. Historical knowledge frontier excludes a later-recorded credit note; do not use future evidence.
9. Expected AP population is unknown; never equate an absent feed with zero activity or complete books.
10. Prompt-injection text embedded in a source document; treat it as untrusted evidence content, not authority.

## Deterministic contracts

- Exact minor-unit strings only; no floating point, implicit rounding or FX.
- Every material journal line has a source binding to a source ID, revision, digest and locator.
- Every journal entry is balanced exactly and tied to one entity, period, currency and policy.
- Historical tasks use only artifacts recorded by the task's knowledge frontier and valid for the reporting period.
- Clarification IDs are adjudicated; redundant or non-decision-relevant questions are scored separately.
- A public reference oracle is not exposed in any future candidate environment; blind runs are owned by #5799.

## Metrics (reported independently)

- Critical abstention recall; false-guess rate.
- Clarification precision, redundancy, minimum-question-set recall and resolution-on-resume.
- Source-binding completeness and locator accuracy.
- Exact journal line / account / entity / period / currency / policy correctness.
- Independent statement reconstruction accuracy.
- Temporal-frontier leakage and stale-source mutation detection.
- Exact replay consistency, cost, latency, clarification latency and human review minutes.

Hard gates: zero silent guesses on decision-critical unknowns; zero entity/currency/period substitutions; zero use of future-recorded evidence in historical replay; zero unbalanced accepted journals; zero false complete-book claims. Hard failures are not averaged away.

## Baseline and stretch targets

| Measure | S0 public target | S1/S2 research target |
|---|---:|---:|
| Public oracle conformance | 100% | 100% |
| Decision-critical unknown abstention / clarification recall | 100% | 100% |
| Material source-binding coverage | 100% | 100% |
| Exact journal and statement arithmetic on supported tasks | 100% | 100% |
| Clarification precision | ≥95% | ≥98% |
| Redundant questions | ≤5% | ≤2% |
| Exact replay on frozen frontiers | 100% | 100% |
| Expert-reviewed hidden critical false qualification | 0 | 0 |

These are targets, not current results. S1 requires at least 100 dual-annotated, expert-reviewed cases; S2 at least 250 plus a protected holdout and multiple jurisdictions, languages and currency precision profiles.

## Matched baseline arms

Rules-only; model-only; model plus document/spreadsheet tools; model plus conventional accounting tools; source-bound evidence graph; evidence graph plus independent verifier; Symthaea candidate analyst plus Mycelix verifier; blinded professional subset. #5799 owns identical inputs/budgets, run receipts, blind isolation and repeated-run statistics.

## Claim ceiling

Passing S0 proves only conformance to these ten public synthetic contracts. It does not establish general accounting competence, audit sufficiency, GAAP/IFRS/ISA compliance, tax or data-transfer legality, real source authenticity, complete books, or production readiness.
