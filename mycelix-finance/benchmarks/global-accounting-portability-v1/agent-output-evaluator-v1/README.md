# Evidence-Qualified Accounting Agent Output Evaluator v1.0.0

This package implements [FIN-AGENT-EVAL-001 / #5864](https://github.com/Luminous-Dynamics/mycelix/issues/5864). It evaluates structured agent submissions for the synthetic accounting S0 corpus. It does not run models, post ledger entries, issue accounting advice or establish professional performance.

## Why this layer exists

The accounting S0 validator checks whether the **public candidate/oracle package** is internally consistent. It does not test whether an agent can produce a good answer or ask the right question.

This evaluator consumes four distinct artifacts:
1. candidate tasks and evidence;
2. a supplied oracle;
3. structured agent submissions;
4. an evaluator report.

The bundled 20 gold-control submissions are harness self-tests mechanically constructed from the public oracle. They are explicitly **not model outputs or benchmark results**. A real experiment must substitute a frozen, independently generated agent-submission bundle and keep private oracles unavailable to the agent.

## Paradigm: evaluate the decision boundary

A capable accounting agent must do four things independently:

- **Choose the right disposition:** deliver, clarify, abstain or escalate.
- **Ask a useful question:** identify the decision-blocking fact, cite the documents that contain the relevant evidence, ask a bounded question and specify the answer needed.
- **Produce a grounded deliverable:** source-bound journal entries and statement lines that independently balance and reconstruct.
- **Preserve evidence-to-decision continuity:** give every supplied fact a terminal disposition (journaled, questioned, reviewed, abstained or explicitly excluded), with a non-empty reason, and ensure discovered contradictions, future facts, integrity failures and superseded revisions are not silently dropped.
- **Expose epistemic limits:** distinguish supported facts from assumed, unknown or disputed inputs and never hide a decision-critical assumption.

A correct number is not sufficient. A correct question type with an unrelated source citation is not sufficient. A balanced but source-unsupported journal is not sufficient. A natural-language question can be syntactically valid while still being professionally poor; machine metrics cannot establish question clarity or sufficiency without expert annotation.

## Files

- `agent_submissions_v1.0.0.schema.json`: closed Draft 2020-12 schema for agent output.
- `agent_evaluation_report_v1.0.0.schema.json`: closed report schema.
- `fixtures/gold_control_agent_submissions_v1.0.0.json`: 20 synthetic gold-control submissions derived from the public oracle; test scaffolding only.
- `../evidence-qualified-accounting-v1/fixtures/evidence_qualified_accounting_candidates_v1.0.0.json`: the candidate tasks.
- `../evidence-qualified-accounting-v1/fixtures/evidence_qualified_accounting_oracle_v1.0.0.json`: public synthetic oracle, used only by the self-test evaluator.
- `runner_observed_evidence_trace_v1.0.0.schema.json` and `runner_observed_evidence_trace_report_v1.0.0.schema.json`: separate externally observed event-trace and validation-report contracts.
- `fixtures/runner_observed_evidence_trace_synthetic_v1.0.0.json`: synthetic tool/source/output event trace; not an authentic model run.
- `../rust-resolver/src/bin/evaluate_evidence_qualified_accounting_agent.rs`: deterministic submission evaluator.
- `../rust-resolver/src/bin/validate_runner_observed_evidence_trace.rs`: event-trace validator with optional canonical #5799/Symthaea receipt and transcript binding.

## What the evaluator scores

Per case and aggregate:
- disposition accuracy and primary-reason accuracy;
- false-deliver and non-deliver-with-output safety failures;
- clarification target-fact precision, recall and F1;
- duplicate, irrelevant or scattershot question targets;
- clarification source-document reference precision and recall;
- exact journal match and individual-entry balance;
- material source-fact binding completeness independently for every expected journal line, plus unavailable-source citations;
- exact statement match against the journal-linked oracle;
- independent statement replay from the submitted journal, including exact source-entry set and signed amounts by account normal balance, entity, currency and period;
- decision-critical assumptions not supported by available source facts;
- candidate-declared evidence-trace coverage across every supplied fact, including facts deliberately excluded from the deliverable;
- evidence-to-journal continuity from fact trace to journal line;
- decision-to-question continuity from blocker to actual clarification question;
- explicit terminal reasons for each fact disposition, including superseded, future, disputed, invalidated, unverified and integrity-failed evidence;
- explicit continuity failure codes for dropped, relabelled or unlinked evidence;
- hard failures with explicit failure codes.

Question-source reference scoring maps the expected blocking fact types to the documents in the candidate bundle that contain those facts. A source reference that is real but unrelated still loses precision. A missing relevant document loses recall.

Journal balancing is evaluated per entry and per currency/entity/period group, so two unbalanced entries cannot cancel each other in an aggregate. Source-binding completeness is calculated line-by-line; one line cannot borrow a citation from another line. Each submitted statement line is independently replayed from all matching submitted journal lines: the statement must cite exactly the journal entries that contribute to its account/entity/currency/period, and the signed sum must match the reported amount under the declared normal balance. This replay is separate from oracle exact match, so a numerically expected statement beside a contradictory journal is still rejected. S0 currently tests period-movement income-statement lines; balance-sheet closing balances require opening-balance and carry-forward inputs before they can be replayed correctly. The evidence trace must assign a terminal disposition and reason to every supplied fact, including excluded, superseded, future, disputed, invalidated, integrity-unverified and integrity-failed observations. Blocker traces must link the decision-blocking fact type and its source documents to an actual question; journal-source facts must link to the actual output line. A discovered fact cannot disappear merely because the agent ultimately excludes it. Source availability uses valid time, recorded-at, document received-at, the knowledge frontier and document revision supersession from the accounting candidate bundle.

Hard safety failures remain visible separately; they are not averaged away. The report does not collapse all dimensions into a single leaderboard score.

## Local Rust validation

From `mycelix-finance/benchmarks/global-accounting-portability-v1/rust-resolver`:

```sh
cargo test --locked --bin evaluate_evidence_qualified_accounting_agent
cargo run --locked --bin evaluate_evidence_qualified_accounting_agent
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

Optional arguments:
- `--data-dir PATH`: evaluator artifact directory (default is the sibling `agent-output-evaluator-v1` folder);
- `--output PATH`: write report JSON to a file rather than stdout;
- `--help`: show usage.

The evaluator validates the candidate, oracle, submission and report schemas; requires identical case-ID sets; pins exact input bytes with SHA-256; builds one result per case; validates the generated report; and has no network or ledger side effects. A separate runner-observed trace validator checks append-only event sequencing and continuity, then can bind the exact trace digest, transcript bytes and source input digest to the canonical #5799/Symthaea receipt. It does not authenticate model identity. A self-hash and receipt binding do not replace the canonical signature/trust-root verification owned by the Symthaea evidence stack.

Eighteen authored Rust tests cover gold-control conformance, unsafe delivery, scattershot targets, question-source mismatches, line-level source binding, per-entry imbalance, statement mismatch, unsupported critical assumptions, missing submissions, revision-aware source availability, complete evidence-trace coverage, blocker-to-question continuity, fact-to-journal continuity, contradiction-state preservation, integrity-failure abstention and superseded-revision exclusion. Tests are authored but **not executed**. No Cargo compile, test, rustfmt, Clippy, independent Draft 2020-12 runtime pass or CI run is claimed.

## Static self-test evidence

A separate JS simulation of the fetched candidate/oracle/submission artifacts found:
- 20 candidate cases, 20 oracle rows and 20 submissions;
- 74 source facts and 82 explicit trace entries;
- one terminal disposition and non-empty reason for every source fact;
- 8 clarification questions;
- exact disposition and reason agreement;
- question target precision/recall = 1.0;
- question source-reference precision/recall = 1.0;
- journal exact match and statement exact match = 1.0;
- all expected journal entries balanced;
- line-level source-binding completeness = 1.0;
- evidence-trace coverage = 1.0 across all 74 supplied facts and eight blockers;
- evidence-to-journal continuity = 1.0;
- decision-to-question continuity = 1.0;
- zero continuity failures, false-deliver cases, redundant questions or decision-critical unsupported assumptions;
- zero broken references or case-ID gaps.

This is evidence that the gold-control test data and evaluator contract align. The evaluator also distinguishes a missing unrelated fact trace and a superseded revision deliberately marked as excluded. It is not evidence that any model achieves these scores. Because the trace is part of candidate output, do not call it runner-observed evidence; actual trajectory continuity requires a separately retained tool/transcript artifact bound to the canonical Symthaea run receipt, tracked in #5874.

## Benchmark and comparison design

On the same rights-cleared cases, source snapshots and frozen budgets compare:
1. model-only;
2. model + document/spreadsheet tools;
3. model + deterministic ledger feedback;
4. typed economic-event graph;
5. event graph + independent verifier;
6. Symthaea candidate analyst + Mycelix verifier;
7. qualified professional on a blinded subset.

Keep model/version, prompts, tools, permission scope, retries, token/time budgets, source revisions, candidate code, evaluator version and random seeds fixed. Retain raw outputs and tool traces. Use the canonical #5799 run manifest/receipt; this package deliberately creates no competing run-manifest schema. #5874 adds a separate runner-observed trace artifact whose exact bytes must be bound by the canonical receipt; it does not replace the candidate-declared trace metric in this S0 version.

APEX-Accounting's live 100-task leaderboard currently displays 71.5% as its leading score; the published 160-task paper reports 56.4% Mean Criteria@3. Those cohorts and metric definitions are not directly comparable. Mercor's official July 2026 article reports more than 40 accounting professionals, median 11 years of experience, 13.7 rubric criteria/task and 97% judge/expert agreement; the public sample is one world and ten tasks. It attributes roughly seven in ten top-three-model failures to reasoning rather than retrieval, including discrepancies found mid-workflow but omitted or contradicted in final journals. FINBALANCE reports maximum 46% exact final balance-sheet accuracy on a separate 710-record split and a journal-reconstruction gain of 26–41 points for four models. Do not combine these denominators or metrics into one SOTA score.

Sources:
- APEX live leaderboard: https://www.mercor.com/apex/apex-accounting-leaderboard/
- APEX methodology/results paper: https://arxiv.org/abs/2607.27189
- Mercor APEX benchmark methodology article: https://www.mercor.com/blog/introducing-the-ai-productivity-index-for-accounting/
- FINBALANCE: https://aclanthology.org/2026.findings-emnlp.675/

## Proposed performance gates (not achieved)

S0 is 20 synthetic cases. Next:
- S0.1: 50 synthetic adversarial tasks with independent mutation controls;
- S1: at least 100 dual-reviewed/adjudicated professional cases;
- S2: at least 250 cases plus a hidden holdout across industries, currencies, policy versions, source systems and entity structures.

Proposed S1/S2 goals: zero critical false-deliver on seeded controls; >=95% decision-relevant questions; <=5% redundant questions; 100% source-binding completeness for material journal lines; >90% / >95% exact statement accuracy; >=30% reviewer-time reduction without increased material errors. These are targets, not reported performance.

## Claim ceiling

The current package tests evaluator behaviour on synthetic gold-control submissions. It does not show that an AI agent can perform accounting work, that professional reviewers agree with the questions, that any policy is correct, or that Mycelix exceeds SOTA. Professional question quality, policy correctness and material accounting judgments require expert adjudication.