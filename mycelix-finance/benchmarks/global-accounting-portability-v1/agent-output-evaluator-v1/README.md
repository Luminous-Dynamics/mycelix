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
- **Expose epistemic limits:** distinguish supported facts from assumed, unknown or disputed inputs and never hide a decision-critical assumption.

A correct number is not sufficient. A correct question type with an unrelated source citation is not sufficient. A balanced but source-unsupported journal is not sufficient. A natural-language question can be syntactically valid while still being professionally poor; machine metrics cannot establish question clarity or sufficiency without expert annotation.

## Files

- `agent_submissions_v1.0.0.schema.json`: closed Draft 2020-12 schema for agent output.
- `agent_evaluation_report_v1.0.0.schema.json`: closed report schema.
- `fixtures/gold_control_agent_submissions_v1.0.0.json`: 20 synthetic gold-control submissions derived from the public oracle; test scaffolding only.
- `../evidence-qualified-accounting-v1/fixtures/evidence_qualified_accounting_candidates_v1.0.0.json`: the candidate tasks.
- `../evidence-qualified-accounting-v1/fixtures/evidence_qualified_accounting_oracle_v1.0.0.json`: public synthetic oracle, used only by the self-test evaluator.
- `../rust-resolver/src/bin/evaluate_evidence_qualified_accounting_agent.rs`: deterministic submission evaluator.

## What the evaluator scores

Per case and aggregate:
- disposition accuracy and primary-reason accuracy;
- false-deliver and non-deliver-with-output safety failures;
- clarification target-fact precision, recall and F1;
- duplicate, irrelevant or scattershot question targets;
- clarification source-document reference precision and recall;
- exact journal match and individual-entry balance;
- material source-fact binding completeness and unavailable-source citations;
- exact statement match against the journal-linked oracle;
- decision-critical assumptions not supported by available source facts;
- hard failures with explicit failure codes.

Question-source reference scoring maps the expected blocking fact types to the documents in the candidate bundle that contain those facts. A source reference that is real but unrelated still loses precision. A missing relevant document loses recall.

Journal balancing is evaluated per entry and per currency/entity/period group, so two unbalanced entries cannot cancel each other in an aggregate. Source availability uses valid time, recorded-at, document received-at, the knowledge frontier and document revision supersession from the accounting candidate bundle.

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

The evaluator validates the candidate, oracle, submission and report schemas; requires identical case-ID sets; pins exact input bytes with SHA-256; builds one result per case; validates the generated report; and has no network or ledger side effects.

Eleven authored Rust tests cover gold-control conformance, unsafe delivery, scattershot targets, missing source refs, question source mismatches, per-entry imbalance, statement mismatch, unsupported critical assumptions, missing submissions and revision-aware source availability. Tests are authored but **not executed**. No Cargo compile, test, rustfmt, Clippy, independent Draft 2020-12 runtime pass or CI run is claimed.

## Static self-test evidence

A separate JS simulation of the fetched candidate/oracle/submission artifacts found:
- 20 candidate cases, 20 oracle rows and 20 submissions;
- 8 clarification questions;
- exact disposition and reason agreement;
- question target precision/recall = 1.0;
- question source-reference precision/recall = 1.0;
- journal exact match and statement exact match = 1.0;
- all expected journal entries balanced;
- source-binding completeness = 1.0;
- zero false-deliver cases, redundant questions or decision-critical unsupported assumptions;
- zero broken references or case-ID gaps.

This is evidence that the gold-control test data and evaluator contract align. It is not evidence that any model achieves these scores.

## Benchmark and comparison design

On the same rights-cleared cases, source snapshots and frozen budgets compare:
1. model-only;
2. model + document/spreadsheet tools;
3. model + deterministic ledger feedback;
4. typed economic-event graph;
5. event graph + independent verifier;
6. Symthaea candidate analyst + Mycelix verifier;
7. qualified professional on a blinded subset.

Keep model/version, prompts, tools, permission scope, retries, token/time budgets, source revisions, candidate code, evaluator version and random seeds fixed. Retain raw outputs and tool traces. Use the canonical #5799 run manifest/receipt; this package deliberately creates no competing run-manifest schema.

APEX-Accounting's live 100-task leaderboard currently displays 71.5% as its leading score; the published 160-task paper reports 56.4% Mean Criteria@3. Those cohorts and metric definitions are not directly comparable. Mercor's July 2026 paper/blog reports that more than 40 accounting professionals authored the benchmark, median 11 years of experience, 13.7 rubric criteria/task and 97% judge/expert agreement; the public sample is one world and ten tasks. FINBALANCE reports maximum 46% exact final balance-sheet accuracy on a separate 710-record split and a journal-reconstruction gain of 26–41 points for four models. Do not combine these denominators or metrics into one SOTA score.

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