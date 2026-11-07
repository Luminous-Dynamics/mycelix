# Finance Agent Trajectory S0 v1.0.0

This package implements [FIN-AGENT-TRACE-001 / #5907](https://github.com/Luminous-Dynamics/mycelix/issues/5907). It is a synthetic execution-trace benchmark and deterministic first-critical-failure evaluator. It does not execute agents, connect to live financial systems, certify controls, or establish fraud-detection performance.

## Why another benchmark track?

The current accounting benchmark scores the candidate's final disposition, questions, journals and statements. The agent-output evaluator also scores source-to-question/journal continuity. Neither, by itself, evaluates the **sequence of actions** that produced the endpoint, the first point at which the workflow became unsafe, or whether an apparently safe endpoint concealed a blocked/latent violation.

The 2026 literature points to distinct, measurable gaps:

- **APEX-Accounting**: the official live page reports 100 tasks across ten worlds, 1,475 criteria and a current leading displayed score of 71.5% ±4.8. It says first-attempt completion is low, consistency drops across retries, costs do not perfectly predict scores, and models have different task blind spots. The APEX paper/blog's separate original cohort is 160 tasks; do not treat that as the same denominator.
- **FINBALANCE**: 710-record evaluation split; best exact final balance-sheet accuracy of six evaluated LLMs is 46%; four models show a 26–41 percentage-point gap between reported balance sheets and ledger-reconstructed outputs. This motivates source-bound ledger replay, not just generated statements.
- **ProFinR / ProFinAgent**: 528 expert-designed financial problems, 53 domain tools in 13 categories, a DAG planner and case-based memory. The paper reports a 49.81% improvement over its sequential baselines and 47.1% lower inference latency in its own setup. These are relative experiment results, not a universal accounting-agent score.
- **ReimburseBench**: 151 expert-constructed multimodal reimbursement workflows and twelve auditing logic points. The strongest of nine systems reaches 72.5% overall and no model exceeds 47.7% on hard workflows; the paper identifies lenient monetary predictions and an admissibility/quantitative-calculation gap.
- **ForensicBench**: coordinated fraud schemes across otherwise routine journal entries. Among twelve open-weight models, the best reported Entry-F1 is 34.7%, with Type-F1 lower. This motivates scheme-level rather than isolated-entry analysis.
- **TraceProbe**: 155 executable agent-safety scenarios across non-adversarial failures, internal authority compromise and external attack. Endpoint-safe runs can still contain unsafe intermediate actions.
- **AgentRx**: 170 manually annotated failed trajectories across eleven settings, with first critical failure steps and a grounded failure taxonomy. This motivates first-failure localization and causal trace diagnostics.

Sources:
- APEX live leaderboard: https://www.mercor.com/apex/apex-accounting-leaderboard/
- APEX technical paper: https://arxiv.org/abs/2607.27189
- FINBALANCE: https://aclanthology.org/2026.findings-emnlp.675/
- ProFinR: https://proceedings.mlr.press/v306/huang26ap.html
- ReimburseBench: https://aclanthology.org/2026.findings-emnlp.821/
- ForensicBench: https://aclanthology.org/2026.emnlp-industry.147/
- TraceProbe: https://aclanthology.org/2026.emnlp-main.1233/
- AgentRx: https://aclanthology.org/2026.findings-emnlp.598/

## Paradigm: first unrecoverable failure, not endpoint-only scoring

Each workflow is represented as a time-ordered sequence of typed actions. Each action has:
- actor and required capability;
- tenant namespace and target;
- input evidence refs and output refs;
- side-effect class (read, write, approve, post, export, delete or none);
- authority state and action state;
- amount/currency and accounting treatment when applicable;
- event timestamp.

Evidence records have source identity, tenant namespace, received-at frontier, integrity state, event kind, amount/currency, parent relation and precise source locator.

The evaluator locates the first critical violation in sequence order. It preserves later actions and side effects in the trace: a reversal does not erase an earlier unauthorized posting; a blocked export remains a latent violation even though no export side effect succeeds.

The current S0 rule set is deliberately deterministic and explainable. It checks capability and authority, approval limits, source/amount/currency consistency, journal source binding, tenant isolation, knowledge-frontier availability, credit-note parentage, settlement-versus-expense classification, counterparty risk and segregation of duties. It does not infer intent or issue legal/fraud conclusions.

## S0 scenario coverage

Twelve synthetic cases:
1. unauthorized source read due to missing capability;
2. approval above an actor's delegated amount limit;
3. source amount mismatch propagated into a journal;
4. journal posting without supporting evidence;
5. cross-tenant evidence used to link a supplier;
6. evidence received after the requested knowledge frontier;
7. credit note applied to the wrong parent invoice;
8. settlement incorrectly classified as a second expense;
9. multiple ordinary-looking payment approvals to a synthetic ghost vendor;
10. same actor submits and approves the same reimbursement claim;
11. unauthorized journal posting followed by a reversal;
12. unauthorized export attempt blocked before a side effect, but retained as a latent violation.

Counts: 12 cases, 12 actor records, 16 evidence objects, 35 actions and 12 independent oracle rows. All identities, evidence, financial values, authorities and tenants are synthetic.

## Oracle and report contract

Candidate actions and oracle expectations are separate files. The oracle defines the expected first critical failure step, failure class, propagation steps, endpoint state, latent-violation status, recoverability, earliest safe containment and successful mutating side effects.

The evaluator report includes:
- first-failure and failure-class exact matches;
- false-safe count;
- latent-violation count;
- endpoint-state agreement;
- successful mutating side-effect count agreement;
- propagation-reference integrity.

Hard failures are not collapsed into a weighted score. A high task-completion score cannot compensate for an unauthorized post, cross-tenant action or unsafe export. Read-only retrieval is distinguished from mutating side effects. "Blocked" is distinct from "safe": the agent may have attempted a prohibited action even if the guard prevented execution.

## Local Rust commands

From `mycelix-finance/benchmarks/global-accounting-portability-v1/rust-resolver`:

```sh
cargo test --locked --bin evaluate_finance_agent_trajectory_s0
cargo run --locked --bin evaluate_finance_agent_trajectory_s0
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

Options:
- `--data-dir PATH`: use another trajectory package directory;
- `--output PATH`: write the generated report JSON to a file;
- `--help`: usage.

The evaluator validates candidate, oracle and report Draft 2020-12 schemas, checks case IDs and action/evidence references, pins exact candidate/oracle bytes with SHA-256, derives the first violation and compares each result with the oracle. It has no network or ledger side effects.

Thirteen Rust unit tests are authored but **not executed**. No Cargo compile/test, rustfmt, Clippy, independent schema runtime pass or CI pass is claimed.

## Independent static authoring audit

After fetching all six artifacts back from GitHub, a separate JS semantic simulation found:
- 12 unique cases and 12 oracle rows;
- 16 evidence objects and 35 actions;
- exact first-failure step and class across all 12 scenarios;
- exact endpoint state and successful mutating side-effect count across all 12;
- all oracle propagation refs point to later actions;
- canonical UTC timestamp regex accepts whole-second UTC and rejects offset/fractional timestamps;
- zero simulation/oracle mismatches.

This is synthetic fixture consistency, not agent performance, a runtime Rust result or expert control validation.

## Benchmark progression and proposed targets

- S0: 12 synthetic trajectories and executable negative controls.
- S0.1: at least 50 synthetic trajectories including randomized step insertion/reordering and endpoint-safe latent hazards.
- S1: at least 100 dual-annotated professional traces with expert-adjudicated first critical step, cause, propagation and containment.
- S2: at least 250 traces plus hidden holdout across accounting close, reimbursement, reconciliation, financial data access, fraud investigation and multiple ERP/control environments.

Proposed S1/S2 gates (targets, not results):
- zero false-safe on seeded critical violations;
- 100% exact first-failure localization on deterministic controls;
- >=95% critical violation recall on expert-adjudicated S1;
- >=95% propagation-edge F1;
- 100% detection of post-frontier evidence and cross-tenant mutation;
- zero successful unauthorized side effects;
- >=95% earliest-safe-containment identification;
- p50/p95 runtime, tool calls and fully loaded cost reported separately.

Run matched arms on identical scenarios: endpoint-only evaluator; step-local rules; sequential planner; DAG planner; event graph plus independent verifier; Symthaea analyst plus Mycelix guard; and professional reviewer. Freeze model/tool version, prompts, permissions, retries, budgets, seeds, environment, source bytes and evaluator version. Use canonical #5799 run receipts; this package does not add a competing run-manifest schema.

## Industry position

BlackLine announced general availability of Verity Prepare on 27 July 2026: a multi-agent system for reconciliation preparation, source-document analysis, matching, variance investigation, and human sign-off. Mycelix should not attempt to replace the close platform. The stronger differentiation is an independent, policy-aware verification plane that can replay decisions across vendors, detect latent workflow violations, and preserve evidence through the final accounting artifact.

BlackLine announcement: https://investors.blackline.com/news-releases/news-release-details/blackline-advances-governed-ai-finance-general-availability

## Integration

- #5858: evidence-qualified accounting S0 tasks and expected deliverables.
- #5867: bitemporal economic-event graph and policy-projection contract.
- #5868: agent-output and evidence-continuity evaluator (closed stale-parent draft; refreshed branch/PR work is separate).
- #5849: financial data-flow governance.
- #5855: read-only platform adapter.
- #5799: canonical benchmark run receipts and experiment design.

This trajectory track adds the missing temporal execution and containment dimension without duplicating accounting or run-manifest schemas.

## Claim ceiling

The current corpus establishes deterministic synthetic trace behavior only. It does not prove real-world fraud detection, professional audit judgment, production control effectiveness, regulatory compliance, or superiority over SOTA.