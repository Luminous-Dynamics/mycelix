# Runner-Observed Evidence Trace v1.0.0

This package implements [FIN-TRAJECTORY-TRACE-001 / #5874](https://github.com/Luminous-Dynamics/mycelix/issues/5874). It records externally observable source-read and structured-output events so evidence-to-decision continuity can be checked independently of the agent's final self-reported evidence trace.

It is **not** a new benchmark run receipt, an accounting policy engine, a model-authentication protocol, or a private chain-of-thought recorder.

## Why a second trace is needed

The agent-output evaluator in #5868 checks a candidate-declared trace against the public task/oracle contract. That is a useful consistency check, but it cannot prove which source revisions the agent actually opened or whether an intermediate discrepancy was preserved in the workflow.

This artifact separates two records:
- **Candidate-declared evidence trace** — the agent's structured statement about evidence and decision lineage. Evaluated by #5868.
- **Runner-observed evidence trace** — source reads captured by the tool adapter/runner, plus structured candidate outputs captured from tool/agent protocol events. Validated here.

Runner/tool adapter events and candidate output events have distinct actor roles. A fact-observed event is a structured candidate assertion captured by the runner; it is not independently proven true by appearing in the trace. The trace binds the exact source bytes and transcript, while professional/independent evidence adjudication remains a separate layer.

No private reasoning tokens, hidden chain-of-thought, or free-form internal deliberation are recorded. Only source access, tool response and structured decision/output events are represented.

## Canonical receipt composition — no duplicate receipt schema

The trace uses the canonical Symthaea `luminous.benchmark-run-receipt.v1` contract from [Symthaea PR #10314](https://github.com/Luminous-Dynamics/symthaea/pull/10314) and the layered evidence architecture in [Symthaea issue #10370](https://github.com/Luminous-Dynamics/symthaea/issues/10370).

This Mycelix package deliberately does **not** copy or redefine the benchmark receipt schema, task-result accounting, task universe, canonical receipt digest, DSSE envelope, trust roots or independent regrade protocol.

Receipt-bound mode consumes three external inputs:
1. exact canonical benchmark receipt JSON bytes;
2. exact raw transcript bytes;
3. a canonical receipt validator report with `schema = luminous.benchmark-receipt-validation-result.v1`, `valid = true`, and one matching run row.

It checks:
- trace and receipt use the canonical receipt schema version;
- trace `run_id` equals receipt `run_id`;
- exactly one receipt task row matches trace `task_id`;
- the task row `input_sha256` equals trace `source_bundle_sha256`;
- raw transcript SHA-256 equals both trace `transcript_sha256` and receipt `provenance.transcript_sha256`;
- SHA-256 of exact trace artifact bytes appears in the matching task row `output_sha256`;
- exactly one validation-report row matches the run ID and its `raw_file_sha256` equals the exact receipt bytes.

Receipt-bound mode requires a separately successful canonical receipt validation report. This validator checks the report's schema ID, `valid` flag and raw-byte binding; it does not rerun the canonical receipt validator. A caller must obtain that report from the canonical validator and retain it with the artifacts.

A SHA-256 is an integrity binding, not an identity or authenticity proof. A caller-supplied report can be forged if the canonical validator/runner is not trusted. DSSE signatures, signer/key policy, runner qualification and independent regrade remain with the canonical evidence architecture (#10370/#10369). This package does not upgrade a runner's trust tier.

## Trace event model

The closed Draft 2020-12 schema uses an append-only ordered event stream:

- `source_opened` — runner/tool adapter records document identity, revision, exact source SHA-256, locator and tool-call ID.
- `tool_response` — adapter/runner records a response digest.
- `fact_observed` — candidate emits a structured fact type/value digest/evidence state linked to preceding source-open events.
- `fact_reclassified` — candidate appends a state change with a preceding cause event; prior state must match the current state.
- `contradiction_detected` — candidate links two or more previously observed facts.
- `question_emitted` — candidate links a question digest to observed blocker facts and source reads.
- `fact_terminalized` — candidate assigns each observed fact a terminal state and reason: question, journal, review, abstention or explicit exclusion.
- `journal_line_emitted` / `statement_line_emitted` — candidate output hashes link back to the facts and journal rows they use.
- `human_override` — reviewer role and explicit fact references/reason are recorded.
- `disposition_emitted` / `execution_terminal` — one final candidate disposition and one runner-observed terminal workflow state.

Every event has a contiguous sequence and canonical UTC timestamp. No event is deleted or rewritten; a reclassification appends a new event. Every supplied fact must be terminalized exactly once. Every question blocker must be terminalized to the corresponding question. Every journal-line source fact must be terminalized to that line. Statement lines can reference only preceding journal lines. The disposition must occur after questions, journals, statements and all fact terminalizations.

A `source_opened` event must be emitted by `tool_adapter` or `runner`, not the candidate. A candidate fact/question event must link to a preceding source-open event for each of its source documents. A fact cannot bind two revisions of the same document in one observation; re-read/revision changes require a separate fact event and explicit reclassification.

## Files

- `runner_observed_evidence_trace_v1.0.0.schema.json`: closed trace schema.
- `runner_observed_evidence_trace_report_v1.0.0.schema.json`: closed validation report schema.
- `fixtures/runner_observed_evidence_trace_synthetic_v1.0.0.json`: one positive synthetic trace over the `ACCT-001-COMPLETE-INVOICE` S0 case (two source opens, five facts, five terminal dispositions, two journal lines, one statement, one final disposition and one terminal state).
- `../rust-resolver/src/bin/validate_runner_observed_evidence_trace.rs`: standalone deterministic validator and optional canonical-receipt binding CLI.

The positive fixture's digests are synthetic placeholders; it is not evidence of an executed agent, actual source file or real runner.

## Local Rust validation

From `mycelix-finance/benchmarks/global-accounting-portability-v1/rust-resolver`:

```sh
cargo test --locked --bin validate_runner_observed_evidence_trace
cargo run --locked --bin validate_runner_observed_evidence_trace
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

Receipt-bound mode:

```sh
cargo run --locked --bin validate_runner_observed_evidence_trace -- \
  --trace trace.json \
  --run-receipt canonical-run-receipt.json \
  --transcript raw-transcript.jsonl \
  --receipt-validation-report canonical-receipt-validation.json
```

Other options include `--schema PATH`, `--report-schema PATH`, and `--output PATH`. Run `--help` for the complete interface.

Twenty-one Rust tests are authored. They cover valid positive structure; source-before-fact; one terminal disposition per fact; source revision digest conflict; blocker lineage; fact-to-journal and statement-to-journal continuity; timestamp/sequence regression; incorrect reclassification; receipt transcript mismatch; missing trace output digest; receipt-report raw-byte mismatch; duplicate task/run rows; source bundle input mismatch; and invalid receipt report schema.

**The Rust validator and tests have not been executed.** No Cargo compile/test, rustfmt, Clippy, independent JSON Schema runtime pass, canonical receipt runtime validation or CI pass is claimed. Static authoring checks are not substituted for these runtime checks.

## Metrics and claim ceiling

This validator establishes trace structural continuity and, in receipt-bound mode, exact raw-byte links to receipt/transcript/task-output artifacts. It does not grade accounting truth, verify that a source locator semantically supports a fact, authenticate the model provider, authenticate the runner/adapter, establish secure isolation, or prove that a human reviewer is independent.

S1 should compare candidate-declared trace with runner-observed trace on at least 100 dual-reviewed professional cases. Report source-observation coverage, source/fact lineage completeness, contradiction preservation, blocker-to-question continuity, fact-to-journal continuity, statement-to-journal continuity, dropped-fact count, unsupported source-access claims, reviewer effort and cost separately. Hard integrity failures must not be averaged away.

No SOTA claim follows from a valid trace. The trace only makes the observable path more auditable; answer correctness still requires an independently qualified grader and domain review.