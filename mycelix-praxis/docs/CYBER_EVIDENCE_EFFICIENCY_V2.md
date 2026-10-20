# Praxis Cyber Evidence Acquisition v2 — five-domain sufficiency conformance

## Scope

This versioned follow-on extends the v0.1 acquisition prototype without rewriting the v1 public fixture. The solver projection and the evaluator-only conformance data live in separate v2 files under `examples/cyber-evidence-efficiency/v2/`. The v2 manifest pins both JSON documents and both Draft 2020-12 schemas.

The five solver-visible domains are:

1. Identity and access
2. Network configuration
3. Supply-chain provenance
4. AI/tool authority boundaries
5. Endpoint and response telemetry

Each scenario carries competing hypotheses and bounded candidate evidence requests. The evaluator independently describes the stipulated synthetic world, per-hypothesis output partitions, response status, and evidence sufficiency. It is a public synthetic conformance fixture—not a hidden or held-out qualification set.

## Paradigm shift: two evidence axes

Do not collapse source status and answer sufficiency into one confidence score.

- **Response status** (`current`, `stale`, `conflicting`, `unavailable`) describes the provenance/freshness/consistency of a returned record.
- **Evidence sufficiency** (`full_support`, `partial_support`, `irrelevant`, `absent`, `conflicting`) describes whether that record supports the claim being investigated.

A source can return a current but irrelevant observation. A current record can partially support a hypothesis without proving it. A stale or unavailable request must not narrow the active hypothesis set. A current response tagged irrelevant, absent, or conflicting must also not narrow it. Only current full/partial evidence with an explicit hypothesis-to-observation mapping may update the candidate set.

The dimensions are reported separately so a high information score cannot compensate for a source conflict, unsupported certainty, missing evidence, or an unauthorized request.

## Metric semantics: expectation is not realization

The evaluator uses a uniform prior over each scenario's declared hypotheses; this is a synthetic scoring convention, not an estimate of real incident prevalence.

- **Expected information gain** is computed from the entropy of joint response-label signatures across the full declared hypothesis set. Each usable request contributes the increase in that joint information when its observation partition is added. This value is prior-expected and does not depend on which one synthetic world was selected for the fixture.
- **Realized information gain** records the pathwise reduction for the declared world: for each usable response, it adds \\(\log_2(N_{before}/N_{after})\\) when the surviving hypothesis set narrows. It is one-world evidence reduction, not an estimate of average performance across worlds.
- The exhaustive non-adaptive oracle optimizes the same prior-expected joint-partition objective subject to the declared synthetic cost and latency budget. It is exact only for the *non-adaptive subset* problem in this small fixture. It is not an upper bound on adaptive policies, and it is not a fair human/AI competitor because it sees evaluator-only partitions.
- A serious policy comparison must average predeclared metrics across balanced held-out variants/worlds, with uncertainty intervals. Do not compare a single world's realized reduction to the oracle's prior-expected entropy as though they were the same statistic.

## Executable reference and baselines

The focused integration harness reports requests attempted/executed, blocked authorization/budget attempts, synthetic request cost, declared latency bounds, expected and realized information gain, remaining hypotheses, and response status/sufficiency counts separately.

It includes three solver-visible deterministic traces:

- **No-query baseline:** no additional requests.
- **Fixed playbook:** a declared, domain-specific sequence filtered through the same authority inputs. Identity, network, supply-chain, AI/tool-boundary, and endpoint scenarios now have their own frozen request order; the baseline is not merely an identity-only trace reused across domains.
- **Cheapest authorized first:** deterministic sort by synthetic cost, latency bound, then stable request ID.

A fourth **non-adaptive oracle subset** exhaustively searches authorized, current, full/partial candidate requests under the same synthetic cost/latency budget and uniform hypothesis prior. It may inspect hidden response partitions; therefore it is a privileged mathematical reference, not a deployable policy or fair human/AI competitor. The exact search is capped at 16 usable candidate requests per scenario.

Any human or AI/tool policy must receive the same solver-visible catalog, evidence, authority set, cost/latency budget, and held-out scenarios. Out-of-catalog or unauthorized requests count as violations, not successful evidence acquisition.

## Typed final-decision contract v1

The next evaluation boundary is a typed final submission rather than a raw label or a free-form explanation. The solver-visible `praxis_core::decision_submission` module defines three mutually exclusive outcomes:

- `claim`: one scenario hypothesis, observed-evidence citations, a structured rationale with supporting/contradictory evidence IDs and typed missing-evidence kinds, plus optional confidence.
- `abstain`: an explicit reason, cited observations when available, and structured missing-evidence kinds. An empty abstention without a stated evidence basis is rejected.
- `request_more_evidence`: catalog request IDs and a typed purpose. Validation checks these IDs against the caller-provided authorized/budget-filtered allowlist.

Observed evidence IDs and request IDs are separate namespaces: a request acknowledgement cannot be cited as though it were an observed record. A definitive claim must cite at least one supporting evidence ID; supporting and identified contradictory evidence must be included in its citation set. Confidence, when present, must be finite and in `[0, 1]`.

This validator is **structural, not an evidence adjudicator**. It validates schema/benchmark/scenario identity and reference membership; it does not establish that a cited observation is fresh, relevant, sufficient, truthful, or proof that a command took effect. The caller must construct the context from solver-visible evidence and filter authorized requests before passing the context. The evaluator-only decision rubric, per-domain abstention/over-answer scorer, paired policy traces, and Pareto analysis remain follow-on work; this contract does not claim those are already implemented.

Eleven integration tests cover valid claims, missing support, request/evidence ID confusion, unknown hypotheses and evidence, duplicate citations, uncited contradiction, confidence bounds, structured versus blanket abstention, unauthorized request IDs, scenario/version mismatches, and unknown evaluator-field rejection. These are contract tests, not a capability score.

## Research and industry anchors

- [Evidence Sufficiency Benchmark (2026)](https://doi.org/10.32604/cmc.2026.086343) provides a five-level support/abstention design. Praxis adapts the methodological distinction between full, partial, irrelevant, absent, and conflicting support; the paper's model results are not cyber results and must not be transferred as our performance baseline.
- [Facebook Research AbstentionBench](https://github.com/facebookresearch/AbstentionBench) evaluates abstention across 20 datasets and six conditions, including stale data. Praxis should add cyber-specific paired variants and task/domain reporting rather than infer that generic question-answering results predict incident-analysis performance.
- [NIST NICE Framework Components v2.2.0](https://www.nist.gov/itl/applied-cybersecurity/nice/nice-framework-resource-center/nice-framework-current-versions) includes a Cybersecurity Supply Chain Risk Management work role and updated Cryptography and DevSecOps competency areas. Map future reviewed tasks to a pinned NICE version; alignment shows coverage, not competence.
- [MITRE ATT&CK](https://attack.mitre.org/) supplies behavior vocabulary, not proof that a described behavior happened in a particular system.
- [OWASP GenAI Security Project](https://genai.owasp.org/initiative/owasp-top-10-for-llm-and-genai/) informs AI/tool boundary cases; a retrieved instruction string is not an authority grant, and an attempted action is not evidence that the action succeeded.

## Current conformance boundaries

- The five domains are represented by **five public synthetic cases**, one per domain. This is a contract smoke test, not a balanced statistical benchmark.
- One current evidence request is deliberately insufficient to resolve each scenario. Each stipulated world has a two-request reference path that narrows the remaining hypotheses to one under the declared mapping.
- All costs and latency values are synthetic comparative units/bounds, not production forecasts.
- The evaluator fixture is public; do not count these examples as held-out capability data.
- The exact non-adaptive oracle is not an AI or human competitor. It is a constrained ceiling for the synthetic non-adaptive subset problem only.
- The evaluator/report does not issue credentials, set mastery or trust scores, authorize operations, scan live systems, exploit targets, or trigger remediation.

## Preregistration gate before comparative claims

The v2 fixture folder now contains `qualification-protocol.template.json` plus `qualification-protocol.schema.json`. This is a **template, not a registered study**: its schema deliberately fixes `status=template_only_not_preregistered` and `eligibility=NOT_ELIGIBLE_FOR_COMPARATIVE_SCORE`. Null fields such as the held-out split digest, power analysis, policy artifact digests, analysis method, and independent reviewer must be completed and frozen before held-out cases are accessed.

The exact-head CI receipt binds the preregistration template/schema blobs alongside the scenario fixtures. That proves which protocol template was present; it does not convert the template into a preregistered experiment or validate any future results. The test suite negatively checks that this template cannot self-promote to a registered status and that the 50-case minimum and zero-unauthorized-action safety gate cannot be weakened in the template schema.

## Qualification targets

Before a superiority claim, independently author and review at least 50 domain-balanced held-out cases, with paired benign/malicious hypotheses and controlled full/partial/irrelevant/absent/conflicting evidence. Register domains, splits, endpoints, exclusions, cost/latency budgets, policy prompts/tools, and analysis before running.

Report, at minimum, per-domain hypothesis quality, over-answer rate, justified-abstention rate, evidence coverage, source-status errors, calibration, forbidden/unauthorized actions, requests/cost/latency, and confidence intervals. Preserve raw case accounting and null/negative results. A safety failure is non-compensable; do not hide it in a single aggregate score.

Do not establish a cost/latency improvement SLO until distributions are measured from a real baseline. The current test pass establishes only exact-head contract and public-fixture conformance, not state-of-the-art performance, real incident accuracy, learning efficacy, or production readiness.
