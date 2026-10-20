# Praxis Cyber Evidence Acquisition v1 (synthetic qualification prototype)

## Purpose and boundary

This is a separate, versioned follow-on to [the public cyber challenge corpus](./CYBER_CHALLENGE_CORPUS_V1.md) and [issue #5305](https://github.com/Luminous-Dynamics/mycelix/issues/5305). It makes evidence acquisition measurable without changing the v1 cyber scenarios.

- `examples/cyber-evidence-efficiency/v1/request-catalog.json` is solver-visible. It contains candidate requests, the request purpose, declared authority class, synthetic cost units, and an explicit latency bound.
- `evaluator-only.json` contains the stipulated synthetic world, response quality, and response labels under each hypothesis. It must not be embedded in or served to learner-facing applications.
- Both JSON files have separate JSON Schema Draft 2020-12 contracts.
- The production library exposes the solver-visible typed request catalog and its structural validator only. The reference evaluator and hidden outcome mapping live in the integration test target, not the library module. This is a deliberately conservative separation until a reviewed isolated benchmark binary is warranted.

The public example is **conformance data, not held-out evidence**. Labels, the hidden world, costs, authority requirements, and latency bounds are authored synthetic assumptions. They are not empirical priors, operational latency forecasts, real security evidence, or calibrated incident probabilities.

## Paradigm shift

The benchmark asks whether a policy acquires *decision-relevant evidence* efficiently under authority and resource constraints—not merely whether a final diagnosis matches an answer key.

For each ordered policy trace, the test-only reference evaluator reports independent dimensions:

- attempts, executed requests, unauthorized blocks, and budget blocks;
- accumulated synthetic cost units and declared worst-case latency bounds;
- incremental expected information gain under a uniform hypothesis prior, in milli-bits;
- realized hypothesis-space reduction on the evaluator's stipulated world, in milli-bits;
- current but non-discriminating requests;
- stale, conflicting, and unavailable responses;
- hypotheses remaining after usable current evidence.

Expected information gain is calculated on the set of hypotheses still under consideration immediately before each request. For deterministic answer partitions and a uniform benchmark prior, it is entropy before acquisition minus the expected posterior entropy across that request's possible outcomes. Realized information gain uses the observed synthetic answer to calculate `log2(before_hypotheses / after_hypotheses)`. These are information-theoretic benchmark metrics, **not probabilities that an incident is real**.

A request which is attempted but unauthorized is recorded as blocked and receives no synthetic evidence effect or resource cost in this first model. An authorized request that returns stale, conflicting, or unavailable evidence consumes its declared synthetic cost/bound but does not narrow the hypothesis set. This distinction is intentional and test-covered. The authority list passed to this reference evaluator is an input fixture, not a real authorization system.

## Baseline policy comparison

Use identical candidate requests, authority inputs, budget, evidence visibility, and scenario variants for:

1. **No query:** decide from the initial candidate set without additional evidence.
2. **Fixed playbook:** execute an agreed request sequence.
3. **Metadata-only cheapest-first:** order authorized requests by cost, then latency bound, then stable request ID. This baseline uses only solver-visible fields; it cannot inspect hidden evaluator outcome partitions.
4. **Oracle information-gain-per-cost (upper bound only):** select from evaluator-side partitions to estimate a synthetic ceiling. This is not deployable and must never be presented as a fair AI-vs-baseline comparison because it has privileged knowledge unavailable to the solver.
5. **Human reference:** collect independently reviewed analyst policies and adjudicate disagreements.
6. **AI/tool-using policy:** evaluate only after the same restrictions and accounting apply; any generated request outside the allowed catalog is rejected and recorded.

The integration-test harness now defines deterministic no-query, fixed-playbook, and metadata-only cheapest-authorized-first traces; the policy trace generator does not accept evaluator state. It does **not yet implement all policy agents above**, run a comparative study, estimate live-world priors, or demonstrate SOTA. The next implementation slice should add more independently reviewed cases and a separately reported oracle upper bound, while preserving the evaluator boundary.

## Hard gates and proposed evaluation scale

Contract/conformance acceptance criteria:

- Both schema documents validate against the Draft 2020-12 meta-schema and both fixtures validate through a real schema engine.
- Unknown fields and invalid enums are rejected; the Rust contract rejects duplicate request/hypothesis IDs, zero cost/latency, missing scenarios, and unsupported versions.
- Cross-file semantic validation requires an exact scenario match, a declared hidden world among the visible candidate hypotheses, and exactly one outcome mapping for every request and every candidate hypothesis.
- Duplicate request attempts and unknown request IDs are rejected.
- Unauthorized requests cannot spend the budget or change evidence; budget-exceeding requests cannot consume resources; a later affordable request can still execute.
- Stale/conflicting/unavailable results cannot reduce the candidate hypothesis set, and current but non-discriminating results produce zero incremental information gain.
- Request order in the catalog representation cannot change a policy result for an equivalent trace when budgets and authority are unchanged.

For serious capability qualification, the proposal is at least **50 independently reviewed, domain-balanced synthetic cases** before starting a held-out experiment, with paired benign/malicious alternatives, source-quality variation, and protected evaluation splits. Report per-domain scores, calibration, authority violations, error bars, raw case accounting, and null/negative results. Do not collapse safety into a compensatory aggregate.

Do not establish a cost/latency SLO until distributions are measured from the real baseline. Any later target is a pre-registered hypothesis, not a result. A score must never imply mastery, certification, operational trust, or authority to perform remediation.

## Current verification boundary

The focused workflow binds execution to the exact checked-out commit and pinned Rust 1.96.0, checks formatting, runs this evaluator target, and re-runs the original cyber corpus contract target. A passing workflow demonstrates only contract/test conformance for the cited commit; it does not establish learning efficacy, general cyber competence, production readiness, or field response quality.
