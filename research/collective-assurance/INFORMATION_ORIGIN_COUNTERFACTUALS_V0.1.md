# Information-Origin Counterfactual Replay v0.1

**Status:** reference validator and experiment protocol proposal. No causal effect or benchmark win is claimed.

## Purpose

Collaborative-memory systems often report what was retrieved, which member spoke, or whether a final answer was correct. Those observations do not isolate the informational contribution of a source.

This protocol adds a narrower experiment:

> Hold the system, task, model, prompt, tools, evaluator, budget and random seed constant; remove one authorized source origin and its authenticated derived-origin closure; then compare the resulting decision, confidence and independently scored quality.

The object removed is an **evidence origin**, not a person. One source can be held by several people; one person can hold multiple independent sources. Repetition by multiple holders does not create independent corroboration.

## Relation to the evidence ledger

Compose with the existing Mycelix #6619 ledger:
- the ledger distinguishes information holder from source origin;
- request creation is distinct from delivery;
- disclosure is distinct from verification and recipient exposure;
- rights-based withholding is an explicit outcome;
- claim and decision revisions bind to the exact evidence origins used.

The counterfactual validator consumes authenticated source-origin and removal-closure references. It does not replace the ledger, source registry or independent receipt store. See the [W3C PROV Data Model](https://www.w3.org/TR/prov-dm/) for Entity, Activity and Agent provenance semantics.

## Protocol

For each eligible origin:

1. **Freeze the baseline.** Record the exact benchmark/task, model, tokenizer, system configuration, prompt, tools, decoding, evaluator/scorer, context, output, time, token, compute, memory and agent budgets, seed policy and seed.
2. **Verify eligibility.** The source origin must have been authorized and actually disclosed to the baseline decision context. A source withheld by rights, not delivered, unanswered or of unknown status is not a negative contribution and is not scored as an omission.
3. **Authenticate the source.** Bind the origin ID and digest to an independent source-registry receipt. Do not infer origin independence from a distinct string or holder count alone.
4. **Compute the removal closure.** Remove the origin and every source-origin descendant derived from it, including summaries, paraphrases, embeddings, memory entries and agent-to-agent copies. The closure must be independently attested.
5. **Replay under matched conditions.** Keep all non-target context and all execution-manifest fields unchanged. The replay's only intended difference is the removal closure. Use paired seeds and the same task/evaluator.
6. **Compare outcomes.** Record decision identity, confidence and independently scored decision quality in both runs. A changed decision is not automatically a failure; quality and evidence use determine the interpretation.
7. **Retain the trace.** Bind both ledger roots, source registry receipt, removal-closure attestation, input digests, output digests and evaluator evidence.

## Qualification rules

The Rust validator in `information_contribution.rs` requires:
- distinct baseline and counterfactual run IDs;
- a strict `MatchedSystem` comparison using the existing manifest validator;
- identical system variant/configuration and all model/task/evaluator/resource/seed controls;
- different input digests but identical non-target shared-context digest;
- a valid source-origin digest and an independent source-registry receipt reference;
- valid baseline and counterfactual ledger-root references;
- a removal closure containing the target origin and only origins present in the baseline;
- the exact counterfactual origin set to equal the baseline set minus the declared closure;
- a separate closure attestation reference;
- finite normalized confidence and decision-quality measures.

Duplicate origin IDs, unremoved derivatives, added sources, changed budgets, changed seeds, altered shared context or malformed evidence fail qualification. Missing registry/closure/ledger evidence remains `INDETERMINATE`.

### Rights are not missingness

Only `AuthorizedAndDisclosed` origins are eligible. `WithheldByRights`, `NoResponse` and `Unknown` return `INDETERMINATE` without contribution metrics. They must not be converted into:
- a zero contribution;
- evidence that the holder had nothing relevant;
- a failure by the holder or team;
- permission to pressure the holder into disclosure.

## Metrics

For a qualified paired replay:

- `decision_changed`: whether the decision digest changed;
- `confidence_delta`: counterfactual confidence minus baseline confidence;
- `decision_quality_delta`: counterfactual quality minus baseline quality;
- `removed_origin_count`: number of source origins in the authenticated closure.

A negative quality delta indicates that the omitted origin supported higher-quality performance in this specific replay. A positive delta may indicate that the origin was misleading, stale, irrelevant or poorly integrated. No change may indicate redundancy or non-use. The validator does not choose among these explanations; inspect the evidence trace and run matched replications.

## Interpretation limits

A paired replay is a **mechanistic information-contribution test under frozen replay assumptions**. It is not the causal effect of a person, team member, conversation or intervention. It does not establish that an origin was true, independent, complete or decision-relevant beyond the authenticated registry and evaluator evidence. It does not measure human memory or capability retention.

For model/system studies, replicate across tasks and seeds and report the distribution of effects by dependency class. For human/team effects, use a separate preregistered team-randomized design, analyze teams as the assignment unit and do not treat multiple replays as independent participants.

## Target ladder

Prospective targets, not achieved results:
- 100% rejection of duplicate or repeated-source inflation in deterministic fixtures;
- 100% detection of missing/altered source registry and removal-closure evidence;
- zero false contribution scores for rights-withheld, unanswered or unknown origins;
- 100% strict manifest parity for paired replay comparisons;
- 100% exact removal of authenticated descendants, with zero unrelated source-origin removal;
- report decision-change, confidence and quality deltas separately; do not collapse them into a single contribution score.

## Ownership

- Mycelix #6089 owns distributed information acquisition, rights-aware inquiry, evidence delivery and source provenance.
- Mycelix #6619 owns the source-origin-aware evidence ledger and strict comparison validator.
- Mycelix #6597 owns team-level capability and knowledge retention after AI withdrawal.
- Mycelix #6616 owns randomized information-order and anchoring experiments.
- Symthaea SAT #13574 consumes these evidence and human-outcome contracts; it owns the wider system-level sovereignty and adaptive-control tournament.

## Claim ceiling

A structural `PASS` means only that a paired replay satisfies the declared manifest, rights, source-origin and removal-closure contract. It does not prove a causal human effect, factual truth, production provenance authentication, AI-off isolation or SOTA superiority.
