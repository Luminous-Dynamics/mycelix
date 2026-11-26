# Task-Conditioned Workflow Memory Boundaries v0.1

**Status:** structural reference validator and synthetic paired isolation fixture. No CoMemBench result, production isolation guarantee or SOTA claim is asserted.

## Research motivation

CoMemBench (September 2026 preprint) evaluates 800 executable workflows across four domains and tests collaborative-memory boundaries under matched isolation challenges. It separates terminal success, verified node progress, required-handoff reliability, isolation challenge success and token cost. Its workflow motifs include chain, split–join, reuse, overlap and bounded revision; its corruption operations include sibling crosswire, incompatible payload, unverified result, completed-as-pending, wrong entity and stale version. Source: https://arxiv.org/abs/2609.32192

This is complementary to evidence retrieval. A fact can be relevant to the overall project but unauthorized or inapplicable to one particular workflow node. Mycelix must qualify the handoff at the actual task boundary, not treat semantic relatedness as permission.

## Contract

A workflow graph binds:
- workflow and benchmark identities;
- task nodes and their task digests;
- producer→consumer handoff edges;
- artifact type, entity-scope digest, required version, verification requirement and required/optional state.

Every artifact binds:
- producer node;
- payload digest;
- artifact type and entity scope;
- version and current/superseded lifecycle;
- verification state and (when verified) a verification receipt;
- operation state, where side effects are involved.

Every observed consumer exposure binds:
- consumer node and artifact ID;
- operation state as reported by the consumer;
- consumed / rejected-inapplicable / rejected-stale / rejected-unverified / rejected-wrong-entity / rejected-wrong-branch / rejected-completed-as-pending / unknown disposition;
- exposure receipt and, if consumed, an effect receipt.

The graph digest is domain-separated and deterministic over sorted nodes and handoff contracts.

## Qualification challenge

A pair contains a clean workflow and a polluted workflow. They must match on workflow graph, task, model, evaluator, source-group cluster, random seed and resource budgets, plus the frozen artifact catalog and node declarations. The polluted run must differ by exactly one additional exposure of the declared artifact to the declared consumer.

The declared mutation operator must match the actual mismatch:
- Sibling crosswire: no producer→consumer contract exists.
- Incompatible payload: producer/consumer path exists but the artifact type differs.
- Unverified result: the contract requires verification but the artifact is not verified.
- Completed-as-pending: the artifact operation is completed but the consumer is told it is pending.
- Wrong entity: the artifact's entity-scope digest differs from the consumer contract.
- Stale version: artifact version is below the required version or its lifecycle is superseded.

The clean target node must have a verified-completion receipt. Otherwise the pair is INDETERMINATE for isolation scoring; failure to solve the clean task cannot count as successful rejection of pollution.

A pollution exposure that is consumed fails isolation. An exact expected rejection passes. An unknown disposition remains INDETERMINATE.

## Outcomes — report separately

- Terminal workflow success (SR).
- Verified Node Completion Rate (VNCR).
- Verified Handoff Success (VHS).
- Isolation Challenge Success (ICS).
- Required-handoff omission and incorrect artifact consumption.
- Exposure-trace coverage and disposition counts.
- Repeated side-effect attempts for completed-as-pending artifacts.
- Latency, tokens and total execution/evaluator cost.

Do not combine completion and isolation into one score. More context may improve progress while increasing cross-task contamination; this is precisely the trade-off to measure.

## Integration ownership

- #6089: source acquisition, rights and delivery.
- #6619: evidence lineage, provenance and independent response audit.
- #6688: relational evidence refinement; it cannot expand task-graph consumer eligibility.
- #6700: signed generation-input exposure.
- #6701: relation-edge attribution.
- #6706: inquiry × RER × ACB factorial effects.
- #6710: CoMemBench research owner.
- This module: graph-bound artifact handoffs and paired clean/polluted isolation qualification.

It does not create an alternate memory store, authorization engine, workflow executor or semantic truth oracle.

## Proposed targets (not achieved results)

- 100% deterministic rejection or INDETERMINATE classification for each declared synthetic corruption operator.
- 100% exact graph and artifact identity/version binding.
- Zero unauthorized artifact consumption in the deterministic isolation corpus.
- 100% accounting of required handoffs with valid execution/effect receipts.
- No isolation credit when the clean target node did not succeed.
- Report metrics separately by topology, domain and corruption operation, with workflow-group clustered intervals and sealed holdouts.

Zero failures in a finite synthetic corpus is not a deployment security guarantee. A structural PASS validates the declared graph and receipts, not artifact truth or a compromised executor/witness.
