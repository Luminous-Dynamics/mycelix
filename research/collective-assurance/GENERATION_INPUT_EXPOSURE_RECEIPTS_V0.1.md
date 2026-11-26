# Generation Input Exposure Receipts v0.1

**Status:** additive reference validator and research protocol, stacked on Mycelix #6688. No production witness, model-runtime instrumentation, benchmark result, or SOTA claim is asserted.

## Why this increment exists

The RER/ACB path in #6688 correctly separates candidate selection from response-constraint application. Its bottleneck diagnostic, however, receives a 64-character generation-input receipt digest as an argument. Digest shape alone does not establish that the receipt authenticates the exact evidence inventory supplied to the diagnostic.

This increment adds a typed, signed receipt path. It reuses the signature-verifier and consume-once replay-guard interfaces already defined by Collective Assurance; it does not create a second evidence ledger, cryptographic implementation, permission system, source registry, or model runtime.

## Contract

The generation-side witness emits claims that bind:

- run and generation identity;
- request, responder, and purpose;
- a challenge nonce and pinned signer key ID;
- model snapshot and prompt-template digests;
- the digest of the exact serialized input bytes observed at the model/runtime boundary;
- a timestamp inside an independently preregistered freshness window; and
- the ordered evidence inventory exposed to generation, including request, evidence ID, source digest, subject, scope, authorized responder, status, applicability, verification, and required state.

The claims use a versioned, domain-separated, length-prefixed encoding. The signature verifier validates the signed bytes against a pinned trust policy. The replay guard atomically consumes the run/phase/nonce tuple. An unavailable verifier or replay guard is INDETERMINATE; a malformed, mismatched, invalidly signed, or replayed receipt is FAIL.

The higher-level diagnostic then compares the signed ordered evidence inventory against the actual AvailableEvidenceConstraint inventory supplied to the RER/ACB bottleneck classifier. A mismatch emits no bottleneck classification. Only after the signature, replay, identity, freshness, and exact-inventory checks pass does it invoke the existing #6688 diagnostic.

## Pipeline placement

authorized discovery → retrieval → RER → ACB constraint construction → witnessed generation-input exposure → response grounding → independent rendered-claim audit

Ownership remains separated:

- #6089 owns discovery, inquiry, delivery, rights, and provenance.
- #6619 owns the evidence ledger, source-origin lineage, ACB-shaped constraint binder, and independent response audit.
- #6688 owns bounded relational evidence refinement and the initial retrieval/handoff/application diagnostic.
- This increment qualifies the generation-input exposure boundary and binds its receipt to the exact evidence inventory used by that diagnostic.
- The external witness owns observation of actual runtime input bytes; the configured verifier and replay guard own signature and nonce trust.

## Fail-closed invariants

1. The expected run, generation, request, responder, purpose, signer, model, prompt, nonce, and timestamp window are pinned independently of the candidate receipt.
2. Every evidence item in the signed inventory is unique, request-bound, authorized for the intended responder, digest-valid, verified, and marked required.
3. The signature covers the exact input payload digest and the ordered evidence inventory; changing either invalidates the signature.
4. The exact inventory presented to the diagnostic must equal the signed inventory, including ordering and all constraint fields.
5. A generation receipt nonce is consumed once. Reuse fails; replay-store outages are indeterminate.
6. A mismatch suppresses downstream classification. The system must not report “no gap” when the handoff receipt and supplied inventory disagree.
7. The claims digest is a content identifier, not proof of witness identity. Identity depends on a separately trusted signature verifier and pinned trust policy.

## Trust and claim boundaries

A PASS establishes that the supplied external verifier accepted a signed, fresh, non-replayed claim and that its ordered evidence inventory exactly matches the inventory passed to the downstream diagnostic.

It does **not** independently prove:

- that the witness observed the actual model input rather than a caller-provided reconstruction;
- that the witness host, runtime hook, signing key, or replay store is uncompromised;
- that the model attended to, understood, or used an exposed evidence item;
- that source content or a relation is factually true;
- that natural-language claims were exhaustively extracted or correctly audited; or
- that the answer or collective decision improved.

The external witness must capture the exact serialized request at a declared runtime boundary, hash it before model execution, preserve the inventory-to-input mapping, and emit a signed receipt. Production deployment requires independently operated witness infrastructure, key rotation/revocation, hardware/host qualification where appropriate, durable replay protection, clock/freshness policy, receipt retention, and negative-control tests. The new module supplies contracts, not that infrastructure.

The encoding is versioned reference-test encoding, not canonical JSON or a general wire standard. Production implementations must pin the schema and encoding version and publish interoperability vectors. The test signature adapter merely echoes signed bytes; it is not cryptography.

## Fault-injection and benchmark plan

Add this as an instrumentation factor to the GroupAssistBench RER/ACB ablation, not as a replacement benchmark:

| Arm | Retrieval/refinement | Constraint binding | Exposure receipt |
|---|---|---|---|
| A | baseline retrieval | baseline response path | existing measurement |
| B | RER | baseline response path | existing measurement |
| C | baseline retrieval | ACB | witnessed inventory |
| D | RER | ACB | witnessed inventory |
| E | RER | ACB | signed, replay-protected exact-inventory receipt |

For the primary RER/ACB causal comparison, hold the receipt mechanism fixed across the relevant arms; report the additional receipt arm as a separate instrumentation intervention so that better observability is not confused with better answer quality.

Adversarial cases must include: changed input digest after signing; omitted, added, duplicated, reordered, or substituted evidence; cross-request and cross-recipient reuse; purpose mismatch; model/prompt mismatch; stale/future receipts; invalid signer; replayed nonce; truncated inventory; out-of-band visible evidence; and verifier/replay-store outage.

Report at least:

- receipt qualification rate and INDETERMINATE rate;
- false acceptance and false rejection on a preregistered fault corpus;
- exact-inventory mismatch detection;
- replay detection and nonce-store unavailability;
- evidence handoff loss and out-of-band exposure;
- response constraint application and independent claim-grounding outcomes;
- p50/p95 latency and incremental witness/storage cost; and
- answer quality, over-refusal, privacy, agency, and total deployment cost as separate outcomes.

Synthetic qualification targets (not achieved results):

- 100% rejection of all deterministic, preregistered integrity mutations in the test corpus;
- 100% of inventory substitutions classified as GeneratorInventoryMismatch or ExposureNotQualified, never as a clean result;
- zero accepted nonce replays in the tested replay corpus;
- 100% fail-closed behavior when signature or replay services are unavailable;
- no answer-quality claim based solely on improved receipt coverage.

For random fault-injection trials, publish the sampling frame, independent fault unit, confidence interval, and cluster structure. Zero observed false accepts in a finite corpus is not proof of zero deployment risk.

## Competitive positioning

GroupAssistBench evaluates both recall (L1) and open-ended assistance (L2), including single, distributed, and conflicting memories (M1–M3). Mycelix should reproduce the official version, splits, scorers, and matched baselines before comparing. Its differentiated contribution should be a trace of where evidence is lost or misapplied, with signed exposure telemetry and explicit uncertainty—not a claim that receipt plumbing alone raises L2.

Compose with, rather than collapse into, the other benchmark families:

- GroupMemBench for multi-party updates, ambiguity, temporal reasoning, and abstention;
- COLLABMEM for sharing, access control, routing, provenance, conflict resolution, and retention;
- HiddenBench for distributed-information discovery and latent information asymmetry;
- Common Ground for shared/private task-state access, joint action, and repair;
- AgentDojo/AgentDyn and related adversarial suites for tool, permission, and effect-sink attacks.

Every result card must bind the benchmark-definition certificate, exact run manifests, model and prompt snapshots, data/split identity, scorer/judge version, receipt trust profile, cost telemetry, and statistical inference plan. Questions from the same source group must remain clustered in uncertainty estimates.

## References

- GroupAssistBench, One Assistant, Many Memories: Benchmarking Real-World Group Assistance When Recall Is Not Enough, arXiv:2611.04867: https://arxiv.org/abs/2611.04867
- Kim et al., Capable language models can outgrow the benefits of collaboration, Nature Machine Intelligence (2026): https://www.nature.com/articles/s42256-026-01268-y
- COLLABMEM, EMNLP 2026: https://aclanthology.org/2026.emnlp-main.1194/
- HiddenBench, PMLR 2026: https://proceedings.mlr.press/v306/li26ej.html
- Mycelix #6619, #6688: https://github.com/Luminous-Dynamics/mycelix/pull/6619 and https://github.com/Luminous-Dynamics/mycelix/pull/6688
