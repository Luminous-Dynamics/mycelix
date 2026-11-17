# Protected Effect Trace Contract (PETC) v0.1

**Status:** research contract draft. It defines a normalized trace envelope; it is not a trust protocol, event store, cryptographic attestation scheme or source of legal authority.

## Why this is a separate artifact

AuthorizationBench and SABench establish two important things: user/task authorization can diverge from task success, and successful agents can still cause unauthorized side effects. LISE adds a different boundary: whether the exact external operation was authorized at effectuation time, whether the effect occurred, and whether evidence is adequate to reconstruct it.

PETC gives all three a shared event vocabulary without flattening their semantics. The schema is at [IPB_PROTECTED_EFFECT_TRACE.schema.json](./IPB_PROTECTED_EFFECT_TRACE.schema.json).

## Event model

Each event records:
- sequence and stable event identity;
- event time (occurred_at) and observation/record time (recorded_at);
- source class and capture origin;
- optional correlation and causal links;
- typed principal/session/operation/approval/constitution/policy/resource/route/provider/effect/claim references;
- separate provider outcome, protected-effect state and observation qualification.

occurred_at and recorded_at are not interchangeable. Bitemporal claim facts may additionally carry effective_from, effective_to and knowledge_time.

## Source qualification

A trace event is a statement from a source, not automatically a fact. source_kind says who or what is making the statement; capture_origin says how it entered the trace. An independent_harness event may provide stronger capture provenance than candidate_reported, but independence still depends on the trust boundary, dependencies and what the harness can actually observe.

Provider acknowledgement, provider status query, owner reconciliation and independent observation are distinct event types and source claims. A signed provider acknowledgement remains provider-reported evidence, not a physical observation of the protected resource. Multiple source references with a common dependency do not become independent corroboration.

## Required evaluator checks beyond JSON Schema

JSON Schema validates shape. A PETC trace validator must additionally:
1. enforce unique event IDs and strictly increasing sequence numbers;
2. verify causation_event_ref and all event/action/operation references resolve;
3. check event-type-specific required fields (for example, provider_dispatch needs an operation, attempt and provider contract reference; protected_effect_observation needs an action/effect and observer/source lineage);
4. check that every expected action ID is represented exactly once or is explicitly missing/unknown at reconciliation;
5. reject an observed action that is not in the frozen expected action set unless it is separately classified as an unauthorized extra effect;
6. keep proposal, approval, policy decision, preflight, dispatch, acknowledgement, protected effect and reconciliation as distinct stages;
7. verify that retries follow confirmed no-effect plus provider-qualified same-operation/same-key semantics; an unknown outcome never authorizes a blind retry;
8. preserve effective-time and knowledge-time views without rewriting earlier as-known decisions;
9. verify that every claim of independent observation has source-dependency evidence; otherwise downgrade to unknown/not_assessed;
10. bind the exact trace bytes to the submission's SHA-256 receipt and the exact system/harness/configuration digests.

These checks require source contracts or raw observations; schema validation alone cannot establish them.

## Cross-benchmark mapping

| Existing benchmark surface | PETC mapping | What must remain distinct |
|---|---|---|
| AuthorizationBench task and authorization trace | task, principal, operation, authorization scope, action events | User-granted scope and institutional authority are not the same object |
| SABench goal/task oracle and write trace | task, expected action IDs, tool/action, target, effect state, collateral action | Goal satisfaction is separate from action authorization and effect evidence |
| OPA / Cedar policy evaluation | policy decision, policy revision, source and diagnostics reference | PDP result is not proof the PEP enforced it or the provider effect occurred |
| Zanzibar-style relationship authorization | principal/resource/relationship snapshot, revision or consistency token | Consistent ACL result is not institutional adjudication or provider finality |
| Mycelix / LISE | constitution epoch/digest, owner authority, exact operation digest, PEP route, dispatch, observation, reconciliation | Owner-resolved facts and provider-specific guarantees remain explicit |

For external benchmark reuse, build adapters over legally reusable source artifacts and preserve each benchmark's own oracle. PETC is a transport/normalization contract for trace evidence, not a way to relabel their scores as LISE scores.

## Safety and claim ceiling

- An absent event is not evidence that the event did not happen unless the capture boundary is demonstrably complete.
- A hash binds bytes; it does not prove truth.
- A DENY at a policy engine does not prove the application or alternate route enforced it.
- A provider acknowledgement does not prove the protected effect's final state.
- A trace contract does not authorize a provider action or make an operation atomic.
- PETC conformance is structural/event-semantic evidence, not legal validation, production certification or SOTA superiority.

## Research next step

Implement an independent trace validator that checks the event graph, stage order, expected action coverage, cross-reference integrity and evidence lineage; then replay minimized counterexamples against the fixed LISE panel and matched AuthorizationBench/SABench subsets.
