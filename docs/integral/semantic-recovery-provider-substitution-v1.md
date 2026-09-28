# Semantic recovery and cross-provider substitution continuity v1

Status: **ReferenceModelOnly**

D6K established that archives can preserve historical evidence and seed a qualified cold start without becoming current authority. D6L carries that boundary into provider recovery and substitution.

The governing distinctions are:

```
provider operation identity
!=
Mycelix semantic effect identity

unknown outcome
!=
proof of no effect

historical reconstruction input
!=
current execution authorization
```

## Stable semantic effect

`SemanticEffectV1` is the semantic identity retained across route changes. It binds:

- stable Mycelix effect ID;
- lineage and lifecycle generation;
- request commitment and semantic environment;
- effect class;
- exact resource and tenant;
- exact amount and unit representation;
- authority and consent claim IDs;
- idempotency key.

`SemanticSubstitutionProfileV1` repeats the contract fields as an independently checkable boundary and allow-lists exact provider profile roots. Values are compared exactly. This reference model does not normalize currencies, units, amounts, resource aliases, or provider-specific interpretations.

A provider route has its own route ID and provider operation ID. Provider operation IDs are scoped by provider identity and are not semantic effect IDs. Route generations must advance exactly one step for a qualified transition. A provider operation ID cannot alias the semantic effect ID.

## Outcome semantics

`ProviderOutcomeV1` distinguishes:

- `NotStarted`;
- `RejectedWithoutEffect`;
- `Pending`;
- `Succeeded`;
- `FailedWithPossibleEffect`;
- `Unknown`.

A timeout or transport error must not be silently converted into `NotStarted`. A successful outcome blocks a duplicate continuation. An explicitly evidenced no-effect outcome permits continuation only when the substitution profile permits retry after no effect.

For an uncertain, pending, or possibly-applied outcome, continuation requires one of two distinct forms of evidence:

1. an `OutcomeResolutionV1` bound to the exact prior outcome, route, provider profile, effect, request, idempotency key, semantic environment, and substitution profile, resolving it as `NotApplied`; or
2. a `CrossProviderIdempotencyWitnessV1` under a `ContractWide` idempotency policy, binding the same effect/request/key and both exact provider profiles.

A resolution of `Applied` blocks another effect. An `Unresolved` result remains blocked. Provider-scoped idempotency alone does not qualify a cross-provider retry. The reference model does not verify signatures or independently establish the truth of provider evidence.

## Effect continuity receipt

`EffectContinuityReceiptV1` binds:

- the stable effect ID;
- predecessor route and outcome;
- predecessor and successor provider identities, profile roots, and operation IDs;
- route generations;
- lifecycle generation;
- exact request and idempotency key;
- semantic environment and substitution profile;
- predecessor observation frontier and successor route frontier;
- explicit transition evidence;
- any resolution or idempotency witness IDs;
- a continuity commitment and fixed claim ceiling.

The assessment rejects route, operation, request, profile, frontier, lifecycle, or receipt mismatch. A route transition preserves the same semantic effect; it is not a new effect and is not itself authorization to execute externally.

## Lifecycle and replay integrity

A route may continue only for the currently qualified lifecycle generation. A D6I tombstone for that lineage and generation blocks continuation. The provider-operation ledger keys operation identity by provider ID plus provider operation ID. Exact re-use is classified as replay; re-use for a conflicting effect/route is a conflict.

A successful substitution must preserve existing effect, authority, capacity, and consent claim sets exactly. Any allocation, consent, authority, or lifecycle change requires its own Mycelix semantic transition and cannot be smuggled through failover.

## Archive recovery

`assess_recovered_effect_input` validates D6K archive binding against the D6J cold-start manifest and reconstruction receipt. It checks exact archive snapshot/frontier, environment, historical profile, effect/request, and lifecycle-generation bindings before returning `UsableReconstructionInputOnly`.

The recovery assessment unconditionally blocks `CurrentProviderExecution`. Recovered state is an input to reconstruction and subsequent current Mycelix qualification; it is not permission to send a new request to a provider. A stale or mismatched archive cannot be relabelled as the source for another frontier.

## Symthaea and Mycelix boundary

Symthaea may analyze provider behavior, identify semantic drift, compare route evidence, detect uncertain outcomes, and propose a substitution candidate. It cannot:

- mint or change the stable semantic effect ID;
- infer that a timeout means no effect;
- treat equal visible results as provider equivalence;
- waive resource, tenant, amount, unit, authority, consent, or profile bindings;
- resolve an uncertain outcome without scoped evidence;
- promote archive/reconstruction evidence to current authority;
- clear a D6I tombstone or revive retired capacity;
- authorize current actuation or external execution.

Mycelix remains the semantic root and qualifies current authority, consent, capacity, and effect transitions.

## Adversarial coverage

The reference tests cover unknown and pending outcomes, no-effect continuation, exact outcome resolution, contract-wide versus provider-scoped idempotency, provider-profile drift, semantic field drift, mismatched resolution/receipt bindings, route-generation gaps, tombstoned generations, operation-ID aliasing/replay, claim conservation, and archive-only reconstruction boundaries.

## Claim ceiling

This is deterministic reference semantics only. It does not establish:

- exactly-once external execution;
- durable or globally shared idempotency;
- provider honesty or cryptographic authenticity;
- real-world outcome truth;
- Byzantine fault tolerance;
- legal authority or consent;
- production failover, storage, or distributed-system safety.

The continuity and evidence roots are opaque commitments in this model; their cryptographic construction and verification are outside scope. A source-level test is not an executed test. Execution evidence is required before any claim ceiling is raised.
