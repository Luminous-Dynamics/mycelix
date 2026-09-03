# Binding Governance Execution Lifecycle v0.1

Status: **normative pre-runtime contract**

This crate replaces mutable execution status as the target authority model for binding governance. It is intentionally wire-neutral and contains no Holochain, Symthaea, Phi, reputation, stake, model, network, or external-effect dependency.

## Authority model

A lifecycle event is not authoritative because it exists or deserializes.

A host must independently verify the institutional authority named by each event and only then construct `VerifiedLifecycleEvent`.

Advisory signals may request review or contribute evidence. They cannot directly mint `ReadyAuthorized`, `Claimed`, `Cancelled`, `Completed`, `Failed`, or `Uncertain` authority.

## Exact execution domain

Every lifecycle is bound to one exact `ExecutionDomain` containing:

- proposal ID;
- timelock/reference identity;
- exact proposal-authority reference;
- exact constitutional epoch digest/profile;
- exact executable-action digest/profile;
- exact verified binding-tally reference;
- exact threshold-authorization reference; and
- exact effect-safety policy digest/profile.

The effect-safety policy commits the runtime mechanism under which real-world effects are allowed, for example mandatory downstream idempotency or a trusted serialized single-writer fence. The pure kernel binds this policy but does not pretend to implement an external guarantee itself.

Changing any authority-bearing or effect-safety component creates a different execution domain. Old claims cannot carry forward.

## Append-only lifecycle

The v0.1 semantic path is:

`Registered -> ReadyAuthorized -> Claimed -> Completed | Failed | Uncertain`

Before claim, governance may instead create an authorized `Cancelled` event.

There is no mutable authoritative status field.

There is no automatic retry transition in v0.1.

## Durable pre-effect fence

`Claimed` is the execution fence.

The **claim event ID** identifies the exact immutable claim publication and remains part of lifecycle causality.

The **attempt ID** is different: it is deterministically derived from the exact execution-domain digest plus the exact `ReadyAuthorized` event ID. It deliberately excludes claim publisher, claim timestamp, and claim nonce.

This means two concurrent claim publications from the same exact Ready authority still create `AmbiguousLifecycleFork`, but they derive the same downstream attempt/idempotency domain rather than becoming two independent real-world attempts.

A runtime must:

1. freshly verify constitutional epoch, proposal authority, binding tally, executable action digest, threshold authority, and the exact effect-safety policy;
2. commit the immutable claim in one successful persistence transaction;
3. return the deterministic attempt ID only after that commit succeeds;
4. perform external effects only in a later invocation that re-resolves the exact claim and current authority;
5. derive downstream idempotency keys from `attempt_id + action_index` where supported; and
6. refuse automatic effect execution when the bound safety policy cannot actually be enforced.

A claim and an external effect MUST NOT be treated as one atomic operation. If an effect can occur outside the persistence transaction, the runtime must assume the effect may succeed even if later local work fails.

## Concurrency semantics

A DHT projection alone is not a global mutex. Two callers can race before either observes the other's claim.

Therefore v0.1 requires defense in depth:

- competing verified claims ultimately freeze lifecycle projection;
- all claims from the same exact Ready event derive the same deterministic attempt ID;
- downstream idempotency keys therefore converge across such a race;
- adapters that cannot enforce the bound effect-safety policy must refuse automatic effects rather than relying on eventual fork discovery.

For non-idempotent external systems, a later runtime must use a trusted serialized/single-writer fencing mechanism committed by `effect_safety_policy` before claiming automatic-execution safety.

## Crash semantics

- crash before durable claim: no execution authority was consumed;
- crash after claim but before effect: the claim remains consumed; no automatic second claim;
- crash while/after an external effect where completion is not positively known: project `Uncertain` after reconciliation evidence is available;
- `Uncertain` is terminal for automatic execution;
- `Failed(NoEffectObserved)` is also terminal in v0.1; any later retry protocol must require new explicit governed authority.

The design prefers at-most-once automatic execution plus explicit reconciliation over duplicate real-world side effects.

## Projection rules

Projection consumes the complete verified event set for one exact execution domain.

- exact duplicate semantic publication is harmless;
- every non-root verified event must reference a present parent;
- exactly one root may exist;
- the root must be `Registered`;
- every verified event must be reachable from that root;
- child time may equal or follow parent time, but may not predate it;
- timestamps never choose between competing children;
- DHT arrival order never chooses between competing children;
- publisher identity, Phi, reputation, stake, or model output never chooses a child;
- two distinct verified children of one parent are `AmbiguousLifecycleFork`;
- competing `Completed`, `Failed`, or `Uncertain` outcomes also freeze rather than being timestamp-resolved;
- any child after a terminal state is invalid.

## Attempt binding

`Completed`, `Failed`, and `Uncertain` must:

- be causal children of the exact `Claimed` event; and
- reference the deterministic attempt ID derived from the claim's exact execution domain and Ready parent.

A terminal event referring to another attempt is invalid even if proposal/action data look similar.

## Legacy mutable timelock state

Until the Holochain adapter is implemented, existing `Timelock.status` remains compatibility data only and MUST NOT become the long-term authority source for binding execution.

The target adapter must derive binding state from verified immutable lifecycle events and must fail closed on lifecycle ambiguity.

## Legacy veto / Phi authority

The existing Guardian veto / Phi-weighted override path is not part of this lifecycle authority model.

A future review/cancellation protocol may accept advisory or safety signals as inputs, but a binding `Cancelled` or review event must be justified by explicit institutional authority under the relevant rulebook. Phi, reputation, model scores, or consciousness metrics cannot directly create or restore execution authority.

## Required runtime tests

Before replacing mutable timelock status as the production authority source, integration tests must cover:

- unauthorized direct timelock updates do not affect projected binding state;
- two callers racing to claim produce a fail-closed ambiguity while deriving the same deterministic attempt/idempotency domain;
- a claim is durable before a mocked external effect begins;
- crash after claim does not create an automatic second claim;
- crash after effect but before completion produces reconciliation/uncertainty semantics rather than replay;
- changed constitution, action digest, tally, proposal authority, threshold authority, or effect-safety policy cannot reuse an old claim;
- DHT reordering does not change projection;
- missing-parent and partial verified sets deny;
- competing terminal outcomes deny;
- deterministic downstream idempotency keys remain stable for the same exact attempt;
- an adapter unable to enforce the bound effect-safety policy refuses automatic effects; and
- advisory scores cannot authorize lifecycle events.
