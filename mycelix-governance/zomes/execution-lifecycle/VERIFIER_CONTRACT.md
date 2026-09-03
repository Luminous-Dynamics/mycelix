# Governance Execution Lifecycle Verifier Contract v0.1

Status: **normative interface contract; implementation intentionally absent**

Expected zome name:

`governance_execution_lifecycle_verifier`

This verifier is a composition boundary, not a new governance authority source. It must derive every positive receipt from already authoritative, independently verified facts.

## Design rule

The verifier MUST NOT turn any of the following into binding execution authority by itself:

- caller identity;
- timelock creator identity;
- existence of a DHT record;
- `verified: true` fields accepted from callers;
- Guardian/council membership alone;
- Phi/consciousness metrics;
- reputation or stake;
- Symthaea/model recommendations;
- timestamps or DHT arrival order.

## Required endpoint 1 — `resolve_execution_domain`

Input:

`proposal_id`

Output protocol:

`mycelix-governance-execution-domain-verifier-v0.1`

The positive receipt must contain:

- exact proposal ID;
- complete `ExecutionDomain`;
- recomputable domain digest;
- verification reference/provenance;
- non-zero verification time.

### Required derivation

The domain must be reconstructed from current authoritative facts rather than caller-supplied domain data.

At minimum:

1. resolve the exact immutable/canonical execution plan for the proposal;
2. call `governance_execution_preflight.verify_execution_preflight` on the exact executable action bytes;
3. require the exact current constitutional epoch/proposal authority/approved binding tally/action digest represented by that preflight;
4. obtain an independently verified threshold execution authorization for those exact action bytes;
5. obtain an independently verified executor designation;
6. obtain an independently verified effect-safety policy; and
7. construct the domain only when all references describe the same proposal/action/authority epoch.

### Execution-plan identity

Do not derive `timelock_ref` from mutable `Timelock.status` or blindly from the latest timelock update.

The legacy timelock integrity currently permits mutable fields beyond status. Until that compatibility surface is repaired, the lifecycle domain should use a canonical execution-plan reference bound to the exact proposal + verified action digest, or another independently immutable plan record.

The preflight must still verify the exact action bytes.

### Threshold execution authorization

The threshold receipt must bind at least:

- proposal ID;
- exact executable-action digest/profile;
- committee/institution authority;
- signing-policy digest/profile or epoch;
- threshold/key epoch;
- signature/authorization identity;
- active validity at verification time.

A legacy `verified: true` field is not enough. Cryptographic/institutional verification must occur behind this boundary or an independently trusted verifier boundary.

### Executor designation

The executor designation must bind:

- proposal/execution domain purpose;
- exact executor principal/workload DID;
- granting institution/authority;
- exact capability/scope;
- validity/revocation state;
- rulebook/policy reference; and
- proof/verification reference.

The timelock creator is not automatically the executor.

A signing committee member is not automatically the executor.

An intelligent/advisory system is not automatically the executor.

### Effect-safety policy

The safety-policy receipt must bind the exact policy digest/profile placed into the domain.

A policy should describe which effect classes may be automated and the required fencing mechanism, for example:

- downstream idempotency required;
- deterministic idempotency-key format;
- trusted single-writer/fencing service required;
- adapter supports exactly-once-like deduplication for the named operation class;
- operations are non-idempotent and therefore automatic execution is forbidden.

The verifier must not claim a mechanism exists merely because a policy requests it.

## Required endpoint 2 — `verify_lifecycle_event`

Input must bind:

- proposal ID;
- exact candidate action/reference;
- exact current `ExecutionDomain`;
- exact `LifecycleEvent`.

Positive output protocol:

`mycelix-governance-execution-event-verifier-v0.1`

A positive receipt must echo/bind:

- exact candidate action/reference;
- exact execution-domain digest;
- exact event ID;
- exact event actor;
- authority-class-specific verification reference; and
- verification time no earlier than the event.

### Event authority classes

#### `Registered`

Registration asserts that this exact current execution domain has entered lifecycle tracking.

It may be published by the bound executor or another explicitly authorized registration actor, but the publisher gains no execution authority merely by registering it.

#### `ReadyAuthorized`

Must be backed by fresh execution admission for the exact domain:

- current constitutional epoch;
- current proposal authority;
- approved binding tally;
- exact action digest;
- exact threshold execution authorization; and
- current executor designation.

A Ready event is not a long-lived lease. Final effect execution must later reverify current authority.

#### `Claimed`

Must require:

- exact bound executor principal;
- exact executor authority still active;
- uniquely verified Ready parent;
- exact current domain; and
- exact effect-safety policy verified/enforceable.

The claim event itself is a durable pre-effect fence, not evidence that any external effect happened.

#### `Completed`

Must require:

- exact bound executor principal;
- exact deterministic attempt ID;
- exact causal claim parent;
- durable external effect receipt/evidence matching the claimed action domain.

#### `Failed`

Must require exact executor/attempt/claim binding plus evidence supporting the declared failure class.

`NoEffectObserved` must not be asserted merely because no local completion record exists.

`EffectMayHaveOccurred` is the safe classification when the external result is uncertain.

#### `Uncertain`

Must require exact executor/attempt/claim binding plus reconciliation evidence showing outcome ambiguity.

It is terminal for automatic execution in v0.1.

#### `Cancelled`

Must be backed by an explicit cancellation/review authority under the governing rulebook.

Guardian status, Phi, reputation, stake, or a model warning can be evidence/request inputs but cannot directly mint cancellation authority.

Cancellation after `Claimed` is not valid in lifecycle v0.1; after claim, reconciliation/terminal outcome semantics apply instead.

## Required endpoint 3 — `verify_effect_safety_policy`

Input must bind:

- proposal ID;
- exact execution-domain digest;
- exact executor principal;
- exact executor-authority reference;
- exact effect-safety policy digest/profile.

Positive output protocol:

`mycelix-governance-effect-safety-verifier-v0.1`

A positive receipt must echo the exact domain/executor/policy and include a non-empty verification reference plus current verification time.

This endpoint answers only whether the required safety mechanism is actually available/enforceable for the exact domain. It does not itself authorize the proposal or executor.

## Freshness

All verifier receipts are runtime facts, not timeless certificates.

A consuming boundary must reject:

- zero verification timestamps;
- verification timestamps in the future;
- expired/revoked underlying authority;
- stale constitutional or signing epochs;
- changed executor designation;
- changed safety-policy configuration.

## Failure semantics

Any of the following is denial:

- verifier zome/function absent;
- call failure;
- decode failure;
- missing required authority source;
- malformed receipt;
- inexact proposal/domain/event binding;
- unsupported policy/profile;
- stale or revoked authority;
- ambiguous constitutional/lifecycle lineage;
- inability to prove the required effect-safety mechanism.

There is no best-effort fallback for binding execution.

## Non-goal

This verifier does not execute effects.

It also does not replace the pure lifecycle projector. Its job is to decide which candidate events are qualified inputs to that projector and to reconstruct the exact current execution domain from existing authority sources.
