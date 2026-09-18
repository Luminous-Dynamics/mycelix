# Constitutional Lifecycle Quiescence v0.1

Status: experimental, non-activating bounded formal tranche for MYC-CONST-003C3.

Parent lifecycle subject: `b19667a0399d46e7a21254e931c3149cfbd994fc` (qualified by exact verifier/run evidence).

## Purpose

The qualified claim-lifecycle model establishes bounded safety and explicit lifecycle resolution. This tranche asks a different question: when the bounded protocol is not making an internal constitutional transition, is it legitimately waiting for the environment, intentionally halted, already resolved, at the model horizon, or genuinely stalled despite already having the authoritative inputs needed to progress?

It does **not** reinterpret the qualified lifecycle head and does not claim production liveness.

## Two event classes

The model separates events by provenance.

### ExternalInputStep

External events can make future resolution possible but are not counted as constitutional resolution progress:

- claim submission;
- authenticated finality becoming available;
- revocation evidence becoming available;
- closure evidence becoming available;
- effect executor/dependency becoming available;
- fault evidence becoming available.

Arrival of evidence is not the same thing as exercising constitutional authority over that evidence.

### InternalResolutionStep

Internal resolution steps consume already-present authoritative state:

- resolve accepted finality;
- apply a ready revocation to live pending claims;
- terminalize blocked claims when closure evidence is already present;
- enter an integrity halt when fault evidence is already present;
- apply a finalized effect when its executor/dependency is already available.

## Resolution obligation

`ResolutionObligation` is intentionally defined independently from the `InternalResolutionStep` action disjunction. It describes state-level semantic prerequisites for an internal transition.

`InternalStepEnabled == ENABLED InternalResolutionStep` is then checked against that independent obligation.

This separation is mutation-sensitive: removing an internal action while retaining its semantic preconditions can expose a `ProtocolStall` instead of making the obligation disappear with the action.

## State classification

The classification priority is:

1. `HorizonReached` — `clock = MaxSeq`; bounded-model artifact, not a liveness PASS;
2. `IntegrityHalt` — fail-closed constitutional circuit breaker active;
3. `ResolvedQuiescence` — no live claims and no finalized effect awaiting application;
4. `ProtocolStall` — unresolved work, before horizon, no fault, authoritative resolution prerequisites already present, but no internal resolution step enabled;
5. `AwaitingExternalEvidence` — unresolved work exists but no current internal resolution obligation exists because required environmental input has not yet arrived;
6. `ActiveResolution` — an internal resolution obligation exists and an internal resolver is enabled.

Canonical safety requires `NoPrematureProtocolQuiescence == ~ProtocolStall`.

`AwaitingExternalHasInputEnabled` additionally requires that a state classified as external waiting actually has at least one enabled environmental input path; otherwise it would be an unclassified dead end disguised as waiting.

## Non-vacuity

Qualification must independently demonstrate reachability of:

- `AwaitingExternalEvidence` after real work has been submitted;
- `ActiveResolution` after authoritative input is present;
- `ResolvedQuiescence` after work has actually existed;
- `IntegrityHalt` after work has existed;
- closure-driven terminal resolution;
- finalized effect resolution.

Reachability checks are separate from safety checks. A safety PASS is not allowed to rely on unreachable classifications.

## Mutation sensitivity

The exact qualifier must remove internal resolvers without changing `ResolutionObligation` and require `NoPrematureProtocolQuiescence` to fail.

At minimum:

- remove finality resolution;
- remove closure resolution;
- remove effect application.

These controls test the distinction between “authoritative prerequisites are already present” and “the protocol still has an actual internal path to consume them.”

## Bound sensitivity

Canonical safety is checked at exact finite horizons:

- `MaxSeq = 6`;
- `MaxSeq = 8`;
- `MaxSeq = 10`, subject to the workflow resource ceiling.

The receipt records generated states, distinct states and complete-search depth for every executed cell. Increasing a bound is evidence expansion, not an unbounded theorem.

Qualification language must be of the form `PASS at bounds {...}` and must preserve any resource-limited/skipped cell rather than silently dropping it.

## Fairness and liveness assumptions

No fairness assumption is hidden in `Spec`.

Future temporal liveness checks may introduce named assumption profiles such as finality-service availability, closure-service availability, effect-executor availability, or internal scheduler fairness. Those assumptions must remain visible and independently reviewable.

In particular:

- external witnesses are not assumed eventually available by the safety model;
- dependency/network recovery is not assumed fair by the safety model;
- integrity-halted states are not required to self-recover without a separate reviewed recovery protocol;
- external submission/evidence arrival does not itself satisfy internal progress.

## Evidence boundary

This tranche does not establish:

- unbounded liveness;
- production fairness;
- network availability;
- witness availability;
- runtime/Holochain refinement;
- recovery from integrity halt;
- production activation.

It establishes only bounded evidence that the explicit quiescence classification is internally coherent, non-vacuous at the tested horizon, and sensitive to deliberate removal of required internal resolution transitions.
