# Mycelix Integration Plane v1 — Execution Safety Addendum

Status: Draft normative addendum

Profile: `MYCELIX-INTEGRATION-1`

Parent RFC: `docs/rfcs/integration-plane-v1.md`

## 1. Purpose and precedence

This addendum tightens the outbound execution, retry, reconciliation, authority, and durable-identity semantics of `MYCELIX-INTEGRATION-1` after executable review of INT-02/INT-03.

Where this addendum conflicts with the parent RFC's earlier reference shapes, **this addendum governs**. In particular it supersedes the parent RFC's `OutboxCommitted -> Executing` reference transition and any example in which a durable approved/claimed command is passed directly to a provider executor.

The core rule is:

```text
observation != consequence
decision != authority
durable authority evidence != current capability
queue claim != dispatch
local dispatch != provider commit
provider commit != world postcondition
reconciliation attempt != resolution
```

## 2. I-11 — Claim MUST be distinct from external dispatch

Acquiring durable work MUST NOT itself mean that an external side effect may have occurred.

The minimum outbound causal sequence is:

```text
Proposed
  -> AuthorityChecked
  -> Approved
  -> OutboxCommitted
  -> AttemptPrepared
  -> DispatchStarted
  -> Confirmed | Rejected | Ambiguous
  -> Reconciled
  -> Finalized
```

`AttemptPrepared` means only that one exact worker/attempt has leased committed work. It MUST be recoverable to `OutboxCommitted` if its lease expires before `DispatchStarted`.

`DispatchStarted` means the local execution boundary has durably recorded that the external-call window is open. A crash or timeout after this boundary MUST be treated conservatively because the provider may have observed or committed the request.

No implementation may silently collapse:

```text
OutboxCommitted -> DispatchStarted
AttemptPrepared -> Confirmed
AttemptPrepared -> Rejected
AttemptPrepared -> Ambiguous
```

without an equivalent durable intermediate theorem proving the same causal distinction.

## 3. I-12 — Each attempt MUST have an exact fence identity

Each outbound attempt MUST have a distinct, durable attempt/fence identity bound to at least:

- the outbox/intent identity;
- monotonically advancing attempt generation or equivalent anti-reuse coordinate;
- worker/lease ownership where applicable;
- command and connector identity.

A completion for attempt N MUST NOT complete attempt N+1, even when the same worker identity is reused after restart.

Attempt/fence identity is not necessarily secret and MUST NOT be treated as authority. Its purpose is causal exclusion of stale work.

A late result for an old attempt SHOULD be preserved as evidence. It MUST NOT silently rewrite a newer attempt or an already-established `Ambiguous` state.

## 4. I-13 — Every execution outcome MUST bind one exact logical operation

`Confirmed`, `Rejected`, and `Ambiguous` outcomes MUST all identify the exact logical external operation they describe.

At minimum this identity MUST bind:

- integration command identity;
- connector instance;
- provider operation identity when one has been established.

A rejection or receipt for command B MUST NOT be attachable to command A merely because both used the same provider or worker.

Connector-local transport errors that cannot establish an exact provider outcome remain errors/uncertainty; they MUST NOT be converted into an unrelated `Rejected` result.

## 5. I-14 — Ambiguity MUST preserve unknown commit state

After `DispatchStarted`, timeout, lost response, process loss, or equivalent uncertainty MUST NOT imply `NotCommitted`.

For every side-effecting operation:

```text
Ambiguous -> reconcile before reexecution
```

unless a separately qualified provider execution profile proves that replay of the exact operation under the current conditions cannot create an additional effect.

`Reversible` and `Compensatable` describe repair semantics. They do **not** prove that blind replay is safe.

`ReadOnly` may permit bounded replay because it does not create an external side effect, subject to the owning domain's freshness/cost policies.

## 6. I-15 — Idempotency MUST be a qualified provider property

Presence of an `IdempotencyKey` is data, not proof of idempotent provider behavior.

A provider profile that authorizes idempotent replay MUST bind at least:

- provider/system and endpoint/operation class;
- key scope;
- key retention horizon;
- payload-equivalence rule;
- behavior for duplicate keys with changed payloads;
- failover/account/region scope where relevant;
- reconciliation/query capability;
- profile/version identity;
- qualification/evidence used to trust those semantics.

Therefore:

```text
IdempotencyKeyPresent != EndpointIdempotencyGuarantee
```

A provider-neutral runtime MUST NOT infer an idempotency reconciliation strategy merely because a key is stored.

## 7. I-16 — Reconciliation observations MUST be append-only and non-conclusive by default

A reconciliation attempt is an observation about an unresolved effect. It does not automatically resolve the effect.

The protocol MUST preserve at least:

```text
ConfirmsEffect
ConfirmsNoEffect
StillAmbiguous
```

and MAY support additional explicit dispositions.

`StillAmbiguous` MUST:

- remain unresolved;
- not transition the operation to a finalizable state;
- be preserved as historical evidence;
- permit later reconciliation attempts.

Multiple reconciliation observations MUST NOT be represented only by overwriting one mutable receipt. Implementations MAY cache a latest summary, but durable history MUST remain reconstructable.

Contradictory reconciliation evidence MUST remain visible rather than being silently last-write-wins.

## 8. I-17 — Durable intent and queue claims MUST NOT materialize current execution capability

Durable authority references/commitments are historical provenance. Restarting a process or loading an outbox row MUST NOT recreate current execution authority.

The preferred composition is:

```text
DurableOutboundIntent
    -> ExecutionClaim / AttemptLease       INT-03
    -> fresh CurrentExecutionAuthority
       + qualified provider profile
       + exact command/target/attempt binding
                                           INT-04
    -> provider payload materialization
    -> DispatchStarted
    -> external call
```

Provider-executable payload material SHOULD be withheld from the generic runtime claim object when doing so prevents accidental capability laundering.

The following equivalences are forbidden:

```text
OutboxCommitted == executable capability
ExecutionClaim == CurrentExecutionAuthority
stored authority commitment == current authority
scheduler/queue message == permission to act
successful prior authority check == authority forever
```

Current execution authority MUST be reconstructed/requalified at the execution boundary under current revocation/control state.

## 9. I-18 — Storage schema evolution MUST preserve causal meaning

Numeric/database representations of protocol states MUST be explicitly versioned once durable data exists.

A new implementation MUST NOT reinterpret an old numeric state as a new semantic state simply because the integer matches.

When migration cannot prove the stronger new meaning, it MUST degrade conservatively to an uncertainty state rather than promote certainty.

Example:

```text
legacy Executing
  # old representation cannot prove whether dispatch occurred
  -> Ambiguous

legacy Executing
  -X-> AttemptPrepared
```

Unknown future storage schema versions MUST fail closed unless an explicit migration exists.

Migration itself SHOULD have adversarial/golden fixtures and be included in hosted qualification evidence.

## 10. I-19 — Execution history and admission MUST be bounded

Runtime implementations MUST declare explicit resource/admission bounds for untrusted or externally influenced material, including as appropriate:

- inbound body/normalized payload bytes;
- command material;
- provider receipts/outcomes;
- reconciliation evidence;
- number of attempts;
- execution observations;
- reconciliation-history records;
- worker/lease identifiers;
- batch claim size.

Budget exhaustion MUST NOT be interpreted as success, rejection, or effect absence. It MUST fail explicitly or quarantine work according to policy.

Internal/runtime-generated observations count against the same relevant history budgets as externally supplied observations; implementation-owned code does not receive an unbounded bypass.

## 11. I-20 — Transport execution evidence MUST remain separate from world postconditions

A provider/controller receipt establishes only the semantics promised by the qualified provider profile.

It MUST NOT automatically establish that a physical, social, financial, or institutional postcondition is true beyond that contract.

For physical or otherwise externally observable effects, the preferred loop is:

```text
ActionIntent
  -> dispatch
  -> provider/controller receipt
  -> Reality postcondition observation
  -> qualification
  -> reconciliation/outcome assessment
```

Examples:

```text
controller ACK != valve physically closed
HTTP 200 != downstream workflow completed
payment API receipt != recipient economic outcome
deployment API success != application healthy
```

## 12. I-21 — Exactly-once MUST NOT be claimed generically

The integration plane MUST NOT promise exactly-once external effects across arbitrary providers.

The portable guarantee is instead:

- unique intent/command identity;
- durable attempts;
- exact attempt fencing;
- provider idempotency when independently qualified;
- explicit `Ambiguous`/commit-unknown states;
- reconciliation;
- compensation as a new effect where supported.

`Compensation` MUST preserve the original effect in history. It is not deletion or rollback of the historical attempt.

## 13. I-22 — Cross-layer immutable identity MUST be typed and verifiable

A string that merely looks like `algorithm:value` MUST NOT be described as an exact immutable artifact binding unless the algorithm/value representation is validated and the digest is actually tied to a canonical or otherwise exact byte representation.

Cross-layer references among proposal, decision, authority, execution, and outcome records SHOULD converge on a shared typed content-identity primitive rather than defining independent digest-string grammars.

The system MUST distinguish:

```text
digest-shaped string
    != verified content commitment

matching mutable ID
    != same immutable artifact
```

Where canonical bytes are not owned by the receiving layer, it MUST consume a separately qualified exact artifact identity rather than pretending relational ID checks recompute the digest.

## 14. I-23 — Option-set horizon MUST survive decision composition

A decision selecting one candidate MUST preserve what is known about how the candidate set was generated.

Recommended option-set completeness vocabulary:

```text
Exhaustive
BoundedCandidateSet
HeuristicSearch
HumanNominated
Unknown
```

A decision over a non-exhaustive set MUST NOT be represented as evidence of global optimality.

```text
selected among considered options != globally best option
```

This information belongs in the proposal/decision provenance and MUST survive into execution audit where material.

## 15. Conformance additions

In addition to the parent RFC's connector conformance suite, `MYCELIX-INTEGRATION-1` SHOULD include adversarial cases for:

- crash before dispatch -> reclaim without ambiguity;
- crash after dispatch -> ambiguity;
- stale attempt completion after a new attempt exists;
- same worker ID reused across attempts;
- late provider response after timeout;
- rejection for a different operation;
- `StillAmbiguous` repeated multiple times;
- contradictory reconciliation observations;
- idempotency key present but provider contract absent;
- provider idempotency retention expired;
- legacy state-schema migration collision;
- history/admission budget exhaustion;
- durable authority evidence after current revocation;
- provider receipt contradicting independent postcondition evidence.

A conformance PASS means the exact implementation satisfied that corpus under the recorded test profile. It does not prove provider truth, institutional legitimacy, current authority, exactly-once execution, or physical-world success.
