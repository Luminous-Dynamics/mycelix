# WEB-LEASE-001 — Persistent Attempt Lease State Machine Corpus v0.1

Status: **FROZEN DESIGN CORPUS / NOT EXECUTED / NOT PASS**

Tracks Mycelix issue #2717.

Parent evidence frontier:

- B1 product head: `e36f65a4b90de6c9eb70bfe3404b510d991f6f4b`
- B1 receipt profile: `mycelix:web-target-admission-receipt:v1`
- Lease profile: `mycelix:web-attempt-lease:v1`
- Time profile: `mycelix:web-attempt-lease-time:wall-clock-assertion:v1`

This corpus freezes persistent one-attempt execution semantics **before any storage implementation is allowed to define them accidentally**.

## Governing theorem

```text
non-Clone Rust capability
!= persistent single-use authority
!= restart-safe attempt consumption
!= crash-safe side-effect accounting
```

The durable lease layer exists because process-local ownership disappears when a process crashes or restarts.

## Authority identity

The durable authority key is the tuple:

```text
admission_commitment
+ attempt_id
+ execution_profile_id
```

None of the fields is sufficient by itself.

```text
same attempt_id + different admission commitment != same authority
same admission commitment + different attempt_id != same authority
same commitment/attempt + different execution profile != same authority
```

The deterministic admission commitment is an integrity identifier, not a secrecy primitive. Privacy/export handling remains separately governed by WEB-RECEIPT-PRIV-001.

## State machine

```text
Absent
  |
  | ensure_prepared
  v
Prepared(epoch=0)
  |
  | transactional acquire_lease
  v
Leased(epoch=N, lease_id, holder_id, issued_at, expires_at)
  |   |  \ explicit abort_before_start
  |   v
  |  AbortedBeforeStart [terminal]
  |
  | durable mark_started MUST succeed before any socket side effect
  v
Started(epoch=N, lease_id, holder_id)
  |        |          |             |
  |        |          |             +--> recover_after_crash
  |        |          |                    |
  |        |          |                    v
  |        |          |             IndeterminateAfterCrash [terminal]
  |        |          |
  |        |          +--> FinishedFailure [terminal]
  |        |
  |        +--> FinishedSuccess [terminal]
  |
  +--> never returns to Prepared/Leased under v1
```

An expired `Leased` state may be transactionally replaced by a new lease with a strictly greater epoch **only when the durable state is still `Leased`**.

`Started` is the consumption boundary, not success:

```text
Started
!= TCP connected
!= peer matched
!= TLS succeeded
!= HTTP succeeded
```

A failed network attempt remains consumed.

## Critical connector composition rule

The lease theorem is only safe when composed with a connector that enforces:

```text
successful durable mark_started
BEFORE
first externally observable network side effect
```

Therefore:

```text
durable state == Leased
```

is safe to expire/re-lease only under a separately qualified connector theorem proving no socket side effect can precede `Started`.

If a connector can open a socket before the durable `Started` transition, this lease protocol does not establish no-replay safety.

## Lease token

A lease/start operation must bind all of:

```text
authority key
lease_id
holder_id
lease_epoch
```

A stale holder, stale lease ID or stale epoch must fail closed.

An active holder must also start before its lease expires:

```text
started_at >= expires_at
-> reject LeaseExpired
```

So an expired token cannot begin a network side effect merely because no replacement lease has been acquired yet.

This protects safety even if the wall clock jumps forward and causes premature lease recovery:

```text
old token
+ new current epoch
-> old token cannot mark Started
```

## Clock semantics

The V1 time profile uses a persisted wall-clock **assertion**, not trusted time.

```text
wall-clock assertion
!= trusted time
!= monotonic process time
```

Clock errors must not become safety failures.

- Forward clock jump may expire a `Leased` record early. The replacement lease increments the epoch, so the old token becomes unusable.
- Backward clock jump may delay recovery. That is a liveness loss, not duplicate socket authority.
- Clock movement can never reset `Started` or a terminal state.

Trusted/notarized time may later be attached as separate evidence without changing the core state-transition theorem.

## Expiry semantics

Expiry is evaluated only for `Leased`.

Conceptually:

```text
observed_now >= expires_at
+ durable state == exact Leased record
+ CAS succeeds
-> replacement lease(epoch + 1)
```

Expiry never changes `Started` or terminal states.

## Terminal states

V1 terminal states are:

- `FinishedSuccess`
- `FinishedFailure`
- `AbortedBeforeStart`
- `IndeterminateAfterCrash`

There is no automatic transition from any terminal state back to `Prepared`.

A manual retry after `IndeterminateAfterCrash` requires separately named operator authority and a **new acquisition attempt lineage**. It is not a lease reset.

## Crash semantics

### Crash before lease persistence

No durable lease exists. Another claimant may attempt normal acquisition.

### Crash after `Leased`, before `Started`

After lease expiry, a replacement lease may be acquired transactionally because no network side effect was authorized yet.

### Crash after durable `Started`

The attempt is consumed.

If the final network outcome was not durably recorded, recovery produces:

```text
IndeterminateAfterCrash
```

and V1 does not automatically retry.

### Crash during terminal outcome persistence

Exact repetition of the **same** terminal transition/outcome reference may be idempotent.

A conflicting terminal result must be rejected. Persistence retry may not rewrite history.

## Transaction theorem

The required storage primitive is stronger than an in-memory lock:

```text
Prepared(epoch=N)
+ claimant A
+ claimant B
concurrently
->
at most one current Leased(epoch=N+1)
```

The eventual `AttemptLeaseStore` must qualify this using storage-level transactional/CAS semantics.

## Storage failure rule

Storage failure is fail-closed.

Most importantly:

```text
mark_started storage error
-> no Started token
-> no socket authority
```

Corrupt/unparseable records also emit no lease/start token and are never silently interpreted as `Prepared`.

## Idempotency

`ensure_prepared` is idempotent but is **not a reset primitive**.

A terminal write may be retried idempotently only if every bound terminal field is identical.

```text
FinishedFailure(outcome=A)
+ repeat FinishedFailure(outcome=A)
-> no-op / same history

FinishedFailure(outcome=A)
+ FinishedSuccess(outcome=B)
-> reject conflict
```

## Corpus

The machine-readable fixture contains 27 cases covering:

1. fresh preparation;
2. preparation idempotency;
3. first lease acquisition;
4. concurrent second lease rejection;
5. valid start;
6. stale holder rejection;
7. stale lease ID rejection;
8. stale epoch rejection;
9. success terminalization;
10. failure terminalization;
11. abort before start;
12. expired-lease recovery;
13. expired lease cannot start;
14. stale token after recovery;
15. forward-clock safety;
16. rollback-clock liveness loss;
17. Started never expiring back to reusable authority;
18. crash recovery to indeterminate;
19. indeterminate terminality;
20. exact terminal persistence retry;
21. conflicting terminal rewrite rejection;
22. prepare-after-terminal non-reset;
23. same attempt / different commitment key separation;
24. same commitment / different attempt key separation;
25. execution-profile key separation;
26. storage error on start failing closed;
27. corrupt state failing closed.

Fixture SHA-256 over the exact UTF-8 JSON bytes:

```text
8f057856d4e8399b7ed6a4287699a4387caafb5530086af66743a0ad444aa417
```

This digest identifies this authored corpus file only. It does not qualify any implementation.

## Required future implementation split

The semantic layer should remain independent of the first durable backend:

```text
AttemptLeaseStateMachine
          |
          v
AttemptLeaseStore trait / transactional contract
          |
          +--> qualified local durable backend v1
          |
          +--> possible later distributed backend
```

Do not embed SQLite/SQLx/Holochain semantics into the state model itself.

## Required future connector handoff

The network connector should eventually require both:

```text
ConnectorHandoffV2
+
ActiveAttemptLeaseTokenV1
```

Then the connector must exchange the active lease token for a durable `Started` authorization **before** opening a socket.

The token must be move-only process-local material, but its security ultimately derives from the durable store state and exact epoch binding, not Rust ownership alone.

## Nonclaims

This corpus does not establish:

- a storage implementation;
- persistence atomicity;
- qualified wall-clock behavior;
- global consensus;
- B0 admission correctness;
- B1 receipt correctness;
- B2 admission/receipt binding;
- network reachability;
- endpoint safety;
- TLS/source identity;
- HTTP correctness;
- source authenticity;
- content truth;
- EPI admission;
- operator legitimacy.

A future PASS may establish only the named state-machine + storage-profile properties actually executed and evidenced.

## PASS discipline

```text
design corpus exists
!= implementation exists
!= store atomicity qualified
!= connector ordering qualified
!= PASS
```

Queued, skipped, cancelled, unexecuted, partially exercised, source-mutating or receipt-less qualification remains **NOT PASS**.
