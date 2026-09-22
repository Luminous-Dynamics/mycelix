# FIN-ALPACA-STREAM-001T — Activity SSE publication/replay profile v0.1

## Status

Frozen provider-mapping profile and sanitized deterministic corpus only.

```text
NOT IMPLEMENTED
NOT NETWORK-EXECUTED
NOT PERSISTENCE-QUALIFIED
NOT QUALIFIED
NOT PASS
```

Tracks FIN-ALPACA-STREAM-001 / #2805 and corpus issue #2869.

Reviewed provider documentation date: 2026-09-22.

## Exact provider surface

Provider: Alpaca, US region.

Reviewed semantic family:

```text
GET /v2beta1/events/activities
```

Deployment remains explicit and scoped:

```text
TradingPaper
TradingLive
BrokerSandbox
BrokerProduction
```

The stream theorem never infers deployment from credential bytes or an arbitrary host string.

## Publication cursor theorem

Current Alpaca Activity SSE guidance distinguishes:

```text
event_id
= ULID publication/replay cursor
= provider publication-sequence identity
= provider-documented publication ordering within the exact Activity SSE stream scope

ref_id
= stable activity identity / deduplication key
!= event_id

at
= business-event timestamp
!= publication sequence
```

Alpaca documents that `event_id` encodes the publish-sequence timestamp and governs delivery/replay order. Delays and backfills may therefore produce a later `event_id` whose business timestamp `at` is older than previously delivered activity.

Mycelix may preserve that exact provider publication chronology, but must not promote it into another ordering theorem:

```text
publication order
!= business-time order
!= trusted wall-clock truth
!= cross-account causality
!= settlement order
!= economic finality
```

Publication cursor ordering is valid only after the cursor has passed the frozen ULID validation/canonicalization rule and only within the exact same provider/schema/deployment/account stream scope.

## Stream-scoped publication identity

A publication identity is scoped by at least:

```text
provider
schema profile
deployment profile
account scope
event_id
```

Therefore:

```text
same event_id under TradingPaper
!= universal identity with same event_id under TradingLive

same event_id for account A
!= universal identity with same event_id for account B
```

A future GA schema cannot inherit `v2beta1` checkpoints without explicit migration evidence.

## Publication envelope vs activity identity

STREAM owns publication envelopes. EXECID owns provider activity identity.

```text
same event_id + same bounded envelope
-> duplicate/re-delivered publication envelope

same event_id + conflicting bounded envelope
-> EventEnvelopeConflict
```

Separately:

```text
same ref_id + different event_id
-> two publication observations
-> one downstream activity identity if EXECID admits that relation
```

STREAM must preserve both publication observations. It must not forge a second execution, and it must not erase a second publication merely because the downstream activity identity deduplicates.

## Query-profile rules

Current provider reference freezes these profile rules:

```text
no since / since_id
-> no historical replay; live updates only

since_id present, until_id absent
-> historical replay from cursor, then transition to live

until_id present
-> since_id required

since and since_id
-> mutually exclusive

time-based until
-> time-based since required

until / until_id reached
-> stream terminates successfully with HTTP 200
```

Time-based `since` / `until` business-history coverage belongs to FIN-ALPACA-RECON-001 / #2808.

Cursor arithmetic is forbidden:

```text
since_id + until_id
!= locally enumerable sequence interval
!= inferred number of events
```

## Processing and checkpoint boundary

Alpaca instructs consumers to persist `event_id` only after successful event processing and to reconnect using the last successfully processed cursor.

Mycelix must model more stages than “received” versus “saved”:

```text
bytes received
!= parsed data event
!= bounded publication admitted
!= required downstream processing accepted
!= checkpoint promotion candidate
!= physically durable safe checkpoint
```

The pure STREAM implementation may consume supplied downstream/durability evidence for deterministic transition tests, but a pure in-memory state machine cannot prove physical durability.

Preferred theorem split:

```text
FIN-ALPACA-STREAM-001A
= pure deterministic publication/replay transition algebra

FIN-ALPACA-STREAM-PERSIST-001
= actual durable checkpoint transaction + crash/recovery theorem
```

Pure positive names should remain bounded, for example:

```text
CheckpointPromotionCandidate
ReplayedFromCheckpointUnderSuppliedEvidence
```

A positive type named `DurablyCheckpointed` is reserved for the later persistence theorem.

## Replay and duplicate semantics

The documented resume algorithm is:

```text
persist last successfully processed event_id
-> reconnect with since_id = that cursor
-> process replayed/re-delivered events idempotently
-> deduplicate activities by ref_id
```

This means replay safety depends on both layers:

```text
STREAM
= publication replay/checkpoint continuity

EXECID
= activity identity / dedup
```

Possible duplicate replay is acceptable; silently skipping an unprocessed activity is not.

## SSE control comments

SSE comments are transport/control evidence, not provider activities.

### Heartbeat

```text
:heartbeat
-> liveness evidence only
```

It does not prove continuity, completeness, currentness, or lack of activity.

### Slow consumer

Provider example:

```text
: you are reading too slowly, dropped N messages
```

Mycelix classification:

```text
-> GapObserved
-> publication continuity withheld
-> retain prior safe checkpoint
-> replay required from prior safe checkpoint
```

`N` is informational. It must not be used to synthesize event IDs, activity IDs, economics, or a local cursor interval.

### Internal server error

```text
: internal server error
-> connection termination / reconnect posture
!= known data gap by itself
```

Unknown future comments remain non-economic and cannot silently prove continuity.

## Business-time backfills

Freeze the provider-documented distinction explicitly:

```text
later event_id
+ older at
-> valid publication advancement with business-time backfill evidence
```

This must not be rejected as a publication regression merely because business time moved backward.

Conversely:

```text
latest publication cursor
!= all business events through time T are complete/final
```

Business-time reconciliation remains a separate theorem.

## Canonical ULID rule

The corpus freezes one rule for the implementation to make explicit before ordering:

```text
parse/validate ULID syntax
-> canonicalize to uppercase Crockford representation
-> only then compare within exact stream scope
```

Malformed cursor bytes fail typed and never participate in ordering.

The implementation must not use arbitrary raw-string comparison before validation/canonicalization.

The ULID timestamp component remains provider publication-sequence metadata. STREAM does not convert it into trusted wall-clock time.

## Required state distinctions

Conceptually the pure state needs enough information to represent:

```text
Uninitialized
LiveOnly
ContiguousFromCheckpoint
GapObserved
ReplayRequired
ReplayInProgress
ReplayedFromCheckpointUnderSuppliedEvidence
ContinuityWithheld
```

with at least:

```text
stream scope
last observed publication cursor?
last processed publication cursor?
safe checkpoint supplied by persistence?
checkpoint promotion candidate?
gap evidence?
replay anchor?
```

Do not use unqualified names such as `Complete` or `Current`.

## Frozen corpus

`fixtures/FIN_ALPACA_STREAM_001_V0_1.json` contains exactly 30 cases covering:

1. live-only query;
2. open-ended cursor replay;
3. bounded cursor replay;
4. invalid until_id without since_id;
5. invalid since + since_id;
6. first publication;
7. later publication;
8. same-scope publication regression;
9. exact duplicate envelope;
10. conflicting same-cursor envelope;
11. same activity identity under a new publication cursor;
12. later publication carrying older business time;
13. heartbeat;
14. slow-consumer gap;
15. dropped-count non-authority;
16. internal server error;
17. reconnect without repair;
18. replay from checkpoint with re-delivery;
19. downstream rejection;
20. checkpoint promotion candidate;
21. supplied durability failure;
22. second gap during replay;
23. malformed ULID;
24. canonicalization-before-comparison;
25. cross-deployment scope firewall;
26. cross-account scope firewall;
27. future-schema checkpoint firewall;
28. same cursor text in different scopes;
29. bounded arbitrary-sequence panic freedom;
30. claim ceiling against business completeness/currentness/finality.

## Metamorphic invariants

The implementation and qualifier should mechanically prove at least:

```text
mutate at, hold event_id
-> publication identity/order unchanged

mutate ref_id, hold event_id
-> publication cursor identity unchanged

mutate exact stream scope
-> scoped publication identity changes

known gap + heartbeat
-> gap remains unrepaired

known gap + transport reconnect
-> gap remains unrepaired

later publication cursor + older at
-> admissible backfill

same cursor + conflicting envelope
-> fail closed

downstream failure
-> no checkpoint promotion
```

## Relationship to neighboring theorems

```text
WIRE-001 / #2813
owns bounded SSE framing + JSON projection

SOURCE-001
owns authenticated network-origin context

EXECID-001A / #2791/#2806
owns ref_id / previous_id activity identity

STREAM-001T / #2869
owns frozen stream profile/corpus

STREAM-001A
owns pure publication/replay transitions

STREAM-PERSIST-001
owns physical checkpoint durability/crash recovery

RECON-001 / #2808
owns business-time historical reconciliation
```

No layer may manufacture another layer's positive claim.

## Claim ceiling

A future STREAM-001T PASS means only that this provider-mapping profile and sanitized corpus were frozen and statically verified.

It does not establish:

```text
live Alpaca interoperability
provider authenticity
account ownership
physical storage durability
economic execution identity
business-time completeness/currentness
settlement/finality
market truth
order authority
autonomous Symthaea authority
```
