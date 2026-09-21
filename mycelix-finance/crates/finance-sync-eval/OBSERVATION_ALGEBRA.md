# FIN-SYNC-002A observation-only graph evaluation v1

Status: source specification. Source review or compilation does not establish provider truth, settlement qualification, or synchronization.

## Claim boundary

FIN-SYNC-002A consumes one exact FIN-SYNC-001 `SettlementGraphV1` plus bounded, explicitly **unqualified** leg observations and derives a deterministic observation-level receipt.

```text
observation reported
!= provider truth
!= qualified settlement
!= synchronized completion
!= recovery authority
```

There is deliberately no caller-authored `QualifiedApplied`, `Completed`, or compensation state in this crate.

## Independent observation streams

Every observation binds an exact `observation_stream_ref`.

`observation_revision` is ordered **only within that exact stream**. A revision from one stream never suppresses another stream's current frontier.

```text
stream A revision 100
!= authority to erase stream B revision 1
```

For each leg and stream, V1 selects the highest supplied revision. Same-stream/same-revision semantic disagreement is conflict. Lower revisions remain bound in the receipt evidence set but do not replace the selected frontier.

Exact replay of one unchanged `evidence_id` is idempotent. Reuse of the same evidence identity with changed canonical semantics is conflict even if the changed observation would otherwise be superseded.

## Observation commitment

Domain:

```text
MYCELIX_FIN_SYNC_UNQUALIFIED_OBSERVATION_V1\0
```

Canonical bytes:

```text
domain
|| u16(1)
|| graph_commitment
|| leg_id
|| text(observation_stream_ref)
|| text(evidence_id)
|| evidence_commitment
|| u64(observation_revision)
|| u8(observation_class)
|| observation_profile
```

Observation-class tags:

```text
1 ReportedNotDispatched
2 ReportedDefinitelyRejectedBeforeEffect
3 ReportedOutcomeUnknown
4 ReportedPending
5 ReportedAppliedUnqualified
6 ReportedRejectedUnqualified
7 ReportedReversedUnqualified
8 ReportedIndeterminate
```

## Stream and leg lattice

Each stream contributes exactly one selected frontier class/profile at its highest supplied revision, unless that exact revision is internally conflicting.

Selected stream frontiers are then joined conservatively per leg.

Important rules:

```text
Applied + explicit no-effect/rejection from another current stream
-> Conflicted

Applied + Unknown/Pending/Indeterminate
-> ObservedAppliedWithUncertainty

Applied + Reversed
-> ObservedReversedUnqualified

Unknown + no-effect observation
-> uncertainty remains visible
```

V1 never resolves independent-source disagreement by highest revision, arrival order, or ambient time.

## Graph-level dispositions

Closed V1 output:

```text
NoKnownAppliedEffect
UnknownEffectPossible
PartialEffectObserved
AllRequiredEffectsObservedAppliedButUnqualified
ReversalObserved
ReconciliationRequired
Conflicted
Indeterminate
```

The most important distinction is:

```text
one leg observed applied + another leg outcome unknown
-> UnknownEffectPossible
```

not `PartialEffectObserved`, because the unknown leg may also have applied.

`PartialEffectObserved` requires at least one applied observation plus an explicit current observation-level no-effect/rejection posture for another required leg.

All legs observed applied still yields only:

```text
AllRequiredEffectsObservedAppliedButUnqualified
```

never synchronized or qualified completion.

A reversal with only otherwise applied/reversed legs yields `ReversalObserved`. A reversal mixed with unresolved/non-applied graph state yields `ReconciliationRequired`.

## Missing observations

Missing evidence is represented by `NoObservation`; it is never treated as `NotDispatched` or rejected.

```text
absence of evidence
!= evidence of no effect
```

## Receipt commitment

Domain:

```text
MYCELIX_FIN_SYNC_OBSERVED_GRAPH_RECEIPT_V1\0
```

The receipt binds:

- exact graph commitment;
- evaluation profile;
- evaluation-context commitment;
- sorted commitments for every admitted non-duplicate observation, including lower revisions;
- sorted per-leg dispositions;
- sorted selected stream frontiers per leg;
- exact graph disposition.

Each selected stream binds:

```text
text(stream_ref)
|| selected_revision
|| selected_class
|| observation_profile
|| sorted selected observation commitments
```

Positive receipt/stream/leg-disposition types are serializable but not deserializable caller claims.

## Bounds

Semantic evaluation admits at most:

- 1024 physical observations total;
- 32 physical observations per leg;
- 16 independent observation streams per leg.

These are post-decoding semantic bounds, not a raw parser allocation theorem. FIN-SYNC-001A/#2498 remains the ingress/resource-admission boundary.

## Frozen reference vector

`test-vectors/observed-graph-v1.json` freezes an applied USD observation and an unknown EUR observation over the FIN-SYNC-001 PvP fixture.

Expected:

```text
USD observation
be4da4668876add8de9c6082ef0cbf09a293043074932f61d93945f5bd98963f

EUR observation
513416d303c78fb8b3ca8d4ed27e17d79f3e388ac858961133b2dc2e05efaebf

graph disposition
UnknownEffectPossible

receipt
6c18d39564d417c747dcabbb02ae1a3fcab6d2772162d3f1f2fb8f15e13305ee
```

A later diagnostic/qualification lane must reconstruct these bytes independently rather than using the Rust canonicalizer as its own oracle.

## Nonclaims

A future PASS may establish deterministic observation classification for exact supplied bounded inputs.

It does not establish provider authenticity, current external state, FIN-ECO finality, synchronized PvP/DvP, financial authorization, capacity, recovery/compensation authority, legal discharge, commercial satisfaction, or autonomous Symthaea authority.
