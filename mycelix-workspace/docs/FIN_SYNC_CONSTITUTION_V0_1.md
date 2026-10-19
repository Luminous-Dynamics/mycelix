# FIN-SYNC-000 — Synchronized Settlement Constitution v0.1

Status: **architecture-only / no runtime authority / no settlement PASS**

Tracking issue: #2425

## Purpose

FIN-SYNC defines the authority, evidence, capability, and failure boundaries for coordinating one already-authorized economic effect across multiple settlement legs or infrastructures.

FIN-SYNC does **not** replace FIN-ECO settlement qualification, Business/Commerce agreement semantics, Finance scarce-capacity authority, rail-specific finality, institutional authorization, or rail-adapter qualification.

The core constitutional rule is:

```text
per-leg qualified settlement
!= synchronized multi-leg settlement

all legs submitted
!= all legs applied

all legs observed
!= all legs final

pre-dispatch capacity bundle prepared
!= external settlement graph completed

rail adapter exists
!= rail supports prepare / atomic commit / safe cancellation

compensation available
!= rollback
```

The purpose of this layer is to make partial effect, synchronization, uncertainty, retry, reversal, compensation, and underlying rail capabilities explicit enough that no provider SDK, adapter, model, or orchestration loop can silently promote a weaker fact into a stronger financial claim.

## Ownership map

### Business / Commerce

Business and Commerce retain ownership of:

- agreements and commitments;
- commercial obligations and satisfaction semantics;
- Action Contract / decision / authorization lineage;
- whether a financial outcome satisfies a business obligation;
- legal/commercial discharge references where separately supported.

```text
QualifiedSettlementGraphOutcome
!= commercial obligation satisfied
!= legal discharge
```

### FIN-ECO-002 settlement qualification

FIN-ECO-002 owns per-leg settlement truth under an exact registered profile:

- settlement subjects;
- observations;
- evidence authority/frontiers;
- rail/network binding;
- finality profile identity;
- qualified settlement;
- explicit Unknown/conflict;
- reversal/invalidation lineage.

FIN-SYNC consumes those facts. It does not redefine per-rail finality.

### FIN-ECO-003 / FIN-ECO-004 authority and capacity

These layers retain ownership of:

- exact authorized economic-effect identity;
- Business attempt binding;
- current institutional/action authority;
- Finance reservation consumption;
- scarce-capacity conservation;
- resource fencing/currentness;
- command authorization.

A settlement graph is never a bearer authorization.

### FIN-ECO-004E3 pre-dispatch bundles

FIN-ECO-004E3 owns preparation of several independently scarce resources before dispatch, including explicit saga/compensation semantics when those resources cannot be reserved atomically.

FIN-SYNC owns a different boundary:

```text
004E3 capacity saga
= acquire/release scarce resources before irreversible dispatch

FIN-SYNC settlement saga
= coordinate/reconcile settlement legs once external effect may begin
```

The two may later share evidence/journaling machinery, but they remain different theorems.

### FIN-SYNC

FIN-SYNC owns only:

- canonical multi-leg graph identity;
- exact leg/dependency/group semantics;
- coordination-profile identity;
- rail-capability compatibility for a requested coordination profile;
- synchronization evidence composition;
- graph-level outcome derivation;
- partial-effect and reconciliation-required state;
- graph-level invalidation/reversal lineage;
- orchestration semantics that do not mint underlying authority.

### Xenia / privacy / evidence infrastructure

Xenia and privacy/evidence subsystems own the cryptographic and disclosure profiles FIN-SYNC may consume.

FIN-SYNC does not invent a second signing, ZK, MPC, credential, or encrypted-envelope stack.

### Symthaea

Symthaea may:

- construct candidate settlement graphs;
- compare routes;
- estimate latency/liquidity/slippage/risk;
- detect anomalous or inconsistent outcomes;
- explain why a graph is blocked;
- propose reconciliation or compensation actions;
- simulate counterfactual execution paths.

Symthaea may not mint any of the authority those operations require.

```text
Symthaea recommendation
!= dispatch authority
!= settlement truth
!= compensation authority
```

## Canonical graph boundary

The canonical graph is a semantic description of the exact settlement legs required by one exact economic effect.

Conceptually:

```text
SettlementGraphRefV1 {
    graph_id,
    economic_effect_commitment,
    graph_profile_id,
    graph_profile_revision,
    graph_commitment,
}
```

Every effect-significant mutation must change or reject graph identity, including changes to:

- economic-effect commitment;
- leg set;
- dependency set;
- coordination groups;
- adapter profile;
- rail/network;
- source/destination economic subject;
- amount/asset/unit profile;
- finality profile;
- semantic idempotency identity;
- profile-significant temporal semantics.

Provider transaction identifiers are evidence/correlation identifiers and are not canonical Mycelix settlement-leg identity.

## Coordination profiles

FIN-SYNC must never expose one authority-bearing `atomic: bool`.

### AtomicSingleOwnerV1

This profile is available only when one independently qualified transaction owner can commit every covered leg under one actual atomic storage/execution boundary.

```text
one coordinator process
!= one atomic transaction owner
```

### SynchronizedPvPV1

Payment-versus-payment synchronization is a profile-specific theorem requiring exact paired legs and exact synchronization evidence.

```text
leg A final
+ leg B final
!= PvP synchronization proven
```

Post-hoc proof that both legs eventually settled may be useful reconciliation evidence, but it is not silently promoted to synchronized PvP.

### SynchronizedDvPV1

Delivery-versus-payment / asset-versus-funds synchronization follows the same discipline. Asset-side and funds-side finality remain owned by their exact infrastructures/profiles.

### CompensatingSettlementSagaV1

For independent rails where true atomicity is not established, partial external effect remains first-class.

Compensation is a new authorized economic action with new capacity/settlement lineage unless the exact underlying rail theorem provides native reversible semantics.

```text
compensation
!= rollback
!= deletion of history
```

## Rail capability boundary

Before FIN-SYNC chooses any coordination profile at runtime, each leg must be associated with an exact registered rail/adapter capability profile.

Capabilities that must remain independently represented include as applicable:

- prepare/hold/reserve semantics;
- commit semantics;
- abort/cancel semantics;
- exact-operation query semantics;
- idempotency scope and retention horizon;
- durable commit classification;
- finality-evidence support;
- reversal/return/reorg semantics;
- temporal/deadline semantics;
- resource/rate limits;
- privacy/disclosure constraints.

```text
provider API method named `commit`
!= qualified atomic commit primitive

provider supports capability P
!= adapter implements P correctly

query supported
!= commit truth established
```

A requested coordination profile that the exact leg capability matrix cannot support must fail **before the first external dispatch** or be explicitly downgraded only through a newly authorized weaker profile. It may not silently become a best-effort approximation.

The first typed rail-capability theorem is tracked by FIN-SYNC-003 / #2443.

## Coordinator non-custody and capability confinement

Coordination must not accidentally turn the FIN-SYNC runtime into a discretionary custodian or unrestricted payment principal.

Preferred authority topology:

```text
human / institution authorizes exact economic effect
        ↓
Business + Finance bind exact graph and legs
        ↓
rail-specific narrow execution capabilities / prepared operations
        ↓
FIN-SYNC coordinator may advance only that frozen plan
```

The coordinator must not be able to alter effect-significant fields such as:

- source/beneficiary;
- amount/asset;
- leg membership;
- rail/network;
- finality profile;
- idempotency identity;
- expiry/deadline;
- compensation destination.

Any such change requires new upstream semantics/authorization and therefore a new graph/effect identity as applicable.

```text
coordination authority
!= discretionary custody authority
```

Where external rails support scoped tokens, prepared-operation handles, signed instructions, or equivalent mechanisms, prefer capabilities bound to the exact frozen settlement leg. Audit evidence may reference/commit to those capabilities but must not serialize reusable bearer secrets.

## Graph-state vocabulary

FIN-SYNC must preserve enough state to prevent unsafe retry and false completion.

A profile may refine names, but it must be capable of distinguishing concepts analogous to:

```text
Prepared
DispatchAuthorized
DispatchStarted
PartiallySubmitted
PartiallyObserved
Synchronizing
AllRequiredLegsObserved
AllRequiredLegsQualified
CompletedUnderProfile
PartialEffect
ReconciliationRequired
CompensationProposed
CompensationInProgress
CompensatedUnderProfile
Conflicted
Indeterminate
```

No implementation should collapse this state space into `Succeeded | Failed`.

## Dispatch boundary

Immediately before the first irreversible external dispatch, the strongest path must revalidate every profile-required current input, including as applicable:

- exact Business/Finance economic-effect identity;
- current action/institutional authority;
- active Finance reservations/capacity allocations;
- graph/profile identity;
- each rail adapter/profile and required primitive capability;
- scoped rail execution capability/prepared-operation identity where applicable;
- graph deadline/temporal profile;
- exact semantic idempotency/effect identity;
- required compliance/proof predicates.

After external dispatch may have begun, pre-dispatch rollback semantics no longer apply.

## Per-leg evidence boundary

A leg becomes financially qualified only through its exact rail adapter and FIN-ECO settlement profile.

```text
provider acknowledgement
!= settlement observation authority
!= qualified leg settlement

qualified leg settlement
!= synchronized graph completion
```

Graph-level evidence must retain the exact per-leg proof/evidence identities it consumed.

## Unknown, partial effect, and retry

Unknown is a first-class financial state.

A transport timeout, dropped response, process crash, or provider SDK error does not establish that an external effect was not applied.

```text
transport failure
!= definitely not committed
```

When a leg may have applied but cannot yet be proven applied or absent, FIN-SYNC preserves `Indeterminate` / `ReconciliationRequired` semantics rather than converting uncertainty into retry permission.

A retry must preserve the exact semantic effect/graph/idempotency identity unless a new independently authorized economic action is intentionally created.

Rail-specific idempotency windows and commit-classification capabilities are execution-policy inputs, not implementation trivia.

## Reversal and invalidation

A later reversal, reorg, return, chargeback, or invalidating settlement observation creates explicit successor evidence.

```text
prior qualified settlement
+ later reversal evidence
= historical settlement + successor invalidation/reversal lineage
```

It never means the original observation or qualification is deleted.

Graph-level completion may therefore be historically true and later invalidated under an exact profile.

## Privacy and disclosure

Settlement graphs may expose sensitive relationship, amount, timing, routing, liquidity, and infrastructure-dependency information.

Canonical economic identity and public disclosure identity must therefore remain separable.

A selective-disclosure view may prove only the exact registered proposition while binding to the same underlying graph/effect commitment.

```text
proof graph completed under profile
!= universal disclosure of all leg semantics
```

Likewise:

```text
hidden leg data
!= independently peer-validated hidden semantics
```

unless the selected cryptographic/evidence profile explicitly establishes that theorem.

## Interoperability boundary

External standards/protocols are adapters around canonical Mycelix semantics, not roots of authority.

### ISO 20022

FIN-ISO20022 maps exact external message/profile versions to and from candidate Mycelix/FIN-SYNC semantics through a loss-aware airlock.

```text
ISO message valid
!= sender authorized
!= payment settled
!= FIN-SYNC complete
```

### ACTUS

FIN-CONTRACT maps exact financial-contract patterns/terms into candidate Business/Finance semantics.

```text
ACTUS schedule generated
!= real-world event occurred
!= obligation admitted
!= payment authorized
```

### Proof-carrying compliance

FIN-COMPLY may produce exact policy-relative compliance predicates/proofs.

```text
predicate satisfied under profile
!= legal compliance universally established
!= financial execution authorized
```

## FIN-SYNC invariant registry

The following invariants are normative for successor tranches.

### FIN-SYNC-I01 — Graph non-authority

A valid graph cannot create Business, institutional, Finance, capacity, or rail authority.

### FIN-SYNC-I02 — Per-leg non-promotion

Per-leg settlement qualification cannot by itself create synchronized graph completion.

### FIN-SYNC-I03 — Observation non-finality

Provider acknowledgement or message status cannot become financial finality except through an exact qualified rail/finality profile.

### FIN-SYNC-I04 — Unknown preservation

Absence of decisive evidence cannot be converted to definitely-not-applied or definitely-applied.

### FIN-SYNC-I05 — Partial effect first-class

A mixed applied/non-applied/unknown multi-leg outcome must remain representable without false rollback or completion.

### FIN-SYNC-I06 — No generic atomic Boolean

Atomicity/synchronization is always profile-specific and evidence-bearing.

### FIN-SYNC-I07 — Atomic owner exactness

AtomicSingleOwner claims require one exact independently qualified transaction owner for all covered legs.

### FIN-SYNC-I08 — PvP/DvP profile binding

PvP/DvP labels or membership cannot substitute for synchronization evidence.

### FIN-SYNC-I09 — Capacity/settlement separation

Prepared capacity bundles cannot be promoted to settlement completion.

### FIN-SYNC-I10 — Exact effect binding

Every graph and runtime outcome binds one exact authorized economic-effect commitment.

### FIN-SYNC-I11 — Adapter profile binding

Every external leg binds exact adapter, rail, network, amount/unit, and finality profiles.

### FIN-SYNC-I12 — Append-only reversal lineage

Reversal/invalidation creates successor evidence and never rewrites prior history.

### FIN-SYNC-I13 — Stable retry identity

An indeterminate external outcome cannot be bypassed by silently creating a fresh retry identity for the same intended effect.

### FIN-SYNC-I14 — Compensation non-equivalence

Compensation eligibility/requirement cannot mint compensation authority or imply rollback.

### FIN-SYNC-I15 — Commercial/legal non-promotion

Financial graph completion cannot self-assert contract satisfaction or legal discharge.

### FIN-SYNC-I16 — Privacy non-weakened identity

Redacted/selective-disclosure views must bind the same underlying economic graph/effect and cannot become alternative mutable truths.

### FIN-SYNC-I17 — Interoperability non-authority

External message/contract/schema conformance cannot mint Mycelix authority or settlement truth.

### FIN-SYNC-I18 — AI non-authority

Symthaea may analyze/propose but cannot construct authority-bearing settlement positives from model output alone.

### FIN-SYNC-I19 — Qualification non-inheritance

Qualified ancestors/donors do not automatically qualify a composed FIN-SYNC successor.

### FIN-SYNC-I20 — Bounded hostile-input verification

Any profile accepting externally supplied graph/evidence bytes must define resource bounds and panic/unsafe behavior before claiming routine untrusted-input safety.

### FIN-SYNC-I21 — Rail capability non-inference

A rail/adapter cannot be treated as supporting prepare, abort, idempotency, synchronization, commit classification, finality, or reversal semantics without the exact independently qualified capability profile required by the selected execution plan.

### FIN-SYNC-I22 — Coordinator capability confinement

The synchronizer may receive only the authority required to advance the exact frozen plan; coordination authority must not silently expand into discretionary custody or allow mutation of effect-significant leg semantics.

## Qualification path

The intended evidence ladder is:

```text
FIN-SYNC-000 architecture constitution
        ↓
FIN-SYNC-001 pure canonical graph semantics
       / \
      /   \
     ↓     ↓
FIN-SYNC-002 deterministic graph-outcome kernel
FIN-SYNC-003 typed rail-capability compatibility
      \   /
       \ /
        ↓
effects-disabled synthetic synchronizer
        ↓
Mycelix-controlled integration profile
        ↓
individual external rail adapters
        ↓
real bounded pilots
```

Each rung earns its own qualification. No rung inherits a stronger claim from an ancestor.

The first executable campaign should use deterministic fake rails and fault injection for:

- unsupported coordination profile at plan time;
- scoped-capability field-substitution attempts;
- duplicate submission;
- timeout/unknown outcome;
- partial application;
- stale authority/capacity before dispatch;
- expired idempotency/preparation window;
- conflicting observations;
- delayed finality;
- reversal after completion;
- crash/restart;
- compensation proposal without compensation authority.

No real-money pilot should precede qualification of the exact authority, capacity, rail-capability, settlement, privacy, compliance, and synchronization profiles it relies on.

## External design references

FIN-SYNC is informed by, but does not claim conformance to or endorsement by:

- BIS Project Meridian — synchronisation across conventional and distributed infrastructures;
- BIS Project Meridian FX — cross-jurisdiction PvP synchronization across heterogeneous ledgers;
- BIS Project Agorá — layered exploration of atomic multi-currency settlement using tokenised commercial-bank deposits and central-bank reserves.

These projects are architecture evidence only. Their results do not establish Mycelix settlement correctness, access to regulated payment systems, central-bank money, legal finality, or production readiness.

## Nonclaims

This constitution establishes no executable behavior.

It does not establish:

- distributed ACID across arbitrary infrastructures;
- universal rollback;
- provider honesty;
- bank/RTGS/DLT access;
- settlement finality beyond exact future rail profiles;
- legal discharge;
- accounting/tax/regulatory compliance;
- ISO 20022 or ACTUS conformance;
- anonymous/private transactions beyond exact future privacy profiles;
- autonomous Symthaea authority.
