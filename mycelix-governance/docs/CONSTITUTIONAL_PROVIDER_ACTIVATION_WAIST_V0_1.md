# Constitutional Provider Activation Waist v0.1

Status: **draft, inert cross-lineage admission contract** for MYC-CONST-003CR2A / #1934.

Parent CR2 semantic subject: `f4748da5c60686e9611ce68d60fcd24a4bfa239a`.

CR2's exact verifier is prepared at `25b4942e1be20ef9ce70f55b2b1d15201ec38ca5`, with draft execution wrapper #1954 parked at run `35458597274` as `skipped`. Therefore CR2 is not silently promoted here: CR2A records it as `PreparedNotHostedQualified`.

## Purpose

CR2 separates **qualified evidence** from **qualified concrete↔formal refinement**. CR2A adds the next narrow waist:

```text
qualified constitutional evidence
        +
qualified formal progress/crash models
        !=
runtime permission to dispatch an external effect
```

A future provider adapter must satisfy an explicit activation contract before it may use those evidence lines to dispatch, retry, or declare completion. This tranche freezes that contract only. It does not implement a runtime classifier, durable outbox, Holochain entry type, provider call, or live `GovernanceAction::EmitEvent` path.

## Qualified inputs

### C3 lifecycle quiescence

Exact qualified evidence:

- semantic `b9bb91353788aeb858c4e52422a87e2401d60a0e`;
- verifier `18309d680a5d630bbb7f5443ab103ade9239a557`;
- run `35314384788`;
- artifact `sha256:d26e566503fc1223d879d69ecdb4cf6978fc62664576ed04973067393c8feaab`;
- model `mycelix-governance/specs/ConstitutionalLifecycleQuiescence.tla`;
- model blob `8ca29563bb581cc77958653210e84d55b3aeb364`.

CR2A reconstructs the exact TLA+ bytes from that semantic commit and independently checks the six `QuiescenceClasses`.

The important distinction is that **generic `ActiveResolution` is still not enough to dispatch an effect**. A dispatch candidate must be on the effect-specific internal path:

```text
StateClass = ActiveResolution
AND EffectResolutionRequired
AND InternalStepEnabled
AND ApplyEffect is the internal action
```

The validator binds the source definitions:

```text
InternalStepEnabled      == ENABLED InternalResolutionStep
EffectResolutionRequired == PendingEffect /\ effectReady
```

and requires `ApplyEffect` itself to retain both `PendingEffect` and `effectReady`.

The other five C3 classes are blocked from effect dispatch:

- `HorizonReached` — bounded model horizon, never runtime authority;
- `IntegrityHalt` — fail closed;
- `ResolvedQuiescence` — no unresolved effect work;
- `AwaitingExternalEvidence` — waiting cannot be translated into completion;
- `ProtocolStall` — a missing internal resolver cannot be relabeled as legitimate quiescence.

C3 remains `runtime_refinement_status = NotEstablished`: this profile does not claim a concrete Rust/Holochain classifier computes those predicates correctly.

## D1A crash/outbox discipline

Exact qualified evidence:

- semantic `15b9c89adf0ac3c6c5a73681614d6bfcd368820a`;
- verifier `70fbe906834fdef1ba69a80066dbbbaef200156f`;
- run `35320314578`;
- artifact `sha256:ce9d7176bd1040ef552fa587e44ae8801a13b0363c0013d5428e5f95ff60430c`;
- model `mycelix-governance/specs/ConstitutionalEffectOutbox.tla`;
- model blob `b75edcee5bfc826786148a0455c5b26a8223c38c`.

The validator independently checks the exact eight D1A phases and the source bodies for `CommitAndEnqueue`, work claiming, request start, delivery, timeout, reconciliation, contradiction halt, retry identity, unknown-outcome blocking, and outbox-before-effect.

The required boundary is:

```text
timeout / lost acknowledgement
        ↓
UnknownOutcome
        ↓
NO blind retry
NO completion claim
        ↓
authoritative reconciliation
   ├─ ReconcileSuccess  -> EffectObserved
   └─ ReconcileNoEffect -> EffectPending, same operation identity
```

`ReconcileNoEffect` does not authorize inventing a new operation. D1A's stable `committedOp`/`outbox` identity remains the logical effect identity. That is a logical idempotency model, not evidence of physical exactly-once execution by any external provider.

D1A also remains `runtime_refinement_status = NotEstablished`.

## Identity inheritance

A provider adapter is not allowed to derive fresh constitutional authority from the request it is about to execute.

The profile therefore requires:

- ClaimBinding source: qualified upstream evidence only;
- operation identity source: qualified upstream evidence only;
- action identity source: qualified upstream evidence only;
- no provider-key regeneration;
- no action-ID regeneration;
- no target rebinding;
- no payload rebinding.

This is a cross-lineage contract. The validator uses exact Git object lookup and does not assume C3, D1A, CR2, D1C, or E0 occupy one ancestry.

## Blocking prerequisites

CR2A deliberately records unresolved prerequisites rather than promoting them by association:

- CR2Q: prepared, not hosted-qualified;
- concrete C3 runtime refinement: not established;
- concrete D1A runtime refinement: not established;
- D1C-R1: prepared, not hosted-qualified;
- E0-R1: prepared, not hosted-qualified.

A future successor may update those statuses only by binding new exact receipts. Historical evidence is not rewritten.

## Mutation sensitivity

`validate_provider_activation_waist.py --self-test` must reject at least:

- dropping `EffectResolutionRequired` from dispatch admission;
- allowing `ProtocolStall` to dispatch;
- allowing `IntegrityHalt` to dispatch;
- promoting bounded `HorizonReached` into authority;
- allowing blind retry under `UnknownOutcome`;
- allowing completion under `UnknownOutcome`;
- setting `activation_allowed=true`;
- falsely promoting CR2 to qualified;
- regenerating provider identity;
- regenerating action identity;
- drifting the exact C3 model blob;
- drifting the exact D1A model blob;
- removing the physical-exactly-once non-claim.

The canonical validator also checks the actual TLA+ operator bodies independently from the profile, so a closed JSON document cannot manufacture source evidence by describing rules the models do not contain.

## Intended consumers

CR2A is a reusable prerequisite for provider lanes such as event admission, parameter updates, and finance effects. Those lanes still need their own concrete provider refinements and qualification. CR2A merely establishes the common fail-closed admission waist they must preserve.

## Non-claims

This tranche does **not** establish:

- CR2 qualification;
- concrete C3 runtime refinement;
- concrete D1A runtime refinement;
- D1C-R1 qualification;
- E0-R1 qualification;
- provider qualification;
- Holochain DHT admission;
- live effect wiring;
- physical exactly-once effects;
- deployment currentness.

A future CR2A qualifier may establish only that this exact inert activation contract matches the exact bound formal evidence and is mutation-sensitive within its stated scope.
