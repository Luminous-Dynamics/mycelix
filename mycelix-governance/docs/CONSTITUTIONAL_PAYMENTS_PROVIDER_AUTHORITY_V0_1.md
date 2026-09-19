# MYC-CONST-003D1D-F1B2A — Constitutional Payments Provider Author Authority v0.1

## Status

This tranche is a **pure, inert authority theorem**. It creates no Holochain entry/link types, no `validate` callback, no coordinator extern, no provider query, no payment dispatch, no replay capability, and no Governance/Treasury routing.

Parent semantic subject: F1B2 `1ded163920ef7ea2829a807cf61c89f224751714`.

## Why F1B2A exists

F1B2 deliberately used one `expected_author` for its hostile-write reference model. That is useful for predecessor/index/race semantics but too coarse for live admission.

The journal has distinct authority roles:

```text
OperationIntent      → OrchestratorIntent
DispatchAttempt      → OrchestratorDispatch
OperationIndex       → OperationIndex
Observation          → ProviderObservation
```

The settlement provider must not inherit authority to create constitutional intent or dispatch facts, and the orchestrator must not manufacture provider-origin settlement evidence.

## Constitutional authority remains upstream

The existing `constitutional-authority` model says automated agents do not hold constitutional sovereignty and issued authority is bound to a concrete holder identity. F1B2A preserves that boundary.

```text
Stewardship constitutional capability
  power = ExecuteAppropriation
             ↓
PublicFundsAuthorityAnchor
             ↓
ProviderAuthorGrant
             ↓
authority to author specific evidence/orchestration facts
```

A `ProviderAuthorGrant` is **not** a `ConstitutionalCapability`. It does not transfer value, appropriate funds, or make an automated provider a sovereign constitutional actor.

Version 1 accepts only a validated root Stewardship `ExecuteAppropriation` capability. A delegated constitutional parent chain remains out of scope until its own chain evidence is qualified.

## Provider-author grant identity

Every grant commits to:

- grant ID and epoch;
- exact provider-profile ID and profile commitment;
- jurisdiction;
- exact upstream authority identity/commitment and validity;
- canonical non-empty record roles;
- direct-author or threshold-committee mode;
- validity interval;
- predecessor grant commitment when rotated.

The grant uses a domain-separated BLAKE3 commitment. Wall-clock observation time is not grant identity.

### Important non-claim: grant construction is not grant issuance

`ProviderAuthorGrant::new(...)` constructs and validates the semantic shape of a grant. It does **not** prove that Stewardship actually issued that grant.

A future stateful authority timeline must verify an issuance act against the exact upstream constitutional capability before a grant may authorize a DHT fact. F1B2A therefore records:

```text
issuance_proof_is_qualified = false
```

No caller-created grant object should be treated as a production bearer capability.

## Direct-author mode

A direct grant names exactly one author identity. Future active validation must bind the actual Holochain action author:

```text
action.author == grant.author_id
```

A DID copied into entry data is not sufficient.

## Threshold-author mode

Threshold mode binds:

- committee ID;
- committee epoch;
- threshold and member count;
- committee descriptor commitment;
- public-key commitment;
- scope commitment;
- exact journal-record commitment signed by the threshold proof.

The exact current threshold-signing implementation is useful substrate, but **not provider-author authority evidence yet**. At the frozen F1B2 parent it exposes committee threshold/member count, public key, scope, active state, epoch and threshold-signature records, but its create validators ignore the action author, link validation is permissive, and `check_signature_validity` checks signature structure/length rather than verifying the signature against the committee public key.

Accordingly, a stored `verified` boolean or structurally valid threshold record is not enough. F1B2B must cryptographically verify the attestation and reconstruct the committee's active epoch/scope state at the candidate action time.

## Rotation

A valid rotation edge requires:

```text
successor.epoch = predecessor.epoch + 1
successor.predecessor = exact predecessor grant commitment
provider profile unchanged
jurisdiction unchanged
upstream constitutional anchor unchanged
successor roles ⊆ predecessor roles
successor.valid_from = exact cutover
```

Ordinary rotation cannot amplify roles. A role expansion is a new authorization decision, not key rotation.

Changing direct-author identity or moving between direct and threshold modes is allowed only through such an exact successor edge.

### Important non-claim: pairwise rotation is not a stateful timeline

`validate_rotation(predecessor, successor, cutover)` proves one edge. By itself it cannot discover that a different conflicting successor for the same next epoch exists elsewhere.

Therefore:

```text
same_epoch_competing_successor_allowed = false
stateful_unique_successor_enforced_in_f1b2a = false
```

F1B2B must maintain the authority timeline and fail closed on competing successors.

## Revocation, supersession and historical evidence

Grant state is evaluated at the **candidate action time**, not an auditor's current clock.

```text
action at t=999
revocation effective t=1000
→ historical action may remain valid

action at t=1000
→ rejected
```

The same rule applies to supersession/cutover.

Historical author evidence binds:

- exact grant commitment and epoch;
- exact role;
- exact journal-record commitment;
- exact candidate action time;
- direct author identity or threshold proof metadata.

Later revocation or rotation cannot retroactively invalidate a fact that was valid when authored.

## F1B2 + F1B2A composition

F1B2 remains the reference theorem for hostile DHT writes, exact projection, predecessor ordering, immutable facts and index conflict semantics. F1B2A supplies the role-aware authority theorem that F1B2 intentionally lacked.

The future admission theorem is therefore:

```text
F1B2 canonical-source / race / immutability checks
                     +
F1B2A role-aware author / historical-time checks
                     ↓
F1B2B stateful authority-aware admission model
```

F1B2B must additionally prove grant issuance, reconstruct grant and threshold-committee state at candidate action time, cryptographically verify threshold attestations, and enforce one canonical successor per authority epoch.

## Activation boundary

F1B2A adds no live DHT surface. It does not register `EntryTypes` or `LinkTypes`, activate `validate`, mint replay capabilities, modify the payments zome, or route Governance/Treasury execution.

Only after the authority theorem and the stateful F1B2B composition are independently qualified should active F1B3 registration be considered.

## Non-claims

F1B2A does **not** establish grant issuance qualification, a stateful authority timeline, provider-authority qualification, threshold-signing qualification for provider authorship, Holochain persistence, payments-zome correctness, automated-agent sovereignty, replay qualification, capability minting, physical exactly-once settlement, external finality, live Governance routing, deployment currentness, or qualification.
