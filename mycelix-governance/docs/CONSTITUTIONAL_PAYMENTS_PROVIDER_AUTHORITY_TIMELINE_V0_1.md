# MYC-CONST-003D1D-F1B2B0 — Provider Author Authority Timeline v0.1

## Status

This tranche is a **pure, inert, stateful authority-history theorem**.

Parent semantic subject: F1B2A `d9f1ad518e2943eb64c9991b2589cf9a221fc047`.

It creates no Holochain entry/link types, no `validate` callback, no coordinator extern, no DHT writes, no payment dispatch, no replay-capability minting, and no live Governance/Treasury routing.

## Why F1B2B0 exists

F1B2A established the shape of a bounded provider-author grant and proved pairwise rules for role separation, revocation timing and monotone rotation. It deliberately did **not** prove two stateful properties:

1. that one predecessor has only one canonical successor in accepted history;
2. that grant issuance/revocation events can be reconstructed as one append-only authority timeline.

F1B2B0 adds exactly that stateful layer.

```text
F1B2A ProviderAuthorGrant
       |
       v
append-only AuthorityTimeline
       |
       +-- GrantIssued
       +-- GrantRevoked
       +-- GrantSuperseded
       |
       v
status_at(grant, candidate_action_time)
       |
       v
F1B2A authorize_record(...)
```

## Event ordering

Authority history is ordered by an explicit contiguous logical `sequence` and stable `event_id`.

Wall-clock time is not used to decide append order or authority identity.

An exact event replay at the same sequence and identity is idempotent. A different event at an already-used sequence or reuse of an event ID at another sequence is evidence of contradictory history and installs a sticky integrity halt.

A mere sequence gap is different: it is an invalid candidate and is rejected without poisoning the accepted history.

## Issuance evidence

A genesis provider-author grant must have exact issuance evidence binding:

- issuance ID;
- exact grant commitment;
- upstream constitutional capability ID;
- upstream concrete holder ID;
- upstream jurisdiction;
- upstream authority commitment;
- issuance time;
- external issuance-proof commitment.

The issuance evidence must match the `PublicFundsAuthorityAnchor` already frozen into the F1B2A grant.

Issuance must occur no later than `grant.valid_from_us` and while the upstream constitutional authority is valid.

### Important boundary

F1B2B0 validates **the binding to a proof commitment**. It does not cryptographically verify the external issuance proof.

Therefore:

```text
proof_commitment present and correctly bound
        !=
issuer signature cryptographically verified
```

The latter remains a required later tranche before live authority activation.

## Genesis and multiple authority roles

The timeline may contain more than one epoch-1 root grant. This is intentional: F1B2A separated orchestration authority from provider observation authority.

For example:

```text
orchestrator root grant
    roles = intent / dispatch / index

provider root grant
    roles = provider observation
```

These are independent authority lineages unless an upstream authorization deliberately grants the same identity both role sets.

A grant accepted through `issue_genesis` must have:

```text
epoch = 1
predecessor_grant_commitment = None
```

A successor grant cannot bypass rotation by entering through the genesis path.

## Rotation

Rotation composes the F1B2A `validate_rotation` theorem with accepted-history uniqueness.

A successor must preserve:

- exact provider profile identity/commitment;
- exact jurisdiction;
- exact upstream constitutional anchor;
- role attenuation (`successor.roles subset predecessor.roles`);
- exact predecessor grant commitment;
- `successor.epoch = predecessor.epoch + 1`;
- `successor.valid_from_us = cutover`.

F1B2B0 additionally requires:

```text
one predecessor -> at most one canonical successor
```

Once a successor is accepted, a different successor proposal for the same predecessor is not merely malformed input. It contradicts accepted authority history and therefore installs a sticky integrity halt.

The cutover cannot occur after predecessor expiry.

## Revocation

Revocation evidence binds the exact grant and exact upstream constitutional authority plus:

- revocation ID;
- authorization time;
- effective time;
- external revocation-proof commitment.

The authorization time cannot be after the effective time. The upstream authority must be valid at both times. The effective time cannot precede grant validity or occur after grant expiry.

As with issuance, F1B2B0 does not cryptographically verify the external revocation proof.

## One terminal event per grant

A grant may have one accepted terminal authority event:

```text
Revoked
   OR
Superseded
```

A second contradictory terminal history is an integrity fault.

This distinction is deliberate:

```text
invalid proposed candidate
    -> Rejected

contradiction with already accepted history
    -> IntegrityHalted
```

## Historical status

`status_at(grant, action_time)` reconstructs one of:

- `NotYetValid`;
- `Active`;
- `Revoked`;
- `Superseded`;
- `Expired`.

The query is evaluated at the candidate action time, never at the auditor's current wall clock.

That gives the required historical behavior:

```text
grant active at t=999
revoked effective at t=1000

status_at(t=999)  -> Active
status_at(t=1000) -> Revoked
```

and similarly for rotation cutover.

A later authority change therefore does not erase valid earlier evidence.

## Authorization composition

`authorize_at` first reconstructs timeline status at `HistoricalAuthorEvidence.action_time_us`, then calls F1B2A `authorize_record` with the resulting `ProviderGrantStatus`.

This intentionally preserves the existing F1B2A checks for:

- exact grant commitment/epoch;
- provider profile ID/commitment;
- jurisdiction;
- role;
- direct-author or threshold-proof mode;
- record commitment;
- validity timing.

If the timeline has an integrity fault, authorization fails closed regardless of the requested historical grant.

## Threshold boundary

F1B2B0 preserves F1B2A's threshold descriptor and proof target but does not add cryptographic threshold verification or historical committee-state reconstruction.

Those remain explicit false profile claims:

```text
cryptographic_threshold_attestation_verified = false
threshold_committee_state_reconstructed = false
```

## Next tranche

`MYC-CONST-003D1D-F1B2B1` should compose this timeline with F1B2 hostile-write admission.

That later work must also establish, before live provider-author activation:

1. an actual cryptographic issuance/revocation proof verifier;
2. actual Holochain action-author binding;
3. cryptographic threshold-attestation verification;
4. historical threshold committee state/epoch reconstruction;
5. role-aware admission for each F1B0/F1B1 record class.

It must still not activate payment dispatch merely because evidence persistence becomes available.

## Non-claims

F1B2B0 does not establish provider-authority timeline qualification, cryptographic grant issuance, cryptographic revocation, threshold attestation verification, threshold committee-state reconstruction, Holochain persistence, live admission composition, payments-zome correctness, replay qualification, capability minting, exactly-once physical settlement, Governance routing, deployment currentness, or qualification until an exact-head verifier passes.
