# Justice Adjudication Reducer v0.1

This crate is the pure conflict-aware bridge between authenticated Justice adjudication evidence and the panel/vote snapshots consumed by `justice-resolution-verifier`.

It does **not** fetch Holochain records and does not authenticate DHT authorship. That belongs to the Justice runtime evidence layer introduced by PR #349.

## Frozen reduction profile

`justice.authenticated-adjudication-reduction@1`

The reducer consumes only:

- one exact Arbitration reference;
- one exact Decision reference;
- the exact selected-panel identities;
- already-authenticated participation evidence;
- already-authenticated vote evidence whose exact Accepted participation basis is named explicitly.

There is no API for legacy `Arbitration.accepted/recused` flags or embedded `Decision.votes[]` aggregates, so the reducer cannot silently fall back to weaker authority.

## Conflict semantics

Reduction is deterministic and set-based, never arrival-order-based.

For one selected actor:

- zero participation attestations means selected but currently inactive;
- exactly one `Accepted` attestation means active;
- exactly one `Recused` attestation means recused;
- two or more participation attestations fail closed, even when they repeat the same disposition;
- a vote requires exactly one `Accepted` participation attestation;
- the vote's `acceptance_attestation_ref` must equal that exact accepted-attestation ref;
- two or more vote attestations from one actor fail closed, even when they repeat the same choice.

Evidence refs are globally unique within their respective participation/vote sets. Reusing one attestation ref across different actors is denied.

This deliberately preserves ambiguity instead of inventing `latest wins` semantics.

## Output

Success returns `ReducedAdjudicationEvidenceV1`:

- sorted `PanelMemberSnapshotV1` values for the downstream verifier;
- sorted `DecisionVoteSnapshotV1` values;
- `AdjudicationReductionReceiptV1`.

The receipt binds the exact Arbitration/Decision refs, reduction profile/version, selected panel, every reduced participation attestation + disposition, and every reduced vote attestation + exact acceptance basis + choice.

The same semantic input set in a different order returns the same output and receipt.

## Claim boundary

A successful reduction means only:

> this exact already-authenticated evidence set is structurally unambiguous under reduction profile v1 and can be represented as one exact panel/vote snapshot.

It does **not** establish:

- DHT authenticity or completeness;
- that no unseen conflicting DHT evidence exists;
- quorum or decision-rule satisfaction;
- policy authority;
- remedy semantics;
- appeal/finality;
- Finance execution;
- Business closure.

The intended chain is:

```text
#349 authenticated DHT participation/vote evidence
        -> justice-adjudication-reducer
        -> #339 justice-resolution-verifier
        -> future authenticated finality
        -> future Justice -> Finance/Business bridge
```
