# mycelix-stewardship-cases

STEW-005 represents joint, contested, under-review, and unresolved stewardship cases without ranking claimants or selecting a winner.

## Core theorem

```text
multiple claims
!= automatic winner
!= highest reputation wins
!= highest stake wins
!= majority vote establishes cultural legitimacy
```

A case groups claim references around one explicit target/domain and records the current **case state**, not a verdict on truth or legitimacy.

## Case states

- `SingleRecordedClaim` — exactly one claim is currently in the case. This does not mean uncontested in the real world.
- `CompatibleMultipleClaims` — multiple claims are recorded as currently compatible. This does not itself establish joint authority.
- `Contested` — at least two claims are recorded as conflicting.
- `UnderReview` — one or more claims are undergoing an external review process.
- `Unresolved` — one or more claims exist and no stronger structural state is asserted.

There is deliberately no `Winner`, `PrimarySteward`, score, rank, or selected-claim field.

## Status evidence

Any state stronger than `Unresolved` requires at least one status-basis reference. STEW-005 does not evaluate those references. For example, a `CompatibleMultipleClaims` state might point to a signed joint-stewardship agreement, while `Contested` might point to mutually incompatible claim statements.

## Referential firewall

The theorem does not dereference claim IDs and therefore does not prove that referenced claims actually match the case target/domain. That belongs in a later admission/resolution layer. This avoids pretending that structural grouping is verification.

## Non-claims

No stewardship legitimacy, cultural authority, legal title, community mandate, ownership, copyright, access right, policy authority, joint agreement validity, dispute resolution, or real-world consent is established.
