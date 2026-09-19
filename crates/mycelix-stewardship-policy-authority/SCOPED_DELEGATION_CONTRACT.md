# STEW-013 — Scoped Delegation Authority Contract v0.1

## Purpose

Freeze the evidence and scope boundaries required before a principal other than an admitted stewardship claimant may act as a policy issuer or authority delegate.

STEW-012B deliberately requires:

```text
asserted_policy_issuer == admitted_claimant
```

That is safe but intentionally narrow. STEW-013 defines what a later delegation theorem must establish before relaxing that restriction.

## Core theorem

```text
delegation record exists
!= delegator had authority to delegate
!= delegate identity authenticated
!= delegation current
!= delegation covers requested target/action
!= subdelegation permitted
!= runtime authority granted
```

and:

```text
represented_collective != delegation
membership != delegation
employment != delegation
title/role label != delegation
DID control != delegation
key possession != delegation
reputation/stake != delegation
```

## Delegation is scoped, never scalar

A future `StewardshipDelegationV1` should bind at minimum:

- delegation identity;
- asserted delegator;
- asserted delegate;
- exact stewardship domain(s);
- target scope;
- action/capability scope where applicable;
- purpose/context scope where applicable;
- explicit subdelegation policy;
- validity/currentness references;
- revocation/supersession references;
- mandate/delegation evidence;
- evaluation profile reference.

There should be no generic `is_admin`, `can_manage_everything`, or scalar authority level in the stewardship core.

## Target scope

Delegation target breadth must be explicit and use the same structural distinctions already frozen by STEW-004:

```text
ExactRepresentation
Revision
Subject
```

A delegation covering one exact recording does not silently cover another representation, later revision, translation, restoration, remix, or derivative.

Subject-wide delegation, when allowed, must be visibly broad rather than inferred from one artifact.

## Domain scope

Delegation in one stewardship domain cannot expand into another:

```text
Preservation delegation
!= AccessPolicyParticipation delegation
!= CulturalProtocol delegation
!= RightsContext delegation
!= Reciprocity delegation
```

A later STEW-012B successor may accept a delegated issuer only when the delegation explicitly covers `AccessPolicyParticipation` and all other required policy-authority checks still succeed.

## Action scope

Where a delegation grants authority over use-policy decisions, action scope should remain explicit.

Examples:

```text
may issue View policy
!= may issue TrainAi policy

may authorize preservation copy
!= may authorize public disclosure

may manage metadata
!= may authorize commercialization
```

A delegation with no action coverage for a requested consequential action fails closed.

## Purpose / context scope

Delegation may be purpose- or context-limited without embedding sensitive prose in the generic theorem. Opaque references can identify admitted context profiles such as:

- archival preservation;
- classroom/educational use;
- community research;
- emergency conservation;
- public exhibition;
- commercial licensing;
- AI training or inference.

The generic layer carries those references but does not decide their legitimacy.

## Delegator authority ceiling

A delegate cannot receive more authority than the delegator is independently established to possess.

```text
delegated_scope <= evaluated_delegator_scope
```

This is a later evaluation theorem, not something a delegation record can self-certify.

A broad delegation document from a principal with narrow or unresolved authority remains unresolved.

## Subdelegation

Subdelegation is never implicit.

A closed direction for a later profile is:

```text
Forbidden
ExactScopeOnly
NarrowerScopeOnly
ProfileGoverned
```

A delegate may not infer subdelegation merely because it can exercise the underlying authority itself.

Delegation chains must retain every hop and their evidence. A later graph theorem should bound chain depth and reject cycles for runtime authority evaluation.

## Currentness, revocation, and succession

Delegation must model currentness independently from historical existence.

```text
historically delegated != currently delegated
revoked now != historical delegation erased
expired != never existed
superseded != deleted
```

A future evaluator should represent activation, expiry, revocation, supersession, and succession evidence explicitly.

Succession after death, institutional dissolution, role turnover, or community governance change must be profile-governed rather than inferred from account possession or newest-record-wins.

## Collective representation

STEW-004 `represented_collective` remains only a claim field.

```text
claimant says "I represent collective X"
!= X delegated policy authority to claimant
```

A community mandate may be evidence for delegation, but the generic protocol must not fabricate community consent from the field itself.

## Technical capability firewall

Delegation remains independent from Xenia/Holochain/key capability.

```text
can sign != delegated authority
can decrypt != delegated disclosure authority
holds capability grant != cultural authority
controls DID != community mandate
```

Technical capability can enforce an already-admitted delegation, but cannot manufacture the delegation.

## Conflict and multiple delegates

Multiple delegations may coexist.

The generic layer must not silently infer:

- exclusive authority;
- newest delegate wins;
- highest reputation wins;
- senior job title wins;
- longest chain wins;
- shortest chain wins.

Overlapping or conflicting delegations must feed the explicit cross-policy / authority conflict layer rather than being collapsed automatically.

## Privacy

Delegation evidence may itself be sensitive. A public delegation identifier or descriptor must not require the underlying mandate, membership roster, sacred governance process, personal information, or protected community records to be publicly disclosed.

```text
delegation verifiable
!= delegation evidence public
```

Protected metadata/existence rules from STEW-020A/B continue to apply.

## Synthetic qualification corpus for an executable child

A future typed theorem should include at least:

1. admitted claimant delegates exact `View` policy participation for one exact representation -> structural candidate;
2. same delegation used for `TrainAi` -> reject;
3. preservation-only delegation used for policy issuance -> reject;
4. exact-representation delegation used for later revision -> reject;
5. expired delegation -> reject for current authority while retaining history;
6. revoked delegation -> reject current authority;
7. represented-collective field without delegation evidence -> reject;
8. decryption/signing capability without delegation -> reject;
9. subdelegation without explicit permission -> reject;
10. conflicting active delegations -> unresolved/conflict state, no automatic winner;
11. delegation chain attempts to expand scope -> reject;
12. private delegation evidence referenced without publishing protected bytes -> structurally representable.

## Deliberate non-claims

This contract establishes no delegator authority, delegate identity, community consent, cultural legitimacy, legal agency, employment authority, copyright/title, currentness, revocation correctness, succession legitimacy, subdelegation authority, policy authority, access permission, AI-use permission, or runtime capability.
