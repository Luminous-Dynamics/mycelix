# STEW-012 — Policy Authority Binding Contract v0.1

## Purpose

Freeze the composition theorem needed before any STEW-003 knowledge-use policy can become an authority-bearing input to runtime authorization.

```text
policy exists
!= policy issuer authenticated
!= policy issuer authorized
!= stewardship authority
!= cultural authority
!= runtime authorization
```

STEW-003 intentionally classifies actions without authenticating the policy issuer. STEW-004/008 intentionally record and profile-relatively admit stewardship claims without turning them into universal authority. This contract defines how those lines may later compose without collapsing their non-claims.

## Required composition

A later executable `PolicyAuthorityBindingV1` should bind at minimum:

1. the complete exact `KnowledgeUsePolicyV1`;
2. an asserted policy issuer identity;
3. one or more exact STEW-008 admission records whose underlying claim includes `AccessPolicyParticipation`;
4. an explicit delegation/mandate basis connecting the admitted stewardship relationship to the asserted issuer;
5. a versioned authority-evaluation profile;
6. explicit currentness/supersession/revocation evidence;
7. evidence references for the binding decision.

The binding result must remain an **authority candidate** until every referenced admission, delegation, identity, currentness, and profile requirement has been independently evaluated.

## Domain firewall

A stewardship claim/admission for one domain cannot manufacture another domain.

```text
admitted Preservation
!= AccessPolicyParticipation

admitted Attribution
!= AccessPolicyParticipation

admitted CulturalProtocol
!= AccessPolicyParticipation
```

Only an explicitly claimed and admitted `AccessPolicyParticipation` domain may participate in the v1 policy-authority binding path.

This still does not make the claimant or represented collective a universal policy authority.

## Target-coverage theorem

The admitted stewardship target and the STEW-003 exact policy target must be compared structurally.

A later theorem may recognize only these coverage relations:

```text
ExactRepresentation claim
  covers policy target
  iff exact STEW-001 identity matches

Revision claim
  covers policy target
  iff subject + revision match

Subject claim
  structurally contains policy target
  iff logical subject matches
```

But structural coverage is not authority scope.

```text
claim target structurally contains policy target
!= claimant authorized to govern every use of that target
```

The admitted domain, admission profile, delegation/mandate, rights/cultural context, and currentness must still support the requested authority.

## No derivative inheritance

An authority binding for one exact policy target does not silently propagate through STEW-006 provenance.

```text
policy-authority(work/representation A)
!= policy-authority(translation B)
!= policy-authority(remix C)
!= policy-authority(restoration D)
!= policy-authority(future revision E)
```

Any propagation must be an explicit later theorem with its own evidence and policy.

## Issuer / represented-collective separation

A STEW-004 claim can say that principal P asserts representation of collective C.

Even if that claim is admitted under a profile:

```text
P claims to represent C
+ claim admitted under profile X
!= P may author every policy for C
```

A separate mandate/delegation basis must bind the asserted issuer to the specific policy-authority scope.

Likewise, the collective identifier itself must not be treated as a signing key or executable principal.

## Admission relativity

STEW-008 decisions are explicitly profile-relative. Therefore a future policy-authority binding must retain the exact admission profile and cannot rewrite:

```text
AdmittedUnderProfile(P)
```

into:

```text
UniversallyAuthorized
```

Different admission systems may disagree. Mycelix must retain those differences rather than silently selecting the highest-reputation, most-funded, government-issued, or most-recent record as universally correct.

## Currentness and revocation

Authority must be evaluated for the relevant decision time/epoch, not by `latest write wins`.

A later executable theorem must define explicit semantics for:

- activation/effective state;
- supersession;
- revocation;
- expiry where applicable;
- historical validity;
- conflicting successors;
- missing predecessor/currentness evidence.

Core invariant:

```text
revoked now
!= historical policy never existed

historically admitted
!= currently authorized
```

Unknown currentness must fail closed for consequential authorization.

## Multiple authority bases and disagreement

Real cultural/legal/stewardship contexts can contain multiple relevant authorities that disagree.

The v1 composition must not define:

- highest reputation wins;
- largest stake wins;
- majority vote automatically wins;
- government record automatically defeats community stewardship;
- community record automatically defeats every legal right;
- newest record automatically wins.

Conflicting authority candidates should produce an explicit unresolved/conflict disposition until a domain-appropriate resolution profile supplies evidence.

## Policy conflict boundary

STEW-003's prohibition dominance is **inside one policy object**.

It does not imply that, across independently authored policies:

```text
any prohibition anywhere globally overrides every permission
```

Cross-policy composition needs a later explicit authority/conflict theorem that compares issuer authority, scope, target, profile, currentness, rights context, and cultural protocol.

## Cultural and legal plurality

Policy authority may have multiple non-substitutable dimensions:

```text
legal right
!= cultural legitimacy
!= stewardship relationship
!= technical key possession
```

A runtime can require several dimensions simultaneously without pretending they are interchangeable.

For example, legal permission may be necessary but insufficient for a culturally restricted use; conversely a cultural stewardship relationship does not automatically create copyright ownership or waive applicable law.

## Unknown / missing evidence

The later executable binding must fail closed when required authority evidence is missing or indeterminate.

```text
no authority evidence
-> no policy-authority candidate

conflicting unresolved authority evidence
-> unresolved, not authorized
```

Absence of a competing claim is not proof of uncontested authority.

## Relationship to STEW-020/020B

Policy authority and metadata/payload disclosure remain separate checks.

```text
policy author authorized for View
!= envelope existence may be published

policy author authorized for Retrieve
!= metadata may be indexed

technical decryption capability
!= policy authority
```

A protected-content runtime should compose, rather than collapse:

- STEW-020 technical protected-content binding;
- STEW-020B metadata publication candidate where a public envelope is appropriate;
- STEW-003 action classification;
- STEW-008 stewardship admission evidence;
- STEW-012 policy-authority binding;
- later constraint/duty satisfaction;
- identity/delegation/currentness verification.

## Relationship to reciprocity

A policy duty may reference STEW-010 reciprocity obligations, but:

```text
policy duty references obligation
!= obligation legitimate
!= obligation satisfied
```

STEW-011 receipts remain reported evidence rather than satisfaction truth. A later authorization process must not grant access merely because a claimant submitted a `ReportedFulfilled` receipt.

## Synthetic qualification corpus for executable child

1. exact-representation admitted access-policy claim + matching mandate + matching target -> authority candidate;
2. admitted preservation-only claim -> reject policy-authority binding;
3. matching subject but different exact representation under an exact-only claim -> reject;
4. revision-wide claim structurally covering exact target but missing delegation -> reject;
5. subject-wide claim + explicit narrow delegation -> candidate only for delegated scope;
6. admitted claim under profile A but rejected under profile B -> explicit conflict, no universal winner;
7. revoked/superseded mandate -> no current authority candidate while history remains intact;
8. policy issuer possesses decryption key but no policy mandate -> reject;
9. policy duty has a reciprocity receipt but satisfaction unresolved -> no promotion;
10. derived translation/remix has no explicit authority propagation -> original binding does not transfer.

Use synthetic cultural/community fixtures only.

## Implementation sequencing

Do not implement the typed composition while STEW-003 and STEW-008 remain isolated sibling draft lineages if doing so would require duplicating either theorem into an unrelated branch.

Preferred sequence:

```text
qualify/review STEW-003
qualify/review STEW-004 -> STEW-008
establish shared ancestry/composition branch
-> implement STEW-012A typed authority binding
-> implement STEW-012Q adversarial qualification
```

The composition branch must retain exact predecessor identities rather than reconstructing the types from memory.

## Non-claims

This contract does not establish policy authority, identity proofing, delegation validity, stewardship legitimacy, cultural authority, copyright/title, legal compliance, conflict resolution, currentness, revocation correctness, runtime authorization, access permission, AI-use permission, or community consent.

It freezes only the composition rules and fail-closed boundaries required before those claims can be attempted.
