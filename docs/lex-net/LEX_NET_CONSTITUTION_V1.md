# LEX-NET Constitution v1

Status: normative research architecture; no legal effect claimed.

## Purpose

LEX-NET defines a cross-sovereign institutional interoperability layer for Mycelix. It is inspired by the way maritime systems, Internet standards, private international law, and cross-border trade frameworks allow independently governed actors to cooperate without requiring one global sovereign.

The governing theorem is:

```text
shared protocol convention
!= law
!= treaty
!= jurisdiction
!= adjudicative authority
!= external-effect authority
```

Mycelix may carry, bind, verify, translate, and locally qualify evidence about institutions, identity, agreements, standards, transactions, assurance, and external legal processes. It may not manufacture legal effect by protocol fiat.

## Constitutional relationship to SOV-DPI

LEX-NET sits above the sovereign-deployment and institutional-authority foundations.

```text
SOV-DPI / local institutional constitution
        ↓
local authority + local recognition policy
        ↓
LEX-NET cross-sovereign evidence exchange
        ↓
locally qualified evidence
        ↓
ordinary local authority/effect qualification
```

A foreign artifact can become locally usable evidence only through an explicit local recognition policy. Foreign evidence does not directly import foreign authority.

## LN-001 — protocol convention is not law

A LEX-NET profile is a technical/institutional interoperability convention. It is not legislation, a treaty, a court order, a regulation, or legal advice.

A profile may reference an external legal instrument or standard by exact identity/version. That reference does not establish that the instrument applies, has been enacted, is in force, binds the parties, or produces a particular legal result.

## LN-002 — foreign evidence is not local authority

Authentication, integrity, provenance, or foreign authorization may establish facts about the origin context. They do not directly create local `AuthorityGrant`, administrative authority, adjudicative authority, execution authority, or external-effect authority.

```text
foreign authenticated evidence
+ local recognition policy
        -> locally qualified evidence
        != local authority
```

Any consequential local action still traverses the ordinary local authority theorem.

## LN-003 — recognition is explicit and local

Recognition is never inferred from:

- a valid signature;
- protocol participation;
- shared software;
- matching identifier strings;
- validator majority;
- reputation;
- stake;
- capital contribution;
- AI output; or
- Luminous Dynamics endorsement.

A positive recognition result binds the exact foreign profile, exact local recognition policy, exact scope, relevant freshness/currentness evidence, and explicit nonclaims.

## LN-004 — legal metadata is evidence, not a verdict

LEX-NET may carry structured references to governing law, jurisdiction, choice of court, arbitration, mediation, treaty/convention profiles, reservations, exclusions, and legal-review evidence.

These are evidence inputs for legitimate institutions and professionals. Mycelix does not decide that a choice-of-law clause is valid, that a court has jurisdiction, that an award is enforceable, or that a treaty applies merely because the fields are well formed.

## LN-005 — external legal process remains external

Courts, arbitral tribunals, mediators, ombuds, regulators, and other legitimate forums remain external authority domains unless a specific lawful integration is independently established.

A cryptographically authentic judgment, award, settlement, or regulatory decision is still distinct from:

- recognition in another jurisdiction;
- enforceability;
- remedy authority;
- execution authority; and
- confirmed external effect.

LEX-NET may package and preserve the evidence chain. It does not replace the forum.

## LN-006 — identity and trust-service equivalence is profile-bound

Cross-border identity/trust-service evidence must preserve trust domain, assurance/reliability profile, issuer/provider identity, subject binding, currentness, and local recognition semantics.

```text
same textual identifier
!= same legal/operational identity
```

Identity evidence remains distinct from role, capability, citizenship, residency, legal status, and authority.

## LN-007 — transferable-record integrity is not legal title

LEX-NET may support technology-neutral control/integrity/history profiles for electronic transferable records and trade documents.

```text
digitized document
!= qualified transferable-record control history
!= legal title
```

No token, hash, NFT, DHT entry, or holder field is sufficient by itself to establish property rights, negotiability, secured-credit priority, or legal control.

## LN-008 — assurance is plural and locally recognized

Independent assurance/classification providers may issue bounded attestations under exact methods and scopes. No provider is globally trusted by default.

A local recognition policy decides whether a given provider/profile is accepted for a given purpose. Self-recognition cannot bypass that policy.

Conflicting assurance remains conflicting evidence; it is not collapsed into an opaque universal trust score.

## LN-009 — external standards remain externally owned

LEX-NET prefers adapters and conformance mappings over reinvention. External organizations remain authoritative for their own standards and instruments.

Initial reference families include:

- UNCITRAL Model Law on Electronic Commerce (1996);
- UN Convention on the Use of Electronic Communications in International Contracts (2005);
- UNCITRAL Model Law on Electronic Transferable Records (2017);
- UNCITRAL Model Law on the Use and Cross-border Recognition of Identity Management and Trust Services (2022);
- UNCITRAL Model Law on Automated Contracting (2024);
- HCCH Choice of Court / Judgments instruments for external jurisdiction/recognition concepts;
- New York Convention and Singapore Convention for external arbitration/mediation recognition/enforcement concepts;
- IETF SCITT / RFC 9943 for signed-statement transparency architecture.

A Mycelix mapping must not imply endorsement, official conformance, or legal compliance unless the relevant external authority independently establishes that status.

## LN-010 — conformance is not authority

A LEX-NET TCK may establish that an implementation maps one frozen external profile into one frozen Mycelix profile as specified.

```text
TCK PASS
!= legal compliance
!= official certification
!= trusted issuer
!= local authority
!= production readiness
```

Conformance tooling must be effects-disabled and authority-neutral unless a later independently qualified consumer explicitly composes it.

## LN-011 — convention governance is scope-limited

Stewardship of LEX-NET profiles does not create authority over participant institutions.

The target governance model is narrow, open, versioned, reviewable, forkable, and multistakeholder. It should coordinate only where shared interoperability materially helps.

Luminous Dynamics, one government, one corporation, one assurance provider, capital holders, validators, or compute owners must not receive unilateral global convention authority merely through their status.

## LN-012 — AI remains advisory in legal interpretation

Symthaea or another model may:

- summarize instruments;
- compare profiles;
- detect contradictions;
- propose mappings;
- identify missing evidence;
- model consequences; and
- assist authorized professionals/institutions.

Model output is not binding legal interpretation, adjudication, recognition, or authority by default.

## Maritime design analogy

Maritime governance is a useful analogy, not a source of Internet law.

| Maritime concept | LEX-NET design analogue |
| --- | --- |
| Flag/source jurisdiction | Origin institution/jurisdiction profile |
| Port State Control | Local admission/reverification policy |
| Classification society | Independently recognized assurance provider |
| Bill of lading | Transferable-record/trade-document profile |
| Convention/MoU | Versioned interoperability/recognition profile |

The analogy does not imply that maritime law applies to digital interaction or that LEX-NET has equivalent statutory status.

## Threat model

LEX-NET-000 freezes twelve foundational threats:

1. foreign authentication promoted directly into local authority;
2. technical conformance promoted into legal validity;
3. caller-selected law/forum metadata treated as dispositive;
4. stale/superseded external profile reused as current;
5. assurance provider self-recognition;
6. cross-border identity equivalence inferred from matching strings;
7. treaty/convention status inferred from software configuration;
8. external decision evidence treated as self-executing enforcement authority;
9. AI legal interpretation treated as binding decision;
10. private Mycelix governance attempting to override mandatory external/local law;
11. evidence translation dropping reservations, exclusions, uncertainty, or nonclaims;
12. global reputation/validator majority promoted into universal recognition authority.

## Continuation

The intended sequence is:

```text
LEX-NET-000  constitution / threat model
LEX-NET-001  local recognition profile
LEX-NET-002  cross-border identity/trust-service bridge
LEX-NET-003  governing-law/forum/jurisdiction metadata
LEX-NET-004  dispute evidence + external adjudication handoff
LEX-NET-005  electronic transferable-record profile
LEX-NET-006  independent assurance-provider recognition
LEX-NET-007  external standards conformance TCK
LEX-NET-008  convention governance / scope limits
LEX-NET-Q1   adversarial cross-sovereign campaign
```

No child may infer legal effect from a parent technical PASS.

## Qualification boundary

A green LEX-NET-000 qualification establishes only that this exact constitution, manifest, validator, and workflow are internally coherent under the frozen machine corpus.

It does **not** establish:

- legal validity;
- treaty status or treaty compliance;
- statutory or regulatory compliance;
- applicable law;
- jurisdiction;
- recognition or enforcement by a court/state;
- official endorsement by UNCITRAL, HCCH, IETF, IMO, ICANN, or any other external body;
- production security or readiness;
- international adoption; or
- authority for any external effect.

The network remains infrastructure for institutions. It is not the sovereign.