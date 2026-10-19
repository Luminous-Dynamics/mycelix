# STEW-000 — Human Knowledge Stewardship Constitution v0.1

Status: preregistration / architecture boundary only

## Purpose

Freeze the cross-domain boundary for stewarding human knowledge, arts, cultural memory, scientific work, educational material, oral tradition, software, and other creative or knowledge-bearing artifacts before adding a new runtime or generalized rights system.

The purpose of STEW is not to create a central owner of human knowledge. It is to let existing Mycelix domains compose around explicit stewardship, provenance, access, rights, preservation, and reciprocity boundaries without collapsing those concepts into one another.

The initial implementation direction is a small dependency-light shared stewardship kernel consumed by existing domains such as Knowledge, Attribution, Commons, Identity, Music, Praxis, and later Symthaea.

## Core theorems

```text
knowledge exists
!= knowledge is public
!= permission to discover
!= permission to retrieve
!= permission to reproduce
!= permission to transform
!= permission to commercialize
!= permission to train AI
!= permission to disclose
```

```text
preservation
!= disclosure
!= reuse
!= extraction
```

```text
epistemic authority
!= stewardship authority
!= access authority
!= rights authority
!= cultural authority
!= preservation authority
!= reciprocity authority
```

```text
EpistemicRelationship
!= ProvenanceRelationship
!= StewardshipRelationship
!= RightsRelationship
!= ReciprocityRelationship
```

No score, credential, reputation system, governance outcome, ownership record, access grant, or preservation record may silently substitute for another authority domain.

## Existing Mycelix systems to compose, not duplicate

### Knowledge

Knowledge already owns epistemic claims, evidence, challenges, classification, consensus, claim dependencies, ontologies, and belief-graph relationships. STEW must not turn provenance, copyright, access, cultural legitimacy, or preservation state into epistemic truth scores.

### Attribution

Attribution already demonstrates usage receipts, privacy-preserving usage attestations, reciprocal pledges, and stewardship-oriented contribution accounting. STEW should eventually generalize the reusable theorem beneath that system rather than building a second reciprocity subsystem.

### Commons

Commons already contains domain knowledge such as `water-wisdom::TraditionalPractice`, including the vocabulary `Public | CommunityOnly | ElderApproved | Sacred`. That vocabulary is useful UX/domain metadata, but STEW must distinguish such labels from actual confidentiality or access-control enforcement.

### Identity

Identity should remain the source of principals, DIDs, credentials, revocation, and selective-disclosure identity proofs. A credential can prove an identity or delegated role; it does not itself prove cultural legitimacy, content rights, or permission to disclose a protected artifact.

### Governance

Generic governance may adopt policies, but voting power, MATL score, token stake, or institutional authority must not automatically manufacture cultural stewardship legitimacy. Community-specific authority must be separately evidenced and scoped.

### Music / creative domains

Creative-domain accounting and rights systems should project onto canonical stewarded subjects and provenance lineages rather than redefining artifact identity independently for every domain.

## Public claims and protected knowledge

Mycelix should preserve strong public auditability where public claims or exercised authority affect others while also supporting legitimate confidentiality.

The constitutional rule is:

```text
public claim / exercised authority
=> auditable evidence and authority lineage

protected knowledge
=> no requirement to disclose protected content merely to make its policy auditable
```

A safe public descriptor may exist without publishing the protected payload.

Therefore `transparent authority` and `transparent content` are explicitly different propositions.

## Stewarded subject identity

A future STEW core should separate at least:

```text
logical work identity
!= revision identity
!= representation identity
!= content digest
```

Examples:

- a composition is not identical to one score edition;
- a score edition is not identical to a performance;
- a performance is not identical to a recording;
- a historical text is not identical to one translation;
- a software project is not identical to one package release;
- a traditional practice is not identical to one recorder's transcription.

This distinction is required for correct provenance, preservation, attribution, rights, and derivative tracking.

## Knowledge-use actions

A future policy theorem should reason over actions rather than one scalar access level. Initial vocabulary should consider:

```text
discover
inspect_metadata
retrieve
view
quote
translate
perform
reproduce
transform
research
commercialize
train_ai
infer_from
generate_derivative
redistribute
archive
```

The governing rule is capability separation:

```text
permit(view) != permit(reproduce)
permit(research) != permit(commercialize)
permit(view) != permit(train_ai)
permit(reason) != permit(disclose)
permit(derive) != permit(redistribute)
```

For protected content, unknown or unrecognized actions should fail closed.

## Community and cultural stewardship

STEW must support stewardship that is individual, collective, delegated, contested, temporally bounded, or unresolved.

A future `StewardshipClaim` should bind an exact subject plus claimant, represented community if any, scope, basis, supporting evidence, credentials, and validity interval.

Multiple claims must remain representable:

```text
multiple stewardship claims
!= automatic winner
```

Initial lifecycle vocabulary should include at least:

```text
Uncontested
JointlyStewarded
Contested
Unresolved
UnderReview
```

The system may preserve and compare evidence while explicitly declining to invent legitimacy it cannot establish.

## Provenance

STEW should not overload Knowledge's epistemic claim graph. A future provenance graph may include relations such as:

```text
DerivedFrom
TranslatedFrom
RestoredFrom
DigitizedFrom
PerformedFrom
RemixedFrom
MigratedFrom
QuotedFrom
TaughtBy
RecordedBy
```

Provenance should preserve exact predecessor identity and evidence when possible.

```text
derivative exists
=> predecessor lineage remains inspectable
```

## Preservation

Preservation must be a separate authority surface from truth, access, and ownership.

A future preservation manifest should be able to bind exact content digests, representation/format, replica attestations, fixity observations, migration lineage, recovery dependencies, and a preservation profile.

Core theorem:

```text
migration != replacement
```

When format migration occurs, the desired evidence shape is:

```text
original representation
+ migrated representation
+ transformation evidence
```

not silent replacement of historical bytes.

## Reciprocity

Reciprocity is not limited to money and should not be inferred from generic reputation.

Future reusable contribution vocabulary may include:

```text
Financial
Time
Translation
Teaching
Documentation
Preservation
Infrastructure
Compute
ResearchAccess
CommunityInvestment
RevenueShare
InKind
Other
```

Where money is represented in protocol theorems, canonical integer/fixed-point amounts should be preferred over floating-point authority-bearing values.

A future knowledge-use receipt may bind the exact subject/revision, actor, permitted action, purpose, policy version, provenance, resulting derivative, and triggered obligations.

## Restricted traditional knowledge: immediate safety rule

Current `water-wisdom::TraditionalPractice` includes `CommunityOnly`, `ElderApproved`, and `Sacred` access labels while its payload is represented as an ordinary public Holochain application entry and generic query surfaces may expose those records.

Until a qualified protected-content architecture exists, Mycelix should fail closed for newly submitted restricted plaintext.

The immediate repair target is:

```text
Public
-> existing public entry path may remain available

CommunityOnly | ElderApproved | Sacred
-> plaintext publication on the current public entry path must be rejected
```

The integrity zome, not only an official coordinator, must enforce this boundary so a modified coordinator cannot bypass it.

Official query surfaces should also avoid returning legacy restricted-labeled records by default. This is defense in depth only: it does not make already-published DHT bytes confidential.

## Historical disclosure cannot be undone by relabeling

Changing metadata or removing an index cannot retroactively create secrecy for content that was already published to a replicated public substrate.

Therefore:

```text
publicly published plaintext
+ later restricted label
!= retroactive confidentiality
```

Migration/remediation work may reduce future exposure through official application surfaces, but any stronger erasure or confidentiality claim requires separate evidence and must not be implied by STEW.

## Protected-content architecture direction

The later confidentiality tranche should receive an independent threat model and qualification program. A likely direction to investigate is separating public commitments/descriptors from protected plaintext and decryption authority:

```text
publicly replicable layer
    safe descriptor
    content commitment
    policy commitment
    optional ciphertext

protected authority layer
    decryption authority
    membership/delegation evidence
    key envelopes
    revocation/currentness
```

This document does not freeze a cryptographic construction.

## AI / Symthaea boundary

Symthaea should consume typed knowledge capabilities rather than assume that retrieved information is freely disclosable, trainable, or redistributable.

A later capability vocabulary should distinguish at least:

```text
discover
retrieve
reason
disclose
derive
train
redistribute
```

Core theorem:

```text
retrieve != disclose
reason != disclose
reason != train
train != derive
derive != redistribute
```

Model confidence, intelligence, or inference quality cannot widen the underlying authority.

## First proving scenarios

The first proving corpus should use synthetic fixtures rather than real restricted cultural material.

### Scenario A — public historical work

```text
public artifact
-> discover
-> retrieve
-> permitted derivative
-> exact provenance retained
-> attribution/reciprocity receipt
```

### Scenario B — synthetic restricted oral tradition

```text
safe descriptor discoverable
-> protected payload absent from public plaintext path
-> authorized steward may access through later qualified path
-> unauthorized actor cannot retrieve plaintext
-> AI may reason only within granted capability
-> disclosure denied when disclosure authority is absent
```

### Scenario C — contested artifact

```text
claim A
+ claim B
-> both retained
-> evidence retained
-> no automatic winner
```

### Scenario D — migrated archival artifact

```text
original digest
+ migrated representation
+ transformation evidence
-> both lineages inspectable
```

## Initial tranche map

### Foundation

- `STEW-000` — this constitution.
- `STEW-000A` — fail closed on restricted `water-wisdom` plaintext and harden official queries.
- `STEW-001` — canonical stewarded-subject identity theorem.
- `STEW-002` — stewardship envelope core.
- `STEW-003` — knowledge-use action/policy theorem.
- `STEW-004` — collective stewardship claims.
- `STEW-005` — contested/joint stewardship lifecycle.

### Provenance and preservation

- `STEW-006` — typed provenance graph theorem.
- `STEW-007` — preservation manifest theorem.

### Reciprocity

- `STEW-010` — generic reciprocity core beneath software-specific Attribution.
- `STEW-011` — reciprocal knowledge-use receipts and obligation evidence.

### Confidentiality

- `STEW-020` — protected-content architecture preregistration and threat model.
- `STEW-021` — `water-wisdom` adapter onto qualified stewardship/access policy.

### Creative and living heritage

- `STEW-040` — creative-work and derivative lineage.
- `STEW-041` — living transmission event model for teaching/oral/cultural continuity.

### Symthaea companion stack

- `KNOW-AUTH-000` — knowledge authority constitution.
- `KNOW-AUTH-001` — typed knowledge capability theorem.

## Mechanical properties to qualify later

```text
restricted policy
=> no protected plaintext accepted by public-entry admission

EpistemicScore
!= StewardshipAuthority

StewardshipAuthority
!= AccessAuthority

Access(view)
!= Access(train_ai)

PreservationAuthority
!= DisclosureAuthority

successful derivative
=> exact predecessor provenance retained

revocation
=> future authority removed
!= historical provenance erased

contested stewardship
=> no automatic winner

unknown protected-content action
=> deny

protocol monetary reciprocity
=> no floating-point authority amount
```

## Deliberate non-claims

This document does not establish copyright ownership, legal title, cultural legitimacy, Indigenous/community consent, protected-content confidentiality, cryptographic key management, deletion from previously replicated public data, archival durability, authenticity of any cultural claim, reciprocity fairness, payment settlement, AI training legality, governance legitimacy, or any permission to use real restricted knowledge.

It creates no new runtime authority and changes no existing access rights.

It only freezes the intended separation of authority domains and the immediate fail-closed repair direction so later implementation cannot silently collapse preservation, truth, access, ownership, cultural legitimacy, reciprocity, and AI use into one score or one database.
