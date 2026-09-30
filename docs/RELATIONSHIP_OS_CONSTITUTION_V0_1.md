# Relationship OS Constitution v0.1

Status: architectural proposal
Tracking: #3650
Scope: vertical-neutral relationship substrate

## 1. Thesis

Mycelix should not reproduce a centralized CRM object model.

The foundational object is a **relationship**: a time-bounded, provenance-bearing association among identified participants, shared context, assertions, evidence, commitments, consent, and delegated authority.

Symthaea is a cognitive consumer/producer of bounded relationship context. It does not become the authority for identity, institutional decisions, commercial settlement, civic legitimacy, or external-world truth.

## 2. Core model

Conceptually:

    Relationship
      ├── identity
      ├── participants
      ├── context
      ├── assertions
      ├── evidence
      ├── commitments
      ├── consent/disclosure
      ├── authority/delegation
      ├── events
      └── temporal lineage

A relationship is not a universal person profile and does not imply that every participant can see every relationship field.

### 2.1 Stable identity

A logical relationship identity MUST be distinct from:

- a Holochain action hash;
- a particular revision;
- a UI record;
- an external CRM identifier;
- an external organization's account number.

The first accepted creation event establishes the relationship lineage. Later revisions MUST preserve that logical identity.

### 2.2 Participants

Participants are references to owning identity/organization domains.

The relationship layer MUST NOT duplicate authoritative identity attributes merely for convenience.

Possible participant roles include:

- person;
- organization;
- team;
- agent;
- service/provider;
- external-system principal.

Role vocabulary is extensible and domain-owned.

### 2.3 Assertions

An assertion is an attributed statement about relationship state.

Minimum conceptual fields:

- assertion identity;
- relationship reference;
- author/source identity;
- assertion class;
- epistemic status;
- observation/assertion time;
- evidence references;
- revision/currentness metadata.

The relationship layer MUST NOT collapse assertion into universal truth.

## 3. Epistemic boundary

At minimum, implementations SHOULD distinguish:

- observed;
- reported/declared;
- corroborated;
- challenged;
- inferred;
- unknown/stale.

These labels describe provenance/evaluation state. They are not substitutes for domain adjudication.

The system MUST preserve:

    observation != assertion != inference != authority

A Symthaea inference MUST remain an inference unless an owning domain produces a qualified authoritative result.

## 4. Evidence

Consequential relationship projections SHOULD be reconstructible from exact evidence references.

Evidence references MUST remain distinct from the claim they support.

The system MUST preserve:

- source identity;
- exact source/reference identity;
- observation or publication time;
- evaluation context where relevant;
- currentness/revision frontier where relevant.

Evidence reuse does not automatically authorize obligation identity reuse, authority reuse, or disclosure reuse.

## 5. Commitments

Commitments represent expected future actions or states between participants.

Examples:

- request;
- promise;
- deliverable;
- milestone;
- appointment;
- acceptance condition;
- follow-up.

A commitment being recorded does not establish:

- payment;
- settlement;
- legal discharge;
- external completion;
- beneficiary satisfaction;
- institutional authority.

Those semantics remain owned by the appropriate domain.

## 6. Consent and disclosure

Relationship data MUST be partitionable by disclosure policy.

A future disclosure policy should be capable of binding:

- purpose;
- requesting principal;
- intended audience;
- data class;
- minimum necessary fields;
- expiry;
- revocation;
- authorization reference.

Public DHT visibility MUST NOT be treated as equivalent to unrestricted human disclosure.

## 7. Authority and agents

Agent cognition and institutional authority are separate layers.

The following equivalences are prohibited:

    agent proposal       != authorization
    model confidence     != authority
    successful execution != external effect
    local approval       != institutional authority
    agent identity       != represented organization

Delegated authority SHOULD bind an explicit authority epoch and, for consequential effects, an execution-time revalidation/fencing boundary.

## 8. Relationship events

Events SHOULD be append-oriented and attributable.

A relationship projection should be derived from event lineage rather than mutable UI state.

Where currentness matters, the projection MUST identify:

- the exact source frontier;
- the projection revision;
- whether the result is current, stale, superseded, or unknown.

A cached value is not a currentness proof.

## 9. Domain ownership

The Relationship OS is a substrate, not an authority vacuum.

Indicative ownership:

| Concern | Owning layer |
| --- | --- |
| identity | Identity |
| organizational authority | Governance / authority domain |
| commercial agreement | Commerce / Business |
| financial state | Finance |
| operational support state | Support / Operations |
| civic procedure | Civic / administrative domain |
| evidence | Evidence/provenance subsystem plus source domain |
| agent cognition | Symthaea |
| external-system state | external source + integration adapter |

Relationship projections may reference these domains but MUST NOT silently become their authoritative owner.

## 10. Relationship 360 projection

A future read model may expose:

- participants;
- recent relationship events;
- open commitments;
- relevant evidence;
- unresolved questions;
- authority/consent boundaries;
- external references;
- attributed insights;
- proposed next actions.

The projection MUST preserve source references and epistemic status.

It MUST be possible to answer:

> Why does the system believe this?

with an inspectable chain back to the relevant source/evidence.

## 11. Symthaea boundary

Symthaea may:

- summarize relationship context;
- detect patterns;
- identify missing evidence;
- surface contradictions;
- propose follow-ups;
- rank retrieval relevance;
- generate candidate actions within a bounded policy.

Symthaea MUST NOT silently:

- manufacture evidence;
- promote inference to fact;
- grant itself authority;
- disclose protected relationship data;
- treat operational success as external-world truth.

A cognitive answer SHOULD expose uncertainty and provenance for consequential claims.

## 12. CRM as a projection

Sales, support, account management, opportunity management, and similar CRM functionality should be implemented as domain/application projections over the substrate.

For example:

    Account View
      -> organization identity reference
      -> relationship references
      -> commercial commitments
      -> support references
      -> evidence
      -> external-system references

This prevents a "CRM account" from becoming a second identity system.

## 13. Federation

External CRM/ERP/service systems are sources or participants in the relationship graph.

An external ID MUST remain an external reference unless an explicit adapter establishes a stronger mapping.

Examples:

    local RelationshipId != Salesforce AccountId
    local ServiceIssueId != municipal ticket number
    local commitment != provider acknowledgement
    provider acknowledgement != verified external effect

Adapters should preserve source identity and currentness rather than overwrite them.

## 14. Privacy and minimization

The architecture SHOULD prefer:

- references over duplicated profiles;
- selective disclosure over broad replication;
- purpose-bound reads;
- least-necessary projections;
- encrypted private state where appropriate;
- explicit retention/expiry semantics.

No relationship feature should require universal aggregation of personal data.

## 15. PR ladder

### ROS-001 — relationship identity

Add the minimal stable relationship identity and participant binding primitives.

Acceptance:

- stable logical ID distinct from action/revision hash;
- immutable creation lineage;
- deterministic identity validation;
- revision references;
- adversarial sibling/substitution tests.

### ROS-002 — evidence/provenance

Add an evidence envelope and exact source binding.

Acceptance:

- exact source identity;
- author/source attribution;
- epistemic status;
- observation time;
- currentness/revision binding;
- substitution/replay tests.

### ROS-003 — commitments

Add a vertical-neutral commitment surface.

Acceptance:

- stable commitment identity;
- participant/role bindings;
- lifecycle transitions;
- temporal constraints;
- explicit separation from settlement/legal discharge.

### ROS-004 — consent/disclosure

Add policy-bound disclosure references.

Acceptance:

- purpose/audience/data-class binding;
- expiry/revocation;
- default deny for unbound disclosures;
- substitution and confused-deputy tests.

### ROS-005 — delegated agent authority

Add agent delegation references.

Acceptance:

- principal/delegate binding;
- authority scope;
- epoch/fencing;
- execution-time revalidation;
- proposal/authority separation;
- stale-delegation tests.

### ROS-006 — Relationship 360

Build deterministic projections from exact source references.

Acceptance:

- source frontier recorded;
- stale/unknown state explicit;
- no duplicated authoritative state;
- deterministic reconstruction tests.

### ROS-007 — Symthaea adapter

Expose bounded relationship context to Symthaea.

Acceptance:

- evidence references preserved;
- uncertainty preserved;
- no authority escalation;
- provenance-backed answer fixtures;
- adversarial hallucinated-evidence tests.

### ROS-008 — CRM vertical

Build account/contact/opportunity/support experiences only after the substrate is qualified.

Acceptance:

- CRM objects remain projections;
- external IDs remain references;
- domain ownership is explicit;
- end-to-end workflow qualification demonstrates value without creating a parallel authority stack.

## 16. Qualification rule

No tranche is considered complete merely because code compiles.

Each tranche requires:

1. exact-head execution;
2. positive tests;
3. negative/adversarial tests;
4. ownership review;
5. provenance review;
6. privacy/disclosure review where applicable;
7. explicit nonclaims.

## 17. Design principle

The target is not:

    "Salesforce, but decentralized."

The target is:

    "A relationship substrate where humans, organizations, agents,
     evidence, commitments, and authority can coordinate without
     confusing a record with reality."

Salesforce-like applications become one proof surface for that substrate, not the definition of the substrate.
