# Institutional Semantics v0.1

Status: **normative candidate / documentation only**

## 1. Purpose

This document defines the smallest institutional vocabulary currently required to compose Mycelix domains into a coherent organization or business without collapsing domain authority into a central Business service.

The model is deliberately narrower than an ERP ontology. It describes how institutional meaning is established, acted upon, observed, reconciled, and projected.

Lifecycle, replay/idempotency, closure-class, semantic-version, and reference-privacy rules are specified more precisely in [`STATE_MACHINES_V0.1.md`](./STATE_MACHINES_V0.1.md).

## 2. Four planes

Mycelix Business semantics are divided into four planes that MUST NOT be silently collapsed.

### 2.1 Normative plane

Answers: **What ought, may, must, or must not happen?**

Contains:

- commitments and obligations;
- rights derived from normative sources;
- policies and constraints;
- institutional powers;
- agreements/resolutions/law/policy that establish or modify the above.

Normative state does not prove that performance occurred.

### 2.2 Intent and authority plane

Answers: **What is being proposed, and does the actor have legitimate authority to attempt it?**

Contains:

- intents;
- invocation capabilities;
- institutional-power evaluation;
- approvals;
- delegations;
- revocations;
- scope, purpose, and operation-specific constraints;
- domain-owned authorization decisions.

Permission to invoke an operation and institutional power to create a recognized effect are distinct concepts.

### 2.3 Operational and epistemic plane

Answers: **What did Mycelix execute, and what do we have evidence to believe happened?**

Contains:

- domain commands;
- execution/action records;
- domain-owned state transitions;
- observations/claims/attestations;
- evidence;
- reconciliation;
- conflict and dispute records.

An observation is a claim by a source, not automatically accepted institutional truth.

### 2.4 Projection plane

Answers: **What conclusion or task-oriented view may be presented to a human or application?**

Examples:

- `Invoice.status = Paid`;
- "3 orders need attention";
- profit and loss;
- customer relationship;
- compliance report;
- AI explanation.

A projection MAY compress or summarize underlying state. It MUST NOT strengthen it.

## 3. Candidate transport-neutral vocabulary

These concepts are candidates for the future transport-neutral Business kernel. A semantic primitive does not necessarily require one stored Rust object; implementations should prefer the smallest representation that preserves the invariant.

### 3.1 `PartyRef`

A stable/contextual reference to a participant recognized by an owning identity/organization domain.

`PartyRef` does not contain authoritative identity credentials, and possession of a reference does not imply authority to dereference protected identity state.

### 3.2 `OrganizationContextRef`

A reference to the institutional context within which commitments, powers, policies, actions, and evidence acquire meaning.

An organization context does not own every record associated with the organization. It binds the context in which independently owned domain records interoperate.

### 3.3 `SubjectRef`

A typed reference to the subject of a commitment, obligation, power, action, observation, reconciliation, or assertion.

### 3.4 `Commitment`

A normative undertaking/specification concerning **performance or maintenance of a condition** by an identified party under defined terms.

`Commitment` is intentionally narrower than earlier drafts. It does **not** mean permission, institutional power, or generic constraint; those belong to `PowerGrant` and `Constraint`.

A commitment SHOULD identify the applicable subset of:

- party expected to perform/maintain the condition;
- beneficiary/institutional context;
- subject;
- activation conditions;
- performance conditions;
- due conditions;
- satisfaction rule;
- evidence requirements;
- validity interval;
- source agreement/policy references.

A commitment may later instantiate one or more active obligations.

### 3.5 `Obligation`

An active institutional duty requiring an explicitly defined disposition.

An obligation MAY arise from:

- an activated commitment;
- an agreement;
- policy or law;
- an authorized resolution;
- another domain-owned normative source.

This is deliberate: tax liabilities, remedial duties, or statutory obligations need not pretend they were voluntarily "committed" first.

An obligation SHOULD retain its exact normative source and satisfaction/discharge policy.

Obligation state is not satisfaction evidence. `Satisfied`, `Waived`, `Breached`, `Disputed`, `Superseded`, and `Terminated` remain semantically distinct as defined by the state-machine profile.

### 3.6 `PowerGrant`

A normative grant that permits a holder, under specific constraints, to cause a recognized institutional state transition.

A power grant SHOULD bind:

- principal;
- holder;
- institutional effect class;
- resource or subject scope;
- purpose;
- constraints;
- limits;
- delegation semantics;
- revocation/supersession lineage;
- validity interval;
- evidence/authority basis.

A `PowerGrant` is not itself a current per-operation authorization decision. The owning authority subsystem must evaluate it against the exact operation and current policy/state.

### 3.7 `Constraint`

A condition that narrows the admissible use of a commitment, obligation, power, intent, workflow, action, or disclosure.

A child/delegated constraint MAY become stricter. It MUST NOT silently widen inherited authority.

### 3.8 `Intent`

A typed proposal to attempt an operation or establish a domain-owned institutional effect.

Intent is not authority and does not prove execution.

Consequential intent must have semantic identity distinct from transport delivery and execution-attempt identity; see `STATE_MACHINES_V0.1.md`.

### 3.9 `ActionRecord`

A source/domain-attributed record of an operation that Mycelix attempted or executed through an owning domain.

An action record is operational evidence; it does not by itself prove an external effect.

### 3.10 `Observation`

A typed source-attributed statement that some external or independently owned condition was observed.

Examples:

- PSP reports settlement;
- bank reports deposit;
- carrier reports delivery;
- registry reports legal-entity status;
- worker/time system reports work performed.

An observation SHOULD retain source identity/reference, source-event identity where available, subject, observation time, evidence reference, semantic profile, and any validity/freshness boundary required downstream.

Repeated delivery of the same source event is not a new fact.

### 3.11 `EvidenceRef`

A reference to evidence supporting an assertion, action, observation, reconciliation, resolution, authorization, or closure qualification.

An evidence reference is not itself authority, and possession of it does not imply unrestricted access to its protected contents.

### 3.12 `Reconciliation`

A domain/policy-governed relation that establishes how one or more accepted observations/domain outcomes correspond to an exact institutional/economic subject.

Example:

`bank deposit -> reconciled to payment obligation -> accepted settlement`

Reconciliation does not erase the original observations and MUST retain the relevant source set, subject, policy/profile, authority/evidence basis, and unresolved variance.

### 3.13 `Resolution`

A decision, under an explicitly identified policy/authority process, that establishes an institutional disposition for competing claims or a dispute.

Resolution changes what the institution recognizes or requires going forward. It MUST NOT rewrite historical claims, actions, observations, or evidence as though they never existed.

### 3.14 `Projection`

A derived human/application view over underlying institutional state.

Projection is always downstream of the truth/authority/evidence it summarizes and must obey viewer/purpose disclosure policy.

## 4. Derived concepts, not Business-kernel authority primitives

The following SHOULD initially be represented as compositions, domain-owned semantic records, typed views, or projections rather than foundational Business-kernel truth:

- Agreement;
- Right;
- Claim/CounterClaim;
- AuthorizationDecision;
- EconomicEvent;
- Customer;
- Supplier;
- Employee;
- Contractor;
- Invoice;
- Order;
- Payment;
- Project;
- Subscription;
- Warranty;
- Purchase order;
- Refund;
- domain-specific proposals;
- Business dashboard status.

### 4.1 Agreement

`Agreement` is a structured establishment or modification of commitments, obligations, powers, rights, or constraints supported by acceptance/authority evidence.

It does not need a second generic authority system inside Business.

### 4.2 Right

`Right` is a normative view of what a beneficiary/holder may claim, require, receive, or exercise because of underlying obligation, commitment, power, policy, agreement, resolution, or law.

The projection of a right does not manufacture its underlying authority.

### 4.3 Claim

A claim/counterclaim is a source-attributed assertion represented through the existing observation/assertion/evidence vocabulary. A dispute does not require a second truth store merely to label an assertion as contested.

### 4.4 Authorization decision

A positive authorization decision is domain-owned output from evaluating current power/capability/policy/constraints against an exact operation. Business may reference that decision but does not mint it.

### 4.5 Economic event

`EconomicEvent` is a typed accounting-facing projection over already accepted domain-owned operational/reconciliation facts.

It communicates economic meaning to Accounting; it does not establish that the underlying sale, delivery, wage, payment, tax, or asset event happened.

### 4.6 Relationship labels

`CustomerRelationship`, `SupplierRelationship`, `EmployeeRelationship`, and similar labels are contextual projections over a `PartyRef` plus agreements, obligations, powers, and domain activity.

A single party can participate in several relationships without being duplicated into several identity silos.

### 4.7 Documents

`Invoice`, `purchase order`, `receipt`, and similar documents are structured projections/records over domain-owned semantics. Rendering or signing a document does not allow the document layer to redefine the underlying obligation/authority state.

## 5. General agreement/performance pattern

A useful cross-domain pattern is:

`Proposal/Intent -> Agreement/Normative Source -> Commitments/Obligations -> Authorized Action -> Performance/Observation -> Evidence -> Reconciliation -> Closure/Resolution`

Retail, services, employment, subscriptions, procurement, regulation, and long-running projects differ primarily in timing, policy, authority, and the obligations they instantiate.

A domain-specific proposal is normally a typed `Intent` or domain proposal record. It does not require a new Business-kernel primitive merely because a product-sale, procurement, or workforce UI gives it a different name.

## 6. Capability, power, and authorization are non-substitutable

A capability answers:

> May this principal invoke this operation surface?

A `PowerGrant` answers:

> Under what institutional authority may this holder cause this class of recognized transition?

A current authorization decision answers:

> Does this exact operation satisfy the currently applicable capability, power, policy, scope, purpose, validity, revocation, approval/quorum, and action-specific conditions now required by the owning domain?

A consequential operation MAY require all three.

Therefore:

`capability != PowerGrant != current authorization decision`

## 7. Claim and observation discipline

Externally observed or cross-domain facts SHOULD remain claims/observations until the owning domain accepts or reconciles them.

Forbidden shortcut:

`provider callback -> business truth`

Required shape:

`source event -> observation -> owning-domain validation -> reconciliation/accepted domain state -> projection`

Examples:

- `ProviderObservedSettlement` is not `ReconciledSettlement`.
- `CarrierObservedDelivery` is not necessarily `FulfillmentSatisfied`.
- `RegistryObservedEntityStatus` is not locally minted legal authority.
- duplicated webhook delivery is not a second settlement.

## 8. Temporal and semantic-profile semantics

Institutional truth is time-scoped and profile-scoped.

Authority-bearing, evidence-bearing, and materially stateful records SHOULD distinguish the relevant subset of:

- `occurred_at`;
- `effective_from`;
- `effective_until`;
- `observed_at`;
- `accepted_at`;
- `verified_at`;
- `revoked_at`;
- `superseded_at`;
- `reconciled_at`;
- semantic profile/version.

A downstream claim MUST NOT outlive any upstream prerequisite required for that claim.

When a required freshness boundary is unknown, consequential positive qualification SHOULD fail closed rather than inventing a default lifetime.

Representation/schema migration MUST preserve historical institutional meaning. A changed semantic meaning requires explicit adaptation/supersession/requalification rather than silent reinterpretation.

## 9. Workflow semantics

A Business workflow is not an authority-owning distributed transaction coordinator.

It is a causal orchestration record whose closure is derived from explicit predicates over domain-owned outcomes, obligations, policy, reconciliation state, authority decisions, and retained evidence.

A workflow MAY:

- request a typed domain action;
- retain logical-intent/attempt/source-event causality;
- observe domain outcomes;
- evaluate closure candidates;
- request compensating actions;
- expose partial progress;
- retain disputes and exceptions.

A workflow MUST NOT:

- mint financial authority;
- mint identity authority;
- mint governance authority;
- manufacture inventory, work, fulfillment, or legal-registry truth;
- convert an unverified external observation directly into accepted truth;
- decide semantic equality for domain operations without the owning domain/profile;
- mark itself complete merely because its internal step counter reached the final node.

## 10. Closure receipts

Consequential workflows SHOULD eventually produce a `WorkflowClosureReceipt` whose meaning is:

> These exact domain-owned outcomes, obligation dispositions, accepted reconciliations, authority references, evidence, exception set, and closure policy justify this exact closure class.

A closure receipt SHOULD retain enough causal/profile information to recompute or audit that conclusion.

Terminality and satisfaction are not synonyms. The exact closure reason/class remains visible as specified by `STATE_MACHINES_V0.1.md`.

## 11. Compensation is not rollback

Once an effect reaches or may have reached the external world, Mycelix MUST NOT model compensating behavior as if history were erased.

Examples:

- settled payment followed by refund;
- shipped goods followed by return;
- filed record followed by correction;
- breached delivery followed by replacement/credit.

The original action and evidence remain part of history. Compensation creates new obligations/actions/evidence with independent causal identity linked to the earlier outcome.

## 12. Conflict and dispute

Disagreement is expected institutional state, not database corruption by default.

Mycelix SHOULD be able to retain:

- source-attributed claim;
- counterclaim;
- evidence sets;
- contested subject/obligation/action;
- applicable resolution policy;
- authority used to resolve;
- resolution;
- remedial obligations;
- appeal/supersession lineage where relevant.

A dispute MUST NOT destroy the original claims or observations.

## 13. Privacy and semantic graph traversal

Semantic composability MUST NOT imply universal observability or universal correlation.

Cross-domain references SHOULD support:

- least disclosure;
- purpose-limited traversal;
- capability/authority-gated dereference;
- contextual/pairwise identifiers where appropriate;
- selective revelation of evidence;
- separation of identity, finance, workforce, commerce, health, and other sensitive contexts.

A Business read model MUST NOT become a universally readable join table over every domain's sensitive state.

Possession of a reference is distinct from authority to dereference it.

## 14. AI boundary

AI systems, including Symthaea, MAY produce:

- institutional queries;
- proposed intents;
- proposed workflows;
- explanations;
- anomaly observations;
- risk assessments.

AI MUST NOT be treated as a source of authority or truth merely because it generated a plausible plan.

Core rule:

> AI may synthesize intent. AI does not synthesize authority or truth.

## 15. Institutional assertion envelope

A future shared assertion envelope MAY normalize provenance fields such as:

- subject;
- predicate/type;
- value/object;
- issuer/source;
- source-event identity;
- semantic profile/version;
- authority basis where applicable;
- validity interval;
- evidence references;
- epistemic status.

This is a semantic envelope, not a mandate to convert Mycelix into an RDF/triple-store architecture. Domain APIs SHOULD remain strongly typed.

## 16. Qualification target

Before runtime implementation, the ontology MUST express all six v0.1 golden paths without adding a new foundational business primitive:

1. product sale;
2. service sale;
3. procurement;
4. employee/contractor compensation;
5. refund/dispute;
6. month-end close.

It must also preserve the state-machine non-substitutabilities frozen in `STATE_MACHINES_V0.1.md`.

Failure to do so means this semantic corpus is still provisional.
