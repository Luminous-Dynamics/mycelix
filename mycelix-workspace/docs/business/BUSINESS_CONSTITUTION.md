# Business Constitution v0.1

Status: **normative candidate / documentation only**

These invariants constrain every future Mycelix Business orchestrator, workflow, adapter, cockpit, automation, and AI integration.

The Business layer is non-authoritative by default. Any positive authority must originate from, and remain attributable to, the domain that owns that authority.

## A. Authority invariants

### AUTH-001 — No financial authority manufacture

Business orchestration MUST NOT create, widen, refresh, or impersonate Finance/payment/treasury authority.

### AUTH-002 — No identity authority manufacture

Business orchestration MUST NOT create or strengthen identity, credential, authentication, or legal-entity authority.

### AUTH-003 — No governance authority manufacture

Business orchestration MUST NOT create, strengthen, or bypass governance authority, policy, delegation, quorum, or constitutional state.

### AUTH-004 — No inventory/fulfillment authority manufacture

Business orchestration MUST NOT create authoritative stock, custody, provenance, fulfillment, or delivery state.

### AUTH-005 — Capability does not imply institutional power

Possession of an invocation capability MUST NOT be treated as sufficient evidence that the principal has current institutional power to cause the requested effect.

### AUTH-006 — Positive authority is domain-derived

Every consequential positive authorization MUST retain a causal reference to the domain-owned authority/evidence that justified it.

### AUTH-007 — Delegation only attenuates

A delegated/child authority MAY become narrower. It MUST NOT silently increase scope, purpose, duration, consequence, resource limits, or delegation depth.

### AUTH-008 — Revocation is prospective authority change

A later revocation changes future authority and MAY trigger reconciliation. It MUST NOT silently rewrite what authority evidence existed at an earlier action time.

### AUTH-009 — Grant existence is not current authorization

The existence of a `PowerGrant`, role, delegation record, approval template, or capability MUST NOT be treated as a current per-operation authorization decision. Consequential execution requires the authority-owning subsystem to evaluate the exact operation, scope, purpose, policy, revocation/supersession state, validity, and other required constraints.

## B. Truth and evidence invariants

### EPI-001 — Observation is not accepted truth

An external/provider observation MUST NOT become authoritative internal state until the owning domain validates or reconciles it under explicit policy.

### EPI-002 — Evidence is not authority

Evidence, proofs, receipts, attestations, confidence, consensus, reputation, or AI inference MUST NOT become authority merely by being available.

### EPI-003 — Provider self-certification is insufficient

A provider MUST NOT be able to both propose a consequential fact and unilaterally qualify that same fact as accepted Mycelix truth unless the owning domain explicitly defines that provider as the authority source and preserves that policy/evidence boundary.

### EPI-004 — Contradiction remains visible

Conflicting observations/claims MUST remain distinguishable until an authorized reconciliation/resolution process establishes the institution's accepted conclusion.

### EPI-005 — Historical evidence is append-preserving

Reconciliation, correction, compensation, or resolution MUST NOT erase the evidence and claims that preceded it.

### EPI-006 — Projection cannot strengthen truth

Presentation MAY compress or summarize state. It MUST NOT represent a stronger conclusion than the currently justified underlying authority/evidence/reconciliation state.

### EPI-007 — Unknown remains unknown

Unknown, ambiguous, unavailable, or unresolved outcomes MUST NOT collapse into success, failure, absence, non-application, or completion without explicit evidence.

### EPI-008 — Duplicate delivery is not new evidence

Repeated transport, webhook delivery, queue delivery, replay, or observation of the same exact source-event identity MUST NOT multiply value, quantity, quorum, confidence, obligation satisfaction, or institutional truth merely because the event was received more than once.

### EPI-009 — Resolution changes disposition, not historical reality

An institutional resolution MAY establish the institution's accepted disposition for a dispute or contested subject. It MUST NOT rewrite earlier claims, observations, actions, or evidence as though they never existed.

## C. Temporal and semantic-version invariants

### TMP-001 — Authority is time-scoped

Consequential authorization MUST be evaluated against the authority/policy state applicable to the action's relevant time boundary.

### TMP-002 — Downstream validity cannot outlive prerequisites

A derived authority, closure, or accepted-truth lease MUST NOT outlive any required upstream authority/evidence/freshness prerequisite.

### TMP-003 — Unknown required freshness fails closed

If currentness/freshness is required for a consequential positive conclusion and the required boundary is unknown, positive qualification MUST fail closed.

### TMP-004 — Supersession is explicit

Changed policy, agreement, authority, reconciliation, or projection semantics MUST retain explicit supersession/version lineage where the distinction affects institutional meaning.

### TMP-005 — Causal order is preserved

Qualification/reconciliation timestamps MUST NOT be sampled or interpreted in a way that permits later-produced evidence to appear causally available before it existed.

### TMP-006 — Representation migration cannot rewrite semantics

Schema or representation migration MUST preserve the historical institutional meaning of the source record. It MUST NOT silently reinterpret `Waived` as `Satisfied`, `OutcomeUnknown` as failure/non-application, stale authority as current authority, or an old semantic profile as though it were created under a newer policy.

### TMP-007 — Material semantic change requires explicit lineage

When a semantic profile changes institutional meaning, the new interpretation requires explicit supersession, adaptation, or requalification. Code upgrades alone MUST NOT retroactively change what a historical record meant.

## D. Workflow and causal-identity invariants

### WF-001 — Workflow coordinates; domains decide

A workflow MAY request domain operations and evaluate their results. It MUST NOT impersonate the authority-owning domain.

### WF-002 — Completion is derived closure

A workflow MUST NOT become `Closed`/`Completed` merely because an internal sequence reached its final step. Closure MUST be justified by explicit closure predicates over domain-owned outcomes and evidence.

### WF-003 — Partial success remains visible

A workflow with mixed outcomes MUST preserve partial state. It MUST NOT collapse partial performance into complete success or generic failure.

### WF-004 — External effects are not rolled back by fiction

Once an external effect may have occurred, compensation MUST be modeled as a new causal action/commitment rather than erasing the original effect.

### WF-005 — Ambiguous effect is not retry permission

An outcome whose application is unknown MUST NOT be blindly retried as though non-application were proven.

### WF-006 — Re-execution is idempotent or causally guarded

Repeated delivery, replay, restart, or retry MUST NOT duplicate a consequential institutional effect unless the domain explicitly permits repeated application.

### WF-007 — Workflow closure is auditable

A consequential closed workflow SHOULD produce or retain a closure receipt referencing the exact authority, domain outcomes, reconciliations, and evidence that satisfy its closure policy.

### WF-008 — Failure does not erase obligations

Workflow failure MUST NOT silently delete commitments or obligations that were already created. Remaining duties must be satisfied, waived, superseded, disputed, breached, or compensated explicitly.

### WF-009 — Logical intent, execution attempt, and source event are non-substitutable

The identity of the requested institutional effect, the identity of a concrete execution attempt, and the identity of an externally/domain-generated source event MUST remain distinguishable. A transport retry or new attempt identifier MUST NOT silently mint a new logical effect.

### WF-010 — Material semantic change changes logical operation identity

Changing a material operation field such as subject, beneficiary, amount, quantity, destination, purpose, or policy-relevant parameter MUST NOT retain the same logical-operation identity unless the owning domain's exact operation profile explicitly defines that field as non-semantic.

### WF-011 — Compensation has independent causal identity

A compensating action MUST have its own logical intent/effect identity and causal reference to the outcome it addresses. It MUST NOT reuse the original effect identity as though compensation were rollback.

### WF-012 — Closure reason remains explicit

A terminal workflow state MUST retain why it closed. Satisfaction, waiver, compensation, termination, and policy-approved closure with exceptions MUST NOT collapse into one semantically indistinguishable `Completed` outcome.

### WF-013 — Obligation discharge reason remains explicit

`Satisfied`, `Waived`, `Breached`, `Superseded`, `Terminated`, and `Disputed` MUST remain distinguishable where applicable. Waiver or termination MUST NOT project as proof that performance occurred.

### WF-014 — Closure with exceptions does not satisfy the exceptions

A closure policy MAY explicitly permit a workflow or accounting period to close with identified unresolved/deferred/write-off/exception items. Such closure MUST retain the exact exception set and MUST NOT strengthen those items into satisfied, failed, or proven-not-applied facts.

## E. Domain-boundary invariants

### DOM-001 — One authoritative owner or explicit reconciliation rule

Every consequential field exposed by Business MUST have either:

1. one identified authoritative domain owner; or
2. an explicit reconciliation/resolution rule when no single source can directly own the fact.

### DOM-002 — Reference instead of duplication

Business structures SHOULD reference domain-owned records rather than copy authoritative credentials, balances, inventory counts, governance state, or similar truth into a new Business-owned source of truth.

### DOM-003 — Bridges do not strengthen semantics

Cross-domain transport/bridge success MUST NOT strengthen the semantic authority of the payload it carries.

### DOM-004 — Domain absence is not negative truth

Failure to retrieve a domain record MUST NOT automatically mean the record does not exist or the corresponding condition is false.

### DOM-005 — Semantic equality belongs to the owning domain/profile

A generic Business orchestrator MUST NOT decide that two domain operations, records, or outcomes are semantically equivalent merely because their transport bytes or incidental identifiers resemble one another. Equality/deduplication for consequential meaning must follow the owning domain's registered semantic profile.

## F. Privacy invariants

### PRIV-001 — Composability does not imply universal observability

A coherent organizational graph MUST NOT require every domain or Business projection to see every linked sensitive record.

### PRIV-002 — Least disclosure

Cross-domain queries/actions SHOULD disclose only the facts required for the declared purpose and policy.

### PRIV-003 — Purpose-limited traversal

Possession of one `PartyRef`, `BusinessRef`, or workflow reference MUST NOT imply unrestricted traversal into Identity, Finance, HR, Commerce, or other sensitive domains.

### PRIV-004 — Business read models are not universal join tables

Caches/read models MAY improve UX but MUST preserve source attribution, authorization boundaries, and field-level disclosure semantics.

### PRIV-005 — Reference possession is not dereference authority

A component that may carry, compare, or store a reference MUST NOT automatically gain authority to retrieve the protected record to which that reference resolves.

### PRIV-006 — Disclosure policy applies before sensitive projection

A Business projection MUST evaluate viewer/purpose disclosure policy before exposing cross-domain joined sensitive state. Building a universally readable unredacted join and merely hiding fields at the UI layer is insufficient.

### PRIV-007 — Correlation is purpose-scoped

Where universal identity correlation is not required, implementations SHOULD prefer contextual/pairwise references or protected binding mechanisms. The ability to correlate a party across domains MUST itself follow explicit purpose and authority constraints.

## G. AI and automation invariants

### AI-001 — AI synthesizes intent, not authority

AI MAY propose actions, plans, explanations, classifications, or anomalies. AI output MUST NOT itself mint institutional authority.

### AI-002 — AI inference is not domain truth

An AI inference MUST remain distinguishable from verified domain state and external observation.

### AI-003 — Consequential execution is typed

Business automation MUST execute consequential effects only through explicit typed capabilities/domain operations with independently evaluated authority.

### AI-004 — Human review policy is external to the model

Whether human approval is required MUST be determined by institutional policy/authority rules, not by the AI deciding that its own confidence is sufficient.

## H. Exit and sovereignty invariants

### EXIT-001 — Every material business record has an exit path

Mycelix Business MUST provide documented export semantics for material business records and evidence.

### EXIT-002 — Export preserves provenance

Export SHOULD retain stable identifiers, semantic profiles/versions, timestamps, source-domain attribution, causal references, closure/discharge reasons, and evidence references needed to understand the exported institutional history.

### EXIT-003 — Exit is not authority loss by design

Leaving Mycelix SHOULD NOT require surrendering access to the organization's own records in order to preserve vendor lock-in.

## I. Global theorem candidates

Future executable qualification SHOULD attempt to establish the following system properties.

### THEOREM-BUS-1 — No projection strengthening

> No Mycelix organizational projection may claim an institutional outcome stronger than currently valid authority, domain-owned state, accepted observations, reconciliation policy, and retained evidence justify.

### THEOREM-BUS-2 — No orchestration authority promotion

> No workflow, Business orchestrator, automation, AI planner, frontend, bridge, or external provider may independently promote itself into an authority-owning domain.

### THEOREM-BUS-3 — No lease widening

> A downstream Business composer may preserve or shorten a required authority/evidence validity horizon; it must never lengthen it.

### THEOREM-BUS-4 — History survives compensation

> A compensating action may address an earlier institutional outcome but must not erase the original action, observation, obligation, or evidence from institutional history.

### THEOREM-BUS-5 — No replay amplification

> Re-delivery, retry, restart, or duplicated observation of the same exact logical/source event may not create additional institutional value, quantity, authority, satisfaction, quorum, or effect unless the owning domain explicitly establishes a distinct permitted operation.

### THEOREM-BUS-6 — Closure preserves disposition

> Every terminal consequential workflow must preserve whether it ended through satisfaction, waiver, compensation, termination, or explicitly policy-qualified exceptions; terminality alone cannot strengthen the underlying obligations or evidence.

### THEOREM-BUS-7 — Historical semantics are stable

> A representation or software upgrade cannot retroactively strengthen or reinterpret the institutional meaning of a historical record without explicit semantic lineage and any required requalification.

## J. Claim boundary

This constitution is not executable enforcement yet.

A future implementation MUST NOT claim these invariants are enforced merely because this document exists. Each invariant promoted to runtime status requires an identified enforcement boundary and qualification evidence.
