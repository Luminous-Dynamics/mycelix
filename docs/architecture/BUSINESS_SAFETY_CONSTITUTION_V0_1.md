# Mycelix Business Safety Constitution v0.1

Status: architecture contract. This document defines safety and ownership invariants for future business coordination and Symthaea integration. It does not establish runtime support, qualification, or autonomous authority.

## Purpose

Mycelix may coordinate commercial, financial, workforce, supply-chain, commons, governance, identity, justice, and external-system state without becoming the authoritative owner of all of those domains.

The Business Fabric is therefore a reference, projection, coordination, and qualification layer. It must not become a universal mutable business database or a source of institutional authority.

Symthaea may observe, estimate, forecast, simulate, explain, recommend, and plan. Those capabilities do not grant institutional permission to act.

## Domain ownership

The authoritative domain remains the source of truth for its own state. In particular:

- Commerce owns commercial exchange, agreements, commitments, claims, and commercial settlement semantics.
- Finance owns ledgers, balances, treasury, accounting treatment, and financial reservations.
- Supply Chain owns physical lots, custody, provenance, production, logistics, and physical-resource reservations.
- Praxis owns work, workforce, skill, schedule, and workforce-capacity reservations.
- Commons/Property owns shared-resource and rights/interest state.
- Governance/Xenia owns policy authority, capabilities, delegation, revocation, and institutional authorization.
- Identity/Lawful Identity owns identity and legal-entity bindings.
- Justice owns dispute and adjudication state.
- Symthaea owns no institutional authority merely by reasoning about these domains.

Business code may hold references to authoritative records. A projection must not silently become an authoritative record.

## Capability is not authority

A capability descriptor answers what an actor, system, machine, or organization can do. Authority answers what it is currently permitted to do.

Capability discovery MUST NOT grant authority.

A business profile MUST be a compositional preset, not an authority-bearing identity and not a hard-coded industry ontology.

## Execution intersection

A consequential action may execute only when every required independent gate passes:

1. epistemic gate: enough qualified evidence exists to justify attempting the action;
2. policy/safety gate: applicable hard constraints permit the action;
3. authority gate: the actor has current, scoped institutional authority;
4. coordination gate: shared reservations and aggregate constraints remain valid;
5. runtime gate: execution-time preconditions remain valid.

No gate may compensate for another. Higher model confidence cannot compensate for missing authority, policy failure, stale state, or an unavailable reservation.

## Typed epistemic boundary

The following categories are semantically distinct and MUST NOT be implicitly promoted into one another:

- Observation: evidence that a source claims something was observed.
- Estimate: a derived inference about current or past state.
- Forecast: a prediction about future state.
- Proposal: a suggested action.
- Authorized intent: an action approved by the authority domain under an exact scope.
- Execution attempt: an attempt to perform the authorized intent.
- Execution receipt: evidence about the attempt's externally observed result.
- Reconciliation: an authoritative-domain treatment of potentially conflicting observations and receipts.

In particular, inference cannot promote itself to observation, and recommendation cannot promote itself to authorization.

## Triple frontier

Consequential decisions SHOULD identify the relevant independent frontiers they relied upon:

- observation frontier: evidence/domain state used in reasoning;
- policy frontier: policies, jurisdiction profiles, hard constraints, and rule versions used;
- authority frontier: grants, delegations, revocations, and authority epochs used.

These frontiers may advance independently. A decision prepared under a prior authority or policy frontier must be revalidated according to its Action Contract before execution.

## Decision capsules

A Decision Capsule is an immutable lineage record binding, by reference where possible:

- subject and action contract identity;
- observation cut and unresolved conflicts;
- exact model/configuration lineage;
- assumptions and uncertainty;
- alternatives considered;
- predicted outcomes and affected parties;
- hard constraints evaluated;
- required authority;
- freshness and revalidation contract;
- recommendation;
- authorization evidence;
- execution identity and receipt;
- observed outcome and calibration evidence.

Human-readable explanation is advisory output. Generated prose MUST NOT become authorization evidence or alter machine-verifiable decision fields.

## Action contracts

Every autonomous or approval-bearing action class SHOULD be identified by a versioned semantic contract, for example `mycelix.procurement.place-order.v1`.

An Action Contract defines at minimum:

- required observations and evidence qualifications;
- freshness/revalidation requirements;
- required authority/capability scope;
- policy and hard-constraint requirements;
- aggregation semantics;
- reversibility/compensation semantics;
- expected execution and outcome evidence;
- privacy-minimal evidence projection;
- compatibility rules across contract versions.

The reasoning system must not choose to relax its own Action Contract.

## Prepared actions

A Prepared Action is a short-lived execution candidate, not evidence that execution has occurred.

It SHOULD bind:

- exact Action Contract digest;
- Decision Capsule reference;
- observation/policy/authority frontiers;
- authority lease reference;
- reservation references;
- intent digest;
- fencing token;
- idempotency identity;
- preparation and expiry times.

Prepared actions must fail closed when required frontiers, leases, reservations, or runtime preconditions are invalidated.

## Authority leases and delegation

Autonomous authority SHOULD be bounded, scoped, revocable, and time-limited rather than represented by broad persistent credentials.

Delegation MUST attenuate authority. A child grant may never exceed its parent in capability scope, subject scope, risk ceiling, budget, time window, or policy permissions.

Revoking an ancestor MUST invalidate descendant authority.

Sibling delegations that draw on a shared parent budget MUST consume a shared aggregate envelope; duplicating the same per-child ceiling must not amplify effective authority.

Delegation depth SHOULD be bounded by policy.

## Reservations and coordination

Observation is insufficient to coordinate scarce state under concurrency.

Authoritative domains SHOULD be able to issue short-lived reservation references for scarce capabilities, including examples such as:

- spend/budget capacity;
- stock or production capacity;
- workforce capacity;
- shared-resource capacity;
- contractual or operational capacity.

The Business Fabric coordinates references to reservations. It does not become the owner of the reserved state.

## Aggregate and anti-structuring policy

Policies MAY apply over an aggregate economic intent rather than a single API call.

Splitting an action into smaller actions MUST NOT make an otherwise prohibited aggregate economic effect permissible.

Policies MAY aggregate by counterparty, resource class, organizational unit, time window, authority lineage, action family, or other explicitly versioned dimensions.

## Quorum and separation of duties

High-consequence actions MAY require independent approval roles. One credential or authority identity MUST NOT satisfy multiple independent roles unless policy explicitly permits it.

Policy MAY require diversity across identity, role, device, model/operator, organization, or failure domain.

## Objective contracts

Symthaea MUST NOT optimize a naked KPI as if the metric were the objective itself.

An Objective Contract distinguishes:

- desired outcomes;
- diagnostic metrics;
- protected constraints/rights/floors;
- resilience floors;
- forbidden or unreliable proxies;
- evaluation horizon;
- required outcome evidence.

The optimizing system may propose changes to an Objective Contract, but MUST NOT unilaterally redefine its protected constraints or success criteria during an evaluation period.

Hard constraints define the feasible region. Optimization occurs inside that region.

## Resilience and slack

Operational slack may be an intentional resilience reserve rather than waste. Profiles/policies MAY establish floors for backup suppliers, staffing, inventory, cash, compute, energy, or other capacity.

Optimization MUST NOT consume protected resilience floors without an explicit policy-authorized exception.

## External execution outcomes

Transport success is not equivalent to economic success.

Adapters SHOULD distinguish at least:

- transport receipt;
- provider acknowledgement;
- economic observation;
- authoritative reconciliation.

When the external outcome cannot be established, the result remains unknown. Unknown MUST NOT be silently converted into failure and blindly retried.

Retries SHOULD use stable idempotency identity when supported. Otherwise, reconcile before issuing a potentially duplicative economic effect.

## Circuit breakers and degradation

Automation SHOULD degrade through explicit states such as:

`Autonomous -> ApprovalRequired -> RecommendOnly -> ObserveOnly`

Missing/stale evidence, unresolved conflicts, weakened authentication, cold calibration, broken adapters, or incident state may preserve or reduce autonomy but MUST NOT enlarge it.

Circuit breakers MUST dominate outstanding Prepared Actions.

After a breaker trips, autonomy MUST NOT silently restore itself. Re-arming requires the policy-defined evidence and a fresh authority decision/lease when required.

## Upgrade and model lineage

New code, model, policy, or Action Contract versions do not automatically inherit prior autonomy qualification.

Prepared actions are bound to the semantics under which they were prepared. An action prepared under `vN` MUST NOT silently execute under incompatible `vN+1` semantics.

Policy migration MUST make every predecessor invariant explicit as retained, strengthened, or exceptionally waived under bounded authority. Absence from a migration plan is not implicit approval to remove an invariant.

## Recovery

Recovery/break-glass authority SHOULD be independently governed, scope-limited, incident-bound, replay-resistant, and short-lived.

Recovery authority MUST NOT silently become ordinary production authority.

Compensation or rollback MUST NOT erase the original action, authorization, evidence, or outcome lineage.

## Constitutional invariants

The following identifiers are normative names for qualification tests:

- BIZ-I01 Authority Conservation: projections, recommendations, inference, synchronization, or explanation cannot increase authority.
- BIZ-I02 Epistemic Monotonicity: degraded evidence cannot enlarge autonomous action space.
- BIZ-I03 No Inference Promotion: estimate/forecast/proposal cannot become observation without new evidence.
- BIZ-I04 Policy Independence: confidence cannot bypass hard policy constraints.
- BIZ-I05 Frontier Binding: consequential actions identify relevant observation, policy, and authority contexts.
- BIZ-I06 Stale Action Rejection: materially invalidated preconditions prevent execution.
- BIZ-I07 Replay Safety: retrying the same execution identity causes at most one intended economic effect.
- BIZ-I08 Unknown Preservation: indeterminate external outcomes remain unknown until reconciled.
- BIZ-I09 Conflict Preservation: contradictory observations are not overwritten solely by arrival order.
- BIZ-I10 Explanation Non-Authority: generated explanations cannot grant authority.
- BIZ-I11 Learning Containment: learned models cannot modify their own authority or hard-policy envelopes.
- BIZ-I12 Privacy Minimality: action evaluation does not require unrelated sensitive attributes.
- BIZ-I13 Revocation Dominance: current revocation fences older outstanding authority.
- BIZ-I14 Compensation Is Not Erasure: compensating actions preserve original lineage.
- BIZ-I15 Capability Is Not Authority: discoverability/physical possibility does not imply permission.
- BIZ-I16 Compositional Safety: individually valid actions cannot jointly violate shared hard constraints.
- BIZ-I17 Anti-Structuring: splitting an action cannot bypass aggregate limits.
- BIZ-I18 Delegation Attenuation: every delegation is no broader than its parent.
- BIZ-I19 Delegated Budget Conservation: sibling grants cannot collectively exceed the parent's shared envelope.
- BIZ-I20 Revocation Closure: ancestor revocation invalidates descendant authority.
- BIZ-I21 Quorum Independence: one authority identity cannot satisfy multiple required independent roles unless explicitly allowed.
- BIZ-I22 Objective Integrity: the optimizer cannot redefine protected objectives/constraints during evaluation.
- BIZ-I23 Resilience Floor: optimization cannot consume explicitly reserved slack without authorized exception.
- BIZ-I24 Portfolio Constraint Preservation: aggregate exposure, concentration, and velocity constraints hold across concurrent agents.
- BIZ-I25 Breaker Dominance: a circuit breaker invalidates or fences affected outstanding Prepared Actions.
- BIZ-I26 Explicit Re-arm: tripped autonomy does not silently restore itself.
- BIZ-I27 Qualification Non-Inheritance: new model/action versions require explicit compatibility or qualification evidence before inheriting autonomy.

## Qualification dimensions

Business capability claims SHOULD distinguish at least:

Semantic qualification:
- S0 representable;
- S1 domain-owned;
- S2 interoperable;
- S3 multi-policy/jurisdiction capable;
- S4 succession/conflict safe.

Operational resilience:
- R0 happy path;
- R1 retry/idempotency safe;
- R2 stale-state safe;
- R3 partition tolerant;
- R4 degraded-mode safe;
- R5 recovery/reconciliation proven.

Autonomy:
- A0 observe;
- A1 explain;
- A2 recommend;
- A3 draft;
- A4 bounded delegated single action;
- A5 bounded multi-step orchestration.

Implementation is not qualification. Passing unit tests is not field qualification.

## Extension rule

A new industry SHOULD be modeled by composing capabilities, profiles, schemas, policies, and adapters.

Adding a country, currency, payment provider, legal form, brand, or vertical-specific transaction enum to a universal Business core is presumed to be an architectural failure unless a cross-domain semantic necessity is demonstrated.

The target property is not that all businesses behave alike. It is that heterogeneous organizations can expose evidence-bearing capabilities, retain authoritative domain ownership, reason under partial knowledge, and add bounded autonomy without allowing intelligence to manufacture authority.
