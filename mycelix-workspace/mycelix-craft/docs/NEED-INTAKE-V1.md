# Craft Need Intake V1

- **Work item:** CRAFT-NEED-INTAKE-001 (#6741)
- **Status:** Reference core; host and field qualification pending
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## 1. Problem contract

Need Intake converts an expressed community need into a structured **proposal**, not an authoritative project definition.

Three layers remain distinct:

1. **Community intent:** statements, priorities, constraints, trade-offs and desired outcomes, owned by the people and institutions expressing them.
2. **Machine-assisted decomposition:** proposed outcomes, measures, work packages, capability requirements, assumptions, inferences, contradictions and unknowns.
3. **Local ratification:** an exact, reviewed proposal revision adopted by the competent local authority before it can be transformed into a project-ready input for Project Formation V1.

A model can help ask questions, extract candidate statements, find relationships, identify missing evidence and prepare alternatives. It cannot decide what people need, invent facts, establish consensus, approve safety constraints, appoint assessors, reserve resources, ratify a project or authorize work.

## 2. Provenance and epistemic types

Every outcome and capability requirement must have at least one explicit traceability link:

- **Source:** exact source identifier and digest. This establishes where the claim came from, not whether the source is true or authoritative.
- **Assumption:** a separately recorded proposition with an accountable owner and risk-if-false statement. It is not presented as fact.
- **Inference:** a stable inference identifier, method/model version and explicit source list. The output remains an inference; a model confidence score does not convert it into evidence.

Unknown, disputed and contradictory information must stay visible. A missing baseline is represented as unknown, not filled with a plausible-looking estimate. Source text and sensitive participant records remain in their data-controller-owned stores; this contract keeps references, digests and visibility-profile identifiers.

This structure should map to W3C PROV concepts for entities, activities, agents, derivations, bundles and versioned provenance. Craft must not create a competing general provenance standard.

## 3. Proposal structure

A proposal contains:

- original need identifier, statement digest, submitter and visibility profile;
- source references with type, digest and visibility profile;
- explicit assumptions and accountable owners;
- outcomes with indicator, baseline, target, time horizon, outcome owner and acceptance-authority reference;
- capability requirements classified as SafetyCritical, Required, Developmental or Preferred;
- work packages, outcome links, requirement links and dependency edges;
- explicit contradictions and any resolution decision references;
- open questions, including which ones block ratification;
- safety review requirement;
- review records bound to the exact proposal digest.

The contract validates reference integrity and rejects cyclic work-package dependencies. It does not interpret natural language or verify that a source actually supports the statement. That semantic check belongs in the reviewed retrieval/assessment process and its benchmark.

## 4. Review and ratification state machine

Prerequisite review roles are distinct:

- **Community:** confirms need, priorities, trade-offs and representation of stakeholder intent.
- **DomainExpert:** reviews technical feasibility, standards and domain-specific interpretation.
- **Provenance:** checks source links, assumptions, inference labels and evidence coverage.
- **Safety:** required when the proposal has a safety review flag.
- **LocalRatifier:** uses the institution's actual governance process to adopt the exact proposal digest.

A role cannot replace another role. Approval by a model, proposer or generic project facilitator is not community approval.

The evaluator only counts reviews that bind to the exact proposal digest and are current at evaluation time. A changed proposal digest invalidates all previous approvals. Expired, withdrawn, future-dated or differently scoped authority references must be rejected by the host before the review snapshot reaches this core.

Unresolved stakeholder-priority conflicts or blocking questions produce NeedsCommunityClarification. Technical/source contradictions require appropriate domain review or recorded resolution. A missing safety review cannot be replaced by a domain review. All prerequisite reviews lead only to ReadyForLocalRatification. Only a separate LocalRatifier approval can produce Ratified.

## 5. Versioning and invalidation

Revision 1 has no predecessor. Every later revision must pin its previous proposal digest. The host computes the digest over a canonical, versioned representation and authenticates the proposal bytes. The pure reference crate checks that a digest is present and that review records bind to it; it does not canonicalize, hash, verify signatures or prove storage immutability.

Any change to outcomes, capability classes, safety flags, sources, assumptions, dependencies, jurisdictional scope, cost assumptions, acceptance criteria or disclosure profile must create a new revision and invalidate earlier reviews.

## 6. Authority boundaries

- Community members own the expression and priority of need.
- Craft owns the workflow structure and traceability references.
- Symthaea may assist with decomposition, retrieval, questioning and explanation.
- Source owners and data controllers govern access and disclosure.
- Standards bodies and statutory authorities own their standards and legal applicability.
- Domain and safety authorities own their technical/safety determinations.
- Local governance owners ratify the proposal.
- Project Formation V1 evaluates the ratified project's requirement/coverage envelope.
- Resource owners control inventory, bookings, custody and release.
- Finance, issuers and payment providers control economic effects.

No output from this crate is a work order, construction permit, professional licence, budget approval, resource reservation, payment instruction or safety authorization.

## 7. Threat model

V1 explicitly considers:
- fabricated or mismatched source citations;
- paraphrase drift and omitted qualifiers;
- source disagreement or stale policy references;
- invented stakeholder consensus;
- a preferred capability silently becoming mandatory;
- optional credential/degree prestige acting as a hidden gate;
- omitted safety work or jurisdictional mismatch;
- false baselines, targets, costs or unit conversions;
- unresolved assumptions being described as facts;
- changes after review or ratification;
- reviewer role confusion or untrusted authority snapshots;
- model-generated outputs being treated as human approval;
- privacy leakage through public source or participant rosters.

The host remains responsible for authentication, access control, privacy, canonical serialization, cryptographic commitments, local policy and replay resistance.

## 8. Qualification

The reference core must pass exact-head format, compile, tests and Clippy. Before production:
1. Build a frozen dataset of community need statements with expert-reviewed claim and provenance annotations.
2. Include ambiguous, contradictory, incomplete and adversarial cases, not only clean requirements.
3. Measure source support precision/recall at claim level, unsupported claim rate, omission rate, contradiction retention and abstention calibration.
4. Measure safety-constraint recall and dangerous false-negative rate separately.
5. Measure community correction effort, consent comprehension, dissent preservation and perceived agency.
6. Compare against human facilitation, structured intake, generic LLM, retrieval-grounded LLM and multi-agent orchestration baselines.
7. Reproduce the same proposal digest and review classification in an independent implementation.
8. Run a prospective field pilot with independent measurement of actual outcome, not only project completion.

A benchmark pass does not confer legal authority or prove community benefit. See NEED-INTAKE-BENCHMARKS-V1.md.
