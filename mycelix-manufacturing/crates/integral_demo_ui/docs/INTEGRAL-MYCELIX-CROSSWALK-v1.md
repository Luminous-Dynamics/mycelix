# Integral ↔ Mycelix Crosswalk v1

## Purpose

This document is an architectural research artifact, not a claim that Mycelix implements Integral or that Integral has adopted Mycelix.

The goal is to identify **generalizable protocol primitives** exposed by attempting to model Integral's five-system cybernetic loop:

`CDS → OAD → COS → ITC → FRS → CDS`

and to keep Integral-specific economic/governance semantics separate from Mycelix substrate capabilities.

**Claim ceiling: ReferenceModelOnly.**

## Core principle

> Integral should own its economic and social semantics; Mycelix can provide reusable identity, provenance, evidence, authorization, federation, deliberation, contribution, and audit primitives.

Neither auditability nor provenance is equivalent to legitimacy, consent, human endorsement, or truth.

## Crosswalk

| Integral concept | Candidate Mycelix primitive | Relationship | Status |
|---|---|---|---|
| CDS | Deliberation + Decision | Strong conceptual correspondence | Proposed/generalize |
| DecisionPacket | Decision artifact | Natural substrate mapping | Proposed |
| OAD | Design Lifecycle | Strong conceptual correspondence | Existing demo seam + extend |
| CertifiedDesign | Design certification | Certification remains distinct from authority/execution | Existing boundary + extend |
| COS | Execution Intent + Evidence | Strong correspondence | Existing demo seams |
| LaborEvent | ContributionEvent | Generalizable beyond time credits | Proposed |
| MaterialConsumptionEvent | ResourceConsumptionEvent | Generalizable accounting substrate | Proposed |
| ITC | Contribution valuation | Integral-specific semantics over generic contribution evidence | Adapter, not substrate |
| FRS | Assessment + Recommendation + Human Disposition | Strong correspondence | Existing demo seams + extend |
| FRSSignalPacket | Feedback/Assessment packet | Natural evidence-bound mapping | Proposed |
| Human decision | HumanDecision | Explicit authority/contestability boundary | Existing D5 |
| Federation | Federated delivery + observation binding | Strong substrate correspondence | D6E |
| Disagreement | Conflict + Disputes graph | Strong substrate correspondence | D6E |
| Symthaea assistance | Explanation/Simulation view | Non-authoritative cognitive assistance | HXA/D5 |
| Human flourishing | HXA evidence ladder | Outcome research, not a legitimacy primitive | Research only |

## Proposed generalized cybernetic loop

A reusable Mycelix coordination loop can be expressed as:

`Intent → Decision → Design → Authorization → Execution → Observation → Assessment → Recommendation → Human Disposition → Revision`

The transitions are deliberately non-automatic.

- A decision does not become authorization merely because it exists.
- Authorization does not imply execution.
- Execution does not imply successful outcome.
- Observation does not become qualification.
- Assessment does not mutate source evidence.
- Recommendation does not become authority.
- Foreign evidence does not become local-origin evidence.
- A deterministic ordering does not imply causality.
- Human disposition is explicit rather than inferred from an AI recommendation.

## Research-derived PR sequence

### A1 — Generalized Coordination Loop

Introduce domain-neutral artifacts for Decision, Design, Authorization, ExecutionIntent, Observation, Assessment, Recommendation, Disposition, and Revision.

Acceptance criteria:
- explicit provenance class;
- explicit generation;
- no implicit authority inheritance;
- replay identity;
- supersession;
- challenge/appeal path where consequential.

### A2 — Feedback Loop

Connect Observation → Assessment → Recommendation → HumanDisposition → new Decision without laundering recommendation into governance authority.

Acceptance criteria:
- recommendation remains recommendation-only;
- source binding is immutable;
- disagreement is preserved;
- a new decision is required for consequential change.

### A3 — Adversarial Loop Conformance

Exercise stale, rejected, foreign, conflicting, mutated, replayed, and recommendation-as-authority fixtures.

### B1 — Design Lifecycle / Design Commons

Generalize OAD's lifecycle concepts:

`Proposal → Review → Simulation → Validation → Certification → Deployment → Observation → Revision`

Certification, authorization, execution, and outcome remain distinct.

### B2 — Certification vs Execution Authority

Formalize:

`CertifiedDesign ≠ Authorization ≠ Execution ≠ SuccessfulOutcome`

### B3 — Supersession + Feedback Lineage

Bind observed performance and review findings to design generations without making the newest artifact automatically correct.

### C1 — General Contribution Event

Generalize Integral's labor/material accounting into source-bound contribution events.

Possible shape:

`ContributionEvent { contributor, activity, resource, evidence, authorization, context, quantity, uncertainty, provenance, outcome_ref }`

### C2 — Contribution → Outcome Lineage

Connect contribution evidence to tasks, designs, execution and observed outcomes without assuming simple causal attribution.

### C3 — Non-Scalar Contribution Semantics

Preserve uncertainty, context and competing interpretations. Do not reduce a person to a single contribution score.

### D1 — Deliberation Object Model

Represent Question, Context, EvidenceSet, Alternative, Argument, Concern, Proposal, Decision, Dissent and Appeal as explicit objects.

### D2 — Decision Provenance Graph

Make the reasons and evidence behind a decision inspectable without treating explanation as proof.

### D3 — Contestability

Provide Challenge → Review → Reaffirm/Revise/Reverse paths with durable provenance.

### E1 — Integral Compatibility Adapter

Only after the generalized primitives exist, map Integral's five systems onto them.

The adapter should preserve Integral's semantics rather than redefining them.

## What should remain Integral-specific

The following should **not** be silently absorbed into Mycelix's substrate:

- the definition of ITC;
- post-monetary economic assumptions;
- Integral's specific governance/consensus rules;
- Integral's normative goals;
- Integral-specific valuation policies;
- claims about real-world economic viability.

Mycelix should expose primitives that allow those semantics to be implemented and audited.

## Evidence and claim discipline

Every compatibility claim should identify:

1. the Integral source;
2. the exact semantic correspondence;
3. the Mycelix implementation;
4. assumptions introduced by the implementation;
5. unresolved divergence;
6. evidence level;
7. claim ceiling.

The reference node must never imply that an implementation choice is an Integral-ratified rule.

## Immediate D6E bridge

The current federation work already demonstrates two important pieces of this crosswalk:

- interpretation hypotheses can be compared without becoming trace evidence;
- heterogeneous observations can remain distinct and explicitly dispute one another without a synthetic winner or causal edge.

The next integration point is therefore not another transport feature. It is the **feedback/governance boundary**: conflict → assessment → recommendation → explicit human disposition.

### Federation source/evidence conservation

D6E now makes the source/evidence boundary explicit at the federation seam. A delivery carries both `source_ref` and `evidence_ref`; an observation and its federation binding carry both fields; replay receipts preserve both; and the D5 projection maps them independently. A source reference is therefore never treated as evidence identity merely because both values happen to be strings or because the delivery was accepted locally.

The federation validator also fails closed when either reference is missing, and rejects mutations to either field during replay/binding. This is a reference-model invariant intended to prevent provenance compression at the adapter boundary.

## Open questions for Integral maintainers

1. What exactly constitutes OAD→COS semantic admission?
2. Which COS observations are eligible inputs to ITC?
3. What evidence must an FRS recommendation preserve?
4. Can foreign observations directly participate in a local FRS assessment?
5. Which events constitute governance authority?
6. How should conflicting observations be resolved, and which body has authority to resolve them?
7. Which parts of the cybernetic loop are normative Integral semantics versus implementation-neutral interface requirements?
8. What is the ratified status and migration policy for each pending interface contract?

## Non-claims

This crosswalk does not establish:

- Integral ratification;
- Mycelix conformance to Integral;
- economic correctness;
- production readiness;
- human flourishing;
- governance legitimacy;
- causal attribution of contributions;
- superiority of either project.

It is an engineering research bridge intended to make those questions explicit and testable.


## A1 implementation status

The reference model is implemented in `cos_conformance::integral_demo_coordination`.

It introduces typed coordination artifacts for Intent, Decision, Design, Authorization, ExecutionIntent, Observation, Assessment, Recommendation, HumanDisposition, Revision, and Appeal. The validator checks explicit parent lineage, generation monotonicity, provenance-bearing evidence, uncertainty preservation, authority boundaries, and challenge/recovery requirements for consequential accepted dispositions.

This is a bounded semantic model. It does not prove the evidence is true, the authority is legitimate, or any physical action occurred. The validator also does not yet replace D5's richer graph validator; integration and shared fixture coverage remain follow-up work.


### A1 ↔ D5 alignment boundary

A bounded cross-check now compares A1 coordination artifacts against D5 trace events by stable identity and explicitly shared fields: kind, provenance, origin, generation, source reference, **evidence reference**, authority reference, uncertainty, challengeability, reversibility, recovery reference, and explicit human disposition. `source_ref` and `evidence_ref` remain distinct fields; the bridge never equates them implicitly. It first runs the D5 trace validator.

This is intentionally **not** a conversion. D5 graph relations and A1 parent links express different things; event order is not used to invent parentage. A direct Observation → FRS Assessment trace transition is permitted so a valid graph-native conflict assessment does not require an artificial ITC projection stage.

The alignment is still a reference-model cross-check. It does not establish that the two representations are semantically complete or that external evidence is true.


### A1 governance-disposition and pair-validation boundary

A recommendation is not treated as governance authority merely because a human disposition artifact exists. The reference helper recognizes only explicit **Accepted** or **Rejected** dispositions as a governance disposition; **Deferred** remains a distinct unresolved state and is not coerced into execution authority.

The A1↔D5 bridge now also exposes a combined validation gate: A1 parent lineage is validated by the A1 coordination validator, D5 graph structure is validated by the D5 trace validator, and shared fields are then cross-checked by stable identity. No parent link is inferred from trace ordering. This keeps model-specific semantics explicit while still making a shared fixture capable of failing closed when either representation is malformed.


### Explicit A1↔D5 pairing for shared fixtures

For fixtures that carry both representations, the bridge now supports explicit artifact/event pairs. The caller supplies the semantic A1 artifact and the exact D5 event it represents; the bridge verifies identity and shared semantics but does not derive parentage, causality, or authority from serialization order. This makes the integration suitable for adversarial fixtures while keeping lineage ownership with the model that declares it.


### Canonical-event binding

Explicit A1↔D5 pairs must bind to the canonical event already present in the validated D5 fixture. A caller cannot supply a forged or detached event with a matching ID and have it treated as the fixture's evidence. The bridge therefore validates the D5 fixture first, resolves the event by identity, and compares the supplied pair event to that canonical record before applying semantic alignment.

### Authority non-escalation at projection/replay boundaries

Authority is treated as a semantic boundary, not as metadata that can emerge while an artifact crosses subsystem representations. The D5 reference model now exposes an explicit `authority_reference_is_conserved` invariant for projection/replay paths: an existing authority reference must remain identical, and an absent reference cannot become authoritative merely through serialization, replay, federation, or trace projection. Governance transitions remain separate: A1 explicitly requires authority on Decision/Authorization/ExecutionIntent/HumanDisposition where applicable, while Recommendation is permanently advisory and cannot carry authority.

The A1 coordination validator now makes the stronger transition rule explicit: **authority creation is confined to governance-bearing stages** (Decision, Authorization, HumanDisposition), while ExecutionIntent must conserve the exact authority reference from its Authorization parent. Evidence stages, Recommendation, Revision, and Appeal cannot acquire authority through ordinary parent linkage.

This separates three operations that are easy to conflate:

1. **Authority creation** — permitted only at an explicit governance transition.
2. **Authority conservation** — permitted when a stage legitimately carries the existing reference forward.
3. **Authority mutation/escalation** — rejected when a transition changes, removes, or introduces authority outside the allowed governance boundary.

These are reference-model invariants, not claims about Integral's ratified governance semantics.


### D6F semantic capability boundary

D6F makes the capability boundary explicit rather than leaving it implicit in stage names or authority fields. The A1 reference model now classifies Observation as **Evidence**, Assessment as **Assessment**, Recommendation as **AdvisoryRecommendation**, HumanDisposition as **HumanDisposition**, Authorization as **Authorization**, and ExecutionIntent as **ExecutionIntent**. Capability is deliberately distinct from authority: evidence and assessment can be validly represented without gaining permission to authorize or execute.

The capability transition validator rejects direct jumps from Evidence, Assessment, or Recommendation to Authorization or ExecutionIntent. Execution capability requires the explicit Authorization → ExecutionIntent transition. HumanDisposition remains a governance disposition boundary rather than an execution capability, so a disposition cannot be treated as an authorization merely because it is accepted.

This is a structural reference-model constraint. It does not claim that these exact capability names or transitions are Integral-ratified governance rules, nor does it establish truth, legitimacy, or real-world execution.


### D6G provenance relation algebra

D6G makes provenance relations explicit in the D5 graph instead of overloading event sequence or A1 parentage. The reference relation vocabulary now includes GeneratedBy, DerivedFrom, Used, AttributedTo, and Revises, alongside existing governance/dispute relations such as Authorizes, RespondsTo, Disputes, Supersedes, Appeals, and Reopens.

The direction of a provenance edge is part of its declared meaning. Validation checks the relation semantics directly; it does not infer causality from serialization order, and it does not require unrelated source references to match. Revises additionally requires an explicit newer Design generation. This keeps several concepts separate: ordering ≠ causality, parentage ≠ provenance, provenance ≠ authority.

The change is deliberately additive to the D5 reference model. A1 parent_ref remains model-owned compatibility data for now; D6G does not silently reinterpret it as a provenance edge. A future migration can introduce explicit relation fields in the coordination model once the cross-boundary vocabulary is stable.

This remains a ReferenceModelOnly invariant. It does not establish causal truth merely because a DerivedFrom or GeneratedBy edge is structurally valid.


### D6H canonical replay and semantic equivalence

D6H adds an explicit semantic-equivalence boundary for replay. A replay may reorder events or relation records and may use different sequence values without becoming a different semantic trace. Canonicalization sorts events by stable event identity and relations by their declared endpoints and relation kind; sequence is intentionally excluded from semantic identity because **ordering is not causality**.

Semantic equivalence nevertheless preserves the fields that can change the meaning or authority of a trace: event identity, kind, provenance, actor, origin, source reference, evidence reference, generation, uncertainty, authority reference, reversibility, challengeability, recommendation-only status, recovery/appeal references, disposition, status, and relation endpoints/kinds. Independent mutation of source or evidence, origin laundering, authority mutation, safety-field mutation, uncertainty loss, or relation mutation therefore makes a replay non-equivalent.

The canonical helper also rejects duplicate event identities and duplicate relation records rather than normalizing malformed input into an apparently valid replay. This is intentionally stronger than byte equality while remaining weaker than a truth claim: semantic equivalence says that two reference-model representations describe the same declared trace semantics; it does not prove that the underlying evidence is true, that a causal relation actually occurred, or that an authority is legitimate.

D6H therefore establishes three separate properties:

1. **Replay identity** — an exact event or fixture can be replayed without mutation.
2. **Semantic equivalence** — permitted serialization/order differences do not create false semantic divergence.
3. **Semantic mutation detection** — changes to provenance, evidence, authority, uncertainty, origin, safety, or graph relations are observable as non-equivalence.

The canonical representation is a comparison aid, not a canonical election of history. It does not select a winner among conflicting observations or turn deterministic ordering into causal truth.

### D6I conflict branches and explicit reconciliation

D6I makes unresolved disagreement addressable as a first-class branch set. A conflict produces two separately identified branches, each retaining its observation identity, origin, work identity, and quantity. Branch labels are navigational only: they do not rank observations, choose a winner, or imply that either source is more credible.

A reconciliation record must bind to the exact conflict and an explicit human decision artifact that names both observation identities and the current generation. It must also identify a distinct reconciliation artifact and a revision at a strictly newer generation. Missing, stale, mismatched, or identity-colliding records fail validation. The source observations remain intact; reconciliation creates a new governance/revision path rather than rewriting history.

This is a structural reference-model gate, not a resolution algorithm. It does not decide which observation is true, does not treat an accepted disposition as proof of factual correctness, and does not imply that a revision was executed. The model preserves the difference between disagreement, human disposition, and subsequent revision.
