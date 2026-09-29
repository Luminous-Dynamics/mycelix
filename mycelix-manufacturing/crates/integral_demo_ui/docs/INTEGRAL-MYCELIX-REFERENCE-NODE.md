# Integral × Mycelix — Reference-Node Compatibility Note

## Purpose

This document explains how the current Mycelix Integral reference-node work relates to the public Integral v0.1 architecture.

It is intentionally a **compatibility and interpretation note**, not a claim that Mycelix is the Integral implementation.

The goal is to make three things explicit:

1. what Integral currently specifies;
2. what this reference node implements as a bounded executable model;
3. what remains an interpretation, extension, or open question.

The distinction matters because Integral's own development guide emphasizes **architectural faithfulness versus functional completeness**, and because its current interface contracts remain under Phase-0/Phase-1 specification and governance.

## Source baseline

This note is aligned against the public Integral v0.1 materials available at the time of writing:

- White paper: https://integralcollective.io/documents/whitepaper.html
- Development guide: https://integralcollective.io/documents/devguide.html
- Technical specifications: https://integralcollective.io/documents/specifications.html
- Community contribution guidance: https://integralcollective.io/community/contribute.html
- GitHub organization: https://github.com/Integral-Collective

Integral currently identifies six draft core data structures (CertifiedDesign, LaborEvent, MaterialConsumptionEvent, ITCLedgerEntry, FRSSignalPacket, DecisionPacket) and three pending interface contracts: OAD→COS, COS→ITC, and FRS→CDS. The public specifications page identifies these interface seams as priority work for systems architects, backend developers, and protocol designers.

The Integral specifications repository README additionally states that repository interaction is currently gated to approved contributors. This note therefore treats the public documents as the architectural source baseline and does not assume that the Mycelix reference model has been ratified by Integral.

## The five-system correspondence

| Integral system | Reference-node treatment | Status |
|---|---|---|
| CDS | Explicit human governance decision and disposition | Bounded reference seam |
| OAD | Versioned design/admission boundary | Bounded reference seam |
| COS | Production admission plus provenance-bound observation model | Bounded reference seam |
| ITC | Source-bound projection from COS observation | Bounded reference seam |
| FRS | Assessment/recommendation and return-to-governance boundary | Bounded reference seam |
| Federation | Foreign/local origin, replay, reconciliation, and conflict preservation | Reference-model extension supporting federation |
| Human experience | Cockpit, progressive disclosure, challenge/recovery metadata | Reference-model extension |
| Symthaea | Explanation/recommendation assistance only | Non-authoritative assistance layer |

The correspondence is deliberately not one-to-one at the implementation-module level. The reference node is currently testing the **seams and invariants between systems**, not claiming to implement all 45 Integral modules.

## What the implementation is actually testing

### 1. Authority does not emerge from evidence

The reference model keeps these concepts separate:

- observation;
- assessment;
- recommendation;
- human decision;
- authorization;
- execution intent.

An FRS recommendation cannot become governance authority merely because it is present in the same trace. A human decision is represented as a separate event with explicit disposition and authority metadata.

This is consistent with the architectural role of FRS as a system that diagnoses operational reality and routes recommendations back to CDS, rather than silently replacing CDS governance.

### 2. Provenance survives federation

D6E distinguishes:

- evidence origin;
- authority origin;
- local versus foreign source;
- logical delivery identity;
- retry/attempt identity.

A foreign observation can be accepted under an explicit local governance context without becoming locally originated evidence.

This is an implementation choice designed to preserve the distinction between **where evidence came from** and **which node is currently acting on it**.

It should be treated as a proposed federation invariant for discussion, not as a claim that Integral has already ratified this exact representation.

### 2a. Source identity and evidence identity are separate

The federation seam now carries two explicit references rather than reusing one identifier for both meanings:

- `source_ref` identifies the source/binding context;
- `evidence_ref` identifies the evidence artifact.

The distinction is conserved in delivery envelopes, replay receipts, observation bindings, and D5 trace projection. Missing references and replay mutations fail closed in the reference validator. This is deliberately a proposed engineering invariant, not an Integral-ratified rule.

### 3. Reconciliation does not mean selecting a winner

When two observations refer to the same work but disagree:

1. both observations remain present;
2. their independent origins remain present;
3. their evidence references remain present;
4. the conflict is represented explicitly;
5. deterministic ordering does not decide which observation is correct;
6. a later governance decision is a separate artifact.

This is important because deterministic replay is useful for reproducibility but does not constitute epistemic authority.

### 4. Replay cannot mutate identity

D6E separates:

- logical delivery identity;
- attempt identity;
- observation identity;
- payload digest;
- origin;
- authority origin.

An exact retry may replay. A retry that changes payload, origin, or other identity-bearing fields is rejected.

This gives the reference model a deterministic failure boundary around append-only federation evidence.

### 5. Transport co-occurrence is not causality

The federation trace projection does not invent `Supports` or other causal relations merely because observations arrived together or happen to be adjacent in a canonical ordering.

Causal relations must be supplied by the layer that has evidence for them.

For example, a reconciliation layer may establish an explicit `Disputes` relation between observations. The transport layer itself does not get to manufacture that meaning.

This distinction is now a deliberate design rule in the reference model.

## Interpretation laboratory

D6E now includes a small executable **interpretation laboratory** in `integral_demo_interpretations.rs`.

The laboratory does not choose a “correct” interpretation. It makes competing semantics explicit so they can be reviewed against Integral's own governance/specification process.

Each unresolved seam is represented with three deliberately distinct variants:

| Seam | Minimal / faithful | Strong-safety | Federation-aware |
|---|---|---|---|
| OAD→COS | Current certified design may cross a bounded admission seam | Certification, admission authorization, generation freshness, and retry identity are distinct | Adds explicit evidence-origin / authority-origin separation and foreign-authority rejection |
| COS→ITC | Operational observation can cross a projection seam; projection is not economic eligibility | Source/observation identity and current generation are checked; mutation is rejected | Foreign origin remains foreign and privacy projections cannot become source observations |
| FRS→CDS | FRS returns a recommendation to governance | Recommendation cannot carry operational authority; explicit governance disposition is required | Foreign recommendation provenance remains intact; conflicting observations await explicit decision |

These are **not three maturity levels** and the ordering is not a ranking. They are alternative semantic hypotheses.

### Divergent fixtures

The laboratory now makes semantic divergence executable rather than merely descriptive.

| Fixture | Minimal / faithful | Strong-safety | Federation-aware |
|---|---|---|---|
| Superseded OAD design | Admitted by the minimal hypothesis | Rejected because generation freshness is explicit | Rejected under the federated admission boundary when freshness/authority checks apply |
| Foreign authority | Outside the minimal positive boundary | Outside the explicit positive boundary | Rejected as an authority-origin violation |
| Mutated retry | Outside the minimal positive fixture | Rejected as non-idempotent mutation | Rejected as mutation of logical delivery identity |
| Conflicting observations | Outside the minimal projection fixture | Outside the bounded source-projection fixture | Rejected from automatic projection; disagreement remains explicit |
| FRS recommendation | Returned to governance | Requires explicit governance disposition | Requires governance while preserving foreign provenance |

The exact outcomes are **fixture semantics of this reference harness**, not claims about what Integral itself must do.

The useful review question is therefore:

> **Which semantics did Integral intend at this interface, and what evidence should distinguish the alternatives?**

The executable laboratory makes that question testable. A future fixture can be added whenever a specification question has multiple plausible readings.

The implementation should not silently promote one hypothesis into an architectural fact.

## Compatibility matrix

| Area | Integral public baseline | Mycelix reference treatment | Classification |
|---|---|---|---|
| Five-system loop | CDS → OAD/COS/ITC; OAD → COS; COS → ITC; COS+ITC+OAD → FRS; FRS → CDS | D1–D6 vertical reference flow | Faithful bounded interpretation |
| OAD→COS | Pending interface contract | Explicit admission boundary with version/generation and authority checks | Implementation proposal |
| COS→ITC | Pending interface contract | Source-bound observation → ITC projection | Implementation proposal |
| FRS→CDS | Pending interface contract | Recommendation separated from human governance decision | Implementation proposal |
| CertifiedDesign schema | Draft public data structure | Reference model uses versioned design identity but does not claim full schema compatibility | Partial / not yet conformant |
| LaborEvent | Draft public data structure | Not claimed as a complete production LaborEvent implementation | Not yet implemented |
| MaterialConsumptionEvent | Draft public data structure | Not claimed as a complete production material-event implementation | Not yet implemented |
| ITC ledger | Draft public data structure | Projection seam only; not a complete economic ledger | Not yet implemented |
| FRSSignalPacket | Draft public data structure | FRS finding/recommendation seam | Partial |
| DecisionPacket | Draft public data structure | Human decision trace and authority boundary | Partial |
| Federation | White paper describes federated nodes and exchange | D6E provides deterministic foreign/local evidence and replay model | Reference-model extension |
| Append-only history | Development guide emphasizes append-only collections | Federation delivery/observation replay model | Faithful principle, bounded implementation |
| Interface versioning | Development guide and specifications require versioning | Schema generation is checked at admission/binding boundaries | Faithful principle, different representation |
| Human governance | CDS is the democratic governance layer | Human decision remains explicit and separate from machine recommendation | Faithful architectural constraint |
| AI assistance | Not a required Integral authority mechanism | Symthaea is recommendation/explanation only | Deliberate non-authoritative extension |

## Important non-claims

The current reference node does **not** establish:

- Integral ratification;
- conformance with any ratified Integral interface;
- economic correctness of ITC;
- correctness of the proposed governance model;
- network reliability;
- cryptographic authenticity;
- privacy compliance;
- production scalability;
- real-world cooperative viability;
- improved human outcomes;
- legitimacy of any governance decision.

Its current claim ceiling remains:

**ReferenceModelOnly**

The purpose is to expose assumptions and failure modes early enough that they can be challenged before they become hidden implementation commitments.

## Questions for Integral maintainers

The implementation surfaced several questions that should be answered by Integral's own governance/specification process rather than silently decided by an external reference implementation.

### Q1 — OAD→COS admission

What exactly constitutes an authorized transition from a certified design into COS?

In particular:

- Is certification itself sufficient?
- Which identity is authoritative?
- What happens when a node receives a design from another node?
- How are superseded design versions rejected?
- What are the required retry/idempotency semantics?

### Q2 — COS→ITC evidence semantics

What is the authoritative boundary between:

- a COS operational event;
- a verified contribution;
- an ITC-eligible event;
- an FRS signal?

The public specifications identify LaborEvent and MaterialConsumptionEvent, but the exact evidence/verification semantics remain an important implementation question.

### Q3 — FRS→CDS recommendation semantics

Can an FRS recommendation ever directly dispatch an operational change, or must consequential governance changes always pass through CDS?

The reference model assumes the latter because it keeps recommendation and governance authority distinct.

### Q4 — Federated disagreement

When two nodes report conflicting observations of the same work:

- should both observations remain canonical evidence?
- who is permitted to resolve the conflict?
- is resolution a new governance artifact?
- how is the resolution represented without rewriting either observation?

The reference model currently preserves the conflict and requires an explicit later decision.

### Q5 — Provenance versus authority

Should Integral formally distinguish:

**origin of evidence**

from

**authority exercised over that evidence**?

D6E treats these as independent dimensions because conflating them permits foreign evidence to be laundered into local evidence merely by being accepted or processed locally.

### Q6 — Determinism versus causality

Should deterministic ordering ever imply semantic relations?

The reference model deliberately says no: ordering provides reproducibility; explicit evidence provides causality.

## Suggested next step

The most useful next step is not to claim completion.

It is to compare this reference model against the ratified/pending Integral contracts once contributor access is available, then turn any genuine mismatch into a small, reviewable specification question.

That keeps the relationship healthy:

**Integral defines the architecture and governance.**

**The reference node tests interpretations against executable constraints.**

**Implementation findings become questions or proposals for Integral's own review process.**

That separation is intentional.


## Interpretation cockpit projection

The D6E interpretation laboratory is now connected to the cockpit projection boundary through `integral_demo_interpretation_cockpit.rs`.

For each unresolved seam, the cockpit can expose the three semantic hypotheses side-by-side and attach them to the same underlying D5 trace context:

- **Minimal / Faithful hypothesis**
- **Strong-Safety hypothesis**
- **Federation-Aware hypothesis**

The projection deliberately does **not** create a trace event for an interpretation. The hypothesis is analysis; the D5 trace remains the evidence-bearing reference artifact. This prevents the UI from laundering an interpretation into evidence, authority, causality, or an observed outcome.

The view therefore exposes two distinct layers:

1. **What the reference trace contains** — event/relation counts and the trace reference.
2. **What each semantic hypothesis would conclude from that fixture** — outcome and reason from the executable interpretation laboratory.

Ordering is deterministic for serialization/UI stability only. It is not a ranking, maturity ladder, or recommendation.

This is intentionally a thin seam for the future Leptos UI: a user can select an unresolved interface and adversarial fixture, compare the three hypotheses, then inspect the same underlying trace rather than receiving a single hidden interpretation.


## Graph-native federation disagreement

The A1↔D5 boundary now carries an explicit `evidence_ref` alongside `source_ref`. These are distinct semantic fields: source/binding identity is not silently treated as evidence identity, and evidence-bearing Observation/Assessment events must retain an explicit evidence binding.

D6E also projects a reconciled heterogeneous conflict into the D5 graph without collapsing it into a winner. Two source observations remain distinct, retain their local/foreign origin, and receive explicit reciprocal dispute relations. A local FRS assessment may respond to both observations and remains an assessment with no authority reference.

An important validator correction accompanies this: a dispute is allowed to cross federation origins because heterogeneous origin is the *subject* of the disagreement, not a provenance mutation. The schema generation must still match, preventing evidence from different schema epochs from being treated as one dispute set.

Delivery order is not causal order. The projection canonicalizes observation presentation by identity and emits only relations justified by the conflict itself. Governance resolution remains a separate human decision artifact.
