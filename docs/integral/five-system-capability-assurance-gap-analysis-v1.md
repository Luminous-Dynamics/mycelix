# Integral Five-System Capability & Assurance Gap Analysis v1

**Status:** Research baseline / source-grounded draft  
**Issue:** #3339  
**Source authority:** Integral Development Guide v0.1 (public draft)  
**Mycelix role:** neutral adapter/evidence/assurance substrate; not Integral policy authority

## Source-status rule

`white-paper concept != DevelopmentGuide proposal != public draft schema != ratified specification != implementation != conformance evidence`

The public Development Guide explicitly describes itself as a proposed bridge from theory to implementation, says unresolved questions remain, and requires assumptions to be tested in real contexts. This matrix therefore records **what is currently explicit in the reviewed public material**, not what Integral definitively lacks.

## Classification

- **Explicit** — directly represented in the reviewed public guide.
- **Partial** — represented, but important semantic/assurance dimensions remain implicit or deferred.
- **Deferred** — the guide explicitly postpones the capability.
- **NotExplicit** — no explicit treatment was located in the reviewed guide; this is not evidence that the concept is absent elsewhere.
- **ExternalNormative** — belongs to Integral's policy/governance/economic semantics and should remain adapter-owned.
- **MycelixReuse** — existing neutral Mycelix capability is a plausible reusable substrate, subject to refinement evidence.
- **GapCandidate** — candidate for further engineering/research, not a claim of an Integral defect.

## Core findings

### 1. The largest opportunity is not another fifth/sixth system

The five systems already provide a functional decomposition. The stronger Mycelix contribution is a **cross-cutting assurance plane** that follows a claim through:

`source observation -> evidence -> validity -> interpretation -> recommendation -> decision -> authorization -> effect receipt -> outcome -> correction/supersession`

This is deliberately orthogonal to CDS/OAD/ITC/COS/FRS.

### 2. The public guide already identifies the oracle problem

The guide explicitly calls out strategic misrepresentation, peer/community verification, physical triangulation, and outcome-based verification as hard problems. That makes evidence provenance and verification architecture a first-class research target rather than an invented concern. citeturn2view0

### 3. Several high-value lifecycle controls are explicitly deferred

The guide defers automated OAD lifecycle/maintainability modeling, feasibility simulation, cross-node certification recognition, automated verification, dispute workflows, and cryptographic federation integrity. These are particularly compatible with Mycelix's existing evidence, formal-assurance, federation, and productive-loop work. citeturn2view1turn2view2turn2view3

### 4. Raw evidence vs derived value is already visible in Integral's schema

The guide explicitly notes that raw LaborEvent and weighted contribution are conceptually distinct even though the Phase-2 record consolidates them. It similarly keeps MaterialConsumptionEvent as a COS-owned record while ITC consumes it. This gives us a concrete seam for evidence-preserving adapters rather than asking Integral to adopt an entirely new model. citeturn2view2turn2view3

## Cross-cutting capability matrix

| Dimension | CDS | OAD | ITC | COS | FRS | Mycelix opportunity |
|---|---|---|---|---|---|---|
| Evidence provenance/source ownership | Partial | Partial | Partial | Partial | Partial | EvidenceEnvelope + source-owned observations |
| Temporal validity/currentness | Partial | Partial | Partial | Partial | Partial | ValidityWindow, freshness, supersession |
| Uncertainty/conflict/indeterminate | Partial | Partial | Partial | Partial | Partial | explicit Unknown/Conflicting/Stale/Indeterminate |
| Claim ceilings | NotExplicit | NotExplicit | NotExplicit | NotExplicit | Partial | ClaimCeiling attached to derived claims |
| Observation → interpretation separation | Partial | Partial | Partial | Partial | Partial | typed transition graph |
| Decision → authorization → effect | Partial | Partial | Partial | Partial | Partial | authority/effect receipts |
| Capability vs qualification vs availability | Partial | Partial | Partial | Partial | Partial | ProductiveLoop + capability graph |
| Dependency provenance | Partial | Partial | Partial | Partial | Partial | dependency/replacement closure |
| Repair/replacement/reproducibility | Partial | Deferred | NotExplicit | Partial | Partial | CIV-BOOT + repair/service loops |
| Failure history / learning | Partial | Partial | Partial | Partial | Partial | immutable failure lineage + correction |
| Federation origin vs recognition | Partial | Deferred | Deferred | Partial | Partial | origin/recognition separation |
| Formal/reproducible assurance | NotExplicit | NotExplicit | NotExplicit | NotExplicit | NotExplicit | assurance manifests + proof witnesses |
| Prediction/counterfactual boundary | Partial | Deferred | Partial | Partial | Explicit/Partial | non-authoritative Symthaea analysis plane |
| Emergency bounded authority | Partial | Partial | Partial | Partial | Partial | expiring authorization + review receipt |
| Dispute/review/correction lineage | Partial | Deferred/Partial | Deferred | Partial | Partial | append/correct/supersede lineage |
| Physical/outcome anti-oracle evidence | Partial | Partial | Partial | Partial | Partial | triangulation/evidence binding |

**Important:** the table is a research baseline, not a scorecard. The classifications are intentionally conservative and should be revised as additional primary Integral specifications become public.

## High-value neutral primitives

### EvidenceEnvelope

Binds:
- source identity;
- observation identity;
- origin node;
- observed-at time;
- valid-from / valid-until;
- schema/interface generation;
- provenance lineage;
- supersession/conflict state;
- evidence class;
- claim ceiling.

### AssuranceReceipt

Binds:
- exact source commit;
- build identity;
- artifact identity;
- verifier/tool version;
- command/profile;
- result digest;
- assumptions;
- scope;
- claim ceiling.

A formal receipt never becomes production qualification merely because a solver succeeded.

### CapabilityGraph

Separates:
- capability;
- qualification;
- current availability;
- authorization;
- execution;
- outcome.

It also records imported dependencies, replacement paths, repairability, calibration continuity, and single points of failure.

### EpistemicTransition

Makes the following explicit and non-interchangeable:

`Observation -> Interpretation -> Recommendation -> Decision -> Authorization -> Effect -> Outcome`

This is particularly valuable for FRS/Symthaea outputs.

### FederationRecognition

Stores:
`origin != recognition != local observation`

A foreign claim can be recognized under an explicit contract without rewriting its origin.

## Priority adversarial corpus

Run negative cases before positive cases:

1. stale evidence accepted as current;
2. peer verification treated as physical truth;
3. FRS/Symthaea prediction written as source observation;
4. recommendation directly produces an effect;
5. OAD certification becomes production authorization;
6. capability becomes current availability;
7. one success becomes general capability;
8. later success erases earlier failure;
9. external dependency disappears from local capability;
10. foreign evidence becomes local evidence;
11. federated recognition becomes local provenance;
12. timeout becomes definite failure;
13. retry creates duplicate semantic effect;
14. correction rewrites history;
15. emergency authority survives expiry;
16. derived summary becomes source of truth;
17. weighted labor replaces raw labor evidence;
18. ledger entry becomes physical execution proof;
19. output existence becomes safety qualification;
20. throughput/share claim lacks denominator.

## Formal assurance tranche

Candidate obligations for the next engineering slice:

- ASSURE-FV-001: projection cannot strengthen evidence class;
- ASSURE-FV-002: stale evidence cannot satisfy current requirements;
- ASSURE-FV-003: interpretation/recommendation cannot become source observation without authorized transition;
- ASSURE-FV-004: decision != authorization != effect;
- ASSURE-FV-005: capability != availability != qualification;
- ASSURE-FV-006: external dependency != local capability;
- ASSURE-FV-007: foreign origin != local origin after recognition;
- ASSURE-FV-008: later success != erased historical failure;
- ASSURE-FV-009: indeterminate delivery != success/failure;
- ASSURE-FV-010: emergency authorization expires and remains reviewable;
- ASSURE-FV-011: correction/supersession preserves history;
- ASSURE-FV-012: derived projection != source-of-truth state;
- ASSURE-FV-013: formal witness != runtime/production qualification;
- ASSURE-FV-014: prediction/counterfactual != observation.

## Symthaea integration boundary

Symthaea can add:
- anomaly detection;
- forecasting;
- counterfactual analysis;
- causal/hypothesis exploration;
- formal proof artifacts where applicable;
- cross-node pattern discovery.

But every analytical result should carry explicit epistemic status and source references. Analytical output must not silently become:
- an observation;
- a certification;
- a decision;
- an authorization;
- an effect receipt.

This preserves the existing Symthaea non-authoritative analytical-plane architecture.

## Research questions still open

1. What public Integral source defines the complete semantics of certification acceptance versus production authorization?
2. What exact freshness/currentness rules will apply to cross-system data?
3. How are disputes represented without rewriting the original labor/material/decision history?
4. What is the intended federation recognition contract?
5. What constitutes sufficient physical triangulation for different evidence classes?
6. How are emergency triggers, scopes, expiry and post-event review represented?
7. How is expertise authority constrained and audited?
8. What evidence closes a capability dependency or replacement-closure claim?
9. Which of the twelve seams receive ratified technical specifications?
10. Which assurance properties belong to Integral policy versus neutral substrate?

## Nonclaims

This artifact does **not** claim:
- Integral is incomplete;
- any Integral policy choice is correct or incorrect;
- Mycelix should be selected as Integral's implementation;
- public draft material is ratified;
- synthetic conformance proves real-world productivity, safety, fairness, ecological outcomes, or deployment readiness;
- formal proofs establish social/economic outcomes.

The intended result is a portable assurance vocabulary and conformance workload that an independent Integral implementation can use while replacing the Mycelix runtime.
