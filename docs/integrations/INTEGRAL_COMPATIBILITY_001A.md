# MYC-INT-001A — Integral Compatibility Contract

Status: Draft interoperability contract

## Purpose

This document defines a neutral interoperability boundary between Mycelix and Integral's five-system architecture (CDS, OAD, ITC, COS, FRS). It is intentionally not an adoption of Integral's political-economic model. The goal is to test whether Mycelix can provide reusable identity, evidence, provenance, authorization, federation, and audit primitives while Integral retains its own governance and economic semantics.

## Core boundary

Mycelix may provide:

- decentralized identity and credentials;
- signed claims, attestations, evidence references, and provenance;
- federated, append-oriented records and cross-node references;
- generic decision-lineage and review-lineage records;
- capability-scoped authorization and execution envelopes;
- generic resource, contribution, observation, outcome, and certification events;
- protocol-level interoperability among heterogeneous organizations.

Integral retains ownership of:

- CDS consensus and governance semantics;
- ITC weighting, decay, allocation, and reciprocity policy;
- COS production and labor-allocation policy;
- OAD design acceptance and certification policy;
- FRS diagnostic taxonomies and recommendation policy;
- all normative judgments about fairness, ecological limits, legitimacy, priority, and acceptable trade-offs.

Symthaea or other analytical engines may provide models, simulations, forecasts, optimizations, and recommendations. Such outputs are evidence-bearing advisory artifacts, never governance authority.

## Structural invariants

### INV-1 — Recommendation is not authorization

A `Recommendation` MUST NOT directly cause an externally consequential action. An independent `Authorization` issued by the appropriate authority is required.

### INV-2 — Prediction is not observation

Predicted, simulated, inferred, reported, and directly observed facts MUST remain distinguishable in provenance and type semantics.

### INV-3 — Evidence is not authority

An evidence object may support or contradict a proposal, finding, or decision, but possession of evidence MUST NOT imply authority to decide or execute.

### INV-4 — Governance policy is adapter-owned unless generic

Integral-specific consensus, weighting, decay, allocation, ecological, and fairness rules MUST remain outside Mycelix core types unless the same primitive is independently useful to unrelated governance/economic models.

### INV-5 — Human resolution remains representable without algorithmic substitution

Human deliberation, mediation, and adjudication outcomes may be referenced and recorded, but Mycelix MUST NOT require that such processes be reduced to algorithmic scoring.

### INV-6 — Cross-system actions use explicit seam points

No analytical or domain subsystem should directly mutate another subsystem's authoritative records. Cross-system effects flow through typed requests, acknowledgements, decisions, and authorizations.

### INV-7 — Provenance survives federation

Federated import/export MUST preserve source actor, source node, source object identity, version/lineage, timestamp or logical-order metadata, and integrity evidence sufficient to distinguish original evidence from a local copy or interpretation.

## 45-module compatibility classification

Legend:

- **A** — strong existing Mycelix substrate analogue;
- **B** — partial fit; generic primitive is useful beyond Integral;
- **C** — Integral-specific semantics; adapter/application-owned;
- **D** — analytical/engineering concern better supplied by Symthaea or a domain engine;
- **E** — deliberately kept outside algorithmic/substrate authority.

### CDS — Collaborative Decision System

| Module | Class | Mycelix boundary |
|---|---|---|
| Issue Capture & Signal Intake | A | identity-backed issue/proposal intake |
| Issue Structuring & Framing | B | generic issue/claim/dependency graph |
| Knowledge Integration & Context | B | evidence/provenance bundles |
| Norms & Constraint Checking | B/C | generic constraint-result envelope; policy semantics adapter-owned |
| Participatory Deliberation | B | arguments, objections, alternatives, amendments |
| Weighted Consensus | C | Integral owns consensus policy; Mycelix may carry ballots/credentials |
| Decision Recording & Accountability | B | decision lineage and rationale/evidence references |
| Implementation Dispatch | B | authorization envelope + capability-scoped execution |
| High-Bandwidth Human Resolution | E | preserve references/outcomes, do not algorithmically replace |
| Review, Revision & Override | B | outcome-linked review and supersession lineage |

### OAD — Open Access Design

| Module | Class | Mycelix boundary |
|---|---|---|
| Design Submission & Specification | B | evidence-bearing artifact/version records |
| Collaborative Design Workspace | B | provenance, attribution, version lineage |
| Material & Ecological Coefficients | D | analytical/domain engine |
| Lifecycle & Maintainability Modeling | D | analytical/domain engine |
| Feasibility & Constraint Simulation | D | analytical/domain engine |
| Skill & Labor-Step Decomposition | D | planning/manufacturing engine |
| Systems Integration | D | engineering systems layer |
| Optimization & Efficiency | D | analytical/domain engine |
| Validation, Certification & Release | B | generic certification/test evidence records |
| Knowledge Commons & Reuse | A/B | commons + attribution + provenance |

### ITC — Integral Time Credits

| Module | Class | Mycelix boundary |
|---|---|---|
| Labor Event Capture & Verification | B | generic contribution/work attestation |
| Skill & Context Weighting | C | Integral policy |
| Time Decay | C | Integral policy |
| Labor Forecasting | D | analytical engine |
| Access Allocation & Redemption | C | Integral application semantics |
| Inter-node Reciprocity | B/C | generic federation/equivalence evidence; policy adapter-owned |
| Fairness / Anti-Coercion Safeguards | B/C | generic finding/escalation envelope; criteria adapter-owned |
| Ledger & Auditability | A | append-oriented provenance/audit records |
| Integration & Coordination | A/B | typed bridge/event contracts |

### COS — Cooperative Organization System

| Module | Class | Mycelix boundary |
|---|---|---|
| Production Planning / WBS | D | planning/manufacturing engine |
| Labor Organization & Skill Matching | C/D | cooperative policy + planner |
| Procurement & Materials | C | supply-chain/commerce application |
| Workflow Execution | C | production application |
| Capacity & Constraint Balancing | D | analytical engine |
| Distribution & Access Flow | C | application policy |
| QA & Safety Verification | B | test/certification evidence primitives |
| Inter-Cooperative Integration | B | federation contracts |
| Transparency & Audit | A/B | event/provenance lineage |

### FRS — Feedback & Review System

| Module | Class | Mycelix boundary |
|---|---|---|
| Signal Intake & Semantic Integration | B | generic observation/signal envelope |
| Diagnostic Classification | D/C | analytical engine; taxonomy application-owned |
| Constraint Modeling & Simulation | D | analytical engine |
| Recommendation Routing | B | non-executive recommendation primitive |
| Democratic Sensemaking Interface | B/D | evidence UI + analytical explanation |
| Longitudinal Memory | B | outcome/evidence lineage |
| Federated Intelligence & Learning | B | federated evidence/model exchange |

## Generic primitives proposed for follow-up

The following names are provisional and MUST be checked against existing Mycelix types before implementation:

- `ActorRef`
- `NodeRef`
- `ArtifactRef`
- `Claim`
- `EvidenceRef`
- `Attestation`
- `Issue`
- `IssueFrame`
- `Position`
- `Objection`
- `Alternative`
- `Decision`
- `Recommendation`
- `Authorization`
- `Observation`
- `Outcome`
- `ReviewTrigger`
- `Certification`
- `ResourceEvent`
- `ContributionEvent`

## Minimal proof-of-neutrality scenario

The first adapter experiment SHOULD model a water-system maintenance problem:

1. an issue is opened by an authenticated actor;
2. observations and evidence are attached;
3. competing claims and alternatives are recorded;
4. at least one objection targets a specific claim or alternative;
5. an analytical engine produces a recommendation with explicit evidence/model references;
6. the recommendation has no execution authority;
7. CDS (or another governance process) produces a decision;
8. a separate authorization permits a bounded implementation action;
9. observations capture the resulting outcome;
10. review compares outcome against the original assumptions and may supersede the decision.

The proof succeeds only if the Mycelix substrate can answer, without knowing Integral's political-economic rules:

- who asserted each claim;
- what evidence supported it;
- which parts were observed, inferred, simulated, or predicted;
- who possessed authority to decide;
- who possessed authority to execute;
- what action was actually taken;
- what outcome occurred;
- whether later evidence contradicted assumptions or justified review.

## Explicit non-goals

This tranche MUST NOT:

- implement ITC economics in Mycelix core;
- encode Integral consensus policy in generic governance types;
- grant Symthaea, FRS, or any recommender execution authority;
- treat algorithmic confidence as legitimacy or voting weight;
- collapse identity, trust, expertise, reputation, stake, and authority into one scalar;
- assume a single governance model for municipalities, cooperatives, companies, households, or federations;
- claim production readiness from schema compatibility alone.

## Follow-up tranche

1. **MYC-EVID-002A** — inventory existing Mycelix evidence/claim/attestation types and define the smallest generic evidence vocabulary without duplication.
2. **MYC-GOV-003A** — deliberation graph: issue → claim → evidence → objection/alternative.
3. **MYC-GOV-003B** — decision → implementation → observation/outcome → review/supersession lineage.
4. **MYC-AUTH-004A** — structural separation of recommendation, decision, authorization, and execution receipt.
5. **MYC-INT-005A** — adapter proof using the water-system scenario.

## Qualification expectations

This document is an architectural contract only. It does not establish executable compatibility. Follow-up PRs must each state which invariants are represented structurally, which are only documented, and which remain unverified.
