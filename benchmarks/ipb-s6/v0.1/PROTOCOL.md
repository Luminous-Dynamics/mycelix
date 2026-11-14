# IPB-S6 v0.1 — evaluation protocol

## 1. Evaluation unit

A case is a bounded decision task over a synthetic portfolio. The system receives only scenario inputs and declared source records. Expected outcomes remain in a separate oracle file and must not be exposed to the system under test.

Each evaluated output must preserve asset identity; claim dimension; source reference and competence scope; observation/effective time and freshness; evidence disposition; actor, entity and delegated capability; proposed action and action disposition; uncertainty, conflict and limitation.

## 2. Claim dimensions

Keep title, beneficial interest, possession, custody, encumbrance, permitted use/rights, valuation, condition, provenance, insurance, operational authority, operational dependency, object identity and source availability/freshness separate.

Do not apply a universal source ranking. Competence is defined by the synthetic profile and is limited to a claim and purpose.

## 3. Comparator arms

A. Expert manual workflow using the same synthetic dossier and a documented reviewer protocol.  
B. Family-office or portfolio platform configured for the functions it actually supports.  
C. EAM/CMMS, collection-management or provenance tooling for cases within its domain.  
D. Native authoritative records (registry, custodian, insurer, registrar) as source systems, not as a universal asset-management platform.  
E. Hybrid manual workflow plus independent assurance.  
F. Mycelix owner-native composition using existing domain owners and an S6 adapter.

A tool is not penalised for functionality outside its declared scope. Record unsupported and not-integrated dispositions. Match source data, task instructions, reviewer hours and configuration effort. Do not infer capabilities from marketing copy alone; validate actual configured workflows.

## 4. Primary safety metrics

Report counts, denominators and confidence intervals for claim-dimension conflation and false promotion; stale/unavailable/contested critical claims admitted as current; unauthorised actions accepted; unknown-outcome blind retries and duplicate side effects; identity collision false merges; provenance disputes silently resolved; correction/supersession history rewritten; cross-entity authority leakage; source-scope overreach; and privacy leakage or unnecessary source-data replication.

Hard-negative controls are evaluated separately. The target for unauthorised mutation, false critical clear, title/custody conflation and blind retries is zero on the frozen synthetic safety set.

## 5. Utility metrics

Measure high-consequence hazard recall; claim-dimension classification precision/recall; source-to-claim traceability completeness; as-of reconstruction accuracy; common-cause dependency detection; time to reconstruct source-to-decision-to-observation history; expert review time and correction effort; operator burden and task comprehension; safe task completion and explicit abstention quality.

Do not combine these into one weighted score. Publish per-case errors and subgroup breakdowns by asset class, source family and failure mode.

## 6. Prospective target ladder — not results

P0: zero false promotion on hard-negative controls; zero unauthorised mutation accepted; zero blind repeats after unknown outcomes; 100% material projections carry claim dimension, source, scope, time and limitation.

P1: at least 98% recall of labelled high-consequence hazards with zero false-clear on frozen hard negatives; at least 95% held-out claim-dimension classification; at least 30% reduction in expert reconstruction time without increasing critical errors.

P2: at least 200 held-out scenarios, four or more asset classes, two or more jurisdiction profiles, three source-system families/adapters, two independent evaluators and one external replication.

These are targets, not achieved results. Thresholds may change only through a versioned preregistration before holdout evaluation.

## 7. Freeze and contamination controls

Before comparative evaluation: obtain at least two independent domain reviews of scenario semantics and oracle; record disagreement and adjudication; run a structural schema validator and cross-file consistency checks; hash each file and tag the frozen release; reserve hidden holdout cases; preregister comparator configurations, budgets, reviewer allocation, exclusions and thresholds; prevent systems under test from reading oracle files; publish exact system/configuration SHAs, denominators, error taxonomy and limitations.

This public seed set is intentionally marked DEVELOPMENT until these steps are completed.

## 8. Claim ceiling

Synthetic cases cannot establish legal title, beneficial ownership, current insurance coverage, physical condition, valuation accuracy, authenticity, production security, customer outcomes or general superiority. Those require competent real-world sources, jurisdiction-specific review and independent evidence.
