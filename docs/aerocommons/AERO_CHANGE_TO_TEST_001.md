# AeroCommons change-to-test qualification — v1

Status: synthetic research contract; proposal-only  
Tracker: https://github.com/Luminous-Dynamics/mycelix/issues/6350

## Purpose

Turn a declared configuration change into a traceable evidence-impact closure, then rank optional experiments that could improve the next engineering decision.

This is deliberately two-stage:

1. **Mandatory evidence impact:** consume the canonical AeroEvidence ChangeSet and impact classifications. Required invalidation/review obligations are not negotiable.
2. **Optional decision-value planning:** only after mandatory obligations have been frozen, rank admissible additional experiments by expected decision improvement and total cost.

A cost optimizer must never remove a required requalification obligation.

## Canonical composition

- AeroEvidenceV1, ChangeSetV1, EvidenceImpact and `classify_impact` remain the source of truth for structural impact.
- AP242/QIF adapters provide semantic feature/requirement/process/instrument identity through the semantic-interoperability contract.
- ComponentBuildPacketV1 provides serialized-unit/build/inspection/test references.
- Symtropy AERO-VOI-001 #2890 owns the generic aerospace decision-aware experiment-selection method.
- Symtropy AERO-INSPECTION-OBSERVABILITY-001 #4165 owns inspectability/NDE and sensing-method context.
- Symtropy AEROWEFT Tracks T/U/V/W and utility calibration #5510 own model applicability, change requalification, candidate admissibility and utility calibration.
- Mycelix owns the downstream proposal/receipt binding and federated evidence lineage.

No second evidence ontology, change-impact classifier, OED engine, QIF parser, inspection solver or release authority is introduced.

## Four non-substitutable outcomes

- `MANDATORY`: a direct/transitive evidence dependency or controlling requirement was changed. Required requalification must be retained.
- `CONDITIONAL_REVIEW`: a declared applicability rule or qualified reviewer must adjudicate.
- `UNKNOWN`: missing/ambiguous dependency coverage, source rights, subject identity, temporal scope or semantic mapping. Unknown cannot become unaffected.
- `OPTIONAL_EXPERIMENT`: new evidence could improve a decision but is not a substitute for mandatory obligations.

A fifth state, `UNAFFECTED`, may be emitted only when dependency coverage is sufficient and the canonical impact owner supports that classification.

## Required plan contents

A plan binds:
- exact predecessor and proposed configuration identities;
- exact ChangeSet identity and recomputed SHA-256 over canonical versioned JSON (sorted reference arrays);
- bitemporal evaluation cutoffs and source sequence/lineage;
- exact evidence records and typed dependency paths;
- expected canonical impact for each record;
- mandatory, conditional and unknown obligations;
- optional candidate tests and exact admissibility receipts;
- estimator/method identity, confidence and calibration status for each decision-value estimate;
- resource cost including setup, instrumentation, operator/facility, test, data adjudication, compute and teardown;
- deterministic ranking/tie-break policy;
- confirmation that no optional policy has modified the mandatory obligation set.

## Decision-value semantics

The planner should optimize improvement in the *next declared engineering decision*, not generic information gain.

Illustrative decision questions:
- Does this process revision preserve dimensional capability for this feature family?
- Does changing a joint/material/process alter which inspection method can detect the declared defect family?
- Which additional coupon test would change the choice among admissible process or inspection alternatives?
- Does a sensor-layout experiment change the design-for-inspection trade-off?

If probabilities are uncalibrated, label the estimate as a proxy and do not describe it as calibrated expected value. Use a finite exact oracle on small cases and held-out regret on larger ones.

Mandatory requalification remains a hard gate and must be completed before an optional acquisition policy can claim a new configuration is adequately evidenced.

## Executable adapter

The isolated Rust crate at `tools/aero-change-to-test` consumes canonical impact classifications and checks that declared obligations match them; it does not implement another dependency classifier. It requires non-demotable obligations, explicit dependency coverage and reason paths. Optional candidates are filtered by all five admissibility gates. Only candidates with calibrated value bounds are scored by conservative lower bound per total cost, and only after obligation closure is structurally recorded.

The synthetic fixture includes four evidence records, three OPEN mandatory obligations and two optional candidates. All optional candidates remain blocked in the checked-in baseline. The 15 authored Rust tests cover mandatory preservation, unknown impact, closure receipts, candidate admissibility, calibrated ranking, mutation resistance and input-order invariance. These are authored tests; they are not evidence of an executed Rust pass.

## Benchmark protocol

Compare:
1. retest-everything;
2. direct-dependency invalidation;
3. typed transitive dependency closure;
4. manual expert checklist;
5. mandatory closure plus decision-value optional-test ranking;
6. independent exact finite-graph oracle.

Score:
- mandatory-impact recall;
- false-unaffected count (hard gate);
- wrong-subject/revision joins;
- false invalidation / over-test burden;
- unknown/unsupported rate;
- reason-path trace completeness;
- mandatory-obligation preservation under optimizer mutations;
- optional test cost, decision flip rate, held-out regret and budget usage;
- p50/p95 plan-generation latency and graph size.

## Frozen synthetic scenario

`fixtures/aero-change-to-test/change-to-test-v1.json` uses a synthetic change to a process plan, a serialized-unit build record, a downstream inspection record and an optional NDE candidate. It is not a real aircraft change, manufacturing qualification or authorization.

## Proposed acceptance targets

- 100% mandatory-impact recall against an independently enumerated finite oracle.
- Zero direct/transitive changed dependencies labelled unaffected.
- Zero mandatory obligations removed or demoted by optional cost/utility ranking.
- 100% traceable reason paths from ChangeSet to affected evidence to obligation.
- Deterministic replay under input permutation and as-of cutoffs.
- 100% abstention/unknown preservation when a dependency edge, semantic map or evidence scope is missing.
- Decision-value superiority claims only on sealed held-out cases with matched total cost and calibrated estimators where probabilities are used.

These are prospective software/benchmark targets, not achieved product results or regulatory acceptance criteria.

## Claim ceiling

A pass establishes only that the change-impact and test-planning semantics are correct on the frozen synthetic graph and declared adapters. It does not assess physical adequacy, approve repairs, accept parts, authorize inspection substitutions, establish certification/airworthiness or authorize tests or flight.
