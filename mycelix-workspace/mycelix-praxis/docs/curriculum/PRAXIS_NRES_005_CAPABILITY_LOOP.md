# PRAXIS-NRES-005: Exercise-to-improvement capability loop

**Status:** synthetic benchmark specification for review. No real exercise, municipal capability, or educational effect has been measured.  
**Fixture:** `examples/courses/civic-resilience-capability-loop-fixture-001.json`.  
**Composes:** PRAXIS-NRES-001 through NRES-004, the existing Praxis content/privacy contracts, and the external practices below.

## 1. Research shift: from assessed learning to verified improvement

The previous benchmark measured what a learner can explain and produce in a scenario. That is necessary but not sufficient to claim that a programme improves resilience.

NRES-005 adds an explicit loop:

**Objective → exercise → observation → evidence → after-action finding → corrective action → independent verification → retest → revised learning.**

This does not make Praxis an emergency management authority. It makes the learning record connect to a structured improvement process while leaving decisions and official exercise governance with authorized institutions.

## 2. Compose with established practice; do not claim to replace it

| Existing practice | What it already provides | Praxis composition |
|---|---|---|
| FEMA HSEEP | Exercise design, conduct, evaluation, capability objectives, after-action reporting and improvement planning | Stable objective IDs, learner-level evidence, corrective-action references and repeatable assessment data |
| IFRC Preparedness Framework / PER | Preparedness assessment, prioritisation, work planning, action and accountability | A learning loop that teaches the process and records a transparent, reviewable practice exercise |
| ISO 22301 | Organizational business-continuity management and continual improvement | Explicit continuity measures, evidence, unresolved dependencies and retest history |
| UNDRR 2026–2030 | Risk knowledge, locally led DRR, financing and recovery readiness | Local ownership, inclusive measurement and a traceable risk-to-learning pathway |
| UNESCO DARE | Describe, appraise, redesign and evaluate assessment practice | Iterative revision of learning assessments based on observed performance, not merely a terminal grade |

These are complementary reference systems. NRES-005 is a prototype bridge between learning evidence and improvement records, not a substitute, compliance claim, certification, or endorsement by any named body.

## 3. Three measurement layers — keep them separate

### A. Learner competence

Measure factual knowledge, uncertainty classification, evidence handling, rights/authority understanding, resource reasoning, delayed retention and performance on novel held-out learning tasks. Keep pre/post, 30-day, 90-day and transfer outcomes visible separately.

### B. Exercise capability

Measure whether explicitly defined exercise objectives were evidenced; whether each minimum service floor was reached; whether each resource pool stayed within its own limit; whether authority records and accessibility checks were complete; and whether evidence was independently reviewed.

### C. Improvement-system effectiveness

Measure whether after-action findings produce assigned and appropriately prioritized corrective actions; whether actions have accountable role owners and due dates; whether completion evidence is independently verified; whether the capability passes a repeat exercise; and whether the same failure recurs.

A course can improve learning without improving an institution's operational capability. An organization can improve one exercise result without proving the course caused the improvement. A service floor can pass in a tabletop while failing in the real world. Do not conflate these outcomes.

## 4. Non-compensable performance rules

- Every critical service floor is reported individually. The overall critical-floor condition is a conjunction, never an average.
- Budget and trained responder-hour limits are separate constraints.
- A legal-authority evidence check measures documentation completeness only; a qualified legal reviewer determines legal questions.
- Accessibility coverage evaluates service-design requirements, not participant identities or sensitive traits.
- A corrective action is verified closed only when completion evidence exists, an independent reviewer verifies it, and a retest passes.
- An untested condition is unknown, not passed. An open action is not silently removed from the denominator.
- Never collapse the above into one composite score, leaderboard, or personal suitability score.

## 5. Measure definitions

NRES-005's machine-readable measure catalog specifies numerator, denominator, unit, formula and guardrail for:

1. Objective evidence coverage.
2. Per-service minimum-floor attainment.
3. Separate budget and responder-hour utilization.
4. Authority-evidence completeness.
5. Accessibility-check coverage.
6. Corrective-action verification rate.
7. Retested failure recurrence.
8. Learner retention and transfer.
9. Fully loaded cost per independently verified improvement.

For each proportion report numerator, denominator, percentage, missingness and evidence reference. For service floors report required and observed values for each service. For time-to-restore measures, only report when the exercise defines a start event, restoration event, service threshold and measurement method before the exercise.

## 6. Repeated-cycle evaluation protocol

1. Freeze the exercise objectives, versioned scenario, service-floor definitions, resource constraints, evidence rules and evaluation guides before conduct.
2. Observe against objectives using consistent evaluator records. Every observation must point to evidence and distinguish synthetic fact, assumption, estimate and unknown.
3. Produce an after-action finding with impact, evidence, root-cause hypothesis, uncertainty and affected objective. A hypothesis is not a verified cause.
4. Assign a corrective action to an accountable institutional role, with due date, expected completion evidence and an independent verification path.
5. Run a comparable retest. Record scenario changes; do not attribute an improvement to the intervention if the second test is easier or has materially different constraints without a sensitivity analysis.
6. Mark an action closed only after completion evidence, independent review and successful retest. Record failed, late, blocked, not-yet-tested and reopened actions explicitly.
7. Publish the improvement record with source, scenario and rubric digests; preserve negative findings and denominators.

## 7. Public development fixture

The paired-cycle fixture uses the existing fictional 72-hour municipal disruption and the exact service floors from NRES-002. Both cycles and all example observations are synthetic development data. They demonstrate how to calculate metrics and trace actions; they do not estimate any real municipality's performance.

The illustrative second cycle moves three of four service floors above threshold but leaves health-logistics continuity below its floor. Two corrective actions are marked verified after independent evidence and retest; one remains open. This deliberately demonstrates why the system must not declare overall success or hide a critical unresolved condition behind an improving average.

Because the fixture is public, it is not a held-out evaluation set. Confirmatory cases require independent custody under NRES-004.

## 8. Benchmark targets and comparisons

Do not invent universal targets for readiness percentages or corrective-action closure. Establish thresholds from the institution's authorized capability targets, local risk profile and service obligations before confirmatory use. For pilot work, report the raw measures and use them to estimate feasibility, variance and evaluator reliability.

Proposed hard data-integrity gates:
- 100% of scheduled objectives receive an evidence-backed result, an explicit missingness reason or an invalid disposition.
- 100% of critical service floors are reported individually.
- 100% of resource pools are reconciled independently.
- 100% of actions marked verified-closed have completion evidence, independent verification and a successful retest.
- 100% of scheduled corrective actions have a visible disposition; untested or overdue actions remain visible.
- 0 accepted reports use a synthetic fixture as proof of real-world readiness or legal authority.

These are integrity gates for the record, not targets for community service performance.

## 9. Industry comparison and where Praxis can add value

HSEEP already supplies mature exercise objectives, evaluator guides, after-action reporting and improvement planning. IFRC PER already supplies a locally oriented preparedness assessment-to-accountability process. ISO 22301 already supplies a business-continuity management-system standard. Praxis should not claim it has invented these.

The proposed differentiation is the open, privacy-aware linkage between:
- a learner's versioned course and competency evidence;
- the exercise objective and exact evidence artifact;
- the finding, corrective-action lifecycle and independent retest;
- the cost and access burden of the learning programme; and
- the next curriculum revision.

This is valuable only if it improves auditability, transfer and follow-through without creating a surveillance or coercive scoring system. Test it against existing HSEEP/IFRC-style records, not only against a weak bespoke baseline.

## 10. Reference implementation

A standalone Rust reference evaluator lives in `benchmarks/nres-capability-loop/`. It parses the public NRES-002 and NRES-005 fixtures, validates evidence-linked cycle records and action closure, and emits a machine-readable report containing separate measures rather than a composite score.

Run from the `mycelix-praxis` directory:

```bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml
```

The crate is isolated in its own Cargo workspace and is not part of the production application. Tests are authored but have not been executed in the current environment.

## 11. Implementation boundary

- The course and all NRES-005 fixtures remain inactive in the production curriculum graph.
- Do not collect participant political beliefs, enlistment interest, loyalty, health status or personal suitability.
- Keep learner records local-only by default; no federated learning for this programme.
- Do not treat exercise data as live incident telemetry or issue operational tasking.
- Official service thresholds, authority and real exercise participation require the responsible institution's approval.
- A synthetic result, Praxis credential or model-generated recommendation confers no legal authority or professional qualification.

## 12. Sources

- FEMA, [Homeland Security Exercise and Evaluation Program](https://training.fema.gov/programs/nsec/hseep/) and [Exercise Evaluation Guides](https://preptoolkit.fema.gov/web/hseep-resources/eegs).
- IFRC, [National Society Preparedness Framework](https://www.ifrc.org/document/ifrc-national-society-preparedness-framework).
- ISO, [ISO 22301:2019 Business continuity management systems](https://www.iso.org/standard/75106.html).
- UNDRR, [Strategic Framework 2026–2030](https://www.undrr.org/strategic-framework-2026-2030).
- UNESCO, [Assessing learning in education for sustainable development](https://www.unesco.org/en/articles/assessing-learning-education-sustainable-development).

The sources guide the design crosswalk; they do not validate this prototype or endorse Praxis.
