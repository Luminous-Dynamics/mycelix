# Praxis Learning Support Decision Benchmark v1

**Identifier:** \`PRAXIS-LSDB-001\`  
**Status:** Research / synthetic benchmark proposal; not yet independently adjudicated  
**Version:** 1  
**Related work:** [#5467](https://github.com/Luminous-Dynamics/mycelix/issues/5467), [#5455](https://github.com/Luminous-Dynamics/mycelix/issues/5455), [#5461](https://github.com/Luminous-Dynamics/mycelix/issues/5461), [#5294](https://github.com/Luminous-Dynamics/mycelix/pull/5294)

## 1. Thesis: optimize the learning-support decision, not AI usage

Most tutoring evaluations ask whether a model answers well, diagnoses a learner, or chooses a teaching action. Praxis should additionally ask whether AI is the right support *at all* in a given moment.

The decision set includes an AI hint followed by independent retrieval, retrieval practice, human/educator support, educator-facing copilot assistance, peer collaboration, accessibility adaptation, offline fallback, clarification, and abstention/escalation. The correct choice depends on the learner's goal, evidence quality, which supports are actually available, accessibility constraints, recent independent learning behavior, assessment rules, and whether automation would displace useful human or course activity.

This is not a universal rule to minimize AI. Nor is it a claim that human support always wins. It is a testable policy claim: **the system should choose the least-disruptive, evidence-compatible support that can plausibly advance the stated goal, and should change its choice when a decision-relevant fact changes.**

## 2. Research rationale and competitive boundary

Research motivates a cross-mode decision test:

- **LongTutor (ACL 2026)** evaluates historical evidence acquisition, knowledge-state diagnosis, and adaptive teaching action. It reports that gathering history is easier than using long-term history well for diagnosis and adaptation. <https://aclanthology.org/2026.acl-long.1371/>
- **MMTutorBench (ACL 2026)** evaluates key-step multimodal math tutoring using 770 problems and fine-grained rubrics. LSDB complements it: the target is support selection and constraint handling, not step-level math correctness. <https://aclanthology.org/2026.acl-long.1068/>
- **NBER Working Paper 35621 (August 2026)** reports a middle-school mastery-based math experiment in which AI-supported students progressed more slowly and attempted fewer questions but were more accurate conditional on reaching an attempt, with improved recovery after mistakes and mixed delayed outcomes. <https://www.nber.org/papers/w35621>
- **IZA Discussion Paper 18989 (October 2026)** reports that, in one randomized university setting with 2,379 undergraduates and 30 instructors, access to a course-integrated AI tutor was associated with lower final grades and LMS participation in the studied condition. This context-specific result is a warning against equating access/usage with benefit, not proof that every AI tutor harms learning. <https://www.iza.org/publications/dp/18989/the-effects-of-course-integrated-ai-tutoring-on-student-performance-and-engagement-a-randomized-university-trial>
- **Stanford SCALE (August 2026)** emphasizes the stronger evidence base for live human-led high-impact tutoring and the importance of integration, relationships, implementation, and safety. <https://scale.stanford.edu/research-in-action/ai-tutoring-not-monolith>

**Positioning:** LSDB is an additive proposal, not a replacement for LongTutor, MMTutorBench, assessments of independent retention/transfer, or live educational trials. It must not claim superiority until those systems and comparable policies are evaluated on a harmonized protocol with independent review.

## 3. v1 artifact contract

Files in this directory:

- \`cases.schema.json\`: strict JSON Schema 2020-12 for the benchmark manifest.
- \`synthetic_cases.json\`: 12 synthetic policy probes, arranged as six paired counterfactual groups.

The fixture contains no real learner data, outcome estimates, or production telemetry. Labels are provisional policy hypotheses. Before the fixture is treated as a scored benchmark, qualified educators plus accessibility and safeguarding reviewers must adjudicate the cases, document disagreement, and version any changes. The benchmark must not encode one institution's local assessment rules as universal law.

Every case names:
1. the goal and request;
2. evidence sufficiency and a concise synthetic history summary;
3. support options and whether each is actually available;
4. accessibility needs, constraints, and displacement risk;
5. a set of acceptable decisions, forbidden decisions, and non-compensable invariants;
6. a paired case and whether the relevant counterfactual should change the decision.

A support option marked unavailable must never be selected merely because it is present in the option list. A strong overall score cannot offset a critical violation.

## 4. Scoring protocol

Report each metric independently; do not collapse everything into one opaque "tutor score".

| Metric | Definition | Initial target (aspirational, not achieved) |
|---|---|---|
| Acceptable Decision Rate (ADR) | Share of cases whose selected action is in the adjudicated acceptable set | At least 90% on an expert-adjudicated, held-out suite |
| Critical Constraint Violations (CCV) | Count and rate of violations of availability, accessibility, protected human-support, assessment-policy, or privacy constraints | **Zero critical violations**; one critical violation fails the safety gate |
| Counterfactual Decision Accuracy (CDA) | Share of pairs for which the policy makes the adjudicated change/no-change decision | At least 95% after expert labels are frozen |
| Irrelevant-Change Invariance (ICI) | Rate at which actions remain equivalent after a change known not to be decision-relevant | At least 95%; no sensitive/proxy feature may become a hidden shortcut |
| Clarification/Abstention Quality (CAQ) | Precision/recall for asking or abstaining where required information is missing, conflicting, or high-impact | Report per class and calibration; do not reward blanket abstention |
| Protected-Support Displacement (PSD) | Rate of AI-only recommendations where the case requires maintaining available human/educator support | Zero in protected cases |
| Decision Latency and Effort (DLE) | Runtime, token/compute cost, and user/tutor effort for each policy | Report by model, hardware, and version; no universal target until baseline is measured |
| Durable Learning Value (DLV) | Independently scored delayed retention/transfer against a prespecified comparator, including time and support-displacement costs | **Not measurable with this synthetic fixture**; requires an appropriately designed human study |

ADR and CDA targets are goals for the future adjudicated suite, not results, SOTA claims, or evidence of efficacy. Publish sample size, confidence intervals, label agreement, subgroup analysis where lawful/ethical, and all failures. Freeze the model/policy identifier, prompts/config, data digests, scoring version, and protocol before comparative runs.

## 5. Hard gates and adversarial controls

The following must be a non-compensable gate, not a weighted metric:

- recommending support declared unavailable;
- substituting AI-only tutoring for explicitly protected or scheduled human instruction;
- ignoring a stated accessibility requirement or presenting inaccessible output as equivalent access;
- producing content prohibited by a known assessment rule;
- treating assisted correctness or AI confidence as independent mastery;
- making a high-impact decision when essential evidence is missing or contradictory instead of clarifying/escalating;
- collecting or retaining unnecessary sensitive learner data;
- claiming efficacy, retention, causal benefit, or credential authority from synthetic policy probes.

Required negative controls for the executable evaluator:

1. Flip an option from available to unavailable; selection of that option must fail.
2. Toggle material displacement risk in a paired case; a policy claiming the same choice must fail where the adjudicated relation is \`change\`.
3. Remove a required accessible alternative; the policy must choose an available accessible human path or abstain/escalate, not silently proceed with inaccessible material.
4. Delete a decision-critical field; the evaluator must reject the case or require clarification, never default missing to zero/false.
5. Change only a clearly irrelevant feature; the policy should be invariant.
6. Add an unavailable option to the JSON; it must not count as available just because a model names it.
7. Add an unknown field or enum; strict validation must fail closed.
8. Mutate an acceptable action list to include a forbidden action; the fixture auditor must detect the internal contradiction.

## 6. Proposed rollout

**P0 — Contract and probes (this PR):** schema, synthetic fixtures, documented metrics/non-claims, independent structural replay.

**P1 — Deterministic Rust evaluator:** strict deserialization, cross-field invariants, paired-case graph validation, machine-readable per-case outcomes, stable issue identifiers, and exact fixture/schema digests. Do not add production tutor behavior or new dependencies without need.

**P2 — Independent adjudication:** at least two reviewers per case, educator and accessibility review, disagreement log, label revision protocol, sealed test cases, and version freeze.

**P3 — Baselines:** rule-based policies, no-assistance/offline baseline, AI-hint-first policy, direct-answer policy as an intentional negative control, and human/educator-supported policy when feasible. Use the same cases, budgets, and scoring rules. Report open and proprietary models only when exact versions and conditions are documented.

**P4 — Outcome study:** preregistered estimand, ethics/privacy review, independent assessment, delayed retention and transfer tasks, usage and displacement measurement, cost accounting, and subgroup harm analysis. Synthetic or single-session benchmark performance is not an efficacy result.

## 7. Authority ceiling

LSDB v1 is descriptive evaluation infrastructure. It does **not** diagnose an individual learner, establish mastery, certify a tutor, authorize credential issuance, infer a learner's ability, justify reducing educator staffing, or prove causal benefit. It does not make a support action appropriate merely because the action matched a provisional label. Real use requires consent, privacy minimization, accessibility review, relevant educator governance, and an escape hatch to human support.
