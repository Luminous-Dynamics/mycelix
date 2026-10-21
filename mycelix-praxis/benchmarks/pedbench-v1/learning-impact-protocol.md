# Praxis Learning Impact Protocol v1

**Status:** proposed protocol for educator and research review  
**Purpose:** stop benchmark performance, learner satisfaction, and immediate aided correctness from being mistaken for durable learning. This protocol is not ethics approval or evidence that Praxis currently improves outcomes.

## 1. Four distinct evidence layers

Report the following layers separately. A release decision must not replace a missing layer with a stronger-looking score from another one.

| Layer | Evidence | Permitted claim |
|---|---|---|
| A — Content validity | Exact-content digest, standards/source mapping, answer-key checks, educator/rights/accessibility review | The named asset passed the listed content checks/reviews |
| B — Tutor behaviour | Held-out PedBench and compatible external benchmark tasks, dual-annotated cases, uncertainty | The named configuration achieved the reported response-quality results |
| C — Learner outcomes | Pre-registered study with fair comparator, independent assessments, delayed unaided retention and transfer | The named intervention changed the measured learner outcome in the studied population/context |
| D — Replication and transport | Independent replication across materially different course, population, language or deployment conditions | The effect replicated across the specifically listed contexts |

A digest binds evidence to bytes; it does not validate the judgement. A benchmark score describes responses on benchmark cases; it does not establish that a learner learned. A randomized study can support causal claims only when assignment integrity, implementation, attrition, analysis and interference assumptions justify them. Individual learner state should use the private [Learner Mastery Evidence v1 contract](../../schemas/learner-mastery-evidence-v1.schema.json): the `mastered` state requires an independently scored, unaided transfer observation and an independently scored, unaided delayed-retention observation, at least three evidence records, a decision rationale, and a scheduled review. Do not put this per-learner record in the public source repository; publish only privacy-reviewed aggregate study outcomes.

## 2. Default outcome hierarchy

For a confirmatory study, select **one primary endpoint and one primary estimand** before looking at outcomes. The default proposed endpoint is:

**Unaided delayed near-transfer performance at approximately 7 days after the last intervention session**, on independently authored or securely held-out items that require the same capability but use unfamiliar surface details, measured without tutor assistance.

Where feasible, pre-specify a 28-day delayed assessment as a key secondary outcome. Also report:

- immediate unaided performance, separate from performance with hints;
- retention on practised items and transfer to unpractised items;
- misconception recurrence and correctness of tutor feedback;
- error recovery: probability the next attempt is correct and attempts required after an error;
- time and practice opportunities to demonstrated competency;
- assessment completion, attrition, missingness and protocol deviations;
- confidence calibration, learner burden and age-appropriate well-being measures;
- language, disability/access mode, grade/prior-knowledge, device and connectivity outcomes where consent, sample size and privacy allow;
- educator/caregiver workload and **cost per learner who demonstrates the target capability**, not cost per generated lesson or completed chat.

Do not silently drop learners who stop using the tutor. Report intention-to-treat when randomization supports it, with justified sensitivity analyses for missing data and uptake.

## 3. Aided-to-unaided transfer gap

Record assisted and independent performance as separate quantities. When both measures use a defensibly comparable scale, compute:

**Aided-to-Unaided Gap (AUG) = immediate aided performance − delayed unaided transfer performance.**

Report the two component scores, AUG, uncertainty and the assessment-scale limitations. A smaller gap is not necessarily better if both scores are low; pair it with delayed transfer itself. The metric is diagnostic, not a replacement for a pre-specified primary endpoint.

This exposes a central failure mode: a tutor can produce a high rate of correct answers during sessions by over-scaffolding, revealing answers or doing the reasoning, yet produce little independent capability afterward.

## 4. Comparator design

Where feasible, compare equivalent objectives and learner populations across:

1. **A strong non-AI baseline:** the existing lesson/courseware plus equivalent practice opportunities, schedule and assessment.
2. **Praxis intervention:** fixed, versioned content and assistance policy with logged deviations.
3. **Human-supported condition:** teacher/tutor instruction or a well-defined blended design, when practical and ethically appropriate.

Do not claim the tutor is better than teachers from a comparison against a weak worksheet-only baseline. Do not compare unequal instructional time or help availability without reporting those differences. If studying add-on use, explicitly track whether AI displaces teacher contact, peer interaction, independent attempts or existing effective practice.

Use cluster randomization when classroom spillovers are likely. Freeze the assignment plan, assessment instrument, primary estimand, practically important effect threshold, power/precision assumptions, multiplicity strategy and stopping rule in the preregistration.

## 5. Safety and release gates

Proposed defaults to be reviewed for each study and age group:

- No core release with an unresolved critical content, source-rights, privacy or accessibility defect.
- No tutor-quality score can override a factual/answer-key veto.
- No mastery state based on one heavily assisted answer; require pre-defined evidence from independent checks and transfer items.
- No effectiveness claim from telemetry, completion, satisfaction or next-item correctness alone.
- Report null results and negative outcomes, including material reductions in independent attempts or human-support access.
- Keep child data private by default; obtain jurisdiction-appropriate permissions, consent/assent and research review. Publish approved aggregates, not identifiable learner histories.

## 6. Reporting and replication

Freeze and publish, where lawful:

- protocol, lesson and assessment digests;
- intervention/model version and exact assistance policy;
- assignment, exposure and implementation fidelity;
- sample flow, baseline balance, attrition, missingness and deviations;
- primary outcome estimate, confidence interval, practical-importance threshold and all pre-specified secondary/guardrail outcomes;
- subgroup/equity estimates with privacy protections;
- cost, workload, limitations and analysis code or an independently reproducible specification.

A benchmark improvement is **Layer B** evidence. A pilot without a defensible comparator is exploratory. A causal statement requires an adequately designed and analyzed study. Broad claims require replication in materially different contexts; do not infer national or global effectiveness from a single grade, school, language or course.

## 7. Proposed decision thresholds

These are protocol-design defaults, not empirical claims or universal educational cut-offs:

- Choose a minimum practically important difference with educators and subject experts before data collection.
- Advance to a larger study only if content/tutor safety gates pass and the pilot demonstrates feasible recruitment, assessment completion and accessible delivery.
- Claim a causal improvement only when the pre-specified analysis supports it, uncertainty excludes effects considered educationally trivial under the registered decision rule, guardrails show no material harm, and implementation/fidelity is documented.
- Require at least two materially different contexts for broad efficacy language, with independent analysis and all material regressions disclosed.

The objective is not to maximize a single aggregate score. It is to maximize **independent, durable, equitable capability per unit of learner effort and social cost**, while preserving dignity, agency, privacy and teacher/learner control.
