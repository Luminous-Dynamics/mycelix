# Praxis-PedBench v1: Pedagogical Ability Benchmark Specification

**Status:** proposal for implementation and educator co-design  
**Owner:** Mycelix Praxis  
**Version:** 1.0-draft  
**Scope:** evaluation of lesson assets and tutor responses, not certification of a whole curriculum or proof of learner-outcome impact.

## 1. Why this benchmark exists

A model can solve a mathematics problem yet fail to teach it. It can also generate fluent, plausible text that ignores a learner's misconception, gives away the answer, or reinforces an incorrect rule. Praxis needs an evaluation set that measures teaching behavior rather than general language fluency or subject-answer accuracy alone.

Two useful public comparison points are:

- **MathTutorBench (EMNLP 2025)** is an open benchmark for holistic evaluation of open-ended math tutoring. It explicitly treats subject-solving ability and teaching ability as different capabilities. <https://aclanthology.org/2025.emnlp-main.11/>
- **The BEA 2025 Shared Task on Pedagogical Ability Assessment of AI Tutors** evaluates misconception identification, mistake localization, guidance, feedback actionability, and tutor identity against human annotations. The best scores on the four pedagogical assessment tracks ranged from 58.34 macro-F1 for guidance to 71.81 for mistake identification in three-class tasks—evidence that those abilities remain difficult, not numbers that Praxis can compare directly without matching the task and protocol. <https://aclanthology.org/2025.bea-1.77/>

Praxis-PedBench should reuse compatible concepts from these works, document any differences, and avoid presenting one scalar score as proof of pedagogical quality.

## 2. Evaluation unit and record

The primary record is a **learner state + objective + response opportunity**, bound to a versioned objective and content digest. For example:

- objective: identify equivalent fractions for equal-sized wholes;
- learner state: says (1/4) is larger than (1/2) because 4 is larger than 2;
- tutor opportunity: respond without simply revealing the answer;
- reference annotations: likely misconception, acceptable next-step guidance, prohibited misleading claims, and answer-leakage threshold.

Store synthetic, privacy-safe cases by default. Real learner conversations require independent ethics, consent/assent, minimization and privacy review before inclusion. Never publish student identities, account IDs, raw histories, or small identifiable groups.

Required record fields in the eventual versioned JSON contract:
- `case_id`, `case_version`, `domain`, `grade_or_course_level`, `language`;
- `objective_ref`, `objective_digest`, `case_digest`;
- `learner_state`, `prompt`, `response_under_test`;
- `expected_behaviours`, `prohibited_behaviours`, `rubric_version`;
- independent annotation decisions, adjudication status, and known limitations.

## 3. Pedagogical dimensions

Score each dimension separately. Do not collapse a safety/accuracy veto into a high average score.

| Dimension | Question | Example failure |
|---|---|---|
| Objective alignment | Does the response address the declared capability? | Explains the topic generally but never addresses the target skill. |
| Error localization | Does it identify the actual step or assumption that failed? | Says “try again” without pointing to the invalid transformation. |
| Misconception diagnosis | Does it distinguish an accidental slip from a conceptual misunderstanding? | Treats a place-value misunderstanding as a careless arithmetic error. |
| Guidance actionability | Is the next step specific enough to help, but not needlessly prescriptive? | Gives vague encouragement without a usable next action. |
| Answer leakage | Does help reveal more of the solution than the configured help level permits? | Gives the final answer when a first-step hint was requested. |
| Explanation validity | Are claims and worked steps correct and internally consistent? | The answer is correct but the written steps imply a false rule. |
| Cognitive engagement | Does it require the learner to reason or retrieve at the appropriate moment? | Performs the entire task for the learner and asks only for confirmation. |
| Calibration and escalation | Does the tutor acknowledge uncertainty or request clarification when necessary? | Confidently grades an ambiguous response without asking. |
| Accessibility and register | Is the response comprehensible and usable for the intended age, language and access mode? | Relies on an unexplained symbol or inaccessible visual-only instruction. |
| Learner agency | Does it offer a meaningful next move and preserve learner choice? | Coerces, shames, or locks the learner into a single path without justification. |

Some dimensions depend on context. For example, answer disclosure is appropriate after a completed attempt or in a worked example. Each case must specify the permitted assistance stage.

## 4. Case construction

Build a balanced, stratified corpus across:

- Grades 1–3 foundational literacy and numeracy;
- Grades 4–8 subject understanding and transfer;
- Grades 9–12 advanced school mathematics/science, humanities and research;
- early college, advanced college and vocational/practical learning;
- South African English plus isiZulu and Afrikaans seed cases before claiming multilingual parity.

Include positive, ambiguous and adversarial cases: correct reasoning; a common misconception; an arithmetic slip; multiple plausible interpretations; a missing prerequisite; an incomplete answer key; misleading prompt wording; an irrelevant high-fluency answer; and a response that gives away the solution too soon.

Keep a held-out evaluation split immutable. Avoid leaking benchmark cases into prompt templates or training data. Record model/provider/version, system policy, decode settings, tools used and exact input/output digests for each run.

## 5. Annotation and scoring protocol

1. Two qualified annotators independently label each case against a documented rubric.
2. A third reviewer adjudicates disagreement; record the disagreement, do not silently rewrite it.
3. Measure annotator agreement by dimension and label. Revise the rubric when agreement is poor rather than treating noisy labels as ground truth.
4. Use macro-F1 for categorical dimensions where appropriate, class-specific precision/recall and confusion matrices, calibration metrics where probabilities are emitted, and severity-weighted false-negative rates for critical instructional defects.
5. Report per-grade, domain and language results with uncertainty. Aggregate scores must not hide a weak subject or learner group.
6. Maintain separate reporting for deterministic tests, human-rated pedagogical behavior, and actual learner outcomes.

An LLM judge may assist triage, but cannot be the sole authority for the reference labels or for claims that a tutor teaches well.

## 6. Proposed launch gates

These are internal research targets to validate, not existing results or published industry norms:

- **Known-critical defects:** 100% detection on a frozen initial negative-control suite; add a new regression test for every confirmed escaped defect.
- **Human agreement:** reach a prespecified acceptable inter-rater agreement before treating any dimension as a stable benchmark. A provisional design target is Cohen's (kappa ge 0.70) on adjudicable categorical labels; analyze prevalence effects and do not report this target as achieved.
- **Pedagogical response quality:** establish baseline score and confidence intervals before selecting a numerical improvement target. Then require a prespecified meaningful gain against both the current Praxis baseline and a named external baseline.
- **Reliability:** publish variance across multiple runs and model versions; test prompt paraphrases and small context changes.
- **External claims:** do not claim state-of-the-art from one split, one model, one language, or an LLM-judge-only score. Independently replicate on a held-out set and disclose all material regressions.

## 7. From benchmark score to educational efficacy

A strong PedBench score is a useful diagnostic, not evidence of learning. Once response quality is acceptable, run preregistered learner studies that compare equivalent objectives and assistance policies. The primary outcomes should include unaided delayed retention and transfer, with immediate performance, error recovery, time-to-competency, learner burden and accessibility as secondary or guardrail outcomes.

The desired causal chain is:

**valid content → pedagogically appropriate interaction → learner reasoning and practice → unaided retention and transfer → practical capability.**

Every link requires its own evidence. A failure at one stage must remain visible and must not be papered over by success at another.
