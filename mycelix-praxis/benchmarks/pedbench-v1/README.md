# Praxis-PedBench v1: Pedagogical Ability Benchmark Specification

**Status:** proposal for implementation and educator co-design  
**Owner:** Mycelix Praxis  
**Version:** 1.0-draft  
**Scope:** evaluation of lesson assets and tutor responses, not certification of a whole curriculum or proof of learner-outcome impact.

## 1. Why this benchmark exists

A model can solve a mathematics problem yet fail to teach it. It can also generate fluent, plausible text that ignores a learner's misconception, gives away the answer, or reinforces an incorrect rule. Praxis needs an evaluation set that measures teaching behavior rather than general language fluency or subject-answer accuracy alone.

Useful public comparison points now include several complementary benchmarks:

- **MathTutorBench (EMNLP 2025)** evaluates open-ended math tutoring across subject problem-solving, student-understanding and pedagogy tasks. Its current leaderboard notes that similar solvers can differ by 20+ points on scaffolding and pedagogy-following win rates, especially on hard variants. <https://aclanthology.org/2025.emnlp-main.11/> · <https://eth-lre.github.io/mathtutorbench/>
- **BEA 2025 Pedagogical Ability Assessment** evaluates mistake identification/location, guidance, feedback actionability and tutor identity against human annotations. The best scores across the four three-class pedagogical tracks ranged from 58.34 to 71.81 macro-F1, so mistake remediation remains measurably hard. Compare only identical tracks and splits. <https://aclanthology.org/2025.bea-1.77/>
- **KMP-Bench (AAAI 2026)** evaluates K–8 tutoring through multi-turn dialogue principles and granular skills such as error detection/correction and problem generation; the authors also introduce the 150K-dialogue KMP-Pile. Treat the benchmark/test split separately from any training corpus. <https://ojs.aaai.org/index.php/AAAI/article/view/40578>
- **MMTutorBench (ACL 2026)** evaluates 770 multimodal math-tutoring problems with step-level rubrics across six dimensions and three tasks: insight discovery, operation formulation, and operation execution. <https://aclanthology.org/2026.acl-long.1068/>
- **Scale TutorBench (2026)** contains 1,490 expert-curated prompts across six STEM fields; 828 examples are multimodal and its rubrics cover correctness, explanation quality, tone and personalization. <https://labs.scale.com/leaderboard/tutorbench>

Praxis-PedBench should complement these external tests with educational contexts not covered by math-only evaluation (CAPS alignment, accessibility, rights/provenance, language parity, and assistance dependence). Reuse compatible concepts, document any protocol differences, and never present one scalar score as proof of pedagogical quality.

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

## 6. Proposed launch gates and SOTA comparison protocol

These are internal research targets to validate, not existing Praxis results or published industry norms. Lock an exact benchmark version, test split, model version, prompting/tool policy, decoding regime and scoring implementation before comparing models.

**First reproduce; then try to beat the strongest comparable result.**

1. **Upstream reproduction:** run the unchanged MathTutorBench tasks and matching BEA tracks against the published baselines. Add KMP-Bench for multi-turn K–8 dialogues and MMTutorBench/TutorBench when the system supports their multimodal input formats. Record per-task results; do not average incompatible scores across benchmarks.
2. **SOTA stretch target on BEA:** on at least two of the four pedagogical tracks, exceed the best reproducible published macro-F1 by **at least 5 absolute points**, with a paired bootstrap 95% confidence interval for the improvement excluding zero. On every other track, do not regress by more than 2 absolute points without a documented reason and mitigation.
3. **SOTA stretch target on MathTutorBench:** improve reward-model win rate by **at least 5 percentage points** against the strongest reproducibly comparable baseline on both the hard scaffolding and hard pedagogy-following tasks, while keeping problem-solving accuracy and solution correctness within 2 points of that comparator. The leaderboard's protocol and model-thinking settings must match before the comparison counts.
4. **Critical defect controls:** 100% detection on a frozen initial negative-control suite for incorrect worked steps, answer-key/distractor collisions, forbidden answer leakage at the specified hint stage, and known generic placeholders. Zero observed misses on the suite is a finite test result—not proof that the whole system is defect-free.
5. **Human-label reliability:** two independent educator annotations plus adjudication. Provisional target: Cohen's κ ≥ 0.70 on adjudicable categorical labels; revise labels/rubrics if agreement is weak and report prevalence effects. Do not call the ten synthetic seed cases a validated gold set.
6. **Robustness:** publish scores by grade/domain/language/access modality, repeated-run variance and prompt-perturbation sensitivity. Report all severe failure classes separately rather than letting good average performance hide a safety or pedagogy failure.
7. **Replication:** rerun the improvement on an immutable held-out set and at least one external cohort before any public SOTA claim. Publish prompt/configuration digests, exclusions, confidence intervals, baseline versions and regressions.

A human-grounded reward model is useful for scalable triage, but educator review and learner studies remain separate evidence layers.

The separate [Learning Impact Protocol v1](learning-impact-protocol.md) defines primary delayed unaided transfer, the aided-to-unaided transfer gap, fair comparator requirements, harm guardrails, and evidence layers. This protocol prevents benchmark performance from being treated as learner-efficacy evidence.

Every evaluation should emit a versioned [PedBench run receipt](run-receipt.schema.json). The receipt binds the dataset split, model/version, decoding and tool policies, system and rubric digests, exact per-case input/output digests, annotation state, aggregate scores and uncertainty. Confirmatory-improvement and state-of-the-art claim levels are schema-gated to a held-out split, independent evaluation and confirmatory replication. The JSON Schema expresses the contract; CI tests protect key schema invariants, but a complete Draft 2020-12 meta-schema validation should still be part of the evaluator build before external use.

## 7. From benchmark score to educational efficacy

A strong PedBench score is a useful diagnostic, not evidence of learning. Recent field evidence is mixed: a 2026 two-year school experiment of a coach-configured tutor found small achievement gains that resembled ordinary platform practice and low rates of substantive tutor dialogue; a separate October 2026 university working paper reported worse final grades and lower LMS participation under its course-integrated AI-tutor treatment. These results concern different populations and interventions, so they do not settle the question—but they make a controlled comparison mandatory rather than optional. <https://scale.stanford.edu/ai/repository/one-click-away-ai-tutoring-khanmigo-two-year-school-experiment> · <https://www.iza.org/publications/dp/18989/the-effects-of-course-integrated-ai-tutoring-on-student-performance-and-engagement-a-randomized-university-trial>

Preregister a learner study that compares equivalent objectives and assistance policies against both (a) a well-designed non-AI learning baseline and (b) where feasible, a teacher-facilitated/human-supported condition. The primary outcomes should include unaided delayed retention and transfer; track immediate performance, meaningful learner engagement, usage opportunities, help-seeking, error recovery, time-to-competency, teacher/caregiver workload, well-being and subgroup accessibility as secondary or guardrail outcomes. Measure whether tutor access displaces productive practice, peer interaction or instructor contact.

The desired causal chain is:

**valid content → pedagogically appropriate interaction → learner reasoning and practice → unaided retention and transfer → practical capability.**

Every link requires its own evidence. A failure at one stage must remain visible and must not be papered over by success at another.
