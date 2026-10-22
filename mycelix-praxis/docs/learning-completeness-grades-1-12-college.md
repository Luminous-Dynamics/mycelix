# Praxis Learning Completeness: Grades 1–12 and Open College Access

**Status:** Proposed quality contract and delivery program  
**Prepared:** 2026-10-21  
**Scope:** Mycelix Praxis curriculum graph, lesson assets, assessments, accessibility, evidence of learning, and college-level course access  
**Implementation boundary:** This document defines acceptance criteria and a research plan. It does not claim the existing corpus already meets them, that lessons have been independently validated, or that Praxis credentials confer institutional credit.

## 0. Community authoring as a first-class workflow

People who know a subject, a language, a trade, a community practice or a learner's needs should be able to contribute original, reusable teaching materials without needing to be software engineers. The initial [Praxis lesson authoring kit](../authoring/README.md) establishes a shared lesson-pack format, a realistic starter, a Rust static checker, and a local-first `/author` form that autosaves to browser storage and exports JSON. It lowers the contribution barrier while keeping publication review separate from drafting. The form is an initial prototype; it still needs exact-head compile/test qualification and accessibility review.

The product target is a local-first visual authoring wizard with objective lookup, structured explanation/example/practice/assessment editors, preview, progressive-hint checks, accessibility prompts, source/licence capture, a validation report, autosave and JSON export. Contributors should be able to work without an account or network connection, then submit a reviewable draft. Reuse should preserve original author credit and licence, and any edit to reviewed content must create a new digest and review state.

The first form-based author screen now exists at `/author`; adding a searchable objective picker, flexible item counts, evidence-sidecar generation, and richer validation are the next usability steps. This is not yet a claim that the page has passed browser-based accessibility evaluation or that lessons created there are published automatically.

## 1. Outcome

Praxis should help a learner move from their current knowledge to a meaningful capability, with the evidence needed to know whether that capability was actually learned. The goal is not merely to place a course title or generated lesson beside every standard.

The long-term objective is:

1. **Grades 1–12:** an auditable, locally adaptable curriculum across foundational literacy and numeracy, sciences, humanities, arts, languages, digital/AI literacy, civic understanding, health and practical life skills.
2. **College and vocational learning:** a discoverable map of courses and prerequisites across disciplines, connected to lawful open materials, complete Praxis-authored lessons where coverage is missing, practice and assessment, projects/labs where feasible, and pathways to externally recognized credit where a qualified institution offers it.
3. **Lifelong learning:** entry points based on demonstrated prerequisite knowledge rather than age, school grade, credential status, or assumptions about a learner's ability.
4. **Learner sovereignty:** local-first progress and privacy-preserving sync; usable without a permanent account or continuous connection where technically possible.
5. **Evidence rather than marketing:** every coverage and effectiveness claim can be traced to a versioned source, a test, a review, or a named pilot.

"All courses accessible" means access to a structured course map and a lawful route to meaningful instruction—not a claim that every copyrighted lecture, paid service, laboratory, professional license, or university credit can be provided free by Praxis. Access, learning, assessment, certification, and accreditation are separate states.

## 2. Starting point found in the public repository

At main commit [`1cec9684c09f7bb1d1d55725ff7ae3d71dbb3e54`](https://github.com/Luminous-Dynamics/mycelix/commit/1cec9684c09f7bb1d1d55725ff7ae3d71dbb3e54), the checked-in `mycelix-praxis/apps/leptos/static/content-manifest.json` contains **344 distinct lesson-target IDs and 346 lesson assets**. These are inventory counts, not a curriculum-completion percentage: a lesson-target ID may be inactive, unmapped to an official objective, incomplete, or not field-tested.

The existing `scripts/validate_curriculum.py` checks active embedded-document JSON, duplicate node IDs, dangling graph edges, generated lesson structure, and deterministic manifest consistency. The documented content contract explicitly says that lesson coverage gaps do not, by themselves, fail validation. Therefore:

- **Structural validity is not curriculum completeness.**
- **A lesson file is not proof of lesson quality.**
- **A correct answer key is not proof of durable learning.**
- **A mastery estimate is not a diagnosis or a stable learner trait.**
- **A Praxis completion record is not, by itself, an accredited qualification.**

The repository includes a Common Core Grade 3 Mathematics source file that declares 25 standards, plus CAPS-aligned school and vocational content in the generated lesson manifest. These are promising seed surfaces; their presence does not establish end-to-end coverage or learning efficacy.

Useful current contracts:
- [Curriculum and content contract](content-contract.md)
- [Learning client contract](learning-contract.md)
- [Curriculum validator](../scripts/validate_curriculum.py)
- [Generated content manifest](../apps/leptos/static/content-manifest.json)

## 3. Research principles to implement

### 3.1 Teach and retrieve; do not just present information

The lesson engine should support explanation, worked examples, guided practice, independent retrieval, feedback that explains errors, spaced revisiting, and transfer to new problems. Practice should be adjusted to the learning objective: recall, procedural fluency, conceptual explanation, argument, creation, performance, or physical skill cannot all be assessed with multiple-choice questions.

Spacing and retrieval practice have substantial research support. A review in *Nature Reviews Psychology* describes how these methods can improve learning across domains and age groups; a meta-analysis also examined combined spacing and retrieval interventions. That is evidence to build into a testable instructional policy, not permission to assert that a particular Praxis implementation works without evaluating it.

References:
- Carpenter, Pan & Butler, [The science of effective learning with spacing and retrieval practice](https://www.nature.com/articles/s44159-022-00089-1), *Nature Reviews Psychology* (2022).
- Latimier et al., [A meta-analytic review of the benefit of spacing out retrieval practice](https://eric.ed.gov/?id=EJ1310148) (2021).

The implementation must version each instructional policy and preserve a clear distinction between (a) policy rationale, (b) an assignment to a learner, (c) the observed response, (d) a computed outcome, and (e) a causal claim about efficacy. A correlation from routine product telemetry must not be presented as proof that a teaching method caused improvement.

### 3.2 Design for learner variability

Use [CAST Universal Design for Learning Guidelines 3.0](https://udlguidelines.cast.org/) as a design reference: multiple means of engagement, representation, and action/expression. Support accessible text, keyboard navigation, screen readers, captions/transcripts, adjustable presentation, meaningful choice, and alternative ways to demonstrate a skill when those alternatives preserve the underlying learning objective.

Do not infer a fixed "learning style" or lower a learner's ceiling based on past performance, disability, language, demographic traits, or time taken. Adapt supports and pathways; keep the learning goal explicit and allow a learner to challenge a recommendation.

### 3.3 Make AI a bounded tutor, not the source of truth

AI can draft explanations, examples, translations, hints, and feedback. It must not silently create authoritative standards, invent citations, grade high-stakes work without an appropriate review path, or turn a plausible answer into verified knowledge.

For AI-supported content:
- retain the prompt/model or generator version needed for reproducibility where appropriate, plus the input source references and content digest;
- separate source-grounded claims from generated examples and hypothetical scenarios;
- test mathematical/technical answers with deterministic solvers or independent checks where feasible;
- show uncertainty and offer an escalation path instead of guessing;
- require human review for core lesson publication and high-stakes assessment content;
- never use protected learner data to train a shared model without a separate, explicit, revocable consent and governance basis.

UNESCO's [AI competency framework for students](https://www.unesco.org/en/articles/ai-competency-framework-students) supplies a useful progression across human-centred mindsets, AI ethics, techniques/applications, and system design, moving from understanding to applying to creating.

### 3.4 Treat standards as versioned adapters, not a universal curriculum

Begin with regional curriculum profiles instead of assuming one country's standards are universal. Initial adapter candidates:
- South Africa: [National Curriculum Statements Grades R–12](https://www.education.gov.za/Curriculum/NationalCurriculumStatementsGradesR-12.aspx) and [CAPS](https://www.education.gov.za/Curriculum/CurriculumAssessmentPolicyStatements%28CAPS%29.aspx).
- United States: [Common Core mathematics and English language arts](https://www.thecorestandards.org/).
- United States science: [Next Generation Science Standards](https://www.nextgenscience.org/).
- Additional national, regional, language, vocational, and international profiles should be added with local educators and source maintainers.

A crosswalk is a claim about correspondence, not equivalence. Record framework, jurisdiction, version/date, official identifier, verbatim-versus-paraphrased status, source URL and access date, mapping rationale, reviewer, confidence, and known differences. Do not erase differences between curricula to make a single global count look larger.

## 4. The completeness model

Track multiple independent dimensions for each objective/course node. Do not compress them into a single green check or opaque "AI quality score."

| Dimension | Minimum evidence | What it does not prove |
|---|---|---|
| Source/standards alignment | Exact framework/version/objective IDs, source provenance, mapping rationale, rights metadata | That the lesson teaches the objective well |
| Graph integrity | Valid stable IDs; typed relations; required prerequisites resolve; dependency-cycle policy is explicit | That the chosen prerequisites are pedagogically correct |
| Instructional completeness | Objective, explanation, appropriate example, practice, feedback, check for understanding, transfer opportunity, and an answer key/rubric where applicable | That the lesson causes durable learning |
| Assessment alignment | Every released item maps to an objective and cognitive/performance demand; rubric and scoring policy are versioned | That the assessment measures every aspect of competence |
| Accessibility | Automated checks plus human review against the applicable accessibility target; alternatives preserve the objective | That all learners have equal outcomes |
| Content and rights review | Named review state, sources, license/permissions, attribution, version and expiry/review trigger | That cited sources are infallible |
| Demonstrated learning | Pre-specified outcome measures, learner work, uncertainty, delayed retention/transfer where relevant | That gains generalize to every population/course |
| Access and resilience | Low-bandwidth/offline path, sync behavior, export, language/device coverage and failure-state tests | That a learner has electricity, a device, time or a safe study environment |

### 4.1 Separate lifecycle states

Use explicit per-dimension states rather than implying that every object is equally ready:

- **Unmapped:** no reviewed objective/standard association.
- **Mapped:** source and objective reference exist.
- **Draft:** lesson content exists but is not release-qualified.
- **Machine-validated:** deterministic schema, links, code snippets/tests where relevant, and asset checks pass.
- **Human-reviewed:** qualified reviewers have checked alignment, correctness, clarity, age appropriateness, accessibility, and references.
- **Pilot-tested:** used with real learners under a documented protocol; report sample, context, measures, attrition, and limitations.
- **Independently evaluated:** an evaluator outside the lesson-authoring path reproduced or critically assessed the result.

These are stages of evidence, not ranks of student worth. A valid new lesson may be useful before efficacy is known, but the UI and reports must say so.

### 4.2 Define coverage denominators before reporting percentages

For every release report, pin the source snapshot and scope. Compute at least:

- **Standards mapping coverage:** reviewed objectives mapped / objectives in the pinned official standards scope.
- **Lesson coverage:** objectives with release-qualified lessons / objectives in scope.
- **Assessment coverage:** objectives with at least one aligned formative assessment / objectives in scope.
- **Accessible lesson coverage:** released lessons passing the required accessibility gates / released lessons in scope.
- **Rights-clear content coverage:** released content with validated provenance and redistribution status / released content in scope.
- **Learning-evidence coverage:** objectives with pilot evidence / objectives in scope, separately from content coverage.
- **Prerequisite closure:** target objectives whose required prerequisite chain has a reviewed, reachable route from a declared entry point.

Report numerator, denominator, exclusions, unreviewed objects, version, and uncertainty where appropriate. Never count a generated lesson filename as a fully covered objective. Never treat missing data as success. Show coverage by grade, subject, framework, language, and evidence stage so one strong area cannot conceal an empty one.

### 4.3 Versioned lesson-evidence sidecar

The additive schema at `schemas/lesson-evidence-record-v1.schema.json` defines a sidecar keyed by lesson ID and exact BLAKE3 asset digest. It separates the lesson body from its evolving evidence metadata, allowing current lesson JSON to remain backward-compatible while reviewers and tooling qualify selected cohorts.

The sidecar requires objective references, source records, rights status, a declared lifecycle stage, explicit check states, review receipts, and a collection of learning-study records. Its conditional rules require review metadata for human-reviewed states, learner-outcome studies for pilot-tested states, and an independent-evaluator review for the independently evaluated state. A study can record its comparison condition, outcome type/timepoint, estimate, uncertainty interval, limitations, and evaluator.

Important limitations:
- The schema file currently has a Rust unit test for JSON parsing and key contract fields. That is not equivalent to running a full JSON Schema Draft 2020-12 meta-schema validator or testing all conditional rules.
- The sidecar is not yet integrated into the learner UI, content manifest, or coverage report. Existing lesson files are not automatically elevated to any review state.
- A well-formed review receipt proves the record exists and is bound to a digest; it does not prove that the reviewer is correct or the learning-effect claim is causal.
- Learner-level records, student identities, raw chat logs, and small-cell outcomes must not be placed in public sidecars. Store only privacy-reviewed cohort summaries and protocol/evaluation references.
- Rights metadata must be conservative: unknown is a real state and blocks distribution that requires clear reuse rights.

Next: add a fail-closed Rust validator for sidecar semantics and schema conditionals, bind each record to the exact lesson asset digest, and include sidecar coverage in the release report without mixing review states into a single quality score.

## 5. Minimum release contract for a core lesson

Every released core lesson should contain or reference, as applicable:

1. Stable lesson ID and schema version.
2. Exact learning objective(s), target level, prerequisites, and standards crosswalk.
3. Intended prior knowledge, vocabulary, and common misconceptions.
4. Accurate explanation in language appropriate to the learner, with citations/source references for material factual claims.
5. Worked example or demonstration, including why steps are taken—not only final answers.
6. Guided practice and independent practice suited to the target competency.
7. Feedback that identifies the error and supports correction rather than merely exposing the answer.
8. Formative check aligned to the objective and a clear answer key/rubric.
9. A transfer task, project, performance, or application where appropriate.
10. A revisit/retrieval plan for knowledge that should persist.
11. Accessibility and alternative-representation metadata; captions/transcripts and text alternatives for media.
12. Teacher/parent/facilitator guidance where relevant, including when human supervision is needed.
13. Content provenance, contributor/reviewer history, generator/model metadata where relevant, license/permission, and version digest.
14. A clearly declared state: draft, machine-validated, reviewed, pilot-tested, or independently evaluated.
15. Safety/age appropriateness review for children's content and sensitive subjects.

Not every lesson needs video, a game, AI chat, or an interactive simulation. The required modality follows the competency and the access needs, not product engagement targets.

## 6. Assessment and learning science

Build a separate, versioned assessment contract. A question bank alone is not an assessment system.

- Begin with diagnostic assessment only when useful; allow learners to skip material they demonstrably know.
- Separate recognition, recall, explanation, multi-step problem solving, practical performance, writing, and creative work.
- Track uncertainty. A small set of easy questions must not establish mastery of a broad course.
- Require item quality review, correct answer/rubric evidence, unambiguous wording, appropriate difficulty, and functioning distractors where multiple choice is used.
- Record hints, retries, accommodations, and whether a result was assisted; use that context to interpret evidence without shaming the learner.
- Use delayed retrieval to check retention and unseen problems to check transfer. Do not call an immediate post-test alone "long-term mastery."
- Avoid rewarding speed when speed is not part of the intended competency.
- Keep formative feedback separate from high-stakes decisions. Offer human review or an appeal route for consequential grading.
- Don't let a learner's previous inferred level become a permanent ceiling; provide challenge routes and periodic reassessment.

For science and mathematics, use independently checkable solutions or executable tests where possible. For essays, art, languages, history, ethics, and open-ended projects, use transparent rubrics and qualified review while preserving legitimate disagreement. One numerical score cannot replace meaningful feedback.

## 7. Benchmark plan: quality first, then efficacy

### 7.1 Deterministic corpus and adversarial controls

Create a small, hand-reviewed golden corpus before expanding automated generation. It should contain clean, ambiguous, incomplete, conflicting, malformed, out-of-scope, and license-restricted examples.

Required negative controls:
- duplicate node IDs and dangling required prerequisites;
- an objective with no lesson and an existing lesson with no active objective;
- a syntactically valid but instructionally incomplete lesson;
- a subtly incorrect worked example/answer key;
- an assessment item that targets a different skill from its declared objective;
- a false or unavailable source citation;
- a content asset without known reuse rights;
- malformed locale/accessibility metadata;
- a cyclic required-prerequisite path where cycles are not allowed;
- a mastery estimate that changes without new evidence;
- a translation that changes a mathematical/scientific meaning;
- an AI response that states an unsupported definitive claim instead of abstaining.

The validator must fail on violations of the relevant invariant and identify the file/object/reason. Passing the validator says only that those checks passed.

### 7.2 Release gates

For any content cohort designated "core and release-qualified":

- 100% of objectives carry versioned source and standards references, or an explicit reviewed "not standards-scoped" rationale.
- 100% of released content has a known rights/permission status and required attribution.
- 100% of lessons have an aligned objective and the required instruction/assessment fields for their lesson type.
- 100% of answer keys, automated checkers and executable examples pass their corresponding deterministic tests; high-impact content also receives human subject-matter review.
- Zero duplicate active IDs, unresolved required edges, or unreported missing lesson assets in the pinned cohort.
- All critical accessibility blockers are resolved before release; automated scans supplement rather than replace human review.
- The learner can see what is available offline, what is synced/public, and which actions are disabled or experimental.
- The report separates structurally valid, reviewed, pilot-tested, and independently evaluated content.

These gates apply to the declared release cohort, not silently to a claim that all grades and all colleges have already been completed.

### 7.3 Learning-efficacy protocol

Do not publish a "better than SOTA" claim from internal thumbs-up or unblinded product telemetry. For each instructional change:

1. Write a pre-analysis plan with the learning question, outcome, comparison condition, population, exclusion rules, sample-size/power rationale, analysis, equity checks, and stopping rule before examining results.
2. Compare against a meaningful baseline: the current Praxis lesson, a standard instructional resource, or an established teaching method suited to the objective.
3. Measure immediate learning, delayed retention (for example at one and four weeks, with longer follow-up where feasible), and transfer to new problems or real performance.
4. Report time-on-task, completion/attrition, use of hints, missing outcomes, and learner/educator burden alongside score gains.
5. Use effect sizes and uncertainty intervals, not only p-values or a single pass percentage.
6. Check differential outcomes across language, disability/access needs, age/grade, prior knowledge, connectivity/device conditions, and relevant locally consented groups. Suppress small-cell reporting that could re-identify a learner.
7. Have a reviewer not responsible for creating the intervention check the data and analysis. Publish negative/null findings as well as improvements.
8. Replicate meaningful results before generalized claims; do not assume efficacy on one topic transfers to all subjects.

Initial outcome families:
- foundational reading: decoding/fluency/comprehension at appropriate developmental stages;
- mathematics: conceptual understanding, procedural accuracy, reasoning, and transfer;
- science: model-based explanation, inquiry, evidence evaluation, and application;
- writing/humanities: argument quality, source evaluation, interpretation, clarity, and revision;
- practical/vocational learning: task performance, safety-critical steps, troubleshooting, and durable transfer;
- higher education: course-specific learning objectives plus projects/labs or authentic performance where the discipline requires them.

Targets for improvement should be preregistered per subject and protocol, not invented as a universal percentage before baselines and measurement reliability exist. The first target is **credible evidence and non-inferior safety/learning**, then measurable improvement over a relevant baseline.

### 7.4 Industry benchmark snapshot (research checked 2026-10-21)

Use these as comparison anchors, not directly interchangeable leaderboard scores:

- **AI-assisted secondary mathematics, large field evidence:** Oreopoulos and Low's 2026 NBER working paper reports a two-year cluster-randomized study across 18 Tennessee middle schools. Assignment to Khan Academy plus Khanmigo raised math achievement by about 0.06–0.08 standard deviations over a school year; a secondary estimate for a full year of active participation was 0.14 SD. The paper reports the median student used Khanmigo on about one-third of practice days and during 17% of exercise sessions with an error. This highlights the product gap between availability and productive use. It is a working paper and its treatment includes existing Khan Academy practice, so do not attribute all gains to the AI tutor alone. [NBER Working Paper 35620](https://www.nber.org/papers/w35620).
- **Structured mistake recovery, 6,000+ students:** NBER Working Paper 35621 (August 2026) reports a randomized math practice experiment where AI-supported students attempted fewer questions and took longer, but were more accurate conditional on making an attempt; after errors, AI improved next-attempt correctness and reduced attempts needed to recover. A mastery progression rule alone did not improve one-week delayed learning; the most encouraging delayed results were limited and concentrated in the AI-plus-mastery workflow. Praxis should measure recovery after misconceptions and delayed transfer—not optimize attempt count or correctness alone. This is a working paper, and findings are topic/workflow-specific. [NBER Working Paper 35621](https://www.nber.org/papers/w35621).
- **AI product iteration, high-frequency outcome:** Khan Academy publicly reported a 6.1% improvement in next-item correctness across product tests spanning more than 15 million tutoring threads, by conditioning its tutor on structured learner history. Treat this as vendor-reported product experimentation and a useful short-horizon metric—not an independent estimate of long-term learning. [Khan Academy, May 2026](https://blog.khanacademy.org/how-khan-academy-is-building-a-better-ai-tutor-our-most-recent-learnings/).
- **College physics tutoring RCT:** a 2025 *Scientific Reports* study at Harvard compared a research-informed AI tutor with active-learning classroom instruction in two introductory physics lessons. It reported higher post-test scores in the AI-tutored condition, but the evaluation was narrow and short-term; it does not establish that AI tutors universally outperform teachers or that the result transfers to other subjects. [Kestin et al., 2025](https://www.nature.com/articles/s41598-025-97652-6).
- **Adaptive mathematics software meta-analysis:** a 2021 ALEKS meta-analysis combined 33 studies with 9,238 learners and found performance overall comparable to traditional instruction (Hedges' g = 0.05, 95% CI [-0.01, 0.20]); supplementary use showed a larger estimate (g = 0.43, 95% CI [0.02, 0.83]). This cautions against assuming personalization alone is a breakthrough. [Meta-analysis](https://www.tandfonline.com/doi/full/10.1080/19477503.2021.1926194).
- **System-level comparators:** PISA 2025 (published 8 September 2026) assessed about 760,000 students across 91 countries/economies. The OECD reports average scores of 482 in science, 463 in mathematics and 461 in reading; across 35 OECD countries, reading declined about 14 points and mathematics 9 points from 2022 to 2025, while science was broadly stable. One in five 15-year-olds was low-performing across all three core domains. PISA 2025 also introduced Learning in the Digital World / computational problem-solving results. Use these as contextual system-level needs, not direct claims about a product's causal impact. TIMSS 2023 supplies additional Grade 4 and 8 mathematics/science context. [OECD PISA 2025 performance results](https://www.oecd.org/en/publications/pisa-2025-results-volume-i_73451bc5-en/full-report/student-performance-in-pisa-2025_23e075c2.html); [OECD press release](https://www.oecd.org/en/about/news/press-releases/2026/09/pisa-2025-students-reading-and-mathematics-performance-declined-sharply-across-the-oecd.html); [TIMSS 2023](https://timssandpirls.bc.edu/latest-news/t23-international-results-release.html).
- **Evidence and access guardrail:** UNESCO's Global Education Monitoring report warns that good impartial evidence of education technology's added value is scarce and remote learning failed to reach at least 500 million students, disproportionately the poorest. Product reach and time-on-platform therefore cannot stand in for education outcomes or equitable access. [UNESCO GEM 2023](https://gem-report-2023.unesco.org/).
- **OECD synthesis on GenAI and education (2026):** the *Digital Education Outlook 2026* warns that general-purpose GenAI can raise task performance without producing real learning gains when the pedagogy does not require learners to think and retrieve for themselves. It emphasizes purposeful tutor/partner/assistant roles and teacher agency. This supports measuring independent, delayed performance after assistance is removed, not only aided task completion. [OECD Digital Education Outlook 2026](https://www.oecd.org/en/publications/oecd-digital-education-outlook-2026_062a7394-en.html).
- **Usage and distraction are part of the environment:** OECD reports 46% of students in OECD countries use AI chatbots weekly or more to help them learn, while 28% say classmates are distracted by digital devices during most or every science lesson. Praxis should evaluate learning gains alongside cognitive effort, distraction, wellbeing, and non-AI routes—not maximize AI interaction frequency. [OECD PISA 2025 results](https://www.oecd.org/en/publications/pisa-2025-results-volume-i_73451bc5-en.html).

A reusable preregistration artifact is now checked in at [`experiment-preregistration-template.md`](experiment-preregistration-template.md). It covers immutable protocol identity, intervention/comparator equivalence, primary and delayed outcomes, power/precision, allocation, analysis, attrition, learner privacy, subgroup reporting, release and replication rules. It is a template, not a claim that an experiment has been registered or executed.

### 7.5 Paradigm shift: build a pedagogical compiler, not just a tutor

A learning system should behave more like a compiler and evidence pipeline than a prompt wrapper. A declared objective is the source program; learning assets are build artifacts; validators are type/contract checks; human review is a separate qualification receipt; outcome studies test whether the result actually teaches. Passing one stage must never silently imply that a later stage passed.

**The staged pipeline:**

1. **Objective compilation:** parse a versioned curriculum objective into the required capability, prerequisites, allowable representations, target cognitive operation, and acceptable evidence of competence.
2. **Asset generation:** produce explanations, examples, practice, hints, assessments, answer keys, accessibility alternatives, and source records as separate typed artifacts.
3. **Static semantic checks:** reject missing/generic text, duplicate prompts, incomplete steps, answer-key inconsistencies, assessment/objective mismatches, malformed rights/provenance and malformed accessibility records. Run mathematical or executable checks wherever a task permits deterministic verification.
4. **Adversarial qualification:** run clean examples and negative controls. The pipeline must prove it rejects known bad examples, not merely that it accepts its own generated output.
5. **Review and release:** bind review receipts to the exact asset digest; publish each asset's actual lifecycle state. A high model confidence or coherence score cannot override a failed check.
6. **Learning evaluation:** evaluate independently demonstrated, delayed retention and transfer against a meaningful baseline, then update the instructional policy only through a versioned experiment.

The Rust audit now scans manifest-listed lesson assets for known generic fallback text across explanations, worked examples, practice, hints, vocabulary, misconceptions, assessments and flashcards; it also checks duplicate worked-example/practice prompts and flags correct answers accidentally listed among distractors. Its curriculum coverage report separately indexes manifest IDs that occur in source curriculum graphs not embedded by the app’s current `include_str!` loader. These are **source candidates**, not automatically active or reachable nodes: activating a candidate still requires duplicate-ID, edge-integrity, metadata and curriculum ownership review. The digest now includes JSON under the curriculum source tree so these candidate mappings are reproducible. The qualification workflow uploads full reports even when the strict no-known-blockers gate fails. This is intentionally a *known-defect detector and triage index*, not proof that undetected content is correct; broader semantic, subject-matter and graph-activation checks remain needed.

**Metrics that should distinguish Praxis from typical product leaderboards:**

- **Independent delayed mastery:** change in unaided performance at approximately 7 and 28 days, measured on prespecified items.
- **Transfer rate:** proportion of learners who solve genuinely novel items or complete real tasks that use the learned capability.
- **Error recovery:** independent accuracy after an explanatory hint and the rate at which the same misconception reappears.
- **Assistance-dependence gap:** difference between performance while aided and after the aid is removed. A tutor that raises aided performance but widens this gap may be teaching dependency, not competence.
- **Learning productivity:** independently retained/transferable capability gained per learner-hour and per unit of delivery cost, with teacher/caregiver workload included.
- **Content defect escape rate:** proportion of known defects in a preregistered adversarial corpus that reaches a release-qualified cohort. Target zero for the defined critical defect classes, including known generic placeholders, mismatched answer keys and invalid rights claims.
- **Access parity:** differences in learning, completion, and task success by language, disability/access needs, prior knowledge, device/connectivity, and learner context.
- **Calibration:** whether confidence and mastery status correspond to observed performance, and whether the system abstains or escalates when evidence is insufficient.

A first PedBench v1 specification and ten **synthetic, not-yet-validated** seed cases are checked in under [`benchmarks/pedbench-v1/`](../benchmarks/pedbench-v1/). The cases now include answer leakage, falsely declaring mastery from one assisted answer, multilingual language access, and fabricated scholarly citations in addition to core misconception and accessibility failures. Rust contract tests require stable unique IDs, explicit failure modes and independent educator review. Labels are candidate reference judgments only and cannot be reported as gold-standard labels or benchmark scores. The protocol borrows compatible dimensions from [MathTutorBench (EMNLP 2025)](https://aclanthology.org/2025.emnlp-main.11/) and the [BEA 2025 shared task](https://aclanthology.org/2025.bea-1.77/), aligns with the newer [KMP-Bench (AAAI 2026)](https://ojs.aaai.org/index.php/AAAI/article/view/40578) and [MMTutorBench (ACL 2026)](https://aclanthology.org/2026.acl-long.1068/), and keeps learner-outcome testing separate. The accompanying [run receipt](../benchmarks/pedbench-v1/run-receipt.schema.json) binds scores to model/configuration and held-out evidence; the [Learning Impact Protocol](../benchmarks/pedbench-v1/learning-impact-protocol.md) defines delayed unaided transfer and an aided-to-unaided gap.

**Current benchmark landscape (research checked 2026-10-21):**

- [KMP-Bench (AAAI 2026)](https://ojs.aaai.org/index.php/AAAI/article/view/40578) tests K–8 dialogue-level pedagogical principles and skills such as multi-turn problem solving, error correction, and problem generation; it is a useful fit for the foundational maths cohort.
- [MMTutorBench (ACL 2026)](https://aclanthology.org/2026.acl-long.1068/) provides 770 multimodal, step-centered math-tutoring problems with rubrics over six dimensions. Use it when Praxis evaluates images, handwritten work, diagrams, or screenshots rather than text-only interactions.
- [Scale TutorBench](https://labs.scale.com/leaderboard/tutorbench) includes 1,490 expert-curated prompts across six STEM domains, of which 828 are multimodal. Its rubric measures adaptation, actionable feedback and hint generation, not just correctness.
- [MathTutorBench](https://eth-lre.github.io/mathtutorbench/) and the [BEA 2025 shared task](https://aclanthology.org/2025.bea-1.77/) remain useful reproducible baselines for mistake localization, scaffold quality, and feedback actionability. MathTutorBench explicitly reports different pedagogical win rates for models with similar problem-solving ability.

**Internal SOTA stretch gates (proposed, not yet measured):** reproduce each upstream baseline on its exact split and configuration first. Then target at least **+5 absolute macro-F1 points on two BEA pedagogical tracks**, with a paired bootstrap 95% confidence interval excluding zero; maintain non-target tracks within a 2-point regression margin. On MathTutorBench, target at least **+5 percentage points in both hard scaffolding and hard pedagogy-following win rates** against the strongest reproducibly comparable baseline, while keeping problem-solving accuracy and solution correctness within 2 points. No cross-benchmark grand average; no claim from a custom-only test set.

**Outcome evidence is mixed, so our tutor must be tested against useful alternatives.** The [2026 two-year Khanmigo school experiment](https://scale.stanford.edu/ai/repository/one-click-away-ai-tutoring-khanmigo-two-year-school-experiment) found modest gains when students had access to a coach-configured tutor, but gains resembled practice without AI and substantive dialogue was infrequent. A separate [October 2026 university working paper](https://www.iza.org/publications/dp/18989/the-effects-of-course-integrated-ai-tutoring-on-student-performance-and-engagement-a-randomized-university-trial) reports a negative course-integrated tutor result in its particular setting. These populations and designs differ and do not settle the overall question; together, they make delayed retention, transfer, participation, teacher contact, and subgroup effects mandatory outcomes—not optional telemetry.

There is no single benchmark that can certify an entire curriculum or establish that an AI tutor is "better than SOTA." Use PISA/TIMSS for system-level context, curriculum standards for alignment, subject-specific item banks and authentic tasks for skill measurement, and preregistered trials for causal effectiveness. Keep these layers separate.

### 7.6 Proposed Praxis target ladder

These are proposed internal acceptance targets, not claimed industry norms. For every research comparison, the protocol must name the population, baseline, outcomes, analysis and minimum practically important difference before seeing results.

**Gate A — content integrity and usable access**
- 100% of the declared release cohort has stable objective IDs, reviewed standards/source mappings or an explicit not-applicable rationale, known content-rights status, and an honest publication state.
- 100% of core lessons in that cohort meet their lesson-type schema and reviewed instructional rubric; 100% of answer keys and executable examples pass their tests.
- Zero unresolved critical accessibility blockers at release; target WCAG 2.2 AA conformance supported by both automated and human testing.
- 100% of the core downloaded learning journey passes offline acceptance tests: open content, complete practice, save local progress, and recover/sync without silent loss. This does not claim every feature works offline.

**Gate B — learning efficacy (first adequately powered pilots)**
- Use an established instructional baseline and measure independent performance, not answer correctness while AI help is active.
- Primary outcomes include a delayed assessment (at approximately 7 and 28 days where practical) and transfer to unpractised items or authentic work.
- Initial non-inferiority guardrail: the lower confidence bound for the primary learning outcome must not fall below a preregistered -0.10 SD margin versus the relevant comparison. The margin must be reviewed by educators/researchers for each use case.
- Stretch target: improve delayed retention or transfer by at least 0.15 SD versus the current well-designed baseline, with a preregistered analysis and a confidence interval that rules out no improvement in a confirmatory, adequately powered study. This is a research target, not a prediction.
- Efficiency target: once learning outcomes meet the same delayed-retention and transfer threshold, reduce time-to-demonstrated-competency or delivery cost by at least 20%, without shifting unreasonable work onto teachers, caregivers, or learners.
- Guardrails: no meaningful deterioration in accessibility, learner well-being, misconception rates, or outcomes for prespecified learner groups. Report all subgroup estimates and uncertainty; do not hide a failure behind an aggregate average.

**Gate C — evidence to outperform the field**
- Replicate the main improvement in at least two materially different contexts or cohorts before claiming general effectiveness.
- Compare against both the current Praxis baseline and an established external teaching/resource baseline; ensure equivalent objectives, allotted time, assistance policies and assessment difficulty.
- Publish the protocol, attrition, null results, uncertainty intervals, cost per demonstrated capability, and independent evaluator findings.
- Do not promote a universal tutor or curriculum claim from a short-term single-topic study. Track next-item correctness as a useful diagnostic, but do not substitute it for delayed retention and transfer.

The target of a repeatable learning gain plus lower cost and broader access is more meaningful than claiming one model beats another on a single benchmark.

## 8. Access to all college courses: a lawful, composable catalog

Do not attempt to rewrite every textbook or scrape every university website. Compose existing resources with licensed content and fill measured gaps.

### Course access tiers

| Tier | User-facing meaning | Qualification requirement |
|---|---|---|
| C0 — Discoverable | Course title, discipline, level, prerequisites, outcomes, provider/source, language, access cost and rights notes | Metadata/provenance verified |
| C1 — Resource-linked | Lawful links to books, lectures, notes, datasets, or labs | Per-resource access and license checked |
| C2 — Structured | Course sequence, prerequisite graph, weekly/module map, reading sequence, checkpoints | Qualified course mapping review |
| C3 — Practice-ready | Worked examples, exercises, feedback, assessments, and/or simulations for listed outcomes | Instructional and assessment review |
| C4 — Project/lab-ready | Capstone, laboratory, practical or portfolio route where the discipline requires it | Human-supervision/safety and rubric requirements met |
| C5 — Learning-evaluated | Pilot evidence for specified objectives and learner population | Preregistered or otherwise transparent evaluation |
| C6 — Credit pathway | A named external institution or body has explicitly described how learning may count toward credit/recognition | Current written policy from the recognizing body |

A C0 entry is worthwhile if it is honest; it is not presented as a complete course. The UI must not mark a course "complete" just because all source links opened.

### Rights-aware source strategy

Prioritize:
- open textbooks and courseware with explicit reusable licenses;
- openly licensed lecture notes, datasets and assignments;
- public-domain materials where status is verified for the applicable jurisdiction;
- public syllabi and catalog information only to the extent permitted by their terms and copyright;
- original Praxis-authored explanations, practice, diagrams, simulations, and assessment items;
- partnerships with universities, libraries, vocational bodies, educators, cultural knowledge holders, and open-education communities.

Examples of useful starting points:
- [OpenStax](https://openstax.org/) provides free college textbooks, but its current book license is generally CC BY-NC-SA 4.0; preserve attribution and ShareAlike terms, and do not assume commercial reuse is permitted.
- [MIT OpenCourseWare](https://ocw.mit.edu/) offers free course materials; MIT identifies non-commercial, attribution and ShareAlike requirements for materials under its standard OCW license, while MITx content can have different terms. Inspect each item's stated license rather than treating the entire site as a single license.
- Add other open repositories only after resource-level rights checks. A link being publicly reachable does not mean a file can be mirrored, translated, modified, or commercially redistributed.

Every imported resource needs its original URL/identifier, publisher/creator, license/version, attribution text, content digest and version/date, modifications/translations, retrieval date, and removal/review process. If the rights status is unknown or incompatible, link only when lawful or leave the resource out; do not copy it into the bundled corpus.

### College course coverage

Build a course registry across mathematics, natural and social sciences, engineering, computing, health professions, business, law, education, arts, design, languages, humanities, and skilled trades. For each discipline, map:
- introductory, intermediate, advanced, and research-level courses;
- common prerequisites and multiple legitimate routes;
- prerequisites that are truly required versus recommended;
- theoretical learning and the practical work that cannot be reduced to a reading list;
- required equipment/software, estimated cost, safety requirements, and lower-cost alternatives;
- alternative explanations, practice formats, and learning resources;
- possible external examination, apprenticeship, portfolio, or credit-recognition routes, where officially offered.

A course catalog can aim for very broad discoverability much sooner than full C3/C4/C5 coverage. Report those separate denominators. The long-term objective is broad access to learning and credible demonstration of skills; awarding a regulated degree or professional license remains the responsibility of authorized institutions/bodies.

## 9. Access, inclusion and privacy

- **Offline-first:** core reading, downloaded lesson assets, practice, and local progress should work without continuous network service. Sync conflicts must be explicit and deterministic; a missing service must not look like success.
- **Low bandwidth and device diversity:** deliver text-first alternatives, compressed/downloadable assets, printable activities, mobile-friendly flows, and optional media. Measure load performance on representative low-end devices.
- **Multilingual from the data model onward:** store language tags and source/translation lineage; review specialist vocabulary; test that translated problems preserve the assessed construct. Do not count a raw machine translation as human-reviewed access.
- **Accessibility:** target a stated WCAG level and applicable platform criteria; test keyboard, screen reader, zoom/reflow, captions/transcripts, non-colour cues, and focus behavior.
- **Learner agency:** explain why an activity is recommended, let learners select a different route, permit correction/appeal, and allow export of learner-owned progress and artifacts.
- **Child privacy and safety:** data minimization, private-by-default learning records, age-appropriate interfaces, protected guardian/educator roles where appropriate, retention/deletion policies, and safety escalation. No targeted advertising or public ranking of children.
- **Progress sharing:** never automatically publish individual learning progress. The existing learning contract notes that learner progress is currently a public entry and requires an explicit publish action; preserve a visible disclosure and do not add automatic background publication without an approved privacy migration and fresh consent.
- **AI limits:** explain when generated help is being used, keep non-AI paths available, and provide a no-penalty way to report wrong or harmful instruction.

## 10. Delivery plan

Work on curriculum coverage, quality infrastructure, and access in parallel, while sequencing foundational dependencies deliberately.

### Workstream A — Establish the truth (first)

1. Produce a reproducible inventory from the exact active graph and manifest: node counts by grade/subject/framework, objective mappings, edge validity, lesson targets/assets, orphaned assets, missing lessons, and license/locale/accessibility status.
2. Publish the denominator and limitations before making a percent-complete claim.
3. Add a machine-readable per-object quality report and a compact human-readable gap table.
4. Keep generated curriculum drafts and production/release-qualified lessons distinguishable in both code and UI.

### Workstream B — Gold-standard school pilots

1. Use the existing Grade 3 Mathematics standards file as a constrained pilot corpus, after confirming its source mapping against the exact active graph.
2. Select a foundational literacy slice and a science slice as complementary pilots; define and pin the jurisdiction/framework before writing content.
3. For the pilot objectives, author complete lessons and assessments using the Section 5 contract, with child-safety and accessibility review.
4. Create the golden corpus and adversarial controls from Section 7.1.
5. Observe real teaching/learning use before describing the content as effective. Then expand to the full Grades 1–12 scope with regional adapters and human review.

Foundations should receive priority because gaps in reading, numeracy, and prerequisite knowledge can block later learning. That must not become an excuse to hide the status of later grades: the inventory and development plan should track every grade band from the start, including advanced pathways for learners already ready for them.

### Workstream C — Build the college access graph

1. Ship the metadata registry and C0/C1 course discovery first.
2. Prioritize high-demand foundational courses and prerequisite bridges using learner/community input, not author intuition alone.
3. Attach rights-reviewed open texts/courseware and stable source records.
4. Add structured maps, practice, assessment, projects and simulations as their review gates are met.
5. Work with institutions and professional bodies on external examinations and recognition; advertise only confirmed routes.
6. Keep content contribution decentralized but review and license rules consistent.

### Workstream D — Scientific iteration

1. Version instructional policies independently of course content.
2. Implement preregistration and immutable protocol/outcome references before large personalization experiments.
3. Separate assignments, observations, results, causal conclusions and credentials in the data model.
4. Report performance, uncertainty, subgroup effects, regressions and failed experiments.
5. Promote no strategy to "proven" merely because it has a sophisticated name, a model recommendation, or a small observational improvement.

## 11. Immediate next engineering deliverables

The first implementation slice should be small enough to verify without waiting for a large model or remote runner:

1. **Rust-first coverage plus pedagogical audit:** `crates/praxis-curriculum-audit` now emits machine-readable or Markdown coverage reports and scans the exact manifest-listed lesson assets for known generic placeholders and duplicate worked/practice prompts. The report includes active graph coverage, missing lessons, orphan lesson IDs, findings by field, and a BLAKE3 digest over the active curriculum loader, curriculum JSON, content manifest, and generated lesson corpus. Run `cargo test -p praxis-curriculum-audit --lib` and `cargo run -p praxis-curriculum-audit -- --format=markdown --fail-on-blockers` from the Praxis workspace. Its dedicated GitHub Actions workflow checks format, tests, clippy, the compatibility validator, and the strict report. Keep the existing Python validator; do not remove or weaken it for migration. The new Rust code has not been locally compiled in this execution environment.
2. **Lesson quality schema:** add a versioned, typed schema for objectives, standards references, source/license metadata, instruction/assessment components, accessibility, review state and evidence status. Keep it separate from the legacy lesson shape until a deliberate migration is tested.
3. **Golden corpus + negative controls:** make the validator fail for representative structural, source, rights, answer-key and accessibility metadata defects. Prove that each negative control is rejected.
4. **Pilot lesson cohort:** publish a small, fully reviewed cohort rather than tens of thousands of unreviewed generated pages; use it to exercise the complete learner journey.
5. **Benchmark harness:** emit exact input/output digests, versioned policies, pinned baseline, protocol, inclusion/exclusion counts and measured outcomes. No CI queue state is treated as a passing result; no benchmark is called complete without exact-head evidence.

Do not add authoritative mastery, credential, or educational-outcome semantics to the Holochain integrity layer until the schemas and validation boundaries are specified and reviewed. A provenance record can attest what was authored, assessed, or observed; it cannot prove the educational claim is scientifically true merely by being signed.

## 12. Definition of success

Praxis is succeeding when a learner can find a suitable pathway, understand what it will teach, get an accessible explanation and meaningful practice, receive accurate feedback, demonstrate the skill, revisit it later, export their work, and inspect what evidence supports any progress claim.

At system level, success means:
- coverage gaps are measurable and visible rather than obscured;
- standards differ by jurisdiction without fragmenting learner capability identity;
- lesson assets carry verifiable source, rights, review and version metadata;
- instructional strategies earn trust through replicable results;
- the platform works under constrained connectivity and varied access needs;
- learning access is broad and lawful, while credentials and accreditation claims remain truthful;
- local communities can extend, correct and govern curricula without gaining silent access to private learner records.

The paradigm shift is from **a large catalog of content** to **an evidence-bearing map from learner intent to demonstrated capability**, with rights-aware course access and the freedom to learn across institutional boundaries.

## Research and standards references

- South African Department of Basic Education, [National Curriculum Statements Grades R–12](https://www.education.gov.za/Curriculum/NationalCurriculumStatementsGradesR-12.aspx) and [CAPS](https://www.education.gov.za/Curriculum/CurriculumAssessmentPolicyStatements%28CAPS%29.aspx).
- [Common Core State Standards](https://www.thecorestandards.org/).
- [Next Generation Science Standards](https://www.nextgenscience.org/).
- UNESCO, [AI competency framework for students](https://www.unesco.org/en/articles/ai-competency-framework-students).
- CAST, [Universal Design for Learning Guidelines 3.0](https://udlguidelines.cast.org/).
- Carpenter, Pan & Butler, [spacing and retrieval practice review](https://www.nature.com/articles/s44159-022-00089-1).
- Latimier et al., [spacing plus retrieval practice meta-analysis](https://eric.ed.gov/?id=EJ1310148).
- OpenStax, [college textbook catalog](https://openstax.org/higher-education) and [licensing information](https://help.openstax.org/s/article/Licensing-information-of-OpenStax-textbooks).
- MIT OpenCourseWare, [course materials](https://ocw.mit.edu/) and [requirements of use](https://mitocw.zendesk.com/hc/en-us/articles/4414774353051-What-are-the-requirements-of-use-for-MIT-OpenCourseWare).
