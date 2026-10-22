# PRAXIS-FRONTIER-RESEARCH-001: Competitive Strategy and Evidence-Driven Benchmarks

**Snapshot date:** 2026-10-22  
**Status:** Research and benchmark proposal; not a measured product comparison  
**Related RFC:** [PRAXIS-FUTURE-CURRICULUM-001](FUTURE-CAPABILITY-CURRICULUM-001.md)  
**Repository context:** Public Mycelix / Praxis

## Executive decision

Praxis should not compete by copying every platform category into one product, maximizing course completions, generating the most content, or claiming generic superiority over Wikipedia or all learning management systems.

The stronger thesis is:

> Build an open, learner-controlled system that makes useful knowledge cheap to contribute, hard to misrepresent, easy to reuse across platforms, and measurably more likely to become durable independent capability.

This is a hypothesis to test, not an established result. The initial competitive advantage should be proved on narrow journeys before it is generalized.

The existing Mycelix Praxis proposals already cover community authoring, curriculum auditing, essential capabilities, answer-before-scoring, evidence-aware assessment, and a learning-impact protocol. This document is a research addendum intended to connect these threads into one competitive programme, not a replacement schema, parallel authoring UI, or independent curriculum taxonomy.

## 1. Research snapshot: what the field tells us

### 1.1 Authoring UX is a live competitive area

H5P's 21 August 2026 product roadmap prioritizes faster creation from existing materials, in-editor editing assistance, a clearer draft-to-publish journey, accessibility and consistency, reuse, aggregated usage/difficulty views, exports, and smoother LMS integration. H5P describes several of these as work in progress or exploration, not delivered commitments. The comparison must also reflect shipped capabilities: H5P's 26 August 2026 release announced **Smart Import**, a two-step workflow that turns supplied source material into chosen H5P activities, available on Premium and Enterprise plans.

**Implication:** "AI makes a lesson draft" is not a defensible differentiator by itself. CreatorBench must compare against current Smart Import where available—not a straw-man blank form—and record plan restrictions, supported content types, manual cleanup, rights handling, accessibility, export, and actual time to a valid activity. Praxis must demonstrate a lower total cost from rough idea to reviewed, rights-clear, accessible, portable learning object. Its strongest distinctiveness is likely the combination of provenance, contribution history, curriculum relationships, learner-controlled evidence, and independently measured outcomes, not generation alone.

Sources:
- https://h5p.com/resources/blog/whats-coming-next-for-h5p
- https://help.h5p.com/hc/en-us/articles/38478563354781-Release-Note-August-2026

### 1.2 Tutoring requires pedagogical diagnosis, not merely correct answers

MMTutorBench, published in the ACL 2026 long-paper proceedings, evaluates 770 math problems around pedagogically significant key steps. The benchmark uses problem-specific rubrics, six evaluation dimensions, and tasks for insight discovery, operation formulation, and operation execution. Its evaluation of 12 multimodal models found substantial gaps relative to human tutors; OCR pipelines degraded tutoring quality and few-shot prompting yielded limited gains.

MathTutorBench (2025) also reports that subject-matter problem-solving ability does not automatically translate into effective tutoring. Open-ended dialogue becomes more challenging as tutoring conversations lengthen.

LongTutor, also published in the ACL 2026 long-paper proceedings, addresses a complementary gap: long-term personalization. It uses expert-annotated real-world learning logs and evaluates historical evidence acquisition, knowledge-state diagnosis, and adaptive teaching actions. The authors report that models can acquire historical evidence but struggle to use it reliably for diagnosis and adaptive teaching.

**Implication:** Praxis needs both local pedagogical quality and longitudinal coherence. A future tutor benchmark should use privacy-minimized synthetic or permissioned learning histories to test whether a system distinguishes observation from inference, notices contradictory or stale evidence, identifies an appropriate next action, and avoids overconfident claims about learner state. This does not require sending raw learner records to a central model.

**Implication:** Praxis tutor evaluations must include misconception diagnosis, appropriate hint selection, answer withholding when useful, explanation quality, error recovery, history-grounded adaptation, and long-dialog consistency. Correct final answers and superficial response similarity are insufficient.

Sources:
- https://aclanthology.org/2026.acl-long.1068/
- https://aclanthology.org/2026.acl-long.1371/
- https://arxiv.org/abs/2502.18940

Praxis implementation follow-up: [PRAXIS-TUTOR-HISTORY-001 #5455](https://github.com/Luminous-Dynamics/mycelix/issues/5455) defines a synthetic, privacy-minimized first benchmark slice for history-grounded teaching actions.

### 1.3 AI support can help recovery after errors without guaranteeing broad transfer

An August 2026 NBER working paper reports a randomized field experiment with more than 6,000 middle-school students using an established computer-assisted mathematics platform. Students were assigned to AI versus conventional platform support, mastery versus non-mastery progression, and one of two math topics. AI users attempted fewer questions and progressed more slowly, but were more accurate conditional on attempting and recovered to correct answers in fewer attempts after mistakes. The delayed assessment was mixed: the most encouraging improvement was concentrated on practiced material in the AI-plus-mastery workflow.

This is a working-paper result in a specific population and platform, not proof of a universal AI-tutoring effect.

**Implication:** Separate time, progress, assisted performance, error recovery, unaided near transfer, far transfer, and delayed retention. Do not collapse them into one "learning score." Treat productive struggle and correction as things to measure, not automatically minimize.

Source: https://www.nber.org/papers/w35621

### 1.4 Access does not guarantee adoption or effectiveness

A June 2026 education working paper describes two randomized trials of an AI-literacy platform for elementary students. Nearly half of students in a control condition never used the platform; students who used it averaged only 2–5 minutes per week. Human tutors focused on engagement increased usage and engagement, but reading achievement did not improve.

**Implication:** Measure invitation-to-first-use, task completion, abandonment, support needs, learning outcomes, and longer-term return separately. A usable tool may still fail if its setting, social support, incentives, or instructional design do not lead to meaningful use.

Source: https://eric.ed.gov/?id=ED681902

### 1.5 Retrieval and distributed practice are promising, but domain matters

A systematic review in health-professions education identified 63 experiments across 56 included studies; 43 reported significant benefits from distributed practice and/or retrieval practice over controls or comparisons. The study population and domain limit generalization, but the evidence supports testing spacing and retrieval rather than assuming rereading or content exposure is enough.

**Implication:** Praxis should test scheduled recall, spacing, feedback, and transfer as explicit intervention components. A scheduling algorithm should not be described as scientifically validated merely because it resembles a named learning or forgetting curve.

Source: https://pubmed.ncbi.nlm.nih.gov/37615780/

### 1.6 Standards are an interoperability opportunity, not a reinvention target

1EdTech Common Cartridge describes standardized packaging and exchange of learning resources and assessments; its ecosystem also connects to QTI, LTI, and CASE competency identifiers. CASE 1.1 defines exchange of academic standards, competencies, skills, associations, and rubric structures.

**Implication:** Define Praxis's richer internal meaning and evidence model, then implement adapters that preserve supported meaning in existing formats. Report losses explicitly. Do not create a closed alternative simply to own the format, and do not claim conformance without tests against the actual specification.

Sources:
- https://www.1edtech.org/standards/cc
- https://standards.1edtech.org/case/

### 1.7 Future capability frameworks must be plural and updateable

UNESCO's AI competency framework for students has 12 competencies across human-centred mindset, AI ethics, AI techniques and applications, and AI system design, with progression through Understand, Apply, and Create.

**Implication:** Praxis should import and map useful external frameworks while allowing local communities to revise examples, languages, capability priorities, and routes. No employer survey, AI lab, government, or maintainer should be the sole authority defining what every person ought to learn.

Source: https://www.unesco.org/en/articles/ai-competency-framework-students

## 2. Paradigm shifts worth pursuing

### Shift A — From content volume to evidence-bearing learning objects

A lesson should be a small, versioned, attributable object with a stated purpose, audience, sources, rights, assessment criteria, accessibility metadata, review history, and known limitations. It can be simple and still useful; it should not be presented as reviewed or effective without the corresponding evidence.

**Testable advantage:** lower cost to maintain and reuse content without an increase in factual, licensing, accessibility, or assessment defects.

### Shift B — From AI that completes tasks to AI that builds independence

Use bounded hints, error diagnosis, active retrieval, and scaffold fading. At an appropriate point, remove the assistant and test a new task. Record the type and amount of assistance so the learner's independent performance is not credited to the system.

**Testable advantage:** a smaller gap between assisted practice and unaided delayed performance, while preserving task safety and learner agency.

### Shift C — From one prescribed course to a transparent graph of multiple routes

Represent hard prerequisites separately from recommendations, relationships, alternatives, and specializations. Let the learner choose a goal and inspect why each capability is included. Support multiple routes to the same goal and map the path to external standards only when evidence supports the mapping.

**Testable advantage:** fewer unreachable paths, less unnecessary prerequisite blocking, better goal completion, and transparent route alternatives—without hiding essential safety dependencies.

### Shift D — From an authoring form to a community knowledge-production pipeline

Make creation, preview, review, rights checking, remix, translation, export, and versioned correction one continuous workflow. Support different kinds of contributors: educators, students, parents, practitioners, translators, researchers, and community experts.

**Testable advantage:** more contributions reach an appropriate review state per contributor-hour, with rights comprehension and review quality preserved.

### Shift E — From platform lock-in to portability as a release test

A lesson is not "open" merely because a page is publicly viewable. Authors need clear rights choices and export access. Published materials should survive supported export/import paths, preserve attribution and assessment meaning, and state which features do not round-trip.

**Testable advantage:** high semantic-preservation rates on frozen exchange fixtures and reduced re-authoring time when organizations move or combine tools.

### Shift F — From one overall score to a multi-dimensional quality ledger

Report content accuracy, source support, instructional design, accessibility, rights status, reviewer status, adoption, independent learning, and delayed retention as separate dimensions. A severe rights or safety violation must not be compensated by high engagement or quiz accuracy.

**Testable advantage:** users can see exactly what is known, unknown, reviewed, or unverified without a misleading universal quality badge.

### Shift G — From "global curriculum" to a federated, plural curriculum commons

Use a shared capability graph and versioned identifiers, but support local forks, language variants, curricular mappings, community governance, and visible disagreements. An update to one source should trigger a review signal in derived objects rather than silently overwriting local adaptations.

**Testable advantage:** reuse across communities while maintaining accurate provenance, local relevance, and the ability to contest decisions.

### Shift H — From engagement metrics to meaningful participation and outcomes

Measure whether people can start, continue, receive necessary support, learn, and return to apply a capability. Do not infer effectiveness from streaks, clicks, completion rates, ratings, or time-on-platform alone.

**Testable advantage:** a transparent relationship between accessibility/usability, meaningful exposure, and independently demonstrated capability.

## 3. Industry comparison: compare like with like

Do not publish one leaderboard that mixes article quality, content-authoring speed, tutor dialogue, learning gains, accessibility, and portability. These are different tasks with different validity conditions.

| Comparator | What to benchmark against it | Praxis opportunity | Main fairness constraint |
|---|---|---|---|
| Wikipedia / reference works | Source discovery, citation tracing, correction latency, topic coverage for the same bounded subject. | Connect sources to practice, projects, skill maps, and uncertainty. | Do not claim greater encyclopedic coverage without a comparable corpus audit. |
| H5P | Time to a valid interactive draft, editing iterations, accessible rendering, reuse, export, author task success. | Integrate provenance, rights, review workflow, capability mapping, and learning evidence. | Use equivalent tasks, content types, author experience, and device conditions. |
| OpenStax / reviewed open textbooks | Factual review, coherent explanations, accessibility, coverage, learner outcomes against a common objective. | Enable smaller updateable units, community localization, routes, and practice. | Respect professional review and do not treat author count as a quality measure. |
| LMS / Common Cartridge | Import/export success, setup burden, assessment preservation, course handoff. | Cross-platform portable objects and goal-oriented composition. | Compare supported feature profiles and report lossy conversions. |
| MathTutorBench / MMTutorBench | Tutor pedagogy and dialog/task-quality dimensions on the published benchmark where licensing and protocols permit. | Add diagnostics for scaffold fading, source use, safety, and independent transfer. | Preserve original task definitions; label any added metrics as separate extensions. |
| Conventional resources vs. AI support | Unaided transfer, delayed retention, error recovery, time, and adverse effects. | Hypothesis: learner-controlled, hint-first routes can improve independence. | Use a preregistered study with an active comparator and independent scoring. |

### Fair comparison rules

1. Freeze the competitor version, task set, content corpus, model/tool configuration, device, and evaluation rubric before testing.
2. Use novice participants for creator comparisons; do not infer novice usability from maintainer demonstrations.
3. Keep training/development examples separate from held-out test cases. Do not tune on the final test set.
4. Use blinded human ratings for subjective quality where practical; report inter-rater agreement and adjudication.
5. Publish sample sizes, exclusions, missing data, uncertainty intervals, subgroup limitations, and negative outcomes.
6. Separate verified findings, model-graded findings, static checks, and proposals in the public report.
7. Keep source data and learner telemetry private where required, and release only rights-cleared, appropriately minimized benchmark artifacts.

## 4. Benchmark suite: versioned, independent, and falsifiable

Existing proposals in Praxis already name CreatorBench v1 and PedBench v1. Preserve their versioned targets and artifacts; do not silently change their acceptance criteria. Use this table as a cross-benchmark map and add new targets through a versioned proposal.

| Benchmark | Primary question | Core measures | Required negative controls |
|---|---|---|---|
| CreatorBench | Can a novice create, preview, check, and share a lesson easily? | Median/p90 time, completion rate, abandonments, correction/retry count, review effort, rights comprehension, accessibility task success. | Missing rights, contradictory licence, malformed export, lost imported fields, keyboard trap, undisclosed answer key. |
| ContentBench / curriculum audit | Is an object internally sound, traceable, safe for its declared context, and connected to a real capability? | Factual defect adjudication, claim/source coverage, stale source response, graph integrity, answer-key defects, accessibility findings, reviewer minutes per object. | Fabricated citation, valid unusual content, incorrect key, missing prerequisite, duplicated ID, superseded guidance, prohibited reuse. |
| TutorBench integration | Does a tutor diagnose and guide rather than simply answer? | Key-step accuracy, misconception diagnosis, hint appropriateness, explanation quality, justified answer withholding, error recovery, longitudinal consistency. | Student misconception, ambiguous prompt, unanswerable item, prompt injection, visually misleading input, tutoring request outside competence. |
| LearnBench | Does use produce independent, retained capability? | Unaided near/far transfer, delayed retention, next-attempt recovery, assistance exposure, learner burden, adverse effects. | Reused test item, answer leakage, AI unavailable, no-help arm, assistance mislabelled as independent work. |
| PortabilityBench | Can materials leave Praxis without losing important meaning? | Import/export pass rate by feature, semantic preservation, provenance preservation, rights metadata preservation, manual rework. | Unknown extension, unsupported interaction, malformed package, rights mismatch, dropped accessibility metadata. |
| LocalisationBench | Can a community adapt materials without breaking meaning or provenance? | Translation review time, concept/rubric preservation, local relevance review, source/version linkage, linguistic and accessibility defect rates. | False cognates, changed units, jurisdiction mismatch, erased upstream attribution, stale translated safety advice. |
| ParticipationBench | Do people actually use and return to the resource under real conditions? | Time to first use, abandonment, successful completion, support requested, return/application rate, accessibility and bandwidth differences. | Required support absent, unreliable connection, misleading completion state, excluded participant groups. |

### 4.1 CreatorBench targets: retain v1, test stricter stretch goals separately

The existing CreatorBench v1 proposal lists median time to first exportable draft of at most 8 minutes, p90 at most 15 minutes, and at least 85% novice task success, plus rights, remix, and accessibility measures. Those are **proposed targets not yet measured** according to the associated PR description.

Candidate future stretch targets:
- median time to a valid first learner preview: at most 5 minutes;
- at least 90% novice task completion;
- at least 95% correct answers on a rights-comprehension task;
- zero public releases when a required rights field is missing or contradictory.

These are v2 hypotheses. Test them without replacing v1 metrics. A faster flow fails if it produces more review defects or confuses contributors about what they are allowed to publish.

### 4.2 Learning outcomes: preregister rather than choose the winner after the test

Before a learner comparison begins, preregister:
- a precise target capability and participant population;
- the active comparator and instructional time policy;
- randomization or a justified alternative design;
- primary outcome and scoring rubric;
- delayed measurement windows (for example, 7 and 30 days);
- assistance policy and exact logging semantics;
- exclusion, missingness, subgroup, and adverse-effect plans;
- sample-size rationale and a minimum meaningful-effect or non-inferiority margin chosen with domain expertise.

The primary outcome should usually be **independently scored, unaided performance on a new transfer task**, with delayed retention as a predeclared key secondary outcome. Repeated practice questions should not be recycled as the test. Completion, speed, confidence, and assisted correctness remain useful secondary diagnostics.

Do not set a universal numeric effect-size goal before selecting the domain and study design. For each pilot, justify a minimum meaningful effect and power the study to it. Report confidence intervals; failure to detect an effect is not proof of equivalence.

### 4.3 Tutor evaluation: use external baselines rather than inventing a competing score

Use MathTutorBench and MMTutorBench as external reference tasks where the artifacts, licences, and evaluation terms allow it. Keep published scores distinct from Praxis-specific extensions. Add Praxis evaluations for hint fading, source-grounded instruction, accessibility, and independent transfer as separately reported metrics rather than combining them into an unvalidated total.

For the first internal tutor challenge set, create hand-authored positive examples, failure cases, and counterfactual pairs:
- correct vs. subtly incorrect answer key;
- helpful hint vs. accidental solution reveal;
- evidence-backed claim vs. generated but unsupported claim;
- learner misconception correctly diagnosed vs. merely corrected with an answer;
- supported uncertainty vs. confident hallucination;
- reasonable abstention vs. ungrounded refusal.

Use held-out cases, blind independent ratings, and appropriate domain specialists. Synthetic tests validate evaluator logic; they do not establish real learner benefit.

## 5. Engineering sequence for the next work

This sequence deliberately builds on the current authoring/audit/benchmark PR family rather than waiting for unrelated repository-wide runners or creating parallel infrastructure.

### Work item 1 — consolidate benchmark definitions

- Reconcile #5368, #5406, #5403, #5273, and this RFC against their current exact branch heads.
- Identify the actual benchmark schemas, fixtures, entry points, and CI workflows from the source, not PR descriptions alone.
- Keep CreatorBench v1 and PedBench v1 versioned; document the relationship between the benchmark names and avoid duplicated scorers.
- Create a benchmark inventory with owner, input artifacts, evaluator, output schema, exact-head receipt, current evidence status, and known validity limitations.

**Acceptance:** each metric is either mapped to one canonical implementation or marked explicitly as proposed; no duplicate evaluator or contradictory target is silently introduced.

### Work item 2 — author-to-export contract test ([implementation issue #5450](https://github.com/Luminous-Dynamics/mycelix/issues/5450))

On the existing creator workflow, freeze one small, rights-cleared example set containing:
- one explanatory lesson with a worked example and practice items;
- one hands-on activity with safety and equipment metadata;
- one translated/remixed object with upstream attribution;
- one accessibility-sensitive media example.

Include valid positives and invalid negative controls. Round-trip each valid object through editable draft, learner preview, JSON export, import, and re-export. Compare semantic fields and expected losses, not just whether JSON parses.

**Acceptance:** deterministic exact-fixture tests pass; missing/contradictory rights and malformed assessment data fail closed; unknown supported metadata is preserved or its loss is reported.

### Work item 3 — benchmark the actual novice journey

Create realistic task cards and preregister measures before implementation optimization. Run the same authoring tasks on Praxis and an appropriate H5P/document workflow. Use participants unfamiliar with both systems where possible; report task order and prior familiarity.

**Acceptance:** a published baseline report includes raw task-level outcomes, sample size, time distribution, error/retry counts, rights comprehension, accessibility limitations, and all missed targets. No task-completion claim is inferred solely from maintainer use.

### Work item 4 — first learner study

Choose a narrow, low-risk pilot with an assessable outcome. Compare the existing workflow with the proposed instructional intervention using an active comparator. Obtain appropriate review and consent before involving people or collecting learner data.

**Acceptance:** a preregistration exists before outcome analysis; test items are held out from practice; unaided transfer and delayed retention are measured independently; results include uncertainty, exclusions, and negative findings. No credential or "mastered" authorization flows from a benchmark score.

### Work item 5 — standards adapter spike

Audit Common Cartridge, QTI, CASE, and applicable accessibility metadata requirements against a small source object. Implement the smallest adapter only after mapping exact field correspondence and unsupported features.

**Acceptance:** round-trip fixtures, compatibility profile, loss report, rights metadata checks, and tests against the relevant published schemas/specifications. Do not claim certification/conformance without the appropriate conformance process.

## 6. Non-negotiable architecture invariants

- Draft creation, public release, lesson review, content correctness, learning efficacy, learner mastery, credentials, and real-world authorization are separate states and authorities.
- No platform-wide learner, author, intelligence, employability, or human-worth score.
- No public release if redistribution rights are missing, contradictory, or incompatible.
- No raw private learner attempts published by default for public analytics.
- No AI-generated claim marked sourced or human-reviewed without the supporting trace.
- No "mastered" or authorization decision inferred from a self-rating, time spent, streak, completion, local browser state, or unvalidated posterior.
- Hard prerequisite edges are explicit; recommendations and related-topic links never silently block access.
- High-stakes content includes jurisdiction, source vintage, review requirements, and known limitations.
- Automated evaluators have positive controls, adversarial negative controls, held-out cases, and named claim limits.
- A green structure/schema test cannot substitute for educator review, accessibility testing, empirical learning evaluation, or external standards conformance.

## 7. Decision summary

Prioritize in this order:

1. **Correctness, rights, and creator usability** — make it easy to contribute without making public content untrustworthy.
2. **Portable objects and curriculum composition** — reuse lesson work across contexts rather than increasing platform lock-in.
3. **Evidence-based tutoring and independent learning** — test whether assistance develops durable capability.
4. **External comparison and replication** — make all competitiveness claims narrow enough to be falsified.
5. **Scale and expansive curriculum coverage** — expand only when the preceding contracts and evidence hold up.

The strongest possible claim is not "Praxis has more lessons" or "Praxis is smarter." It is: "For this defined task and population, Praxis makes authoring or learning measurably better under a documented protocol, preserves rights and accessibility constraints, and the result replicates."

That is the standard needed to make the system better rather than merely bigger.

## References

- H5P, "What's coming next for H5P," 21 August 2026. https://h5p.com/resources/blog/whats-coming-next-for-h5p
- Yang et al., "MMTutorBench: The First Multimodal Benchmark for AI Math Tutoring," ACL 2026. https://aclanthology.org/2026.acl-long.1068/
- Macina et al., "MathTutorBench: A Benchmark for Measuring Open-ended Pedagogical Capabilities of LLM Tutors," 2025. https://arxiv.org/abs/2502.18940
- Oreopoulos et al., "Making AI Tutoring Productive: Evidence from a Mastery-Based Math Practice Experiment," NBER Working Paper 35621, August 2026. https://www.nber.org/papers/w35621
- Robinson et al., "Access Is Not Enough: Human Support Improves Engagement with AI Tutoring," EdWorkingPaper 26-1451, June 2026. https://eric.ed.gov/?id=ED681902
- Trumble et al., systematic review of distributed and retrieval practice in health-professions education, 2024. https://pubmed.ncbi.nlm.nih.gov/37615780/
- 1EdTech Common Cartridge. https://www.1edtech.org/standards/cc
- 1EdTech CASE. https://standards.1edtech.org/case/
- UNESCO, AI competency framework for students. https://www.unesco.org/en/articles/ai-competency-framework-students

**Claim ceiling:** This addendum reports published work and proposes competitive hypotheses and benchmark contracts. It does not demonstrate that Praxis has surpassed any comparator or that any proposed target has been achieved.
