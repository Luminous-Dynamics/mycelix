# Praxis Essential Capabilities v1.0 (draft)

## Purpose

This seed proposes a small, cross-domain foundation for adult learning: capabilities that help a person reason about claims, make everyday decisions, protect their agency, collaborate, respond to risk, and keep learning. It is a **curated starting point, not a universal canon**. Learners and communities should be able to extend, contest, localize, or omit material without having a single global score imposed on them.

The machine-readable seed is [`essential_capabilities_v1.json`](essential_capabilities_v1.json). It contains 20 nodes, 39 prerequisite/relationship edges, and six suggested learning paths. Its current graph metadata is a working proposal; it does not claim formal alignment to a national standard.

## Design choices

1. **Capabilities, not content volume.** Each node includes a demonstration prompt. Progress should reflect what a learner can explain or do, not time-on-page, video completion, or recall alone.
2. **Evidence is not authority.** A stored attempt is not automatically an admitted evidence event; an admitted event is not automatically a mastery estimate; a mastery estimate is not automatically a credential or authorization. Keep provenance and assistance visible.
3. **A useful mix of knowledge and action.** Numeracy, epistemic reasoning, health and civic navigation, digital safety, finance, collaboration, systems thinking, emergency readiness, and project delivery are connected rather than isolated subjects.
4. **Localize what changes by jurisdiction.** Legal rights, employment rules, emergency numbers, consumer protections, health systems, taxes, and financial products must resolve to current local sources. This seed deliberately does not encode one country's laws as global truth.
5. **Accessibility is a requirement, not a learner-style score.** Provide language, format, and assistive adaptations as needed. Do not infer cognitive aptitude from presentation preference.
6. **No unearned certainty.** Scientific, medical, civic, financial, and AI-related topics should teach source checking, uncertainty, competing explanations, and appropriate escalation rather than promise a simple answer to every situation.

## Suggested assessment protocol

Use this as a rubric proposal for building assessments; it is not yet an implemented Praxis protocol.

For each capability, create at least two non-identical assessment forms and include:

- **Explanation:** can the learner explain the core idea and its limits?
- **Execution:** can the learner perform a realistic task under stated conditions?
- **Transfer:** can the learner apply it to a new context rather than repeat a memorized answer?
- **Retention:** can the learner still retrieve or apply it after a delay (initial pilot checkpoints: 14 and 30 days)?
- **Calibration and escalation:** does the learner say what remains uncertain and recognize when local, professional, or emergency help is appropriate?

Use a small rubric with explicit anchors, for example: `0 = absent/unsafe`, `1 = incomplete or substantially prompted`, `2 = independently adequate in the stated context`, `3 = adequate transfer plus justified limits`. Publish the task/rubric version and document reasonable accommodations. Do not treat the levels as a universal ranking of people or aggregate them into a single global learner score.

When a performance record is retained, preserve at least the learner-bound source identity, capability and task/version, observed outcome, time when relevant, assistance class, evaluator/tool identity and version, artifact digest where available, and correction/supersession links. A self-report can be useful reflection but should not be silently relabeled as independent demonstration. Store only data needed for the stated purpose; keep sensitive learner records private by default.

## Pilot benchmark plan

These are **proposed research targets**, not achieved results or validated thresholds.

### Gate 0 — corpus integrity

- Every node has a unique stable ID, a domain, difficulty, estimated practice time, and a concrete demonstration.
- Every graph edge resolves to existing nodes; strict `Requires` edges remain acyclic.
- Every learning-path sequence resolves to known IDs and has a total-hour value consistent with its nodes.
- Every external source is dated or versioned where possible; jurisdiction-dependent content has a review owner/date before deployment.

### Gate 1 — assessment quality

- At least two parallel forms for each skill, plus a transfer task that differs meaningfully from instruction examples.
- Scoring guidance is readable and independently reviewable; sample responses include weak, adequate, and strong performance.
- Inter-rater agreement is measured on a double-scored sample before consequential use.
- Assistance is captured explicitly (`Unassisted`, `Hint`, `Scaffolded`, `Collaborative`, `ToolAssisted`, `DirectAnswer`, or `Unknown`) rather than erased from the record.

### Gate 2 — actual learning, not engagement theater

Compare a proposed instructional version with an appropriate baseline, using predeclared outcomes and reporting uncertainty:

- **Primary:** performance on unseen transfer tasks and delayed retention at 14/30 days.
- **Efficiency:** time and number of practice attempts to a predeclared retained-competency criterion.
- **Durability:** performance after gaps in practice, not just an immediate post-test.
- **Calibration:** when predictions are elicited, compare confidence with observed outcomes; don't reward overconfidence.
- **Accessibility and fairness:** report missingness, completion, time, and outcome differences across consented and ethically collected groups; investigate barriers rather than using differences as automatic evidence of learner deficiency.
- **Privacy:** measure data collection and retention against the minimum required to support learning.

Candidate hypotheses for preregistration: improve transfer by at least 0.20 standard deviations over a time-matched baseline, and/or reduce time to durable competency by 15% without degrading delayed retention or access for learners who need accommodations. Treat these as hypotheses to test, not product claims. Report effect sizes, confidence intervals, exclusions, failures, and deviations from the planned analysis. Do not claim an improvement from a single cohort or from activity metrics alone.

### Gate 3 — ecosystem contribution

- Learners can export their own work and evidence in a usable format.
- Communities can propose corrections and alternate prerequisite paths without silently overwriting history.
- Disputed knowledge can retain supporting and contradicting evidence.
- A credential, reputation measure, community vote, or model prediction cannot override failed task evidence or substitute for qualified judgment in a regulated domain.

## Source and adaptation notes

This is a synthesis, not a reproduction of a single curriculum. The following open or public resources inform the scope. They have different audiences and jurisdictions; none alone validates this exact set of capabilities.

- **Media and information literacy:** [UNESCO — Media and Information Literacy Curriculum](https://www.unesco.org/mil4teachers/en/introduction). Covers evaluating content providers and their content, digital citizenship, AI, and misinformation; explicitly supports adapting curricula to local context.
- **Scientific and engineering reasoning:** [National Academies — A Framework for K–12 Science Education (brief)](https://nap.nationalacademies.org/resource/13165/reportbrief.html). Emphasizes evidence, data, models, investigation, explanation, and informed public participation. This seed adapts the practices for adult lifelong learning rather than claiming standards equivalence.
- **Learning practices:** [Institute of Education Sciences — Organizing Instruction and Study to Improve Student Learning](https://ies.ed.gov/ncee/wwc/PracticeGuide/1). Includes evidence ratings for spacing learning over time and use of quizzes/retrieval. Instructional effects can vary by context; Praxis should measure outcomes instead of assuming every method works equally for everyone.
- **Adult financial competence:** [OECD/EU — Financial Competence Framework for Adults in the European Union](https://www.oecd.org/en/publications/financial-competence-framework-for-adults-in-the-european-union_510f133c-en.html). Useful for competence categories; it is EU-oriented and financial education must be localized.
- **Financial decision education:** [U.S. CFPB — Building blocks of financial capability](https://www.consumerfinance.gov/consumer-tools/educator-tool/youth-financial-education/learn/). This is a U.S. resource focused on youth development, included as a source of instructional concepts, not as universal legal or financial advice.
- **Cybersecurity risk structure:** [NIST Cybersecurity Framework](https://www.nist.gov/cyberframework). This is an organizational risk framework, not a personal certification syllabus; use it as a vocabulary/resource for risk management, then write learner-facing tasks appropriate to individuals.
- **Health, employment, emergency, and civic navigation:** use the appropriate current public authority in each locale and record the relevant jurisdiction, source version/date, and review date. These areas must not be taught as a substitute for licensed professional advice, legal counsel, or local emergency services.

## Industry comparison and differentiation hypothesis

This is a capability-learning direction, not a claim of superiority over existing platforms. The following comparison is based on public product/specification descriptions, not independent head-to-head testing.

| System / standard | Publicly documented strength | Praxis opportunity | What would demonstrate an advantage |
|---|---|---|---|
| [Khan Academy mastery system](https://support.khanacademy.org/hc/en-us/articles/360007253831-What-is-self-paced-Mastery) | Skill-level progress, exercises, unit/course challenges, progress reporting, and skill changes after assessment | Generalize beyond a subject/course to portable, cross-domain capability evidence and learner-controlled records | A predeclared cross-domain transfer/retention study, not feature-count comparison |
| [Duolingo efficacy approach](https://blog.duolingo.com/duolingo-efficacy-research-framework/) | Public description of research into taught content and language skills across reading, listening, speaking, and writing | Apply outcome measurement to practical adult capabilities across local contexts and real-world task performance | Independent outcome assessment, delayed checks, transparent uncertainty, and replication outside one subject |
| [Open Badges 3.0](https://www.1edtech.org/standards/open-badges) and [W3C Verifiable Credentials 2.0](https://www.w3.org/TR/vc-data-model/) | Interoperable, verifiable achievement claims, with criteria and evidence fields and portable credential exchanges | Make capability definitions, evidence lineage, assistance, corrections, limitations, and privacy controls inspectable while remaining standards-compatible | Passing relevant interoperability/conformance suites and independent verification; do not invent a proprietary credential island |
| [CAST Universal Design for Learning 3.0](https://udlguidelines.cast.org/) | Framework for access, choice, engagement, multiple representations, expression, and learner agency | Build accessibility and learner control into each assessment protocol rather than add them after outcome measurement | Comparable completion and learning-quality evidence across access needs, with privacy-preserving reporting |
| [IES What Works Clearinghouse study guide](https://ies.ed.gov/ncee/wwc/PracticeGuide/1) | Research-informed recommendations including spacing learning over time and interleaving worked examples with problem-solving | Treat instructional methods as versioned hypotheses, compare them against a baseline, and measure durable transfer | Time-matched comparisons with delayed retention and preregistered outcomes rather than assuming a method will always help |

### Paradigm shift: a capability evidence loop

Build Praxis around this loop:

`Need → Baseline → Practice → Feedback → New-context demonstration → Delayed check → Learner-owned evidence → Real-world application → Revision`

The differentiator should be the quality and portability of this loop, not an AI tutor by itself, a large course catalog, a badge, or a global score. Knowledge nodes need explicit outcomes and provenance. Assessments need parallel forms and transfer tasks. Evidence needs clear assistance and scope. Learners need access to their records and meaningful consent. Communities should be able to contest or improve material without silently overwriting the history of claims.

### Proposed comparative benchmark protocol

1. **Baseline:** compare against (a) the current lesson experience and (b) a time-matched active baseline using retrieval practice and spaced review. Avoid a weak no-instruction control as the only comparison.
2. **Assessment design:** use distinct pre-test, immediate post-test, transfer form, and delayed 14/30-day checks. Do not reuse an identical item as the only evidence of retention.
3. **Primary outcomes:** predeclared rubric-scored transfer performance and delayed retention. Report effect sizes and confidence intervals, not only p-values or completion rates.
4. **Efficiency:** measure time and number of attempts to reach a defined retained-capability criterion. Account for failures and dropout so the analysis does not only count completers.
5. **Calibration:** on tasks where confidence predictions make sense, compare confidence with observed outcomes (for example, Brier score across a sufficient sample); do not force probability scores onto every task.
6. **Assessment quality:** establish rubric clarity, score agreement between independent raters, and form-to-form comparability before using results for high-consequence claims.
7. **Access and safety:** test keyboard/screen-reader-compatible paths where relevant, low-bandwidth/offline recovery, localization, and whether the task mistakenly rewards unnecessary exposure of private data. Report disparities as signals for investigation, not labels of learner deficiency.
8. **Anti-gaming:** include novel variants, realistic case tasks, delayed checks, and documented assistance. Treat speed, streaks, time-on-page, AI-authorship guesses, and error patterns as weak signals—not proof of mastery or dishonesty.

**Initial target hypotheses (not results):** seek ≥0.20 SD improvement on prespecified transfer outcomes or ≥15% less time-to-retained-competency against a credible active baseline, without a meaningful drop in delayed retention or accessibility. Preregister the study and sample-size plan, report negative/null results, and do not claim superiority until independently replicated. The targets should be revised if pilot reliability, feasibility, or learner harms make them inappropriate.

## AI-supported learning: evaluate independence, not just assisted throughput

Recent findings warn that assisted performance can differ from durable independent skill. In a 2025 field experiment with nearly 1,000 high-school mathematics students, GPT-4 access improved practice performance, but performance fell below the no-AI comparison for learners using the less-guarded tutor when access was removed; the hint-oriented, learning-protective design mitigated much of this negative effect ([Bastani et al., PNAS, 2025](https://doi.org/10.1073/pnas.2422633122)). A separate 2025 randomized trial with 120 undergraduates learning AI concepts reported lower correct responses on a surprise 45-day retention test for the unrestricted ChatGPT-assisted group than for traditional-study participants ([Barcaui, 2025](https://doi.org/10.1016/j.ssaho.2025.102287)). These studies have bounded populations and domains; they do not prove that all AI assistance harms learning. The wider evidence is also heterogeneous: a 2025 systematic review and meta-analysis of 68 experimental and quasi-experimental studies reported a positive overall effect (SMD 0.45) but very high between-study heterogeneity (I² = 95%), with effects varying by subject, duration, education level, and instructional design ([Han et al., *Educational Research Review*, 2025](https://www.sciencedirect.com/science/article/pii/S1747938X2500051X)). The correct lesson is not “AI helps” or “AI harms” universally. It is: **measure which instructional policy helps which learner acquire which capability, under which conditions**.

An additional, larger field experiment published as an NBER working paper in August 2026 randomized more than 6,000 middle-school students in Hamilton County Schools across AI versus computer-assisted-learning support, mastery versus non-mastery progression, and two mathematics topics. The report says AI-supported learners progressed more slowly and attempted fewer questions, but had higher correctness conditional on attempting; the most visible mechanism was improved correction after mistakes, alongside more time spent per question receiving structured support. The study included a delayed assessment one week later covering practiced and unpracticed material ([Oreopoulos et al., NBER Working Paper 35621](https://www.nber.org/papers/w35621)). This is a different population and domain from Praxis's adult, cross-domain goal, but it reinforces a key design principle: throughput alone can underrate useful error repair, while immediate assisted correctness alone cannot establish durable learning.

For each material AI tutoring policy, measure separately:

- **Independent baseline and supported task success:** unaided baseline, correctness while the prescribed support is available, time, and number of attempts.
- **Error recovery:** after an incorrect attempt, correctness on the next attempt, attempts until a correct resolution, and time spent on the corrective interaction.
- **Independent transfer:** performance on a novel but comparable task without the tutor, including unpracticed concepts where the study design permits.
- **Delayed independent retention:** performance after a predeclared delay, without reusing the exact training items.
- **Scaffold cost and fidelity:** hint count, direct-answer reveals, learner's own initial attempt, time exposed to support, and mismatch between recorded assistance and the assigned instructional policy.
- **Source reliability and safety:** proportion of consequential factual claims properly verified, invalid citations, unsafe recommendations, and appropriate escalation.
- **Learner explanation:** whether the learner can explain the method, identify its limits, and correct a plausible error.
- **Access and missingness:** who could not complete the task, why data are missing, and whether device, bandwidth, language, or accommodation barriers explain performance differences.

Compare at minimum (a) current instruction plus retrieval practice, (b) direct-answer AI, and (c) hint-first support with scaffold fading, keeping tasks, time, and feedback as comparable as possible. Randomize or counterbalance when feasible and ethical, preserve a pre-analysis plan, and assess with an independent tool-free task. The teaching policy should prefer a learner attempt, then a small hint, then feedback, and only then a worked answer when pedagogically justified. Fading should be adaptive and recorded, not a hidden manipulation.

**Guardrail target:** do not ship an AI-learning optimization based only on immediate scores or completion speed. A reasonable pilot success criterion is no material degradation in independent delayed transfer versus a strong active baseline, plus measurable benefit in at least one predeclared primary outcome. If speed improves but tool-free learning worsens, report that as a regression.

## Known gaps / next steps

- Have educators and adult learners from multiple regions review omissions, language, accessibility, and assumptions.
- Add local variants as separately versioned overlays; do not fork the general concepts into hidden country-specific defaults.
- Attach primary-source references to individual instructional lessons and assessment items, with review and supersession metadata.
- Build a deterministic JSON/graph validator into the repository's test suite so future seed changes verify ID uniqueness, edge/path integrity, enum compatibility, and prerequisite cycles in CI.
- Replace proposed outcome thresholds with preregistered pilot estimates and report null or harmful results as openly as positive results.
