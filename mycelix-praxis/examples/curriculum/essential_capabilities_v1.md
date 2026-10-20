# Praxis Essential Capabilities v1.0 (draft)

## Purpose

This seed proposes a small, cross-domain foundation for adult learning: capabilities that help a person reason about claims, make everyday decisions, protect their agency, collaborate, respond to risk, and keep learning. It is a **curated starting point, not a universal canon**. Learners and communities should be able to extend, contest, localize, or omit material without having a single global score imposed on them.

The machine-readable seed is [`essential_capabilities_v1.json`](essential_capabilities_v1.json). It contains 19 nodes, 37 prerequisite/relationship edges, and five suggested learning paths. Its current graph metadata is a working proposal; it does not claim formal alignment to a national standard.

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

## Known gaps / next steps

- Have educators and adult learners from multiple regions review omissions, language, accessibility, and assumptions.
- Add local variants as separately versioned overlays; do not fork the general concepts into hidden country-specific defaults.
- Attach primary-source references to individual instructional lessons and assessment items, with review and supersession metadata.
- Build a deterministic JSON/graph validator into the repository's test suite so future seed changes verify ID uniqueness, edge/path integrity, enum compatibility, and prerequisite cycles in CI.
- Replace proposed outcome thresholds with preregistered pilot estimates and report null or harmful results as openly as positive results.
