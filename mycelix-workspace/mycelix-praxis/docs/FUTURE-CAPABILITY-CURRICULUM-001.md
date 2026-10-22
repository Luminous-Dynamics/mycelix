# PRAXIS-FUTURE-CURRICULUM-001: A Living Capability Curriculum

**Status:** Proposal for community review  
**Date:** 2026-10-22  
**Scope:** Curriculum architecture, creator workflow, quality evidence, and interoperability  
**Implementation status:** This document is a design proposal. It does not claim that the described workflows, benchmarks, learning outcomes, or standards conformance are implemented.

## Executive proposal

Praxis should build a **Living Capability Curriculum**: an open, versioned, community-maintained map of capabilities that people can learn, demonstrate, adapt, and teach throughout life. It should complement—not attempt to replace—encyclopedias, school curricula, textbooks, simulations, videos, courses, and expert communities.

The central shift is to treat education as more than consuming pages or completing courses:

**A learner chooses a real goal → Praxis composes a learning path → the learner practises and builds something → the learner demonstrates transfer without answer-giving assistance → results inform the next path.**

At the same time, contributing should not require a degree, an LMS account, JSON knowledge, or a large block of free time. A contributor should be able to start with a short explanation, a worked example, a diagram, a local case, an audio explanation, a lab, a source collection, or a translated lesson. Praxis can help shape that material into a portable lesson pack, but must not mistake generated structure for correctness or publishing permission.

### Decision

Build the curriculum in layers, not as a single universal syllabus:

1. **Capability framework:** versioned, observable capabilities and relationships.
2. **Learning objects:** small, reusable, rights-aware lessons, examples, activities, assessments, demonstrations, and translations.
3. **Journeys:** several inspectable routes to a learner-selected goal, with hard prerequisites distinguished from recommendations.
4. **Local adaptations:** jurisdiction-, language-, age-, culture-, resource-, and context-aware variants mapped to existing standards.
5. **Evidence and review:** separate records for content correctness, educator review, accessibility, and demonstrated learning efficacy.

The initial curriculum seed and community authoring work already proposed in related Praxis work should remain the implementation foundation. This RFC defines how those pieces could compose into a coherent future-oriented system; it does not duplicate or supersede them.

## 1. What should Praxis outperform?

We should not make the vague claim that Praxis is already “better than Wikipedia” or “better than every educational platform.” Those services optimize different jobs. Instead, benchmark Praxis against the best relevant alternatives on specific tasks.

| Need | Existing strength to respect | Praxis opportunity |
|---|---|---|
| Broad reference coverage | Wikipedia and other reference works make knowledge discoverable and linked. | Connect reliable references to questions, practice, real tasks, counterexamples, and progression. Do not copy an encyclopedia merely to compete on article count. |
| Reviewed, coherent textbooks | Open textbook projects use expert authorship and editorial review. | Make small reviewed units, adaptations, translations, and corrections cheaper to produce and maintain; preserve reviewer and source provenance. |
| Interactive activities | H5P offers many reusable interactive content types. | Make authoring portable and local-first, with explicit instructional purpose, curriculum links, review state, rights, and outcome evidence. Interoperate rather than recreate every interaction engine. |
| Course delivery | LMSs organize courses, classes, assignments, and teacher workflows. | Let one learning object work across self-study, mentoring, classroom, community, and offline use. |
| Adaptive practice | Existing tutoring and assessment systems support feedback and sequencing. | Optimize for unaided transfer and delayed retention, not only answer accuracy while assistance is present. |
| Credentials | Open Badges and verifiable credential standards support portable achievement claims. | Keep claims tied to criteria and evidence, portable between providers, and explicitly separate from local progress estimates or authorization. |
| Search and AI answers | Search engines and assistants shorten the route to an answer. | Help a learner build the ability to reason, explain, create, and solve the next unfamiliar problem without the answer being supplied. |

The differentiation is the **combination** of learner agency, a community content pipeline, curriculum composition, practical demonstration, durable evidence, localization, offline use, and open exchange. Each part must be evaluated independently before claiming an overall advantage.

## 2. A creator workflow for people who are not instructional designers

### 2.1 Accept rough beginnings

A contributor should be able to choose one of several starting points:

- Explain something you know.
- Show how you solve a real problem.
- Create a hands-on experiment, repair, simulation, field observation, or project.
- Share a source list or annotated reading.
- Translate or localize an existing lesson.
- Improve, challenge, or fork a published lesson.
- Start with a blank lesson.

A first-time creator should be prompted for just the few fields that materially shape the lesson: who it is for, what the learner should be able to do, what the creator wants to contribute, and any important safety or context constraints. Advanced metadata should be progressively disclosed rather than placed in the first screen.

### 2.2 Use an authoring pipeline, not a wall of fields

1. **Capture:** accept text, structured notes, links, diagrams, permitted media, examples, or a simple activity.
2. **Shape:** offer a plain-language structure: goal, explanation, example, learner action, feedback, misconceptions, sources, and what success looks like.
3. **Preview:** show the material as a learner will encounter it, before publication. Include keyboard-accessible and low-bandwidth representations.
4. **Check:** run deterministic checks for missing fields, broken references, duplicate questions, answer/distractor collisions, missing rights, inconsistent identifiers, unsupported prerequisite references, and inaccessible media metadata.
5. **Review:** invite a subject reviewer, educator, local-context reviewer, or accessibility reviewer when the topic or risk calls for it. Let reviewers comment on specific claims or activities.
6. **Release:** create an immutable versioned release only after the selected publication policy is satisfied. Draft, private sharing, community review, and public release are distinct states.
7. **Reuse:** fork, translate, adapt, compare versions, attribute upstream creators, and export a portable package without needing to ask the original platform for permission to retrieve one's own work.

A one-click starter should make a *first preview* easy. It must not imply that an illustrative starter is correct, reviewed, rights-cleared, standards-aligned, or effective.

### 2.3 Make AI an optional assistant, not an authority

AI may suggest clearer wording, examples, alternative formats, questions, translations, misconceptions, and review checklists. It should:

- disclose AI assistance in version metadata;
- distinguish sourced claims from drafts, inferences, and unsupported suggestions;
- retain citations or source locators for claims that depend on external material;
- never silently invent an author, credential, source, licence, approval, or learner result;
- never turn a generated question or answer key into reviewed content merely because it parses;
- keep the learner's attempt before revealing a solution when the aim is unaided learning;
- offer bounded hints and then fade assistance so transfer can be measured;
- let a human inspect, edit, reject, or disable generated suggestions;
- avoid uploading private learner records or unpublished material without explicit, informed permission.

For consequential domains—health, law, financial decisions, emergency response, employment, and physical safety—require appropriate jurisdiction and date metadata, authoritative-source review, and clear limits. The platform should display uncertainty or abstain rather than fill gaps with confident-sounding prose.

### 2.4 Reduce friction without lowering standards

The first draft should be low friction; public claims should be evidence-aware. A short lesson can be publishable as **community-contributed, not yet reviewed** when its rights and minimum safety requirements are satisfied. More consequential claims should require additional review. Do not force every hobbyist contributor through an academic peer-review process before they can share a harmless explanation.

Review should increase reach and trust, not confer personal worth. Do not introduce a single global author or learner score. Show individual dimensions—source quality, correctness review, accessibility, local relevance, and learning evidence—so strengths and gaps remain visible.

## 3. The Living Capability Curriculum

### 3.1 A graph of capabilities, not a single ladder

The curriculum should be a graph that permits different routes and makes relationships explicit. A hard prerequisite is different from a recommendation, a related topic, a specialization, or an alternative route. Only the designated prerequisite relation should block a learner from proceeding.

Each capability should contain:

- a stable identifier, title, version, and change history;
- a plain-language statement of observable performance;
- relevant contexts and boundaries, not an abstract label alone;
- prerequisite, recommended, related, applied-in, alternative, and specialization relationships as distinct typed edges;
- novice-to-advanced examples of what performance can look like, without treating age or seat time as proof of capability;
- multiple learning objects and multiple routes rather than one canonical lesson;
- explicit source, rights, creator, reviewer, locale, accessibility, and safety metadata;
- mappings to external curricula or competency frameworks where a genuine mapping exists;
- evidence maturity and known limitations.

Do not activate every contribution as a curriculum node automatically. New candidates should be checked for duplicates, semantic ownership, broken relationships, unsupported claims, and route integrity before becoming active framework material.

### 3.2 A future-ready core that can be localized

The future curriculum should be a proposed cross-cutting layer, not a replacement for mathematics, language, science, history, arts, or established subject learning. It should connect disciplinary knowledge to the practical ability to understand and improve the world.

Candidate capability families include:

1. **Foundations and communication:** reading, writing, numeracy, mathematics, quantitative reasoning, multilingual communication, and interpreting data.
2. **Epistemic and scientific practice:** source evaluation, uncertainty, probability, causal reasoning, experimentation, measurement, argument, systems thinking, and recognizing when evidence is insufficient.
3. **Digital independence:** operating systems and networks, privacy, cybersecurity, media literacy, data literacy, safe use of AI, and understanding the limits of automated systems.
4. **Making, repair, and problem solving:** design, tools, computing, experimentation, troubleshooting, maintenance, project delivery, and learning from safe failure.
5. **Life and resource stewardship:** personal finance, consumer decisions, housing and energy basics, water and food literacy, sustainability, emergency preparedness, and responsible use of shared resources.
6. **Health, care, and human development:** evaluating health information, navigating appropriate services, self-management, collaboration, empathy, and care—using current local guidance and qualified review for high-stakes claims.
7. **Civic and institutional navigation:** understanding rights and responsibilities, public services, community problem-solving, governance, evidence-based participation, and conflict resolution.
8. **Creative and social capability:** artistic expression, storytelling, music, teaching, mentoring, negotiation, teamwork, and constructive disagreement.
9. **Meta-learning and adaptation:** planning, retrieval, reflection, source seeking, help seeking, deliberate practice, adapting to new tools, and knowing when to stop or ask for expert help.

This is a starting taxonomy to test, not a fixed judgment about what every society or learner should value. Communities should be able to add, question, reorder, contextualize, and fork capability families while retaining transparent provenance and external mappings.

### 3.3 Teach observable capability, not topic familiarity

Every learning journey should answer four questions:

- **Understand:** Can the learner explain the concept in their own words, identify assumptions, and recognize limitations?
- **Apply:** Can the learner perform a relevant task with appropriate tools and feedback?
- **Transfer:** Can the learner adapt the principle to an unfamiliar case, including a case that exposes the limits of a memorized recipe?
- **Teach or create:** Can the learner produce a useful demonstration, explanation, artifact, or feedback for another person?

Not every skill needs an identical four-stage rubric. Safety-sensitive capabilities may require supervised practice and external authorization. Creative capabilities may need a portfolio and transparent human judgment instead of a multiple-choice test. The framework must allow multiple kinds of legitimate evidence.

### 3.4 Compose routes around goals

A learner should be able to say, for example, “I want to understand whether an online claim is trustworthy,” “I want to maintain a home backup-power setup safely,” “I want to learn enough statistics to evaluate a study,” or “I want to teach a skill to my community.” Praxis can propose a bounded route through relevant capabilities and lessons, explaining:

- why each item is included;
- which prerequisites are essential and which steps are optional;
- how much time and what equipment or connectivity are likely required;
- what practice will be done and what counts as success;
- what the route does not cover;
- which local rules, sources, or specialist input matter.

A route is a proposal the learner can inspect and adapt. It is not an opaque ranking, a compulsory monoculture, or an authorization decision.

## 4. Learning-object and curriculum contracts

A learning object should be small enough to reuse but complete enough to inspect. The portable package should represent, as applicable:

- purpose and intended learner;
- capability IDs and versioned framework links;
- explanation and alternative presentations;
- worked example with explicit steps or reasoning;
- learner task and expected response;
- feedback, progressive hints, and solution reveal policy;
- misconceptions and corrective explanations;
- assessment criteria and answer-key provenance;
- transfer task distinct from a repeated practice item;
- sources with stable locators, capture dates where relevant, and claim-to-source relationships;
- creator, contributors, reviewers, AI-assistance declaration, change history, and rights;
- language and localization context;
- accessibility information, transcripts, captions, alt text, keyboard behavior, and reduced-motion or low-bandwidth alternatives when relevant;
- safety classification, required supervision, and jurisdiction/date restrictions where relevant;
- review status, evaluation evidence, known defects, and expiration/review triggers.

Keep format and authority separate. A schema-valid package is structurally well formed; it is not thereby true, safe, open-licensed, accessible, or pedagogically effective.

### Versioning and provenance

Published versions should be immutable references. Corrections and adaptations create new versions with diffs and attribution, rather than silently rewriting what earlier learners saw. A derivative lesson should identify its upstream sources and changed parts. If a source is withdrawn, a licence changes, or guidance goes stale, the system should mark affected content for review instead of pretending the historical release never existed.

For each lesson, expose distinct statuses such as:

- community draft;
- rights and source information complete;
- mechanically checked;
- subject review completed;
- accessibility review completed;
- piloted with a named learner group;
- replicated or externally evaluated.

These are independent dimensions, not one misleading green badge.

## 5. Open by default in access, explicit about rights

The content commons should make copying, remixing, translation, and export straightforward while respecting the actual licence of every contribution. Never infer permission to redistribute from a URL, a public page, an AI-generated response, or the contributor's good intentions.

The author flow should explain licences in plain language, including attribution obligations, share-alike conditions, non-commercial restrictions, and no-derivatives restrictions. Public remix and redistribution should be blocked when the source rights or chosen licence do not permit them. Assets need their own attribution and rights metadata where different from the lesson text.

Design for portable plain-text and structured exports before platform-specific packaging. The target is “author once, export to many places,” not “trap all lessons in Praxis.” Explore standards-based adapters where applicable:

- [1EdTech Common Cartridge](https://www.1edtech.org/standards/cc) for exchanging course materials;
- [1EdTech QTI and LTI](https://www.1edtech.org/specifications) for assessment and tool interoperability;
- [CASE](https://www.1edtech.org/standards) for links to competencies and standards;
- [H5P](https://h5p.com/content-types) for interactive learning content;
- [Open Badges](https://www.1edtech.org/standards/open-badges) and the [W3C Verifiable Credentials Data Model 2.0](https://www.w3.org/TR/vc-data-model-2.0/) for portable, verifiable achievement claims where appropriate.

Interop adapters should preserve meaning or clearly report losses. Do not claim conformance because a file superficially resembles an export format; run the relevant conformance tests before making standards claims.

## 6. Quality, safety, and learner agency

### Separate five different questions

1. **Content validity:** Are the claims accurate enough for this context, sourced, current, and appropriately qualified?
2. **Instructional quality:** Does the content support the stated learning objective, feedback, practice, misconceptions, and transfer?
3. **Access quality:** Can people with different disabilities, languages, devices, bandwidth, and prior knowledge use the material?
4. **Learning efficacy:** Do suitable learners retain and transfer the capability beyond the lesson and beyond assistance?
5. **Authority:** Who is permitted to issue a credential, make a consequential decision, or authorize an activity?

Passing one question must never answer the other four. A local progress flag, a learner confidence rating, a generated score, or completion of a game is not independently verified mastery or permission to operate physical equipment.

Learner records should remain learner-controlled and private by default. Public content quality and curriculum coverage can be measured without exposing a person's raw attempts. Optional research participation needs understandable consent, a specified purpose, appropriate retention limits, and a way to withdraw where feasible. Never use engagement, streaks, speed, or a global “intelligence” score as substitutes for demonstrated learning.

### Accessible from the first draft

Use [CAST's Universal Design for Learning Guidelines 3.0](https://udlguidelines.cast.org/) as a design reference: provide multiple means of engagement, representation, and action/expression; address barriers rather than assuming one format works for everyone. Target WCAG 2.2 AA for applicable web surfaces, with manual keyboard, screen-reader, zoom, contrast, captions/transcripts, cognitive-load, and low-bandwidth checks. A machine check is necessary but not sufficient for an accessibility claim.

Localization must adapt examples, language, units, resources, and relevant institutional context—not just translate words. Preserve both the localized version and its upstream relationship so a correction can flow without erasing local judgment.

## 7. Benchmarks: measure whether this is actually better

All targets below are **hypotheses**, not results. First collect baselines on the same tasks and comparable participant groups. Compare the existing Praxis authoring flow, H5P or another relevant creation tool, a normal document-based workflow, and a reference LMS workflow where the task fits. Publish methods, failures, confidence intervals, exclusions, and exact tested versions.

### CreatorBench: first contribution to reusable lesson

Use novice authors who have not seen the interface. Give them a realistic task: turn something they know into a learner-previewable activity, then review, reuse, or translate it.

Candidate targets for the next benchmark:

- median time to a valid first learner preview: **5 minutes or less**;
- median time to an exportable, structurally valid draft: **8 minutes or less**;
- novice task completion: **at least 90%**;
- rights comprehension: **at least 95%** correctly identify whether the test source can be reused and under which conditions;
- zero public-release acceptance when required rights fields are missing or contradictory;
- measure rather than hide help requests, validation retries, abandoned tasks, and post-task author confidence;
- report results separately for keyboard-only, screen-reader, low-bandwidth, and multilingual tasks.

Record each target as met, missed, or unmeasured; do not silently change the benchmark after seeing results. If the current PR's CreatorBench v1 targets differ, preserve both versions and report the reason for the new version.

### ContentBench: safe and useful reusable objects

Report defect rates by class rather than a single opaque quality score: unsupported factual claims, broken or irrelevant source links, duplicate prompts, answer/distractor collisions, ambiguous rubrics, missing prerequisite evidence, inaccessible media, rights errors, localization defects, stale high-stakes advice, and mismatch between objective and assessment. Add positive controls to ensure the checker does not reject valid unusual lessons merely for being unusual.

Mechanical checks should be deterministic and fast. Human evaluation remains necessary for semantics, factual truth, pedagogy, cultural context, and accessibility.

### LearnBench: retained capability, not clicks

Preregister the learning objective, primary outcome, participant group, exclusions, sample-size rationale, analysis plan, and timepoints before comparing systems. Compare at minimum:

- a static or conventional resource;
- current Praxis practice/sequence;
- Praxis with bounded, hint-first AI scaffolding and explicit scaffold fading, if this arm is available and ethically appropriate.

Use an independently scored, unaided transfer task as the primary outcome and delayed retention (for example, 7- and 30-day follow-ups) as key secondary outcomes. Track next-attempt correction after errors, attempts to correction, time spent on corrective feedback, help requests, and learner-reported burden as diagnostic measures—not as the primary proof of learning. Use new items and scenarios rather than reusing the practice questions as the test.

Report average outcomes and uncertainty, subgroup outcomes where consent and sample size support responsible interpretation, missing follow-ups, assistance exposure, adverse effects, and the limits of external validity. A gain in assisted accuracy without better independent transfer does not establish better learning.

### PortabilityBench: leaving is a feature

Test round trips into and out of supported formats. Verify preservation of meaning, media rights, item structure, answer keys, accessibility metadata, attribution, version identity, and assessment criteria. Publish any information that cannot survive conversion. A successful export should not require using Praxis as the only renderer.

### Equity, privacy, and reliability

Measure who can create and learn, what costs remain, which languages and contexts are represented, whether review burdens fall unfairly on particular contributors, and whether private learner data leaks into public records. Keep results disaggregated and contextualized. Do not compress people's value, ability, or trustworthiness into a universal scalar.

## 8. First curriculum experiment

Avoid trying to author every subject at once. Select one bounded, useful learning journey that is suitable for external review and can be independently assessed. The suggested first pilot is an **Everyday Evidence and Digital Independence** path, because its tasks naturally connect several domains:

1. distinguish a claim, an observation, an explanation, and an opinion;
2. inspect the source, date, context, and corroboration behind an online claim;
3. recognize uncertainty, conflicts of interest, and misleading quantitative presentations;
4. use safe search and AI assistance without treating generated output as a source;
5. protect accounts and personal information when collecting evidence;
6. explain a decision with linked sources and known limitations;
7. teach the method to another person using a local example.

Keep this path separate from high-stakes professional certification. Include examples in more than one language and at more than one reading level; make at least one offline-capable version. Map to external frameworks only when each mapping is defensible. Recruit external reviewers before treating the path as authoritative.

This is a pilot proposal, not a decision that the same path should be first for every community. A local community may choose a more urgent capability, such as numeracy for household budgeting or safe basic maintenance, while reusing the same object, rights, review, and evaluation contracts.

## 9. Phased delivery

### Phase A — specify and audit

- Inventory the existing authoring flow, lesson schema, content auditor, capability graph, curriculum seed, and benchmark contracts on their exact PR heads.
- Identify overlap and dependency order before creating a competing schema or parallel authoring page.
- Freeze the initial capability and lesson metadata contract and a small known-answer fixture set.
- Document the creator workflow in plain language and test it with people who have never edited structured content.

### Phase B — first end-to-end slice

- Make one rough note into a learner-previewable draft, validate it, obtain a review, export it, import it elsewhere, and preserve attribution and version history.
- Add test cases for missing rights, conflicting licence metadata, unsupported claims, answer-key collisions, malformed prerequisites, accessibility gaps, and tampered provenance.
- Preserve an explicit draft/review/published-state distinction. No auto-publication or automatic curriculum activation.

### Phase C — measured pilot

- Run CreatorBench with novice contributors before comparing against alternatives.
- Conduct accessibility review and human subject-matter review.
- Preregister a learner study with delayed independent transfer and retention.
- Publish a transparent report, including negative results and unmeasured criteria.

### Phase D — expand and federate

- Add more capability domains, languages, curriculum mappings, community reviewers, and portable format adapters only as actual use cases justify them.
- Allow communities to fork the capability map and govern local versions without losing source or standards links.
- Introduce credentials only after issuer criteria, learner consent, evidence requirements, verification, status, and privacy semantics are separately qualified.

## 10. Acceptance criteria for this RFC to become implementation

- A novice can create a useful draft without editing code or structured data manually.
- The learner preview matches the exported lesson in all behavior that the format claims to preserve.
- No missing, ambiguous, or incompatible content licence can pass the public-release gate.
- All lesson releases have traceable version identity and creator/source provenance.
- Hard prerequisite edges are distinguishable from soft relationships and cannot be silently invented by an importer or generator.
- A generated answer key cannot be represented as human-reviewed unless an explicit reviewer event supports the claim.
- The platform does not infer verified mastery, credential eligibility, or authorization from local progress, time spent, a self-rating, or activity completion.
- At least one full author-to-review-to-export-to-import journey passes deterministic tests on an exact source revision.
- Creator usability, accessibility, and learning efficacy are reported separately; no unmeasured target is described as achieved.
- The pilot has an independent unaided transfer measure and a delayed-retention plan before any claim that Praxis improves learning over a comparator.

## References and design anchors

- UNESCO, [Open Educational Resources](https://www.unesco.org/en/open-educational-resources): open licensing, reuse, adaptation, and redistribution.
- CAST, [Universal Design for Learning Guidelines 3.0](https://udlguidelines.cast.org/): barrier-aware access and multiple means of engagement, representation, and expression.
- UNESCO, [AI competency framework for students](https://www.unesco.org/en/articles/ai-competency-framework-students): human-centred mindset, AI ethics, techniques/applications, and system design, with Understand / Apply / Create progression.
- 1EdTech, [Common Cartridge](https://www.1edtech.org/standards/cc), [standards overview](https://www.1edtech.org/specifications), and [Open Badges](https://www.1edtech.org/standards/open-badges): existing interoperability and portable-credential work to build on.
- H5P, [content types](https://h5p.com/content-types) and [2026 authoring direction](https://h5p.com/resources/blog/whats-coming-next-for-h5p): interactive-content authoring, accessibility, and reuse are active areas of competition.
- W3C, [Verifiable Credentials Data Model 2.0](https://www.w3.org/TR/vc-data-model-2.0/): standard data model for verifiable claims, including privacy, accessibility, and internationalization considerations.
- World Economic Forum, [Future of Jobs Report 2025 — Skills Outlook](https://www.weforum.org/publications/the-future-of-jobs-report-2025/in-full/3-skills-outlook/): one employer-survey perspective on analytical thinking, adaptability, technological literacy, creativity, and lifelong learning. It is an input, not a universal curriculum authority.
- [CreatorBench, PedBench, and the curriculum-audit/community-authoring proposal (PR #5368)](https://github.com/Luminous-Dynamics/mycelix/pull/5368), the focused [Community Creator Studio proposal (PR #5406)](https://github.com/Luminous-Dynamics/mycelix/pull/5406), and the [essential-capabilities curriculum seed (PR #5273)](https://github.com/Luminous-Dynamics/mycelix/pull/5273): implementation and evaluation context to reconcile against current exact heads before this RFC is implemented.

---

**Claim ceiling:** This RFC defines a proposed architecture, evaluation plan, and research hypotheses. It is not evidence that Praxis currently outperforms Wikipedia, H5P, OpenStax, an LMS, or any learning intervention. That requires measured, reproducible, fair comparisons.
