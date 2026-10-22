# Praxis CreatorBench v1 — contributor usability and quality benchmark

**Status:** design proposal; targets below are not measured results.  
**Owner:** Mycelix Praxis.  
**Goal:** make it possible for a person with useful knowledge—but no programming background—to create, validate, share, and improve a learning resource without weakening quality, accessibility, or reuse-rights review.

## 1. Product hypothesis

Praxis should treat community authoring as a first-class learning pathway. A community member can know how to repair a bicycle, explain fractions, teach a language, grow food, understand taxes, or run a science experiment without knowing JSON, standards metadata, instructional-design terminology, or a code repository.

The interface should translate that practical knowledge into a draft structure, explain gaps in plain language, help the creator check it, and preserve attribution and rights. The exported object remains a **draft** until the appropriate review has approved that exact revision.

## 2. Industry comparison

Use three distinct comparison classes; they solve different problems, so there is no meaningful single “better than H5P” score.

| Comparator | Established strength | Praxis opportunity |
|---|---|---|
| H5P | A large library of interactive content types, import/export and reuse workflows; publishes accessibility information by content type. | Start with a small number of high-quality, format-portable lesson patterns; bind objectives, review receipts and provenance to an immutable asset digest rather than optimizing for widget count. |
| OpenStax | Faculty authorship, subject-expert and editorial review before free textbook publication. | Allow much broader contribution, including practitioner and learner-created resources, while applying risk-based review instead of claiming every contribution is textbook-quality automatically. |
| UNESCO OER guidance | Open licensing and the capacity to create, access, adapt and redistribute materials are core OER principles. | Make rights/attribution understandable inside the authoring flow and preserve remix lineage in exported lesson packs. |
| WCAG 2.2 | A recognized accessibility target, including keyboard access, focus visibility, target size, and accessible form behavior. | Make the **authoring tool** accessible, not merely the lesson output, and provide non-drag, low-bandwidth and alternative-format paths for authors and learners. |

References: [H5P content types](https://h5p.com/content-types), [H5P reuse workflow](https://h5p.org/reuse-h5p-content), [H5P content-type accessibility recommendations (2026)](https://help.h5p.com/hc/en-us/articles/7505649072797-Content-types-recommendations), [OpenStax author/reviewer process](https://help.openstax.org/s/article/What-is-the-author-and-reviewer-process-for-OpenStax-books?language=en_US), [UNESCO Recommendation on OER](https://www.unesco.org/en/legal-affairs/recommendation-open-educational-resources-oer), and [W3C WCAG 2.2](https://www.w3.org/TR/WCAG22/).

H5P’s own 2026 product direction highlights faster creation, easier accessibility, reuse and smoother workflows. Those are table stakes, not a defensible differentiation claim. Praxis should compete on **open portability + author guidance + curriculum alignment + evidence-bound review + proven learning outcomes**.

## 3. Benchmark tasks

Give novice contributors the same short, counterbalanced tasks in Praxis and a selected comparator:

1. Create a lesson about something they know from a blank draft.
2. Start from a template and adapt it for a different age or skill level.
3. Write three practice questions, answer keys, explanations and plausible wrong-answer choices.
4. Remix a permitted open lesson and preserve attribution, licence, and unedited items.
5. Export a file, fix validation findings, and describe what is still needed for publication.
6. Use the editor with keyboard-only navigation and a screen reader; repeat on a narrow mobile viewport or low-bandwidth setup.

Recruit non-programmer novice authors, experienced teachers, technical educators, adult/vocational educators, and authors working in more than one language. Report each group separately. Obtain consent for research instrumentation; do not upload draft contents or learner data by default merely to calculate usability metrics.

## 4. Proposed internal stretch targets

These are testable design goals, not present-day Praxis results or industry averages.

| Metric | Proposed target | How to measure |
|---|---|---|
| Time to first exportable draft | Median ≤8 minutes; p90 ≤15 minutes for the starter task | Screen-recorded consented usability sessions; count from task start to valid JSON export. |
| Novice task success | ≥85% of novice participants export a structurally valid draft within 15 minutes | Preregister success criteria; report denominator and confidence interval. |
| First-pass quality | ≥80% of exported novice drafts pass structural validation without a critical blocker after one normal edit cycle | Run the same versioned validator on exact exports. |
| Rights comprehension | ≥90% correctly explain that a source link is not permission and can distinguish draft from approved publication | Short teach-back, not a confidence-rating question. An undecided licence must never be disguised as permission. |
| Remix safety | 100% preservation of unedited items/unknown metadata in tested supported round trips; zero silent destructive changes | Exact fixtures and field-by-field diff. |
| Critical assessment defects | 100% detection on a frozen negative-control suite for exact/numeric-equivalent answer-distractor collisions and incomplete choice sets | Fault-injected fixtures; report false positives as well as false negatives. |
| Accessibility | No known critical WCAG 2.2 A/AA blocker in core author tasks; 100% completion of core keyboard-only tasks | Automated checks plus human keyboard, screen-reader, zoom/reflow and target-size testing; automated tools alone are insufficient. |
| Review throughput | Reduce median reviewer time per accepted contribution without reducing critical-defect detection | Randomized reviewer workload study with fixed content and blinded rubrics. |
| Equity of contribution | No material success-rate gap by experience, language, disability/access mode, or device without a documented mitigation plan | Report subgroup estimates only where privacy and sample size permit. |

Set the baseline before claiming gains. Publish study task scripts, versioned exports, scoring rules, failures and uncertainty. Do not promote these goals as achieved results.

## 5. Quality and contribution state machine

Keep author UX simple but make every state explicit:

- **Draft:** creator can freely edit; no claim of correctness or approval.
- **Local checks passed:** structural and deterministic checks found no known blocker; not a pedagogical or legal approval.
- **Review requested:** export includes a stable identifier, exact content digest, author credit/pseudonym, language, source/rights notes, and validation report.
- **Human reviewed:** named review role and scope are bound to the exact digest; fixes create a new revision and invalidate the old review.
- **Pilot tested:** consented study/pilot evidence is attached with known limitations.
- **Published/reusable:** release policy is satisfied; approved attribution and licence are visible. Content can be forked without erasing its origin.
- **Superseded/withdrawn:** old identifiers remain resolvable for audit, but a withdrawn or rights-infringing version is no longer offered as a valid learning resource.

Review is risk-based: a low-risk original practice exercise should not face the same process as medical advice, hazardous lab instruction, or safety-critical electrical work. A critical accuracy, rights, privacy or accessibility blocker vetoes publication regardless of average quality score.

## 6. Paradigm shift: contributors create portable learning objects

The contribution unit should be a versioned lesson object rather than a page tied to one platform. Each object carries:

- a human-readable purpose, audience, prerequisite assumptions and learning objective;
- explanatory content, a worked example, practice with feedback and a method to assess understanding;
- language/localization, accessibility descriptions and low-bandwidth alternatives where relevant;
- authorship, sources, licence/rights, remix lineage and exact content digest;
- validation, review receipts, known limitations and (only when available) independently measured learning evidence.

The UI should not require contributors to understand every field. Ask for the practical teaching intent first, then reveal details progressively. Show a learner preview beside the edit form, suggest templates, let users save/reopen locally, and explain each validation finding in terms of what the creator should change. AI may draft or translate, but must label suggestions, keep provenance visible, and never self-approve its own output.

## 7. Current scope and next work

The current Leptos editor at `/author` is local-first: it saves to browser storage, accepts a lesson-pack JSON for remix, performs quick checks, and exports JSON. It does not publish to the live curriculum automatically. The new template starters reduce the blank-page problem, and a live quick-check count makes progress visible.

Next:
1. Add preview-as-learner and a structured, downloadable validation report.
2. Add a guided open-licence chooser with plain-language explanations, source-per-item attribution, and compatibility warnings; do not select a licence on the author’s behalf.
3. Build an exact-digest review-request bundle with the lesson pack, evidence sidecar, checker result and an explicit list of remaining blockers.
4. Run usability sessions against the targets above; iterate based on observed abandonment and errors, not preferences alone.
5. Only then broaden into branching lessons, question banks, richer multimedia and collaborative co-authoring.

The north-star metric is **the number of independently reviewed, reusable learning objects that help learners demonstrate durable capability per contributor hour**, disaggregated by language and audience. Raw exports and generated asset counts are not success measures.
