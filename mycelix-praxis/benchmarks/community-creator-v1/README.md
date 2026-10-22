# Praxis Community Creator Benchmark v1

**Status:** proposed pilot protocol; targets are hypotheses, not achieved results.  
**Purpose:** measure whether ordinary people can author useful, reusable learning content without needing software-engineering skills, while preserving instructional quality and rights.

## 1. Compare authoring workflows fairly

Evaluate at least these task conditions when practical:

1. **Starter-file baseline:** a participant edits the JSON starter with their normal editor.
2. **Praxis Creator Studio:** the participant uses the guided `/create` form and downloads a private draft.
3. **External authoring baseline:** where a task maps fairly to the tool's supported content types, use an established authoring workflow such as H5P and record its export/import limitations.

Give participants the same learning objective, intended learner group, source material and task requirements. Counterbalance task order where feasible. Do not compare a simple single-page lesson with a highly interactive activity as if they were equivalent. Record the tool version and exact task instructions.

## 2. Proposed usability targets

These are internal stretch targets for an initial mixed-experience usability pilot. Revise them based on observed baselines before any public claim.

| Metric | Proposed target | How to measure |
|---|---:|---|
| First-session task success | At least 80% of first-time creators produce a schema-valid draft without live assistance | Count successful participants over all participants assigned the task; report confidence interval |
| Time to valid first draft | Median ≤10 minutes; 90th percentile ≤20 minutes | Start at task instruction, stop at first valid JSON pack; exclude setup time only if excluded for all workflows |
| First-build schema pass rate | ≥95% | Valid first exports / first exports attempted |
| One-revision completion | ≥99% schema-valid after one guided repair pass | Valid exports after first repair / repair attempts |
| Local draft preservation | 100% of tested reload/reopen scenarios preserve saved fields | Frozen test matrix across supported browsers; test normal reload and recoverable navigation |
| Rights/provenance completeness for public-submission candidates | 100% contain attribution, declared licence, rights confirmation and source notes where third-party material is present | Deterministic gate on a frozen positive/negative-control corpus |
| Critical defect controls | 100% detection on the frozen known-defect suite; report false-positive rate on negative controls | Missing fields, empty/duplicate alternatives, exact/numeric-equivalent answer collisions, missing rights data and broken objective IDs |
| Keyboard/accessibility task success | ≥95% for the equivalent authoring task using keyboard-only navigation, with no critical keyboard trap | Manual run plus automated accessibility checks; separately verify screen-reader labels and error messages |
| Educator review quality | ≥80% of submitted pilot drafts pass the agreed rubric after at most one revision; zero known critical factual/answer-key defects in approved assets | Two independent subject-appropriate reviewers plus adjudication; never treat creator success alone as evidence of pedagogical quality |

Publish denominators, exclusion rules, task order, prior authoring experience, confidence intervals and any workflow failures. Do not report only the best-performing cohort.

## 3. Measure the whole authoring loop

Track the authoring stages as separate outcomes:

1. **Draftability:** how quickly someone can explain the intended learning objective in their own words.
2. **Instructional completeness:** worked reasoning, progressive hints, practice, answers/explanations, assessment, vocabulary and misconceptions.
3. **Mechanical validity:** schema validity and deterministic defect controls.
4. **Rights and provenance:** original versus adapted components, source attribution, reuse licence and creator-selected credit.
5. **Reviewability:** whether another educator can understand the objective, verify the answers and review accessibility without asking the creator to reconstruct hidden context.
6. **Activation:** deliberate curriculum ID mapping and manifest registration by a maintainer; no creator draft is automatically activated.
7. **Learning evidence:** a separate research layer requiring independent learner outcomes, delayed unaided retention and transfer. Creator usability does not measure learning efficacy.

Useful cost measures include minutes of author effort, minutes of educator review, number of revision rounds, and cost per *reviewed and reusable* lesson. Cost per generated file is misleading if the file cannot be checked or reached.

## 4. Rights, provenance and collaboration controls

The frozen test corpus should include cases where:

- a public draft has no licence or attribution;
- an author chooses an open licence but has not confirmed rights to embedded media;
- third-party sources are listed without rights notes;
- a translated or remixed lesson drops the original attribution or changes licence metadata;
- a fork silently changes the objective ID;
- an otherwise valid draft has an answer-key or distractor collision.

A public submission must fail closed on missing required rights metadata. A declared licence is not legal proof that the contributor owns the material; human review remains appropriate for uncertain cases.

For collaborative versions, preserve an immutable revision identifier, creator attribution, parent revision, licence and change summary. Derivative work should record what changed and must not overwrite or erase the originator's credit.

## 5. Industry baseline and differentiation

Do not claim that this first Creator Studio is more capable than H5P's mature authoring system. H5P already offers a broad library of content-type templates, browser-based authoring, import/export, multilingual workflows and reuse guidance. Its August 2026 roadmap also emphasizes faster creation, better accessibility, consistency and reuse at scale ([H5P authoring guide](https://h5p.org/documentation/for-authors/), [H5P roadmap note](https://h5p.com/resources/blog/whats-coming-next-for-h5p)). Praxis should use H5P as an interoperability/authoring baseline, not attempt to rebuild every interactive content type.

1EdTech Common Cartridge already standardizes packaging and exchange of learning resources and assessments across compatible LMS/LOR systems ([1EdTech Common Cartridge](https://www.1edtech.org/standards/cc)). Creative Commons licences already formalize public reuse choices and required attribution ([licence chooser](https://creativecommons.org/chooser/)). Praxis should implement compatible exports and clear rights metadata instead of inventing a closed licensing or course-package standard.

Praxis's defensible differentiation should be the *evidence chain around the lesson*: stable objective/node mapping, explicit draft/review/publication states, deterministic answer-key and schema checks, exact-revision provenance, educator review, and eventual learning-outcome evidence. This combination is a product hypothesis to test—not yet a demonstrated competitive advantage.

For the first pilot, compare Praxis with an H5P authoring workflow on equivalent tasks and report task success/time, revision count, accessibility and interoperability losses. Only compare scores on content types and task instructions both workflows can complete fairly.

## 6. Interoperability roadmap

Keep the native Community Lesson v1 JSON straightforward and versioned. Then add adapters rather than locking creators to one platform:

- [**H5P authoring and reuse guides**](https://h5p.org/documentation/for-authors/) for supported interactive content types and its established authoring/reuse ecosystem.
- [**1EdTech Common Cartridge**](https://www.1edtech.org/standards/cc) for packaging and exchanging compatible course resources and assessments with LMS/LOR tools.
- [**Creative Commons licence chooser**](https://creativecommons.org/chooser/) for authors selecting public reuse permissions.
- Portable plain-text/Markdown source with a deterministic converter to the native lesson pack.
- Translation and fork manifests that preserve objective references, source rights and lineage.

Do not claim standards conformance until round-trip fixtures and the relevant conformance requirements are tested. A lossless round trip must preserve the content, assessment semantics, attribution/licence information and objective references or explicitly report any unsupported fields.

## 7. Privacy and research ethics

The first Creator Studio saves draft text locally in the current browser and exports it only when the creator initiates a download. A usability pilot should obtain informed participation, avoid real learner records, and store only the minimum anonymized task timing, validation results and reviewer labels. Do not collect draft text or browser telemetry by default just to compute authoring metrics.

Any public usability claim should be based on a predeclared protocol, representative creator roles, and reported limitations. Benchmarks of creation speed must not override lesson correctness, creator agency, accessibility or rights.
