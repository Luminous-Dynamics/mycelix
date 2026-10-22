# Community Content Commons: authoring, remixing, interoperability and benchmarks

**Status:** proposed architecture and internal targets; not claims of shipped product capability or measured superiority.

## North star

A person with useful knowledge should be able to turn it into a lesson, adapt somebody else's lesson lawfully, translate it for their community, get useful review, and share it without becoming a software engineer or giving up ownership. A learner or educator should be able to trust which revision was reviewed, what it is intended to teach, what rights apply, and what the evidence actually supports.

Praxis should compete on the complete author-to-learning lifecycle, not merely the speed of generating text.

## The paradigm shift

**From a content editor to an evidence-aware, interoperable teaching commons.**

The editor is only one component. The larger system should connect six things without conflating them:

1. **Pedagogical content:** objective, explanation, worked examples, progressively revealing hints, practice, assessment, feedback, vocabulary and misconceptions.
2. **Curriculum mapping:** references to one or more standards/frameworks, the precise mapping status, rationale and source version.
3. **Rights and provenance:** author credit, sources, source-specific reuse conditions, required attribution, transformation history and original lesson lineage.
4. **Quality evidence:** deterministic findings, human-review receipts, accessibility checks and the exact lesson digest that each review covered.
5. **Interoperability:** import/export adapters with a visible loss report rather than a silent best-effort conversion.
6. **Learning outcomes:** studies and reviewed learner-level aggregates, kept separate from structural checks and tutor benchmark scores.

The internal lesson-pack JSON is a useful canonical representation of teaching intent. It should not replace mature exchange standards. Keep adapters separate, versioned and testable so the core format can evolve without trapping an educator inside Praxis.

## Current starting point

The first implementation lives in `mycelix-praxis/authoring/` and the Leptos `/author` route. Authors can use a guided form, local browser autosave, quick checks, JSON preview/download and import an existing lesson pack for remixing. Imported extra items and unknown metadata are retained on export; items not yet editable in the form remain preserved. A Rust command checks individual drafts and emits a digest:

```sh
cargo run -p praxis-curriculum-audit -- --check-lesson authoring/drafts/my-lesson.json --format=markdown
```

This is a real initial authoring workflow, not a claim that all import paths, all content types, the user interface, or every browser accessibility mode is production-ready. Exact-head build/test qualification and usability review are still required.

## Interoperability strategy

Use the correct standard at each boundary; don't force one standard to do every job.

| Boundary | Candidate format/standard | Praxis adapter requirements |
|---|---|---|
| Canonical lesson with teaching intent | Versioned Praxis lesson pack + evidence sidecar | Stable IDs; explicit language/objectives; review state; exact digests; preserve unknown extensions |
| Course/LMS/repository packaging | [1EdTech Common Cartridge](https://www.1edtech.org/standards/cc) | Import/export packages, resource links, manifest and course structure; retain a mapping/loss report |
| Assessment items and tests | [1EdTech QTI 3](https://www.1edtech.org/standards/qti) | Map question types, scoring, rubrics, feedback, accessibility accommodations and response processing; explicitly report unsupported interactions |
| Interactive activities | [H5P](https://h5p.org/getting-started) `.h5p` packages | Preserve licensing, attribution, interaction type and accessible alternatives; never claim round-trip equivalence where semantics differ |
| Secure integration into an LMS | [1EdTech LTI 1.3 / LTI Advantage](https://www.1edtech.org/standards/lti) | Use for authenticated tool launch and supported services, not as a substitute for content portability; minimize personally identifiable data |

Common Cartridge packages learning materials and assessments; QTI standardizes assessment exchange and supports scoring semantics; LTI connects learning tools to platforms with a stronger security model. These are complementary boundaries, not competitors to the internal lesson-pack representation.

### Loss-aware round-trip contract

Every import/export adapter should produce a machine-readable receipt with:

- source file digest, format and adapter version;
- fields and assets imported, mapped, preserved, transformed, or omitted;
- every unsupported interaction and whether it blocks export;
- rights and attribution copied through from each source item;
- output digest and a deterministic normalized diff;
- explicit statement of whether semantic equivalence was verified, merely approximated, or not assessed.

For formats/fields claimed as lossless, the round-trip test must show zero data loss. For unsupported interactions, preserve the source package as an opaque attachment and display a clear warning; don't silently flatten it into a poor approximation. A remix must create a new revision and never mutate the upstream work.

## Author contribution workflow

1. **Start with the learner capability.** Search active objective IDs and framework mappings. If no suitable objective exists, label the ID provisional and request review; don't invent an official standards code or silently activate a graph node.
2. **Create or remix locally.** The browser form should be usable without an account or network connection. Source packs are read locally, remain local until explicit export, and retain untouched fields.
3. **Make good pedagogy easier.** Guided fields should prompt for worked steps, several varied practice tasks, staged hints, misconception repair, feedback/rubrics, accessible alternatives and a meaningful delayed-transfer check.
4. **Check before submission.** Run known-defect filters and structural checks, calculate exact content digests and validate assessment scoring. Static checks must distinguish blockers from advisories.
5. **Record sources and rights.** Require contributor credit, actual licence/rights status, source URLs and scope. A URL by itself is not permission. Respect compatibility constraints when multiple licences are combined. Where rights are unknown, permit private drafting but block public redistribution.
6. **Review the exact revision.** Subject expert, educator, accessibility, rights and age/safety checks are separately recorded. A content edit invalidates the prior content digest and must trigger the relevant checks again.
7. **Publish, translate and branch.** Public lessons have immutable revision IDs; adaptations and translations are child revisions linked to upstream lineage, attribution and local changes. The original author's material is not overwritten.
8. **Learn from use responsibly.** Accept educator/learner issue reports and proposed edits. Share only consented, privacy-reviewed aggregate outcome evidence; never publish raw student records or treat unreviewed clicks as learning gains.

### Contributor dignity and governance

- No account requirement for local drafting; a shareable pull request or package is a valid contribution route.
- Authors choose attribution or an allowed pseudonym. Do not force public identity disclosure.
- Contributors can choose a licence only for content they have authority to license. Third-party items retain separate rights records.
- Preserve the source and credit the original author for remixes and translations.
- Separate contribution reputation from learner worth, public grades and employment access. Appeals and correction history remain available.
- Use local reviewer communities for subject/language context with a transparent escalation path for contested work.

## Benchmarks to beat the field

These are **proposed internal stretch targets**, not current results or industry-wide norms. Establish a baseline on the current app before claiming improvement.

### Authoring usability and speed

| Metric | Initial target |
|---|---|
| First-time author completing a useful draft with no coding | At least 90% task completion in a moderated usability study |
| Time to first structurally valid three-practice-item lesson | Median ≤10 minutes; 90th percentile ≤20 minutes |
| Remix of a simple existing lesson | Median ≤2 minutes to import, change one teaching element and export a reviewable draft |
| Supported schema round-trip | 100% of supported fields/assets preserved; 0 silent loss |
| Unsupported source elements | 100% explicitly reported; no quiet truncation |
| Browser-local privacy | No network upload of drafts without explicit action; automated test confirms the local-only path |
| Accessible author workflow | All core tasks keyboard-operable; test with screen reader and WCAG 2.2 AA review before claiming conformance |
| Rights gate | Zero publication of content whose overall or embedded-source rights status is unknown or incompatible |

### Content and tutoring quality

- Reproduce comparable tasks on [KMP-Bench](https://ojs.aaai.org/index.php/AAAI/article/view/40578), [MMTutorBench](https://aclanthology.org/2026.acl-long.1068/), and the relevant public tutoring tracks before tuning to their scores.
- On two comparable pedagogical benchmark tracks, target at least +5 absolute macro-F1 over the strongest reproducible baseline, with paired 95% confidence intervals excluding zero. Do not regress materially on other tracks or critical-veto rates.
- Detect 100% of a frozen initial set of known critical negative controls (wrong answer keys, correct answer in distractors, equivalent numeric choices, prohibited answer leakage, generic placeholders). Zero misses in that finite set is a measured scope, not proof of universal correctness.
- Report human-label agreement per rubric dimension. Use two independent educator annotations and adjudication; revise weak rubrics rather than treating noisy labels as gold.
- Report results by grade/course, subject, language, disability/access modality and difficulty. Do not hide a language or subgroup regression inside a macro-average.

KMP-Bench tests K–8 mathematical dialogue and foundational teaching skills, and its authors report that models can solve verifiable problems yet struggle to consistently apply pedagogical principles. MMTutorBench evaluates 770 multimodal math-tutoring problems with problem-specific rubrics across six dimensions. These benchmarks are useful diagnostics, but their scores are not direct evidence of learner efficacy.

### Evidence that a learner actually learned

A strong benchmark score is evidence about tutor behaviour, not durable learning. Use the [Learning Impact Protocol](../benchmarks/pedbench-v1/learning-impact-protocol.md) and keep distinct outcomes separate:

- one pre-registered primary endpoint: unaided delayed near-transfer performance around 7 days after the intervention;
- around 28-day retention where feasible;
- immediate aided performance and its gap versus delayed unaided transfer;
- misconception recurrence, attempts to recover from errors and time-to-competency;
- student/educator effort, cost, accessibility, attrition and possible displacement of teacher or peer interaction.

Make causal claims only with a design and analysis that justify them. Replicate material effects in a second materially different context before broad effectiveness claims. The benchmark run receipt must freeze dataset, model/policy, prompt/rubric and analysis digests, uncertainty, limitations and independent replication evidence.

## Implementation order

**P0 — finish the authoring vertical slice:** exact-head Leptos compile/tests, accessible keyboard/screen-reader review, clean static report, source and rights controls, and tests for local-only saving and non-destructive remix.

**P1 — round-trip interoperability:** add an export/import loss-report interface and golden fixture corpus for Common Cartridge, QTI 3, and H5P; begin with one supported subset per format and explicitly classify unsupported fields. Do not claim conformance without the relevant standard's tests.

**P2 — objective and community workflow:** objective search with framework/version tags, source-backed objective mapping proposals, versioned review receipts, attributed translation/fork workflow and local reviewer queues.

**P3 — evidence and efficacy:** immutable benchmark runs, educator-reviewed gold labels, preregistered learner pilots and independent replications. No public superiority claim until the method is reproducible and the relevant outcome evidence supports it.

## External references

- H5P describes current priorities around reducing authoring friction, accessibility, reuse and smoother authoring workflows: [What's coming next for H5P (21 Aug 2026)](https://h5p.com/resources/blog/whats-coming-next-for-h5p).
- [OER Commons remix guidance](https://help.oercommons.org/support/solutions/articles/42000046860-remix-a-resource) emphasizes checking reuse rights and adapting for grade, locale, learning needs and updated information.
- [CAST UDL Guidelines 3.0](https://udlguidelines.cast.org/) emphasize learner agency, multiple means of representation and response, accessibility, collaboration and transfer.
- [1EdTech Common Cartridge](https://www.1edtech.org/standards/cc), [QTI](https://www.1edtech.org/standards/qti) and [LTI](https://www.1edtech.org/standards/lti) supply complementary exchange, assessment and integration boundaries.
