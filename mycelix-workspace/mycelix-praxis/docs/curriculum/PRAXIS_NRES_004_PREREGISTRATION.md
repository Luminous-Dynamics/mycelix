# PRAXIS-NRES-004: Preregistration and benchmark-split governance

**Status:** draft protocol template; no pilot or confirmatory study has been conducted.  
**Purpose:** make claims about learning gains and transfer reproducible, cost-aware, privacy-preserving, and resistant to leakage.

## 1. Scope and non-claims

This protocol evaluates whether a learning experience improves durable civic-resilience reasoning relative to a credible existing baseline. It does not evaluate military performance, recruitability, ideology, political loyalty, or support for any particular service policy. It does not establish that a synthetic municipal plan will work in the real world.

The currently published PRAXIS-NRES-002 fixture and its eight variants are **public development cases**. They are not a held-out set and results on them cannot support a confirmatory generalization claim.

## 2. Benchmark split policy

Maintain three distinct sets with versioned manifests:

- **Development/public:** fixtures and mutations available to authors and learners for transparent practice, schema tests, examples, and baseline debugging.
- **Pilot/calibration:** separate cases used to refine prompts, rubric anchors, accessibility, scorer training and variance estimates. Pilot cases and outcomes must not be reused as confirmatory outcomes.
- **Confirmatory/held out:** cases not available to course authors, participating learners, or assessors before the outcomes and exclusions are locked. The custodian is independent of course authors and has a documented conflict-of-interest policy.

Before confirmatory collection begins, preregister hashes of the course build, rubric, analysis code, case-generator specification, public development manifest, and confirmatory-case commitment. The independent custodian should retain case inputs and answer/scoring keys in controlled storage. Publish the commitment digest and custody procedure. After scoring and the analysis are locked, release the case materials, keys, hashes and exclusion log when lawful and safe, so independent reviewers can reproduce the analysis.

A hash commitment alone does not create a secure holdout. Access control, independent custody, exposure logs, a freeze date and a disclosure policy are required. If confirmatory items leak, record the exposure and exclude or analyze the affected cases according to the preregistered rule; do not silently replace them after seeing outcomes.

## 3. Recommended primary question

Does the course improve learners' ability to produce a feasible, evidence-bounded and rights-aware response to a novel civic-resilience scenario after a delay, compared with an established preparedness course or equivalent instructor-led baseline?

A study must select exactly one primary learning endpoint before confirmatory collection. A useful default candidate is the blinded rubric score on a held-out transfer task at 30 days, with each R1–R8 dimension and every non-compensable flag also reported separately. Treat any total as an evaluation summary, not as a political alignment score.

Critical rights/authority or fabricated-evidence errors must be reported separately and cannot be obscured by an average. Define the error categories and any program-level decision bound before study launch; do not invent an acceptable threshold after seeing results.

## 4. Preregistration fields required before launch

Record, version and timestamp:

1. Research question, target population, eligibility rules and intended deployment setting.
2. Course, baseline and assessment versions, content hashes and delivery conditions.
3. Primary endpoint, operational definition, scoring rule and time point.
4. Secondary outcomes: immediate post-test, 30-day and 90-day retention, held-out transfer, resource-feasibility errors, uncertainty calibration, hard-fail categories, access barriers, completion, attrition and fully loaded cost.
5. Assignment design, randomization unit, allocation procedure, contamination controls and any clustering.
6. Sample-size rationale, minimum practically important effect, desired statistical power, anticipated attrition and assumptions used. Estimate these from a separate pilot or defensible prior data; do not choose them by looking at confirmatory outcomes.
7. Analysis model, covariates, multiplicity handling, confidence intervals, missing-data approach, exclusions, sensitivity analyses and stopping rules.
8. Assessor training, blinding, double-scoring fraction, disagreement resolution, reliability statistic and criteria for freezing the rubric.
9. Confirmatory case custodian, split manifest commitment, access log, leak response and post-analysis disclosure plan.
10. Accessibility accommodations, language handling, informed consent, privacy controls, retention/deletion schedule and responsible contact.
11. Fully loaded cost model and included/excluded cost categories.
12. Publication plan that includes negative, null, harm, dropout and inconclusive outcomes.

No field may be silently changed after outcome inspection. Corrections require a dated amendment explaining whether any outcome data were already observed.

## 5. Outcome model and benchmark dashboard

Report absolute values, denominators, uncertainty intervals and missingness for each metric. Do not report only a single leaderboard number.

| Metric | Operational definition | Minimum reporting requirement |
|---|---|---|
| Immediate knowledge gain | Difference from baseline using a frozen aligned assessment | Estimate, interval, denominator and missingness |
| 30-day retention | Performance on a delayed assessment using a parallel form | Estimate versus comparator, interval and attendance |
| 90-day retention | Performance at a longer delay, if feasible | Same reporting as 30-day; no substitution of self-report for performance |
| Transfer | Blind-scored performance on genuinely held-out fictional cases | Dimension scores, hard-fail categories, uncertainty and case-exposure audit |
| Resource feasibility | Fraction of submissions whose arithmetic and service-floor checks are valid | Errors by type, not just an aggregate |
| Evidence calibration | Correct identification of facts, assumptions, unknowns and source conflicts | Calibration error and examples of systematic failure |
| False confidence | Gap between learner confidence and demonstrated accuracy, using a preregistered confidence measure | Gap distribution and relationship to error severity |
| Rights/authority failures | Predefined occurrence of invented authority or material rights/due-process omissions | Counts and denominators, independent review and no political inference |
| Assessor reliability | Agreement on independently scored work | Dimension-level agreement statistic, interval and prevalence caveat |
| Access and inclusion | Completion and assessment barriers by ethically collected, minimized, relevant variables | Aggregates, small-cell protection and accessibility findings |
| Cost per competent outcome | Fully loaded program cost divided by learners reaching a preregistered competence criterion | Cost boundary, numerator, denominator, sensitivity and comparator |
| Run accounting | Every scheduled attempt has a score, missingness reason or invalid-run disposition | 100% explicit disposition; do not delete failed or absent runs |

All targets tied to educational effect size, attrition, reliability or cost must be chosen before confirmatory collection. A weighted-kappa value of 0.75 can be explored as an initial reliability hypothesis, but it is not asserted here as an external field standard.

## 6. Fair comparison with state of practice

Select the comparator before recruitment. It should be an existing preparedness curriculum or instructor-led program relevant to the target population, delivered with a documented time and support budget. Give both groups equivalent assessment conditions and avoid testing only material unique to Praxis.

Where feasible and ethical, randomize at the learner or class level. If class-level assignment is used, account for clustering. If randomization is impractical, predefine the quasi-experimental design and likely confounders. Report baseline differences and avoid causal language beyond the design.

Compare:
- durable performance and transfer, not completion alone;
- results per learner assigned as well as per learner completing;
- full delivery costs, including instructor, support, translation/accessibility, assessment, technology and follow-up;
- error rates and harms alongside average score;
- access barriers and missingness alongside headline performance.

Do not claim "better than state of the art" merely because the course contains more content, has a higher unblinded post-test score, or wins on the public fixtures that informed its design.

## 7. Privacy, consent and purpose limitation

Default to local-only learning records. For a study, collect only the minimum data required by the preregistered outcomes and ethics review. Do not collect or infer political affiliation, support for conscription, willingness to enlist, loyalty, or personal suitability for coercive service. Do not enable federated learning for this draft course.

Make participation in research distinct from access to public preparedness education. A participant must be able to decline research use without losing access to learning resources. Set deletion and retention dates before collection; publish only aggregates that protect small groups. Any optional accessibility data must be consented, purpose-limited and held separately from scoring where practical.

## 8. Review and stop conditions

Pause interpretation and seek independent review if any of the following occur:
- an assessment is being used for recruitment, political profiling or eligibility for coercive service;
- a material privacy leak or unauthorized secondary use occurs;
- a holdout is materially exposed;
- assessors cannot reliably apply the rubric;
- a task requires unsafe real-world activity, unauthorized investigation, or unqualified medical practice;
- the baseline or course delivery differs materially from the frozen protocol;
- a critical case defect makes the primary outcome uninterpretable.

A paused, null, negative, or inconclusive study is a valid result and must not be represented as success. Any revision creates a new version and, where needed, a new confirmatory cohort.

## 9. External alignment and implementation boundary

The design is informed by the UNDRR Strategic Framework 2026–2030, which emphasizes risk knowledge, locally led disaster risk reduction, financing and recovery readiness, and by UNESCO assessment guidance that recommends a range of assessment methods aligned with learning objectives. These are design references, not endorsements of this course or proof of its effectiveness.

The public course and fixture remain inactive until content review, stable curriculum node ownership, lesson integration, manifest validation and accessibility review are complete. This protocol does not authorize deployment, emergency response, official training credit or certification.
