# Praxis Learning Experiment Preregistration Template

**Purpose:** define a learning evaluation before outcomes are inspected. This template is for educational evaluation, not clinical or safety-critical research clearance. Obtain the required institutional/ethics review, educator/guardian permission, and learner assent/consent appropriate to jurisdiction and age before recruiting participants.

## 1. Immutable protocol identity

- **Protocol ID and version:**
- **Status:** draft / reviewed / preregistered / closed
- **Date and time registered (UTC):**
- **Protocol digest:** BLAKE3 or another explicitly named cryptographic digest of the canonical protocol bytes
- **Lesson/course ID and exact content digest(s):**
- **Instructional policy ID/version and configuration digest:**
- **Tutor/model/provider/version**, if applicable:
- **Assessment instrument and scoring rubric version(s):**
- **Analysis code/version or digest:**
- **Responsible investigator and independent reviewer:**
- **Registration URI or immutable record reference:**

Any material post-registration change must create a new version with rationale, timestamp, predecessor reference, and a clear list of changed fields. Never silently overwrite the hypothesis or primary outcome after observing results.

## 2. Research question and claim boundary

- **Primary research question:**
- **Target skill/capability and learner population:**
- **What result would count as meaningful improvement?**
- **What would count as no benefit or harm?**
- **Claim type:** descriptive / predictive / comparative / causal
- **Claim scope:** exact course, topic, population, language, platform, context and duration
- **What this study cannot establish:**

A random assignment can support causal claims only when its design, assignment integrity, compliance, attrition, and analysis justify them. Observational or product-telemetry results must not be described as causal effects.

## 3. Intervention and comparator

- **Intervention:** exact instructional sequence, AI policy, hints, feedback, prerequisites, spacing, mastery gate, and allowed learner actions
- **Comparator:** current Praxis lesson, a qualified teacher-led resource, standard courseware, or other named baseline
- **Equivalent learning objectives:** list common objective IDs and explain any mismatch
- **Planned exposure:** sessions, minutes, dates, practice opportunities, and revisit schedule
- **Assistance policy:** allowed tutor hints, answer reveal, external tools, accommodations and support
- **Contamination risk:** whether learners or educators can share interventions between groups
- **Randomization unit:** learner / classroom / school / other, with justification
- **Assignment concealment and allocation procedure:**
- **Deviations to record:**

Comparisons must be fair: document differences in instructional time, help available, assessment, content difficulty, teacher support, and device/connectivity conditions.

## 4. Hypotheses and outcomes

For each hypothesis, state direction, outcome, unit, timepoint, estimator, and a practical-importance threshold.

### Primary outcome (choose one primary estimand per confirmatory question)

- **Outcome name and operational definition:**
- **Instrument, items, rubric and content digest:**
- **Unit and scale:**
- **Assessment conditions:** unaided / permitted aids / supervised practical
- **Timepoint:** immediate / approximately 7 days / approximately 28 days / other
- **Target comparison and effect measure:**
- **Minimum practically important difference:**
- **Confidence interval and decision rule:**

### Secondary and diagnostic outcomes

Track distinct metrics rather than combining them into a single score:

- Immediate independent performance.
- Delayed retention, preferably at approximately 7 and 28 days where feasible.
- Transfer to unpractised items, changed examples, or authentic work.
- Error recovery: next-attempt correctness and number of attempts to recover after a mistake.
- Misconception recurrence and answer-key/feedback correctness.
- Time to demonstrated competency and total learner effort.
- Number of questions attempted, skipped, hinted, and retried.
- Lesson completion and attrition.
- Accessibility and performance under low-bandwidth/offline/device constraints.
- Learner-reported confidence calibration, burden and wellbeing.
- Educator/caregiver effort and cost per demonstrated capability.

**Primary outcome must not be replaced after results are seen.** In particular, next-item correctness, lesson completion, time on app, and “mastery streaks” are useful diagnostics but cannot substitute for delayed retention and transfer.

## 5. Sampling and statistical plan

- **Target sample and rationale:**
- **Power / precision calculation:** assumptions, minimum effect, variance, clustering, attrition
- **Eligibility and recruitment:**
- **Randomization and strata:**
- **Primary estimator:** intention-to-treat where random assignment is used; justify alternatives
- **Covariates:** prespecified only
- **Multiplicity strategy** for secondary outcomes:
- **Confidence interval and uncertainty method:**
- **Stopping rule and any interim analyses:**
- **Missing outcomes and attrition plan:** do not silently drop non-completers
- **Protocol violations and per-protocol sensitivity analysis:**
- **Subgroup/equity checks:** prior knowledge, language, disability/access needs, age/grade, device/connectivity and other justified groups
- **Minimum subgroup reporting threshold** to avoid identifying learners:
- **Sensitivity analysis:**

Do not choose sample size or stopping rules in response to interim effect estimates unless a valid sequential design was preregistered.

## 6. Data, consent, safety and governance

- **Data collected and why each field is necessary:**
- **Data minimization and retention/deletion:**
- **Consent/assent and guardian/educator permissions where required:**
- **Private-by-default learner records and publication boundaries:**
- **Security controls and access roles:**
- **Harm/incident reporting and withdrawal process:**
- **Accessibility accommodations:**
- **Reviewer for child safety and age appropriateness:**
- **Research ethics review or determination reference:**

Never place names, student IDs, raw chat transcripts, or row-level education histories in public provenance records. Publish only approved, privacy-reviewed aggregate results; suppress small cells and re-identification risks. Do not train a shared model on study data without a separate, explicit, revocable, governed permission.

## 7. Analysis plan and interpretation rules

- **Primary analysis code/version:**
- **Outcome scoring and exclusions:**
- **Missing data analysis:**
- **Effect size and confidence interval reporting:**
- **Null results and negative outcomes:**
- **Practical-significance rule:**
- **Causal-claim rule and assumptions:**
- **Deviation log location:**
- **Independent reviewer for data and analysis:**

Separate:
1. data observation;
2. computed result;
3. uncertainty;
4. interpretation;
5. causal conclusion, if supported.

These are not equivalent. A cryptographic digest proves which protocol/content bytes were identified; it does not certify the design quality, correctness of the data, or truth of a causal claim.

## 8. Release and replication rule

- **What results are needed to ship this intervention?**
- **What regression blocks release?**
- **Minimum non-inferiority/safety margin:**
- **Replication context(s) required before generalization:**
- **Who can independently reproduce the analysis?**
- **Where will protocol, analysis, limitations and approved aggregate results be published?**

Proposed Praxis default guardrails (review per context before registration):
- Do not regress the primary learning outcome below a justified, prespecified non-inferiority margin.
- Require delayed retention and transfer measures for claims about durable learning.
- Release no core lesson with unresolved critical content, rights, or accessibility blockers.
- Replicate a meaningful effect in at least two materially different contexts before making broad effectiveness claims.
- Report null or negative outcomes, uncertainty, attrition, cost and subgroup limitations.
