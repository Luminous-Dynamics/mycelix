# Naming and paradigm agenda for Mycelix Praxis

- **Status:** Proposed product language and research agenda; not a naming/legal clearance
- **Product focus:** Mycelix Praxis
- **Related documents:** [Continuous Learner Record RFC](continuous-learner-record-rfc.md), [Praxis/Craft boundary and benchmark plan](praxis-craft-boundary-and-benchmarks.md)

## 1. Recommendation: stop calling the whole thing a diploma

A **diploma is one kind of formal qualification**, awarded by an authorized institution or issuer under defined requirements. It is not an accurate name for a private, lifelong record that accumulates many kinds of learning, work artifacts, practice, feedback, research, and demonstrated capabilities.

Use the phrase “continuous diploma” only as an informal description while discussing the idea. Do not make it the product's canonical type or imply that Praxis itself confers a formal degree, regulated qualification, institutional accreditation, or professional licence.

### Proposed name

**Mycelix Praxis Living Record**

One-line description:

> A learner-controlled, lifelong record of learning, evidence, and demonstrated capability.

Short UI label: **My Record** or **Living Record**.

Technical term for the internal domain model: **Learner Record**. Use standard names (e.g. Comprehensive Learner Record, Open Badge, Verifiable Credential) only for the corresponding interoperable object/profile.

This structure separates product identity from exchange formats:
- **Praxis Living Record** is the product experience and learner-controlled collection.
- A **learning observation** is something recorded as having happened.
- **Admitted learning evidence** is an observation accepted under a named, versioned policy.
- An **advisory capability estimate** is a limited, revisable interpretation of specified evidence.
- An **achievement/credential** is a specific assertion made by an authorized issuer.
- A **shareable presentation** is the subset the learner selects for a purpose.
- A **diploma, degree, regulated licence, or formally accredited certificate** remains the exact recognized qualification issued by the competent authority.

The record can include formal qualifications; it must never pretend that the whole record is itself an accredited diploma.

## 2. Why not call it “Human Capability Record” or simply “CLR”?

Terminology is getting crowded. 1EdTech already publishes the **Comprehensive Learner Record (CLR Standard 2.0)** and Open Badges 3.0, which support verifiable achievement claims and learner-curated records. Their official documentation specifically describes learning and employment achievements across education and work, including courses, skills and employer-based milestones. The term **Learning and Employment Record (LER)** is also used as a broader concept for learning/work records; CLR is one implementable standard in that space.

A 2026 sector initiative is using **Human Capability Record (HCR)** for a consent-driven, portable capability-record concept. Praxis should not adopt the same phrase without a deliberate differentiation/brand review.

Sources:
- 1EdTech CLR: https://www.1edtech.org/standards/clr
- 1EdTech Open Badges: https://www.1edtech.org/standards/open-badges
- 1EdTech CLR/LER terminology FAQ: https://www.1edtech.org/clr/faq
- Human Capability Record initiative: https://www.mortarcaps.org/the-future/

**Do not invent a new standards acronym or compete with CLR/LER/VC vocabulary.** Make the product name distinctive and keep the technical interchange standards-native. “Praxis Living Record” is a working recommendation, not a trademark, domain, or naming-availability clearance; complete language, trademark, cultural/translation, and domain checks before formal rebranding.

## 3. Naming alternatives considered

| Name | Strength | Concern | Decision |
| --- | --- | --- | --- |
| Continuous Diploma | Easy metaphor for lifelong learning | Implies a diploma/credential, potentially misleading about authority and accreditation | Do not use as canonical name |
| Human Capability Record | Descriptive, but a 2026 sector initiative uses HCR for a related concept | Confusing overlap; avoid copying established sector naming | Do not use without a deliberate differentiation decision |
| Comprehensive Learner Record | Established technical term | Already names the 1EdTech CLR standard; would imply standards conformance | Use only for the standard/profile |
| Lifelong Learning Record | Clear and descriptive | Broad, generic, and close to established record terminology | Useful explanatory phrase |
| Capability Passport | Easy to understand in employment contexts | Passport metaphor suggests a gatekeeping/permission token and is already common in skills products | Not the preferred identity |
| Capability Portfolio | Strong for curated artifacts | Sounds like a subset assembled by hand; does not clearly convey history, provenance, assessment, corrections, or privacy boundaries | Use as one optional view |
| Praxis Living Record | Clear product-family association, suggests continuity and a learner-owned record without implying a qualification | Requires normal name/trademark/translation checks | **Preferred working name** |
| Praxis Lifegraph | Evokes connected learning and evidence | Sounds technical, possibly surveillance-like, and may overemphasize graph implementation | Could name an internal visualization, not the product |

## 4. Paradigm shift: build a capability compiler, not just a content platform

The most ambitious useful direction is to make Praxis translate a learner-chosen real-world goal into a **transparent learning and evidence plan**, then check the plan against observed results and transfer. Internally, this can be explored as a *capability compiler*:

```text
learner-chosen goal + context + constraints
                  |
                  v
        capability/competency graph
                  |
                  v
   proposed learning and practice route
                  |
                  v
     task + rubric + evidence contract
                  |
                  v
       observations and artifacts
                  |
                  v
   admission + limited capability estimate
                  |
                  v
       independent transfer challenge
                  |
                  v
       learner review and next choice
```

It is not an AI that decides what a person should be. It proposes a plan the learner can inspect, change, reject, or take offline.

### Shift A — from course-first to goal-and-capability-first

A course is one possible means, not the unit of success. Begin with an outcome such as “diagnose a network outage safely,” “calculate and test a small solar installation,” “design an accessible public-space route,” or “play a musical piece to a stated standard.” Map backward to prerequisites and forward to tasks that can demonstrate the intended outcome.

### Shift B — from scalar mastery to a typed evidence graph

A single score obscures the task, context, assistance, uncertainty, recency and provenance. Record evidence per criterion and context; derive any summaries from exact inputs and named rules. Preserve conflicting or incomplete evidence rather than averaging it away. No overall human-worth, trustworthiness or employability score.

### Shift C — from one-time certification to two timelines

Preserve the historical achievement as a dated record. Separately estimate current capability from new evidence and apply issuer-defined expiry/revocation/status rules to credentials. Skill freshness can trigger a new assessment; it does not rewrite history.

### Shift D — from app-contained exams to real-world transfer

Connect learning to practical tasks and unfamiliar situations. A simulation can provide useful evidence about a bounded scenario, not automatic proof of safe real-world competence. Increase assurance only when a stronger assessment actually ran.

### Shift E — from recommendation engines to controlled experiments

Symthaea can propose learning routes and task variants. Praxis must measure whether they improve retention and transfer against the fixed-path and existing Praxis baselines, including negative and subgroup results. Model confidence or plausible explanations are not efficacy evidence.

### Shift F — from a data silo to learner-directed views

Keep a learner-controlled source record; produce narrowly scoped views for a mentor, issuer, employer, community, or personal reflection. A public portfolio is an explicit publication choice, not the default location for raw attempts, sensitive feedback, or full learning history.

### Shift G — from institutional monopoly to plural recognition

Make formal education, informal learning, apprenticeship, open-source contributions, care work, community practice, research, and creative achievement representable, without pretending each kind of evidence has identical assurance or qualifies someone for regulated work. Fairness means accepting more than one learning route while keeping each claim's provenance and limits honest.

## 5. Proposed product shape

The first interface should be organized around the learner's intent, not internal subsystem names:

1. **My goals** — learner-authored outcomes and constraints.
2. **My learning map** — prerequisite and competency graph, with alternate routes and explanations.
3. **Practice and challenges** — lessons, retrieval practice, applied tasks and assessment rubrics.
4. **My evidence** — submitted artifacts, observations, feedback, assistance/provenance, disputes and admission decisions.
5. **My capability estimates** — dimensions and contexts; last supporting evidence; uncertainty; what's missing; what would change the estimate.
6. **My achievements** — formal qualifications and credentials, with independent verification and issuer-defined status.
7. **My record and sharing** — private-by-default record, readable export, selected purpose-specific presentation and clear sharing expiry/revocation limits.
8. **Review and next step** — learner-selected next actions, overdue practice and assessment opportunities; no punitive streak requirement.

A learner should be able to use Praxis meaningfully without publishing a public identity, joining Craft, accepting AI recommendations, or obtaining an institution's credential.

The newest broad policy synthesis is the OECD's [Digital Education Outlook 2026](https://www.oecd.org/en/publications/oecd-digital-education-outlook-2026_062a7394-en.html), published January 19, 2026. Its key warning is directly relevant to Praxis: generative AI can improve immediate task output without producing durable learning, and some studies report weaker performance when assistance is removed. Praxis should therefore distinguish *assisted task success* from *independent delayed transfer*, record assistance explicitly, and include both AI-assisted practice and no-assistance assessment conditions in evaluation. That evidence argues for pedagogically bounded Symthaea advice rather than letting the model complete tasks on the learner's behalf.

## 6. Evaluation programme: compete on results, not adjectives

Do not declare “better than SOTA” in advance. Set a reproducible baseline and publish whether each hypothesis passes.

### Learning and capability

Compare:
- **B0:** fixed curriculum / non-adaptive path;
- **B1:** existing Praxis spaced repetition + Bayesian Knowledge Tracing;
- **B2:** Praxis + bounded Symthaea advice;
- **B3:** an external published knowledge-tracing or adaptive-learning baseline, reproduced with matched datasets, frozen splits and leakage-safe preprocessing.

Track:
- 30-day and 90-day retention;
- transfer success on unseen tasks, independently scored against preregistered rubrics;
- time/attempts to reach a predefined competence criterion;
- Brier score/log loss and calibration curves for probabilistic predictions;
- false-mastery rate and failure to transfer;
- subgroup and accessibility breakdowns with uncertainty intervals;
- learner ability to understand and correct the system's interpretations.

Initial *stretch hypothesis*, not a current result: at least 10% lower median time-to-demonstrated-competence versus B0/B1, with no meaningful loss in delayed retention and improvement in transfer, replicated across more than one domain. Determine sample size and minimum meaningful effect from baseline variance and user cost before running the study.

### Integrity and portability

- All declared official standards/profile conformance fixtures pass before claiming conformance.
- Zero false acceptance in the defined adversarial test suite (tampered payload and score, wrong subject, untrusted key, unsupported method, expired and required revoked/suspended status).
- Two independent verifier implementations agree on the declared test corpus.
- Supported round trips preserve all required fields; unsupported conversions produce a loss report.
- Replaying an evidence receipt does not double-count an achievement.
- All accepted receipt profiles require complete provenance for the fields they consume.
- Public DHT defaults expose no raw private learning evidence, and all sharing requires explicit consent.

### Human value and accessibility

Measure task completion, time-to-understanding, correction/export success, learner comprehension of verification status, offline behavior, keyboard/screen-reader paths, language support and fairness of assessment outcomes. Do not optimize number of badges issued, daily engagement, time-on-platform or click-through as substitutes for learning.

### Comparative research notes

Official standards are a floor for interoperability, not a benchmark for educational efficacy:
- 1EdTech's description of [CLR 2.0](https://www.1edtech.org/standards/clr) explicitly includes course, skill and workplace achievements and a learner-curated record.
- Its [Open Badges 3.0](https://www.1edtech.org/standards/open-badges) specification supports digital achievement assertions with issuer, criteria and potentially evidence.
- The [CLR 2.0 Errata release](https://www.imsglobal.org/spec/clr/v2p0/errata) was issued June 12, 2026. Pin version/context fixtures during implementation; do not assume that a standard profile is immutable.

Praxis differentiation should be measured in the connections between learning, assessment, evidence and transfer, while staying compatible with these existing formats.

## 7. Execution sequence

### Slice 1 — now, documentation and contract fixtures

- Adopt “Praxis Living Record” as working language only in this design; do not trigger a broad product rebrand.
- Keep standards names exact.
- Reconcile the existing PRAX-EVIDENCE proposal stack and migration state before adding duplicate runtime types.
- Define one end-to-end synthetic learner journey and versioned test fixtures: task, rubric, observations, admitted evidence, advisory estimate, credential status, selected export.
- Add negative controls that attempt to promote unsigned/altered/unproven observations into stronger claims.

### Slice 2 — local-first end-to-end proof

- Implement a bounded journey inside Praxis using the existing learning/task surfaces.
- Keep raw evidence local/private by default.
- Produce provenance-complete evidence and a deterministic advisory projection, separate from credential issuance.
- Add a human-readable export and machine-readable versioned representation.
- Run Rust tests and adversarial fixtures locally; CI can confirm independently later.

### Slice 3 — learning experiments

- Compare B0/B1/B2/B3 under a preregistered protocol.
- Measure delayed retention and transfer, not only immediate quiz scores.
- Publish dataset/split hashes, tool/model versions, sample sizes, confidence intervals and negative results.

### Slice 4 — interoperability, then Craft

- Verify the declared VC/Open Badges/CLR profiles against official fixtures and independent verifiers.
- Only then connect Craft as a consumer of learner-selected claims and as a producer of explicitly consented work evidence.
- Keep opportunity decisions and professional endorsements under Craft's authority, outside Praxis's capability-estimation layer.

## 8. Naming and claims policy

Until naming checks are complete, refer to the feature as **“Praxis Living Record (working name)”**. Do not claim that it replaces diplomas, accreditation, professional licences, institutional transcripts, or the 1EdTech CLR standard.

The promise is more useful and more defensible:

**One learner-controlled place to preserve the story of learning and its evidence—across a lifetime—while keeping every credential, estimate, and endorsement honest about who asserted it and what it proves.**
