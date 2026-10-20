# Praxis-first architecture: boundary with Craft, evidence lifecycle, and benchmark plan

- **Status:** Proposed architecture; no runtime behavior changed by this document
- **Parent design:** [Continuous Learner Record RFC](continuous-learner-record-rfc.md)
- **Scope:** Mycelix Praxis first; Craft is a consuming/producing neighbour, not a second learning authority
- **Rule:** Do not implement a new cross-project contract until the relevant existing proposal, migration authority, and Holochain/DNA compatibility have been checked.

## 1. Executive decision

**Praxis owns the learning lifecycle and the semantics of learning evidence. Craft owns the work/opportunity lifecycle and the context in which capabilities are applied.** Neither app is the global source of truth for a person's worth or all human capability.

Praxis answers:
- What did the learner choose to learn?
- What learning or practice was offered?
- What was actually observed or submitted?
- Under which criteria and version was evidence admitted?
- What do those observations support—and not support—about current capability?
- Is an issuer entitled to issue this particular credential, and what exactly does it assert?
- What does the learner choose to keep, export, or disclose?

Craft answers:
- What work, apprenticeship, project, contract, or opportunity is available?
- What evidence and qualifications does the opportunity require?
- What has the applicant chosen to share for this purpose?
- What work was completed, by whom, in which context, and who can attest to it?
- What professional/guild endorsement or opportunity outcome is valid under Craft policy?

Craft must not silently turn a profile field or stale imported score into a credential. Praxis must not become a job marketplace, rank people for employers, or dictate opportunity acceptance.

## 2. Current source audit: integration seam to fix

The current public source contains a strong starting separation: Craft's README says it stores opt-in professional profile metadata, credential pointers and peer attestations, while credentials remain in Identity/Praxis and are referenced by hashes. It also lists jobs, apprenticeship pathways, work history and application lifecycle as Craft scope.

There are two important correctness seams:
1. Craft's credential UI describes credentials themselves as decaying with the Ebbinghaus forgetting curve and invites learners to take retention checks to refresh credential vitality.
2. Craft's credential-publication coordinator verifies a Praxis issuer signature over a defined payload and binds presentation to the subject, but its own code comment explicitly says the caller-supplied `mastery_permille` is not included in what is cryptographically verified and is not yet checked against Praxis's record. The Praxis learning contract separately states that its browser-facing `verify_credential` path fails closed while full canonical-payload, issuer-signature, expiry and status checks remain incomplete.

These are not a reason to abandon the current work. They are a reason to freeze a precise contract before growing it. A proof over fields A–E cannot prove a separate field F, and a public verification badge must not imply checks that did not run.

Source references:
- [Craft README](../mycelix-craft/README.md)
- [Craft credential coordinator](../mycelix-craft/zomes/craft-graph/coordinator/src/lib.rs)
- [Craft credential page](../mycelix-craft/apps/leptos/src/pages/credentials.rs)
- [Praxis learning client contract](learning-contract.md)
- [PRAX-EVIDENCE-001 proposal #2467](https://github.com/Luminous-Dynamics/mycelix/pull/2467) and [PRAX-EVIDENCE-002A proposal #2508](https://github.com/Luminous-Dynamics/mycelix/pull/2508) are related in-flight work. Reconcile with their actual head commits and merge state before implementation; do not assume an open PR is part of main.

## 3. Paradigm shifts

### 3.1 From "credential vitality" to three distinct timelines

Never apply one time-decay function to the whole credential.

| Object | Expected temporal semantics | Owner / authority |
| --- | --- | --- |
| Historical achievement / completed assessment | Immutable history; corrections append a correction or superseding assertion | Praxis or the qualified original issuer |
| Credential authenticity, expiry, suspension or revocation | Evaluated using the credential's proof, issuer policy and applicable status mechanism; not an Ebbinghaus curve | Issuer/credential profile; verifier evaluates it |
| Current capability estimate / skill freshness | A revisable, evidence-bound, task-specific estimate; new practice may strengthen, weaken, or leave it unresolved | Praxis computes an advisory projection; never rewrites the historical achievement |
| Craft reputation / endorsement | Contextual claim made by named peers or a guild under a stated policy; may be disputed or withdrawn according to that policy | Craft / named endorser |
| Work eligibility / job decision | Decision for a specific role and context, based on declared requirements and applicable law/policy | Employer / authorized opportunity owner, not Praxis's learner model |

A credential can remain historically authentic while no longer satisfying a role's current-skill requirement. Conversely, a learner may still be capable after a badge expires. An expiry may be appropriate for a safety licence or a credential issuer's explicit policy; an Ebbinghaus estimate alone is not grounds to rewrite or invalidate a historical award.

**Recommended product language:** replace “credential vitality” with separate status indicators such as “credential verification,” “issuer-defined expiry/status,” and “skill evidence freshness.” A low-confidence or old capability estimate should prompt reassessment, not silently mutate the old credential.

### 3.2 From score-first to claim-and-evidence-first

Do not make a single `mastery`, `employability`, `trust`, or `readiness` scalar the shared cross-project API.

Use the evidence sequence:

```text
observation
  != admitted evidence
  != analysis
  != advisory capability estimate
  != issued credential
  != employer decision / authorization
```

Every transition must identify the actor, input references, named/versioned rule, output, timestamp and limitations. A downstream projection cannot claim stronger scope, confidence, currentness or authority than its exact inputs and policy permit. This builds on the semantics already proposed in PRAX-EVIDENCE-001 instead of adding a competing evidence engine.

### 3.3 From platform lock-in to portable, purpose-limited views

The canonical learner record is not a public profile. Praxis should keep a learner-controlled evidence graph and build specific presentations for specific purposes. A job application, learning plan and public portfolio should disclose different slices. Craft receives the slice a learner chooses to share, not a default feed of all learning history.

### 3.4 From course completion to demonstrated transfer

Completing a course is evidence of completing that course. It is not automatically evidence of competence across every adjacent task. Praxis should progressively add assessment types and measure retention and transfer outside the original item/task context. Recommendations are useful only if they improve outcomes against credible baselines.

## 4. Responsibility matrix

| Capability | Praxis | Craft | Shared dependency |
| --- | --- | --- | --- |
| Curriculum, lessons, retrieval practice, learner-chosen goals | **Owns** | No | Content/knowledge references if needed |
| Source attempt observations and learning-evidence admission | **Owns** | May submit a learner-consented work-evidence candidate | Existing provenance substrate; typed contracts |
| Capability estimates and pedagogical recommendations | **Owns advisory projections** | May consume disclosed bounded claims | Symthaea advice contract; model/rule provenance |
| Assessment rubric and assessment receipt for learning objectives | **Owns contract and record** | Can define work acceptance criteria for a project; sends receipt as an external claim | Versioned rubric/task reference |
| Credential issuance from an authorized issuer | **Owns the learning-side flow when issuer policy permits** | Must not re-issue, rewrite, or increase assurance of the Praxis credential | Identity/issuer trust, cryptographic proof/status |
| Credential presentation / public portfolio | Previews and controls learner-selected learning records | Owns opt-in professional profile and application-specific view | Holder-controlled selective disclosure |
| Work artifacts, task history, peer contribution attribution | Can reference them as learning evidence with consent | **Owns work/opportunity record** | Shared stable artifact references and provenance |
| Guild endorsements / professional reputation | Does not infer them from learning completion | **Owns context and policy** | Named issuer/endorser, evidence, challenge path |
| Job posting, matching, apprenticeship, application/contract lifecycle | No | **Owns** | Capability requirements expressed as explicit, versioned criteria |
| Current skill freshness estimate | Can maintain learning-oriented advisory estimate | Can display a role-specific re-assessment requirement, without changing the credential | Must show source, date, criterion and uncertainty |
| Human worth, global ranking, universal employability score | **Must not own** | **Must not own** | Explicitly out of scope |

## 5. The single integration contract

Introduce no new general-purpose evidence/provenance subsystem. Reuse the current Praxis and shared Mycelix work where review and migration authority permit. At the cross-app boundary, define one versioned concept: an **External Evidence Receipt**. It is a proposal for an adapter DTO, not a claim that this Rust type already exists.

Minimum semantic fields:
- `contract_version` and `receipt_id` (stable/idempotent);
- `producer` and producer version/build;
- subject binding appropriate to this presentation and consent scope;
- `claim_id`, competency URI, task/activity ID, task version and criterion/rubric ID + version;
- evidence references and content digests where appropriate (a digest is not proof of authenticity or accessibility);
- observation/outcome metadata, support/assistance used and assessment method;
- observed time and receipt time, each separately defined;
- derivation lineage / prior receipt IDs, if derived;
- visibility/disclosure class and learner consent reference;
- evaluator/issuer/endorser identity and proof/profile reference, when applicable;
- explicit limits, missing fields, contradictory evidence and verification outcomes.

Use a tagged result state such as `Observed`, `AdmittedUnderPolicy`, `Insufficient`, `Disputed`, or `Rejected`. Do not overload one boolean `verified` to mean authenticity, issuer trust, evidence sufficiency, currentness, and professional acceptance.

An adapter may submit a receipt; it cannot directly set learner mastery, silently issue a credential, or change the user's private record. Praxis validates the receipt under a named/versioned admission profile and asks the learner before associating externally-produced evidence with their record. The receipt is immutable in meaning; corrections are new linked receipts or dispositions.

### 5.1 Credential presentation from Praxis to Craft

Craft must independently validate the presentation it relies on:
1. Validate the credential's declared format/profile and canonical signed payload, including every claim it consumes.
2. Verify the cryptographic proof against an acceptable issuer key and supported proof suite.
3. Bind the credential subject to the authenticated presenting holder, without assuming issuer identity proves holder identity.
4. Apply issuer trust policy, validity/expiry and any required suspension/revocation/status checks.
5. Validate claim-specific requirements for the current opportunity; authenticity alone does not make a credential suitable for every job.
6. If a field such as a score or mastery level is used, prove it is inside the signed/committed payload and conforms to a versioned semantic policy; otherwise ignore/reject it rather than displaying it as verified.
7. Preserve the original signed object or a faithful reference. Never translate a partially verified credential into a stronger local badge.

If a required status service is unavailable, the UI must report “could not establish current status,” not “valid.” Distinguish a known invalid credential from one that could not be checked.

### 5.2 Work evidence from Craft to Praxis

Craft can issue a work-context receipt for an artifact, project outcome, employer assessment or peer attestation. It is a *candidate input* to Praxis, not a learning credential. Praxis should:
- show the learner exactly which artifact, collaborators, employer, location and metadata will be imported;
- collect explicit consent and support private/nonportable evidence;
- keep attribution/dispute states, group contribution boundaries and conflicts of interest;
- map the receipt to a stated competency only under a named rubric/policy;
- avoid treating a job offer, wage, popularity, guild status, or number of endorsements as evidence of skill by itself.

## 6. Standards: use existing exchange formats, keep internal semantics stricter

- **W3C Verifiable Credentials Data Model 2.0** for credential/presentation data model and the issuer-holder-verifier roles: https://www.w3.org/TR/vc-data-model-2.0/
- **1EdTech Open Badges 3.0** for achievement credentials, criteria, evidence, alignment and endorsements: https://www.1edtech.org/standards/open-badges
- **1EdTech Comprehensive Learner Record 2.0** to package related achievements and associations as a learner-controlled record: https://standards.1edtech.org/clr/
- **xAPI** may be useful as an interoperability adapter for learning-experience statements, not as a credential or truth system: https://github.com/adlnet/xAPI-Spec

The W3C model deliberately separates cryptographic verification from the verifier's later validation/business rules; implement this distinction in the UI and API. Standards compatibility is not automatic authority or accreditation. Claim conformance only for an explicit, tested profile and declare unsupported cryptosuites, fields and conversions.

## 7. Industry/SOTA positioning

We should benchmark against **categories and reproducible baselines**, not write that Praxis is “better than the industry” without direct comparative evidence.

| Ecosystem / baseline | What it is already good at | Praxis differentiation to prove |
| --- | --- | --- |
| LMS / course platforms | Course delivery, enrollment, assessment workflows, teacher/learner administration | Learner-owned continuity across courses, work, simulation and informal learning |
| Credential networks (for example, Credly) | Issuer distribution, badge display, credential sharing and workforce use cases | Evidence graph with explicit claim limits, local-first control, cross-context demonstrations and loss-aware portability |
| Open Badges / CLR ecosystem | Portable achievement representation, evidence metadata, endorsements and standardized exchange | Better learning-to-evidence production and evaluation, while remaining compatible rather than competing with their formats |
| Learning Record Stores / xAPI | Capture and exchange of learning-experience statements across tools | Turn selected, consented observations into auditable evidence and justified learning recommendations without treating event logs as skill truth |
| Adaptive-learning / knowledge-tracing systems | Predict future item responses and adapt practice schedules | Calibrated estimates plus real delayed retention and transfer outcomes, with data leakage controls and learner-visible reasons |
| Portfolios / project marketplaces | Real work artifacts, professional identity, peer/community signals, work and opportunity workflows | Reconnect verified work-context evidence to optional lifelong learning without letting marketplaces define the learner's whole identity |

Public vendor claims (including adoption counts) are useful evidence of market distribution, not independent evidence of learning efficacy, credential correctness or privacy. Direct vendor comparisons need an explicit dated test plan, fair access to the same tasks, and separate product claims from measured outcomes.

## 8. Benchmark programme: baseline first, then prove improvement

### 8.1 Learning efficacy

Compare at least:
- **B0:** fixed curriculum / current non-adaptive learning path;
- **B1:** existing Praxis spaced-repetition and Bayesian Knowledge Tracing path;
- **B2:** Praxis plus a bounded Symthaea advisory plan;
- **B3:** an appropriate reproducible external baseline from the knowledge-tracing literature, using the same train/test split and leakage-safe preprocessing.

Measures:
- delayed retention at 30 and 90 days;
- performance on unseen transfer tasks and practical demonstrations;
- time/attempts to pass a pre-registered independent competence criterion;
- mastery-probability calibration (Brier score/log loss and calibration curves);
- false-mastery rate: claims judged competent by the predictor but failed by the independent assessment;
- subgroup and modality/accessibility breakdowns with uncertainty intervals.

Avoid optimizing AUC alone. Knowledge-tracing research documents evaluation/preprocessing inconsistencies and data-leakage risks; use versioned splits, a frozen data manifest and no target leakage. A recent benchmark study provides a relevant reproducibility reference: https://arxiv.org/abs/2403.15304. A 2025 empirical framework likewise highlights inconsistent comparisons and standardizes 21 models across 9 datasets: https://doi.org/10.1109/TKDE.2025.3552759.

**Stretch hypotheses, not claims or universal SOTA thresholds:**
- reduce median time to independently demonstrated competence by at least 10% against B0/B1;
- improve 30/90-day transfer without a meaningful reduction in delayed retention;
- improve calibration and false-mastery rate against the strongest reproducible baseline;
- show positive benefit in more than one domain before generalizing.
Pre-register outcomes, analysis, exclusions, sample-size/power calculation and stopping rules; report null or negative results. The exact minimum effect should be chosen after measuring variance and costs, not retrofitted after observing the result.

### 8.2 Credential and interoperability correctness

- 100% pass rate on the declared official conformance fixtures for each format/profile claimed as supported.
- Every deliberately mutated negative control in the test suite is rejected; report the test corpus size and mutation classes, and do not equate a finite test suite to proof of security.
- 100% preservation of required fields through supported export/import round trips; every non-lossless conversion yields a machine-readable and human-readable loss report.
- Test with at least two independent verifier implementations and real issuer key rotation/status fixtures.
- Explicitly test wrong subject, untrusted issuer, changed signed score, signature/serialization mismatch, unsupported proof suite, expired credential, revoked/suspended status, missing status, duplicate IDs and stale cache.
- Measure p50/p95 verification latency and offline behavior on named reference hardware; choose release budgets from measured baseline and threat model.

### 8.3 Evidence integrity across project boundaries

- 100% of accepted assessment receipts bind all required producer, task, rubric, evidence and consent provenance for their declared receipt profile.
- 100% of malformed, tampered, duplicated, cross-subject and unsupported receipts in the negative-control corpus fail closed.
- Deterministic simulation receipts reproduce the same output digest under the declared reproducibility envelope; nondeterministic workloads must declare tolerances and seeds/inputs honestly.
- Zero silent authority promotion in contract tests: observation → evidence → capability estimate → credential → work decision each require their own rule/authorized actor.
- Zero raw private portfolio evidence published publicly by default; all external disclosure paths have explicit consent tests.
- Idempotent import: replaying one valid receipt does not double-count achievements or alter an earlier result.

### 8.4 User agency, usefulness, and access

Measure completion and error rates for: reviewing why a claim was accepted/rejected, correcting a claim, exporting a readable record, selectively disclosing one claim, and revoking/withdrawing a shared presentation where supported. Test keyboard/screen-reader use, low-bandwidth and offline operation, multilingual criteria, assistive-technology paths and a broad range of device capability.

Track learner trust and understanding: after seeing “verified,” can users correctly explain what was checked, by whom and with what limitations? If not, the status model/UI is not good enough.

## 9. Proposed first vertical slice, focused on Praxis

1. Freeze the semantic contract and add fixtures; no DNA schema change in the first slice.
2. Use an already existing Praxis learning task/attempt as the input, with explicit rubric version, provenance and assistance information.
3. Produce an immutable assessment/evidence receipt and a separate recomputable advisory capability projection.
4. Render the learner-facing difference between “observed,” “admitted under policy,” “estimate,” and “credential issued.”
5. Export one purpose-specific claim in a standards-backed format only if the implemented proof profile is supported and actually verified; otherwise export an honest unverified portfolio item.
6. Test all negative controls locally, including altered score, missing evidence provenance, and wrong subject.
7. Later, consume the result from Craft through the adapter; require explicit learner consent for any work evidence imported back into Praxis.

The first slice is intentionally not a general talent ranking, new credential marketplace, or whole-DNA migration. Confirm the active Praxis Holochain migration/qualification authority and reconcile the open PRAX-EVIDENCE stack before materializing new integrity entries or changing public visibility.

## 10. Definition of done

A learner can complete one bounded learning task, see the rubric and exact evidence behind its result, inspect an advisory estimate without it becoming authoritative, keep the raw evidence private, and export a truthful purpose-specific representation. An independent verifier rejects tampering, unsupported proof methods and wrong-subject claims. If that learner later applies through Craft, Craft verifies the fields it relies on and does not treat an unsigned mastery number, expiring cache, guild popularity, or simulated result as universal competence.

**Success is not the number of credentials Praxis issues. It is the proportion of capability claims that are understandable, appropriately scoped, independently checkable where promised, useful to the learner, privacy-preserving, and predictive of successful performance in the context where they are used.**
