# Craft Work Evidence V1 — qualification and comparative benchmark plan

Status: proposed evaluation protocol and targets, not measured results.

## 1. Correctness and security gates

The following are non-compensatory:
- tampered claim, authority, evidence digest or signature acceptance: **0**;
- unauthorized, revoked, out-of-scope or expired assessor acceptance: **0**;
- self-attestation acceptance where independence is required: **0**;
- replay acceptance after host reservation: **0**;
- future-dated claim or artifact acceptance, or an attestation predating claim creation: **0**;
- duplicate identity counted toward assessor quorum: **0**;
- disputed or unable-to-corroborate result promoted to corroborated: **0**;
- valid frozen fixture rejected: **0**;
- historical claim revision silently overwritten: **0**.

Require 100% execution of the declared negative-control corpus for every release candidate. Expand with property-based generation, mutation testing, malformed serialization, key rotation, issuer/assessor collusion, common-employer dependence, time skew, partial network availability, concurrent replay, and stale/revoked trust snapshots.

## 2. Determinism and interoperability

- Same claim and same schema version produce the same digest across repeated runs.
- Non-semantic ordering of evidence references, scopes, conflicts and limitations does not change signed payloads.
- Any semantic mutation changes the digest or invalidates the signature.
- Publish canonical byte test vectors before adding a non-Rust implementation.
- Round-trip only a declared profile of W3C Verifiable Credentials Data Model 2.0, 1EdTech Open Badges 3.0 and Comprehensive Learner Record 2.0. Report unsupported fields and lossy mappings explicitly; do not imply certification by mapping.
- Run the official standard-specific conformance suites for the chosen profile and independent verifier implementations before claiming interoperability or certification.

## 3. Performance reporting

Do not set an arbitrary SOTA throughput claim before measuring. Freeze:
- CPU, OS, Rust toolchain, dependency lock and build profile;
- fixture digest, artifact count and signature/key sizes;
- trust-store and revocation snapshot size;
- attestation quorum (1, 2, 3, 5, 10);
- verification-only versus artifact-byte hashing;
- warm/cold process conditions and concurrency.

Report P50/P95/P99 latency, throughput, peak memory and size growth at 1, 10, 100 and 256 evidence references, plus 1/2/3/5/10 assessors. Compare the reference verifier against a naïve signature-only verifier and a coordinator implementation that merely counts verification links. The latter are deliberately weak baselines, not acceptable alternatives. No performance superiority is claimed until results are measured on the same frozen workload.

## 3.1 Published reference points and limits

A recent Annual Review table reproducing Sackett et al. (2022) revised selection-validity estimates reports structured interviews at `ρ = 0.42`, job-knowledge tests at `ρ = 0.40`, empirically keyed biodata at `ρ = 0.38`, and work-sample tests at `ρ = 0.33` ([Annual Review table](https://www.annualreviews.org/content/table/10.1146/annurev-orgpsych-020924-072127.t1)). These are reference estimates from a heterogeneous research literature, not universal constants, not direct Craft results, and not an acceptable substitute for a matched evaluation. The table notes that the job-knowledge estimate is from concurrent samples because no applicant-sample meta-analysis was available.

A review first published on 19 October 2026 argues that much criterion-validity evidence still relies on concurrent incumbent samples and that better primary studies of operational applicant settings are needed ([Niessen et al., 2026](https://onlinelibrary.wiley.com/doi/full/10.1111/ijsa.70086)). Craft's evaluation should therefore be prospective, use the applicant population where decisions are actually made, and measure outcomes after hiring.

For credential interoperability, Craft should complement rather than compete with existing specifications:
- [W3C Verifiable Credentials Data Model 2.0](https://www.w3.org/TR/vc-data-model-2.0/) for issuer/subject claims and verifiable presentations;
- [1EdTech Open Badges 3.0](https://www.1edtech.org/standards/open-badges) for individual achievement assertions with criteria and supporting evidence;
- [1EdTech Comprehensive Learner Record 2.0](https://www.1edtech.org/standards/clr) for learner-curated learning and employment records.

A crucial standards distinction: Open Badges verification establishes that a credential is an authentic and timely statement under its proof/status rules; it does not itself establish that the claims are true. Craft must add scoped evidence validation and independent real-world assessment. Do not claim 1EdTech certification until the applicable test suite and staff review process are complete ([Open Badges certification process](https://www.1edtech.org/certification/open-badges)).

For apprenticeship quality, align the operational model to the ILO Quality Apprenticeships Recommendation No. 208: explicit trainer/instructor competence, a suitable balance of off-the-job and on-the-job learning, competency assessment/certification, and regular monitoring of retention, non-completion and success rates ([ILO Recommendation 208](https://www.ilo.org/resource/other/r208-quality-apprenticeships-recommendation-2023)). These should become measurable agreement and programme fields, not an opaque apprenticeship score.

## 4. Professional selection and capability evaluation

The relevant comparison is not only credential formats. Compare the opportunity pipeline against matched baselines:
1. CV / résumé and ordinary reference screening;
2. structured interview;
3. job-relevant work-sample assessment;
4. conventional job-board or professional-network shortlist;
5. evidence-first Craft shortlist with explicit abstention and a human decision-maker.

Use prospective, operational cohorts for real hiring outcomes rather than only retrospective incumbent samples. Freeze job family, minimum requirements, assessment rubric and evaluation horizon before recruitment. Keep the hiring decision with the employer and candidate consent explicit.

Primary outcomes:
- criterion validity against blinded, job-relevant performance at 3/6/12 months;
- calibration and uncertainty (Brier score, reliability curves, selective-risk/coverage);
- precision/recall for each required capability, with false-accept and false-reject rates;
- inter-rater agreement and assessor conflict prevalence;
- time-to-first-opportunity, conversion, offer acceptance and repeat engagement;
- apprenticeship completion, delayed retention and unseen-task transfer;
- pay, mentor compensation, worker control of evidence and appeal outcomes;
- rework, recurrence and safety outcomes for physical repair/productive work;
- subgroup error and opportunity disparities, with lawful, consented, privacy-protective evaluation.

Report confidence intervals, missingness, attrition and subgroup sample sizes. Do not collapse the dimensions into a universal person score. A matching system must expose missing evidence and abstain; it must not infer worth or deny access automatically.

## 5. First end-to-end benchmark candidate

Use a bounded water-pump repair case composed with Manufacturing's existing work-order ownership:
- failure observation and its source;
- diagnosis (kept distinct from the original observation);
- work authorization and agreement;
- technician and mentor roles;
- actual parts/material consumption;
- before/after flow and containment measurements;
- independent in-scope inspection;
- recurrence observation window.

The benchmark must reject “work order completed” as equivalent to “capability restored,” and reject an immediate functional test as proof of recurrence-free operation. No attestation creates compensation; Finance separately decides settlement under its own authorization and journal contracts.
