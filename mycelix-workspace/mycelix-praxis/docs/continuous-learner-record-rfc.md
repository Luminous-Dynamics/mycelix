# RFC: Mycelix Praxis Continuous Learner Record

- **Status:** Proposed; architecture and pilot targets only
- **Owner:** Mycelix Praxis
- **Scope:** Cross-project learning evidence and portable credentials
- **Working product language:** “continuous diploma” / “living capability record”
- **Normative boundary:** This RFC does not assert that production-grade credential verification, privacy-preserving synchronization, institutional accreditation, or cross-repository adapters already exist.

## 1. Summary

Praxis should let a person build a lifelong, learner-controlled record of learning and demonstrated capability—whether it came from a university, apprenticeship, a book, paid work, open-source contribution, care work, community practice, research, a creative discipline, or a simulation.

The record is not one score and does not rank human worth. It is a graph of specific claims, the evidence supporting each claim, who assessed it, under what conditions, what remains uncertain, and when the claim should be reviewed. A diploma, résumé, competency profile, grant application, apprenticeship portfolio, or employer-facing summary becomes a **purpose-specific view** over that record, selected and shared by the learner.

Use the **Mycelix Praxis Continuous Learner Record (CLR)** as the internal product concept. Do not confuse this internal model with 1EdTech's formal Comprehensive Learner Record (CLR) standard; use the standards' full names in documentation.

## 2. Why this is a useful next step

Praxis already has local-first learning surfaces, mastery and recommendation projections, versioned Rust client contracts, and a credential summary surface. Its current learning contract also states that credential signature/revocation verification is not implemented and that a positive verification UI must remain unavailable until canonical-payload, issuer-signature, expiry, and revocation checks run. It also documents learner-progress publication to a public Holochain entry. Those limitations should shape this work, not be hidden by new UI.

The opportunity is to connect learning records to credible demonstrations of work while preserving the project’s existing fail-closed and learner-agency direction.

## 3. Product principles

1. **Learner agency first.** The learner can inspect, correct, export, selectively disclose, and choose when to share evidence. Sharing is an explicit action, not a background side effect.
2. **Claims are scoped.** “Can perform task X under conditions Y to rubric Z” is better than “is an expert.” State context, method, date, and limitations.
3. **Evidence is not authority.** A hash proves neither skill nor truth. A signature establishes an issuer relationship only after cryptographic verification and trust-policy evaluation. An AI-generated assessment is not automatically an accredited credential.
4. **No single human-value score.** Keep distinct capability dimensions, evidence provenance, confidence, context, and recency. Never collapse them into a universal learner/employability score.
5. **Many legitimate ways to learn.** Do not privilege classroom attendance, expensive tools, English-only content, or one assessment modality. Permit artifacts, demonstration, oral defense, apprenticeship, practical performance, peer attestation, formal qualifications, and self-asserted claims—with their provenance labelled.
6. **Privacy by default.** Keep raw evidence local or under learner-controlled storage by default. Public DHT writes need an explicit, comprehensible disclosure choice; avoid publishing identifiable learner activity by default. Treat metadata and predictable content hashes as potential leaks.
7. **Inspectable AI.** Symthaea may explain gaps and suggest practice, but the learner can contest or ignore the recommendation. Preserve model/version provenance and uncertainty. Machine scoring does not silently become final authority.
8. **Accessible, correctable records.** Support assistive technology, multilingual criteria, alternative assessment modes, appeals, corrections, expiry/review, suspension and revocation. Revocation must not pretend that already-exported copies can be recalled.

## 4. Record model: a graph, not a score

Keep an internal versioned model in Rust. Use explicit identifiers and enumerated types; do not expose raw Holochain `Record` values as browser DTOs. Start with the following conceptual entities; the exact wire schema is a later implementation decision.

### 4.1 Capability claim

A claim says what was demonstrated, in which context, against which versioned criterion. Suggested fields:

- stable competency/framework URI and human-readable label;
- claim scope and context (for example: “debug a Rust memory-safety issue in a reviewed patch”);
- criteria/rubric identifier and version;
- assessment method: self-assertion, artifact review, direct observation, practical test, simulation, peer attestation, course assessment, or regulated qualification;
- linked evidence IDs, assessor/issuer, observed date, and provenance;
- outcome per criterion, plus method-specific uncertainty—not a universal score;
- review/expiry policy where applicable; current status (claimed, evidenced, endorsed, disputed, expired, suspended, revoked);
- learner note, correction history, and accessible explanation.

### 4.2 Evidence artifact and assessment receipt

An evidence artifact refers to work or an observation; an assessment receipt records how a specific assessment was run. Keep them distinct. An artifact may be a patch, lab report, performance recording, design, field observation, portfolio item, or signed attestation. A receipt records the criterion, rubric version, task/scenario version, environment, tools, assessor, measurements, result, and limitations.

For reproducibility-oriented assessments, include the repository commit or release, dataset digest, configuration, seed when meaningful, and the digest of results/replay material. Do not assert that a digest makes the artifact accessible, authentic, reproducible, or proof of competence by itself.

### 4.3 Assurance and confidence are separate

Use descriptive evidence/assurance labels (for example, self-asserted; artifact attached; observed by a named assessor; independently reproducible; formally credentialed) to describe **how a claim was supported**, not to order people. Separately record the quality and limits of the method. A formal credential can be out of date; a strong practical demonstration can be valuable without being a formal qualification. Neither should silently overwrite the other.

### 4.4 Portfolio and presentation

The learner may assemble a purpose-specific view: “systems engineering role,” “solar microgrid apprenticeship,” “community water resilience project,” “grant application,” or “personal learning map.” Include only selected claims and disclosures. Support a signed portable export and an ordinary human-readable portfolio. The learner’s full private record is not the export default.

## 5. Standards and interoperability

Prefer interoperability over a new proprietary credential format:

- **W3C Verifiable Credentials Data Model v2.0**: base model for verifiable claims and holder-controlled presentation.
- **1EdTech Open Badges 3.0**: achievement/skill assertions with evidence and possible endorsements.
- **1EdTech Comprehensive Learner Record Standard 2.0**: bundle related achievement assertions into a learner record.

Official references:
- https://www.w3.org/TR/vc-data-model-2.0/
- https://www.1edtech.org/standards/open-badges
- https://www.1edtech.org/standards/clr

These formats complement rather than replace Praxis’s internal record model. Map supported fields losslessly where possible; document every loss, extension, and unsupported proof/status method. Do not advertise conformance until official fixtures/conformance tests pass for the declared profile. In particular, never treat the mere presence of `proof`, a `proof_type` string, a hash, or a credential record as successful verification.

Before showing a credential as verified, the verifier must fail closed on: canonicalization/payload mismatch, unknown or untrusted issuer key, invalid signature, unsupported cryptographic suite, expired credentials where expiry applies, unavailable or invalid revocation/status data where policy requires it, subject mismatch, and schema/profile mismatch. Distinguish “authenticity verified,” “issuer trusted for this claim,” “evidence assessed,” “credential current,” and “professional/licensing authority accepted”; they are not synonyms.

## 6. Cross-project integration

Praxis owns the learner-facing record, consent, portable exports, claim graph, credential lifecycle, and provenance contract. Other projects should emit **versioned assessment evidence** through a small adapter boundary rather than depend on Praxis UI internals or write directly to its DHT.

### Symthaea — reasoning and learning guidance

Use Symthaea to:
- map a learner-selected goal to a transparent competency graph and prerequisite gaps;
- recommend learning resources and propose assessment plans;
- summarize evidence and call out conflicts, staleness, missing criteria, or weak transfer evidence;
- help generate accessible practice variations and explanations.

Contract boundary: every generated plan or interpretation records model/version, input evidence references, output, and limitations. Recommendations are advisory. Symthaea must not fabricate completed work, silently change assessment criteria, infer sensitive traits, or issue an authoritative professional credential merely from its own evaluation. Its uncertainty and any failed/retracted benchmark evidence should remain visible.

### Symtropy — repeatable simulation and practical tasks

Use Symtropy as an assessment environment for tasks where simulated behavior is relevant: robotics, manipulation, vehicle/flight scenarios, manufacturing operations, safety procedures, construction planning, and resource allocation.

A simulation receipt should bind to a scenario ID/version, engine build/commit, relevant configuration, random seed when meaningful, input asset/dataset digests, task rubric, measurements, replay/result digest, and explicit limitations. Label this as **simulation evidence**. Passing a simulated task supports a claim about that bounded scenario; it does not, on its own, establish safe performance in the physical world. Where stakes require it, add supervised real-world demonstrations and assess sim-to-real transfer.

### Sol-Atlas — place-based and civic work

Use Sol-Atlas tasks to support evidence for spatial reasoning and applied community projects: mapping, infrastructure inventory, water/energy resilience, accessibility audits, housing analysis, and scenario comparison. Each task must identify region and dataset/version, data provenance, analysis method, known coverage gaps, uncertainty, and relevant privacy restrictions. A successful map or modeled intervention proves that a deliverable was produced under its stated conditions—not that the modeled outcome has occurred in the real world.

### Mycelix — trust, consent, community recognition

Mycelix can distribute learner-controlled attestations, endorsements, and governance-supported recognition among communities, guilds, employers, and institutions. Separate identity authentication, issuer reputation, assessor competence, and the quality of each claim. Community consensus must not be represented as objective truth, and a majority vote should not be able to erase a learner’s private work or silence a documented appeal.

## 7. End-to-end flow

1. Learner selects a goal and chooses what evidence can be used.
2. Praxis resolves it to versioned competencies/criteria and presents the rubric before work begins.
3. Symthaea offers an explainable learning or assessment plan; learner accepts, edits, or rejects it.
4. An assessment happens in Praxis, Symtropy, Sol-Atlas, or a human/community setting.
5. The adapter submits an assessment receipt and artifact references. Praxis validates schema, provenance, signature where required, subject binding, task/rubric version, consent scope, and policy.
6. The receipt enters the learner’s record with its modality and assurance clearly labelled. Disputes and missing verification stay visible.
7. An eligible issuer or assessor—not the UI alone—may endorse or issue a portable credential under a defined policy.
8. The learner previews a purpose-specific presentation, selects fields/evidence, and explicitly exports or shares it.
9. Verifiers check the credential and issuer trust, current status, and stated scope. They see limitations rather than a misleading green “verified” badge.

## 8. Privacy and threat model requirements

- Do not place raw portfolios, precise location histories, private feedback, or identifiable assessment traces in a public DHT by default.
- Do not infer that Holochain agent-centric storage makes a public entry private. The existing public progress entry needs a separate privacy/migration decision before this model is wired to it.
- Use pairwise/purpose-limited subject identifiers where feasible; avoid reusing one global identifier across every institution and employer.
- Store large evidence under a learner-controlled local/private store or explicitly consented encrypted storage. Treat access metadata, encryption key custody, backups, and deletion as part of the threat model.
- Avoid public unsalted hashes of predictable personal artifacts. Keep commitments/digests private where public publication creates correlation or guessing risks.
- Sharing a credential is an explicit, previewable action. Provide a non-networked export path. Be honest that recipients can copy a credential/evidence after it has been shared.
- Record consent scope and disclosure purpose; do not repurpose the record for hidden employee ranking, insurance, credit, law enforcement profiling, or unrelated behavioral scoring.
- Provide a correction and appeal path that appends a transparent correction/withdrawal status without pretending an already-public immutable record was erased.

## 9. Evaluation gates (targets, not current results)

| Area | Pilot gate |
| --- | --- |
| Credential verification | Invalid signature, changed payload, wrong subject, untrusted issuer, unsupported proof profile, expired credential, and required revoked/suspended status are all rejected; false acceptance in the negative-control suite is zero. |
| Standards mapping | Declared W3C VC 2.0 / Open Badges 3.0 / CLR 2.0 profile passes its applicable official conformance fixtures; unsupported fields and conversions are surfaced, not dropped silently. |
| Export portability | Round-trip tests preserve every field in the declared supported profile or report an explicit, tested loss map. Test with at least two independent verifier implementations before claiming interoperability. |
| Privacy | No raw evidence or precise location appears on a public DHT by default; no external sharing occurs without an explicit learner action; test logs and telemetry contain no private evidence. |
| Assessment provenance | Every accepted external receipt identifies issuer/assessor, rubric and task versions, relevant environment, timestamp, and scope; tampered receipts fail closed. |
| Learning effectiveness | Measure delayed retention and transfer at 30/90 days, time to demonstrated competence, and assessment calibration against a baseline; report sample sizes and uncertainty. Engagement streaks and XP are not evidence of mastery. |
| Fairness/accessibility | Compare false-rejection and assessment outcomes across language, disability/assistive-technology mode, low-bandwidth/offline usage, device class, and prior access to tools. Investigate disparities instead of hiding them in a global mean. |
| Sim-to-real validity | For claims based on simulation, measure prediction/transfer against supervised real-world or independent practical assessment where feasible; label simulator-only findings explicitly. |
| Performance and resilience | Establish measured baselines for local search, export, verification, offline operation, conflict resolution and backup/restore before setting release SLOs. |
| Agency and trust | Learners can inspect why a claim is present, correct it, export it, and control disclosure; usability tests verify that visibility and limitations are understood. |

All numerical thresholds beyond hard correctness/privacy gates should be set only after a baseline and risk assessment. Publish benchmark manifests, dataset versions, test commands, confidence intervals, and negative controls. “CI queued” is not test evidence.

## 10. Incremental implementation plan

### Phase A — contract and privacy boundary
- Add versioned Rust types for capability claims, evidence artifacts, assessment receipts, and verification status.
- Keep these as pure domain/DTO types initially; no cross-repository runtime coupling.
- Document local/private/public visibility and consent transitions.
- Add synthetic fixtures and adversarial tests for missing provenance, tampering, unsupported methods, and subject mismatch.

### Phase B — credential correctness
- Audit the existing credential schema/profile against VC Data Model 2.0 and Open Badges 3.0.
- Implement real canonical payload/signature/key/status verification or keep verification unavailable. Add negative controls before positive UI.
- Add learner-owned export/import and an explicit loss map; do not claim CLR/Open Badges conformance before tests pass.

### Phase C — first evidence adapters
- Start with one low-risk end-to-end adapter using an open-source work artifact, such as a reviewed PR with a clearly versioned rubric.
- Add one deterministic Symtropy assessment adapter and one Sol-Atlas data-analysis assessment adapter only after their receipt contracts and provenance are reproducible.
- Require learner consent before the adapter associates a receipt with a learner record.

### Phase D — guided lifelong record
- Add a capability graph and learner-created, purpose-specific portfolio views.
- Let Symthaea generate inspectable recommendations from evidence gaps; evaluate recommendation quality against learner-chosen goals and baseline strategies.
- Add human/peer endorsement, appeal, expiry/review, and issuer trust policy.

### Phase E — pilot and interoperability
- Run the evaluation gates with negative controls, independent verifier implementations, accessibility tests, and explicit retention/transfer evaluation.
- Pilot across contrasting contexts (formal coursework, hands-on craft, open-source contribution, and a community/civic project) before expanding the competency catalogue.
- Publish results and failures; do not conflate product completion with accreditation.

## 11. Decisions intentionally left open

- Which competency/framework vocabularies should be first-class versus linked as external URIs?
- Which proof suites and status mechanisms will be supported in the first credential profile?
- What data belongs in local storage, a personal vault, private source chain, or a shared DHT after an explicit privacy review?
- Which first task best tests the end-to-end pipeline without pretending a prototype grants professional accreditation?
- Who is authorized to endorse which claim types, how conflict-of-interest and appeals work, and what evidence a verifier must require for a high-stakes role?

## 12. Definition of done for the first vertical slice

A learner can choose a bounded capability, see its rubric, complete an independently reproducible task, inspect a provenance-bearing receipt with limitations, store it locally without publishing private evidence, export a verifiable claim under a declared profile, and have an independent verifier accept valid evidence and reject tampered/unsupported evidence. The flow includes an explicit sharing action, human-readable output, and a tested withdrawal/dispute path.

This vertical slice—not a broad badge catalogue or AI-generated score—is the smallest credible foundation for a continuous diploma.
