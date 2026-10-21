# Open Home Health Aids and Medicine Dossier — Architecture v0.1

**Status:** public architecture proposal; not clinical guidance, a manufacturing instruction, or legal advice  
**Owner proposal:** Mycelix provenance/governance contract; coordinated with Symthaea, Sol Atlas, Symtropy, and existing Mycelix manufacturing work  
**Tracking:** [MED-OPEN-001 #5027](https://github.com/Luminous-Dynamics/mycelix/issues/5027)  
**Research checked:** 14 October 2026

## 1. The idea: open access and reproducible quality, not amateur drug manufacture

We should make useful home care more understandable and affordable, enable people to build low-risk assistive aids, and make pharmaceutical knowledge and manufacturing dossiers more reproducible where rights permit.

Those goals contain distinct trust boundaries:

- **Open home-care knowledge** helps people understand established first aid, prevention, safe use of existing products, warning signs and how to reach qualified care.
- **Open household aids** includes repairable accessibility tools, organizers, labels, care-kit fixtures and other low-risk non-medicinal designs, each scoped to an intended use.
- **Existing medicine access** helps people locate authorized products, compare public price/availability evidence and check current sources. It must not replace a pharmacist or clinician.
- **Open medicine dossiers** make scientific evidence, product identity, testing and stability methods, quality criteria, applicable licenses and manufacturing evidence reproducible for qualified, appropriately authorized makers and independent evaluators.

**Open source is not proof of safety, effectiveness, product quality, regulatory authorization, or permission to manufacture a medicine at home.** WHO describes GMP as controls across the production system because some hazards—including contamination, mix-ups, incorrect labeling and too much or too little active ingredient—cannot be eliminated by testing the final product alone. [WHO — Medicines: Good manufacturing practices](https://www.who.int/news-room/questions-and-answers/item/medicines-good-manufacturing-processes).

This project must not publish AI-generated at-home synthesis or compounding recipes, dose recommendations, extraction methods, or instructions for unqualified manufacture of high-risk medicines. A standardized product used exactly according to its current label is not equivalent to a homemade substitute.

## 2. The first useful user-facing release

Start by improving use of existing, quality-assured products and public care information—not by making medicines.

1. A small, offline-capable library of authoritative, versioned first-aid, prevention and care-navigation cards.
2. Clear distinctions among “follow the current product label,” “general education,” “ask a pharmacist/clinician,” and “seek urgent care.”
3. Care-kit and accessibility-aid plans using ordinary, low-risk products and materials, with clear intended-use limits.
4. Expiration and storage reminders that refer back to a named product and its labeling; don't calculate or invent dosing.
5. A directory of legitimate supply channels and price/availability data with source timestamps and explicit missing/stale status.
6. A safe reporting route for confusing instructions, suspected quality problems, adverse events and recalls.
7. Translations that carry the source version, translator/reviewer, jurisdiction and expiration forward instead of becoming unversioned copies.

WHO's “Medication Without Harm” programme explicitly includes the patient/public and the use of medicines at home, and recommends engagement alongside health professionals. [WHO — Medication Without Harm](https://www.who.int/initiatives/medication-without-harm). The design should support safer decisions, not diagnose, prescribe, or act as a substitute clinician.

### Example of the boundary

WHO's essential-medicines guidance aims for availability, affordability and assured quality. The 2025 WHO Model List includes more than 520 medicines, but inclusion in a model list is not by itself proof that a particular product is authorized for every jurisdiction or appropriate for an individual. [WHO — Essential medicines](https://www.who.int/news-room/fact-sheets/detail/essential-medicines).

For South Africa, the National Department of Health publishes Standard Treatment Guidelines and Essential Medicines List resources, including a primary-care EML/STG PDF updated in August 2026. Treat the official source as a pointer until redistribution rights are confirmed; do not copy the document into our corpus merely because it is publicly accessible. [NDoH — STGs and EML](https://www.health.gov.za/nhi-edp-stgs-eml/).

## 3. A dossier is not a single safety score

A single green badge or composite confidence number would hide what is known, what is unknown and who has authority. Keep these evidence dimensions separate:

| Dimension | Example evidence | What it does *not* prove |
|---|---|---|
| Clinical benefit and harms | Quality-assessed clinical evidence, systematic reviews, guideline references | Product identity, batch quality, or individual suitability |
| Product identity | Exact product/ingredient, formulation, strength, dosage form, jurisdiction-specific identity | That a physical lot matches its record |
| Material quality | Supplier/lot identity, specifications, contaminants, independent analytical results | Clinical efficacy |
| Analytical method | Method version, validation status, sample identity and lab report | That manufacturing followed the qualified process |
| Stability and packaging | Exact formulation/packaging, storage conditions, study reference and expiry basis | That a new package/site/lot inherits the same result |
| Manufacturing capability | Facility/site identifier, applicable QMS/GMP evidence, process revision, equipment/calibration state | Product authorization or batch release |
| Product authorization | Regulator, jurisdiction, exact product/configuration, status/currentness and source | Authorization in a different jurisdiction or for a changed product |
| Batch disposition | Exact output lot, applicable specifications, authorized disposition record | A clinical claim beyond the lot and intended use |
| Access and affordability | Time-stamped offers/prices, procurement/stock status, geography and terms | Quality, availability today, or clinical interchangeability |
| Safety and recall | Adverse-event/quality signal reference, triage, status, affected lots and recall/supersession | That absence of reports means absence of harm |

Use labels such as `Known`, `Unknown`, `Conflicting`, `Stale`, `Restricted`, `Failed`, `Withdrawn` and `NotApplicableWithRationale` per dimension. Never silently convert missing evidence into a reassuring default or infer a product-wide claim from a single lot.

## 4. Versioned contract sketch

The first software deliverable should be small, renderer-neutral and dependency-light. Types below are conceptual names, not frozen wire formats.

### `EvidenceSnapshotRefV1`

Required fields:
- stable provider and source-record identifier;
- canonical source URL and dataset/release/version identifier, where supplied;
- publication/observation time (if known), retrieval time, and source-declared update time as separate fields;
- artifact digest plus an explicit `VerifiedAgainstCapturedBytes` / `DigestNotVerified` state;
- capture method, parser/adaptor version, and snapshot-manifest identity;
- source assertion vs reported result vs derived aggregation vs model hypothesis;
- source lineage, correction, retraction and supersession references.

A digest provides content identity, not source authenticity, endorsement, clinical truth or lawful reuse.

### `RightsAndLicenseAssessmentV1`

Keep access and reuse independent of evidence quality:
- license/terms identifier and version, if known;
- record-level and dataset-level rights where they differ;
- permitted use (view, derive, redistribute, commercial use, model training) only when the source grants or reliable review establishes it;
- attribution, share-alike, patent, database and territorial constraints as applicable;
- review owner, timestamp, source evidence and expiry/recheck date;
- status: `KnownOpen`, `AttributionRequired`, `Restricted`, `Unknown`, `ReviewRequired`, `Expired`.

If redistribution rights are unknown, retain a citation and minimal source metadata only when that capture is itself permissible; do not mirror the underlying file. Public accessibility is not a license.

### `HealthKnowledgeCardV1`

Should bind:
- card ID and semantic/version identity;
- intended audience and jurisdiction;
- scope and explicit non-scope;
- primary sources and exact versions;
- author, clinical reviewer / accountable review body, translation lineage, reviewed date and expiry;
- risk classification and escalation/red-flag reference;
- correction, withdrawal and supersession state;
- user-comprehension/accessibility review;
- distribution license, only if explicitly established.

A language translation is a derivative safety artifact: it needs review, and must not outlive the source it translates. Do not accept a bare AI summary as approval evidence.

### `OpenMedicineDossierManifestV1`

Binds an exact medicine/product/configuration and declares:
- stable product identifier, active ingredient identity where applicable, strength/form/packaging identifiers, and intended-use scope from accountable sources;
- jurisdiction and applicable product authorization reference/currentness;
- patent/license/technology-transfer rights evidence where applicable;
- source snapshot IDs for quality, clinical, analytical, stability, manufacturing and labeling evidence;
- qualified facility/manufacturer and quality-system evidence references;
- applicable test/specification revisions and output-batch links;
- safety-monitoring contact/route, signal and recall linkage, and last-review/expiry status.

The manifest links to evidence; it is not a manufacturing recipe and does not authorize a maker or release a lot.

### `ProductEvidenceDimensionV1` and `SafetySignalAndRecallRefV1`

Each dimension needs an evidence reference, scope, reviewer/issuer, status, validity interval and correction/supersession links. Safety signals must be reportable before causality is established; keep `Reported`, `UnderTriage`, `Confirmed`, `Refuted`, `Closed` and `RecallAction` distinct, with qualified owners controlling transitions. Don't expose reporter or patient identity in public records.

## 5. Explicit lifecycle and authority gates

Use scoped, auditable states rather than a numeric maturity score:

`ResearchOnly` → `SourceLinked` → `ReviewRequired` → `GuidanceApproved`

For a medicine dossier, separate dimensions might independently progress through evidence review while manufacturing disposition follows a different pathway:

`ManufacturerQualificationEvidenceRecorded` → `ProductAuthorizationEvidenceRecorded` → `BatchDispositionRecorded`

Those arrows are process examples, not automatic transitions. Qualified accountable reviewers must authorize each applicable transition. A dossier can be public while its product is unauthorized; a manufacturing facility can be qualified while a product's clinical evidence is incomplete; a valid authorization can expire or be withdrawn; an affected lot can be recalled without deleting history. Preserve all of these distinctions.

### No implicit promotion

- Research, traditional-use reports and hypotheses remain visibly distinct from reviewed public guidance.
- A model-generated confidence score can never authorize a health claim.
- One jurisdiction's authorization never implicitly carries to another.
- Changing formulation, supplier, site, packaging, software-controlled process or intended use triggers review of affected evidence.
- Missing, contradictory, expired, withdrawn or recall-affected evidence fails closed for the decision it governs.
- A failed safety gate doesn't have to make general education unavailable; it blocks only the affected higher-risk pathway.

## 6. Component ownership across the four projects

### Mycelix — identity, rights, provenance and governance

Use Mycelix for versioned evidence snapshots, permission/rights decisions, signed review and disposition events, provenance edges, correction/retraction/supersession, role-scoped approvals, appeals and audit receipts. Build on existing manufacturing execution/lot evidence work (see [MFG-001 #4685](https://github.com/Luminous-Dynamics/mycelix/issues/4685)) rather than introducing a competing manufacturing runtime. Mycelix provenance does not mint clinical truth or regulatory authority.

### Symthaea — bounded synthesis and uncertainty

Use Symthaea to locate candidate sources, structure arguments, find contradictions, identify missing evidence and propose research hypotheses. Each output must distinguish quoted/source assertions, extracted facts, derived results, uncertain interpretations and unsupported hypotheses. It may not independently certify medicines, select a treatment for a person, approve a translation, or authorize a production/release decision. Reuse [HEALTH-MFG-000 #5969](https://github.com/Luminous-Dynamics/symthaea/issues/5969), [MED-PRODUCT-001 #4727](https://github.com/Luminous-Dynamics/symthaea/issues/4727), and the existing public-health epistemic ladder in [issue #135](https://github.com/Luminous-Dynamics/symthaea/issues/135).

### Sol Atlas — source-backed access and geographic projection

Reuse [ATLAS-DSCI-001 #99](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/99) for licensed snapshot metadata and evidence projections. Map aggregated, source-attributed provider capability, procurement/stock signals and logistics dependencies only where the source rights and granularity allow it. Keep observation date, retrieval date, geographic resolution, coverage gaps and stale status visible. Do not map individual patients or precise home medicine needs.

### Symtropy — deterministic counterfactuals, not clinical prediction

Reuse the existing synthetic scenario/replay substrate (e.g. [ENG-SYNTH #1172](https://github.com/Luminous-Dynamics/symtropy/issues/1172)) for non-clinical resilience questions:
- supplier interruption and essential-item stockout;
- cold-chain/storage disruption represented by externally sourced threshold/status events;
- manufacturing-node outage, test failure and batch quarantine;
- constrained transport, power or communications;
- compare redundancy, lead time, procurement diversification, local capacity and total lifecycle cost under stated assumptions.

Scenarios can estimate service/stock continuity under declared model assumptions. They cannot establish clinical outcomes, product quality, dosage suitability or a real facility's capability.

## 7. Rights, privacy and regulatory boundaries

- Link to official and licensed sources first. A public government PDF is not automatically open-licensed; verify its terms before ingestion, mirroring, translation, or model training.
- Separate code/software, raw datasets, clinical guidance, product documentation, patents and traditional knowledge. They may require different licenses and permissions.
- Consider open licenses only after rights review; a license choice cannot retroactively grant patent rights, source-data rights or community knowledge rights.
- Traditional/Indigenous knowledge must not be ingested, translated, modeled, or commercialized without valid permission, community governance, and applicable fair benefit-sharing arrangements.
- Public graph records must not contain patient-identifying health data. Keep private operational and clinical information outside this public dossier contract.
- For South Africa, verify current SAHPRA requirements for each intended activity and product. The current SA Guide to GMP for Medicines is version 10, updated 24 April 2026; SAHPRA's licensing guidance (version 4, updated 13 April 2026) addresses applicants seeking authority to manufacture, import or export medicines/scheduled substances. These sources are a starting point, not a complete legal opinion:
  - [SAHPRA — GMP guide](https://www.sahpra.org.za/document/sa-guide-to-good-manufacturing-practice-for-medicines/)
  - [SAHPRA — Licence to manufacture, import or export](https://www.sahpra.org.za/document/licence-to-manufacture-import-or-export/)
- Medicine-patent licensing and technology-transfer programmes can be relevant to legitimate generic production, but a specific license has its own molecule, territory, licensee and use conditions. [Medicines Patent Pool — agreements with innovators](https://medicinespatentpool.org/progress-achievements/licences).

## 8. First synthetic qualification corpus

No real medicine recipes, patient records, price feeds, or raw copyrighted clinical source files are needed for the first implementation. Use synthetic product IDs and fabricated artifact hashes.

Required negative controls:
1. open-looking URL but redistribution rights unknown → no corpus mirroring;
2. digest exists but captured bytes are absent → `DigestNotVerified`;
3. source revision is retracted/superseded → old record remains addressable but not current guidance;
4. manufacturer evidence matches but product identity differs → no transfer of authorization;
5. current product authorization but changed formulation/pack configuration → scoped review required;
6. laboratory result is valid but the wrong lot was sampled → doesn't qualify target lot;
7. all visible evidence is complete except one required dimension → that dimension remains unresolved;
8. one source is counted twice through derivative feeds → corroboration must retain lineage and not count it as independent;
9. recall signal targets a subset of batches → affected scope is precise, with the recall authority external;
10. Symthaea proposes a plausible use unsupported by source evidence → remains a hypothesis;
11. Sol Atlas receives an expired stock snapshot → displays stale/unknown, not in-stock;
12. Symtropy scenario succeeds synthetically → cannot mark a product or facility as authorized/qualified.

Independent verification should validate the same immutable manifest without importing the producer's verdict function. Freeze canonical identity vectors, versions and negative controls. Exact-head CI must capture source, dependency lock, toolchain, fixtures and result; queued is not passing.

## 9. Success measures

Track real public value without overstating clinical impact:
- source-rights decisions completed and records correctly withheld where rights are unclear;
- proportion of active public cards with current primary sources, accountable review and expiry;
- translation review and comprehension performance;
- catalog data freshness and explicit missingness;
- product/batch provenance and independent evidence reproducibility;
- time from a quality/adverse-event signal to accountable triage, correction or recall action;
- medicine/aid affordability including delivery and waste, not list price alone;
- supply availability, geographic coverage, supplier concentration and recovery time under synthetic scenarios;
- number of false promotions blocked by negative controls;
- zero unqualified automated clinical, regulatory or batch-release decisions.

Do not claim reduced morbidity/mortality or proven access improvement from a prototype or simulation. Evaluate those outcomes only with appropriate governance, data protection, ethics and qualified study design.

## 10. Build order

1. Freeze the public schema and trust/rights boundary.
2. Build a synthetic, dependency-light manifest validator and independent verifier.
3. Add licensed source snapshots and corrections/withdrawals using Sol Atlas's evidence contract.
4. Add Symthaea source contradiction/missing-evidence reports without automated approval.
5. Add Symtropy scenario fixtures for stock continuity and manufacturing resilience.
6. Only after reviews and permissions, pilot with a small number of low-risk public health-aid cards and source-linked South African essential-medicine metadata.
7. Treat any actual medicine formulation/manufacturing work as a separate qualified, jurisdiction-specific programme.

**Bottom line:** make care knowledge open, aid designs reproducible, medicine evidence auditable and qualified production easier to reproduce. Do not make “open-source” or “home-made” a substitute for quality, rights, or accountable clinical and regulatory decisions.
