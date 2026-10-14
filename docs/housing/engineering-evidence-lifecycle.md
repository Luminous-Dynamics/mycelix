# Housing Engineering Evidence Lifecycle

- **Status:** Draft integration design; no new zome entry types are implemented by this document.
- **Date:** 2026-10-13
- **Related design:** [Symtropy housing engineering RFC](https://github.com/Luminous-Dynamics/symtropy/blob/rfc/housing-engineering-evidence-contract-2026-10/docs/built-environment/housing-engineering-rfc.md)
- **Scope:** Mycelix Commons housing inventory, inspection, provenance, approval, and maintenance records.

## 1. Purpose

Mycelix should make housing engineering evidence traceable across a building's lifecycle without trying to turn its civic coordination layer into an engineering solver.

A building model or calculation is produced by the engineering toolchain. Mycelix records which project/building it concerns, which revision was evaluated, the evidence manifest digest, what kind of review was performed, who attested to that review, and whether a newer record supersedes it. That allows the engineering work to be audited and shared without treating the DHT as a source of physical truth.

This is a proposed interface contract, not a statement that the housing commons currently provides this lifecycle.

## 2. Existing structures and the gap

The current Commons sources already provide a useful operational starting point:

- `mycelix-commons/zomes/housing-units/integrity/src/lib.rs` defines `Building` and `Unit` records.
- `mycelix-commons/zomes/housing-maintenance/integrity/src/lib.rs` defines maintenance requests, work orders, and an `Inspection` record with inspector, date, findings, and a `passed: bool`.
- `mycelix-commons/zomes/housing-clt/integrity/src/lib.rs` defines community land trusts, ground leases, resale formulas, and affordability reporting.

These are useful for inventory, stewardship and operations, but an inspection boolean and free-text findings cannot represent a versioned engineering evaluation with separate requirements, solver inputs, output hashes, uncertainty, validity domains, review status, and superseding revisions.

**Do not reinterpret the existing `Inspection.passed` field as overall building-code compliance.** It can describe the scope of that specific inspection. Broader engineering evidence should be represented explicitly and linked to the relevant building and inspection records.

## 3. Proposed record separation

Keep four distinct concepts separate so one cannot be confused with another.

| Record | Purpose | Illustrative states |
|---|---|---|
| **Engineering case** | Identifies a building/project model, jurisdiction, model revision, and evidence manifest. | Draft, submitted, superseded, withdrawn |
| **Evaluation claim** | Reports an individual rule/analysis outcome from the manifest. | PASS, FAIL, REQUIRES_REVIEW, NOT_EVALUATED, BLOCKED, INSUFFICIENT_EVIDENCE |
| **Review or approval attestation** | A named reviewer states what they reviewed and the scope of that decision. | Pending, accepted, rejected, expired, superseded |
| **Operational inspection / maintenance record** | Captures a dated physical observation, defect, repair, or work order. | Existing domain-specific states |

The engineering claim states and reviewer/approval states must not be collapsed into a single `approved: bool`. A calculation can pass a stated criterion while its design still awaits professional review or statutory approval.

## 4. Proposed evidence reference

A future evidence-case record should contain only the minimum index data needed to find and verify the external manifest:

- stable case identifier;
- reference to the Mycelix building record and, where appropriate, the related inspection or work-order record;
- optional versioned site-evidence projection ID and digest, using the read-only contract in the [Symtropy housing RFC](https://github.com/Luminous-Dynamics/symtropy/blob/rfc/housing-engineering-evidence-contract-2026-10/docs/built-environment/site-evidence-adapter-contract.md);
- model identifier and exact revision identifier;
- jurisdiction and rule-pack identifier/status;
- manifest schema version and manifest SHA-256 digest;
- SHA-256 digest of the independently controlled expected-claims contract, which pins the exact schema digest and verifier executable/version, plus case/jurisdiction/model bytes/rule pack/toolchain and mandatory-claim definitions, criteria, scope, approved tools, and exact input/output digests;
- an authorized content URI or retrieval locator, if used, with access/retention policy;
- the manifest-declared aggregate status kept separate from the verifier's evidence-integrity verdict and a count of each claim state;
- author identity reference, creation timestamp, and provenance for the verifier that computed the aggregate;
- immutable supersession/retraction relationship to previous records;
- links to separate review/approval attestations.

The full manifest and large raw artifacts should not be copied into a Holochain entry by default. Keep their canonical bytes in a declared artifact store, then record their digest and locator. The artifact store, digest checker, access policy, and retention policy must be explicit. A digest mismatch blocks verification.

### Site-evidence projection

The site-evidence projection is an optional read-only reference, not a runtime dependency on the Sol Atlas renderer. Follow the [site-evidence adapter contract](https://github.com/Luminous-Dynamics/symtropy/blob/rfc/housing-engineering-evidence-contract-2026-10/docs/built-environment/site-evidence-adapter-contract.md): preserve source/product/version and acquisition time, capture/receive time, coordinate reference system and axis order, native spatial resolution, footprint/coverage, nodata/QA-mask accounting, uncertainty/accuracy, transform-chain and artifact hashes, reuse rights, and prohibited inferences. Distinguish catalogue metadata, captured bytes, processed measurements, and engineering applicability review.

A globe marker, map tile, H3 cell, coarse elevation raster, public utility-layer listing, or satellite water index must not satisfy a parcel boundary, legal access/setback, foundation, groundwater, drainage, or design-flood claim by itself. Keep absent, stale, low-coverage, unlicensed, out-of-domain, or unreviewed site evidence as unresolved. Data-source authority is not an error-free guarantee: the Council for Geoscience [publicly disclosed that an incorrect, unauthorised dolomite-land layer was published on its portal in February 2025 and then withdrawn](https://www.geoscience.org.za/statement-on-the-erroneous-publication-of-an-incorrect-version-of-the-dolomite-land-layer-on-the-cgs-data-portal/). The adapter must retain exact product/version and retrieval time and check custodian corrections/notices before relying on a layer; an official portal presence alone cannot clear a geohazard. Mycelix should store the projection ID/digest and status summaries rather than copy raw raster products or treat the renderer as an authority.

The schema under development is in the [Symtropy RFC branch](https://github.com/Luminous-Dynamics/symtropy/tree/rfc/housing-engineering-evidence-contract-2026-10/docs/built-environment). Treat its schema/version as provisional until fixture-based conformance tests and a review are complete.

## 5. Validation and trust boundaries

Integrity validation can enforce well-formed, coherent, append-only records. It cannot independently establish that a building is safe or that an external artifact is truthful.

Required design constraints:

1. **Identity and authorship:** verify entry authorship and links; identify which authorized identity asserted each claim or review.
2. **Append-only history:** revise by creating a new record linked to the prior record. Do not overwrite the evidence history with a new status.
3. **Typed references:** referenced building, model revision, rule pack, manifest and predecessor records must resolve and have the expected type.
4. **Status integrity:** the manifest's `overall_status` is an untrusted declaration. Preserve `declared_status` and conservative `effective_status` separately from `evidence_integrity_status: EVIDENCE_INTEGRITY_VERIFIED`. The integrity field means the pinned contract, manifest structure and local artifact bytes matched; it does not recompute structural, fire, thermal or code-compliance results. Any computed engineering evaluation must identify its own evaluator/version and remain distinct from professional review and statutory approval.
5. **Fail-closed aggregation:** overall PASS is impossible if any mandatory claim is failed, blocked, not evaluated, insufficiently evidenced, or awaiting required review. Missing claims must not be silently omitted from the expected requirement set.
6. **Digest verification:** check the manifest, independently pinned expected-claims contract, model/input/output/tool bytes, and any provided rule-source snapshots against actual bytes. Every evaluated `PASS` or `FAIL` claim must carry non-empty input and output artifact digest lists, a registered tool with pinned executable/container bytes, and a non-empty measured result. A `FAIL` is an evaluated finding, not a bare status label. A mandatory code-compliance claim cannot be evidence-integrity verified unless its source has a SHA-256 snapshot linked to a registered artifact, the bytes match, and the source metadata identifies a recognized legislation/standard/regulation/technical-guidance type, edition or effective date, clause/location, and public/licensed reuse state. This metadata gate does not interpret the clause or decide applicability. A SHA-256 match does not establish correctness or real-world truth.
7. **Scoped reviews:** an attestation must record what was reviewed, the model revision, scope, applicable qualifications/authority, limitations, and date. It must not imply approval of later revisions.
8. **External approval:** permits, certifications and other statutory decisions are separate evidence records with issuing body, identifier, scope, validity dates and status. Never synthesize or infer an approval.
9. **Privacy:** keep occupant identity, tenancy details, access credentials, health information and unnecessary precise location data out of common engineering manifests. Use separate consent-controlled records when personal data is genuinely needed.
10. **Rights-aware sources:** record pointers, edition/clause identifiers, rights status and hashes where lawful. Do not replicate copyrighted standards text without the appropriate rights.

## 6. Lifecycle

1. Create or identify the Commons `Building` record.
2. Register the engineering case with a specific design/model revision and provisional or confirmed jurisdiction.
3. Produce an evidence manifest through the versioned external engineering workflow.
4. Verify the manifest's schema and artifact bytes against the out-of-band pinned expected-claims contract; preserve the manifest-declared status separately from the verifier's evidence-integrity verdict.
5. Publish an immutable case record with the declared aggregate status, the evidence-integrity verdict, verified artifact/claim counts, verifier version, and contract/manifest digests. Do not relabel package integrity as an engineering result.
6. Request scoped review from a qualified professional, test body, or authority as applicable; store each response as a separate attestation.
7. Link physical inspections and construction records to the exact design revision, recording deviations rather than silently editing the design record.
8. For renovations, changed materials, failed inspections, revised calculations or changed rules, create new records and explicitly supersede only the affected prior evidence.
9. At operation time, connect approved inspections, maintenance work and observed performance to the relevant building elements, subject to privacy and retention policy.

## 7. Acceptance tests before production use

The first implementation should include unit tests and multi-agent Holochain validation tests for at least these cases:

- reject malformed SHA-256 values, invalid status values, missing required IDs, and unknown schema versions;
- reject an evaluated `FAIL` claim whose input/output digests are empty or whose measured result is missing/null;
- reject a record that references a nonexistent building or predecessor;
- reject self-asserted aggregate PASS when any mandatory claim is unresolved or failed;
- reject a manifest, model artifact, claim input, tool executable/container, or provided rule-source snapshot whose bytes do not match its independently pinned digest;
- reject a mandatory code-compliance claim whose rule source lacks source type, edition/effective date, clause/location, an allowed rights state, or a source digest linked to matching artifact bytes;
- reject changes to the embedded schema digest, verifier version/executable digest, case/jurisdiction, model artifact digests, rule pack, toolchain, mandatory claim criteria, subject element scope, approved tool ID, or claim input/output digests when the expected-claims contract is pinned;
- never render `EVIDENCE_INTEGRITY_VERIFIED` as engineering compliance or professional/statutory approval;
- reject a review attestation that names a different model revision from the case under review;
- prevent a superseded record from appearing as the current review result without displaying its supersession state;
- preserve the complete history when an attestation is rejected, withdrawn, or superseded;
- ensure `NOT_EVALUATED` is rendered as unknown—not compliant—and that missing evidence is visible;
- ensure a valid record signature or DHT validation is not represented as proof of physical safety;
- ensure private occupant data is not copied into shared public/common engineering records.

A test passing at the single-agent unit layer is not sufficient for the integrity/link-validation behavior. Include multi-agent tests when the hApp harness supports them, and report any missing harness as a blocker rather than a pass.

## 8. Integration milestones

- **M0 — Contract:** review the manifest schema, status meanings, identifier policy, and privacy model.
- **M1 — Read-only prototype:** consume an example evidence manifest, verify its digest and schema, then render each claim status and unresolved issue without issuing approvals.
- **M2 — Integrity records:** add a versioned engineering-case entry type and validation/link rules without breaking existing Building, Unit, or Inspection records.
- **M3 — Review attestation:** bind qualified reviewer credentials to a specific model revision and scope; support rejection and supersession.
- **M4 — Lifecycle links:** associate work orders, inspections, material batches and as-built changes with the appropriate model revision.
- **M5 — Multi-agent assurance:** test invalid author, invalid links, races/duplicates, supersession, and inconsistent status combinations across agents.
- **M6 — Pilot readiness review:** document external audit needs, operational data rights, recovery/retention behavior, and the applicable professional and statutory approval path before use on a real construction project.

## 9. Non-goals

- This work does not implement BIM authoring, structural/thermal simulation, code interpretation, or permit issuance.
- It does not assert the housing commons, inspection UI, or any model shown in a demo is production-ready.
- It does not let a governance vote override mandatory safety requirements or replace the competent authority.
- It does not treat decentralized consensus as a substitute for independent engineering review, material testing, physical inspection, or statutory approval.
