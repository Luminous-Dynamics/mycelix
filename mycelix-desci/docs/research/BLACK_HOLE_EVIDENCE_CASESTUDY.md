# Black-hole scientific evidence case study

**Purpose:** Apply the existing Mycelix-DeSci signed scientific-event protocol to black-hole research without adding a parallel truth, provenance, or trust system.

## Existing canonical substrate

The canonical implementation is `mycelix-desci/src/core/src/scientific_events.rs`. Reuse its existing types and invariants:

- `ResearchObject` / `ResearchObjectType`: manuscripts, datasets, software, workflows, protocols, and models.
- `AtomicClaim`: one scoped scientific proposition at a time.
- `EvidenceArtifact`: content hash, media type, locator, license, availability.
- `Attestation`: review, computational reproduction, independent replication, critique, conflict disclosure, correction.
- `EvidenceOutcome`: supports, does not support, inconclusive.
- `ScientificEventPayload`: append-only proposal, evidence attachment, attestation, correction, withdrawal, supersession and retraction.

The current protocol already provides signed events, an explicit canonical codec, per-stream hash chaining, optimistic concurrency, idempotency and deterministic evidence projections. Do not create a second DeSci event log for astrophysics.

## Domain manifest

The schema in [black_hole_evidence_manifest_v1.schema.json](schemas/black_hole_evidence_manifest_v1.schema.json) is a domain-specific manifest artifact, not a new canonical Mycelix event type. Attach it to a canonical claim as an ordinary `EvidenceArtifact` (media type `application/vnd.luminous.black-hole-evidence-manifest+json`). The event chain authenticates which manifest bytes were attached; the manifest carries source/data/software/protocol/uncertainty relationships.

Large data files remain in authoritative archives (GWOSC, Zenodo, SXS, EHT); Mycelix records locators, file-level checksums, license/availability and derived-artifact lineage. Do not copy multi-gigabyte data into Holochain DHT entries by default.

## Case: the GW250114 direct-wave dispute

Create separate research objects for:
1. Nature GW250114 direct-wave paper (DOI 10.1038/s41586-026-10696-0);
2. its Zenodo data package (10.5281/zenodo.20017347);
3. the rational-filter software revision;
4. Kankani–McWilliams counteranalysis (arXiv:2607.02380);
5. Kankani–McWilliams BOB paper (PRD 114, 084025; DOI 10.1103/kjjd-nk8f);
6. Dyer–Chung–Moore direct-wave paper (PRL accepted 2026-10-27; DOI 10.1103/xftt-2n6t);
7. SXS numerical-relativity data and exact selected waveform resolutions;
8. LVK GWTC-5.0 Tests of GR data release (Zenodo 21454847);
9. Suzuki et al. orthonormal-QNM paper (PRD 114, 044099; DOI 10.1103/6x5z-q9ml) and Zenodo archive (10.5281/zenodo.19601938).

### Atomic claims must not be merged

| Claim family | Example proposition | Evidence treatment |
|---|---|---|
| Component extraction | A specified filtering pipeline yields a non-QNM residual in GW250114 | Reproduce/replicate the extraction |
| Observable estimation | Its instantaneous frequency/damping has a stated posterior under a declared window | Reproduce numeric posterior with all window/conditioning assumptions |
| Horizon interpretation | Frequency/damping measure Kerr `Omega_H` / `kappa` | Separate disputed claim; attach counter-evidence |
| News/BOB interpretation | Frequency tracks News frequency at peak News-amplitude time / plunge dynamics | Independent competing model and predictions |
| Area-law claim | Direct-wave-derived quantities test Hawking's area law | Do not upgrade unless mapping to horizon properties is independently validated across parameter space |
| QNM overtone claim | First overtone has a stated support under a particular mode basis | Preserve orthonormal vs non-orthogonal basis as separate workflows |

A repeatable residual does not prove its physical interpretation. The Nature report, SXS spin-sweep counteranalysis, BOB/News-frequency result and accepted PRL modeling result remain separate research objects and claims.

## Suggested event mapping

- `ClaimProposed`: propose one proposition with exact scope and source.
- `EvidenceAttached`: attach the schema-validated manifest and individual source/data/code/result artifacts.
- `AttestationRecorded(ComputationalReproduction)`: record an exact pipeline reproduction with a non-empty versioned `protocol_reference` and evidence IDs.
- `AttestationRecorded(IndependentReplication)`: record an independent implementation/team outcome.
- `AttestationRecorded(Critique)`: attach contradictory analysis without replacing the original claim.
- `AttestationCorrected` / `AttestationWithdrawn`: correct or withdraw a mistaken attestation while retaining event history.
- `ClaimSuperseded` / `ClaimRetracted`: explicitly supersede or retract a claim if warranted.

A failed or inconclusive reproduction is still an event and evidence artifact. Do not encode "no reproduction yet" as a positive assessment or let event counts/reputation determine scientific status.

## Integrity is not truth

- A content hash identifies bytes; it does not prove the contents are correct.
- A valid signature authenticates who attested; it does not prove the attestation is accurate.
- A reproducible computation can consistently implement an incorrect physical model.
- Agreement between implementations is weaker when they share code, assumptions, data conditioning, or training corpora.
- Governance and reputation may authorize actors or route review; neither substitutes for scientific evidence.

## Acceptance

1. A JSON Schema validator accepts the manifest and rejects missing source/version, data hashes, conventions, uncertainty dimensions or claim ceiling.
2. Every reproduction attestation references a versioned protocol and evidence.
3. Every claim links to sources and result artifacts; contested interpretations retain contradictory evidence.
4. File-level hashes and software commits survive upstream version changes.
5. Supersession, correction and withdrawal preserve history.
6. No new trust score, token-weighted scientific consensus or raw-data storage path is introduced.
