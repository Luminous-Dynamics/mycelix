# Black-Hole Research Evidence Integration

**Status:** integration contract / documentation; no new event store or parallel epistemic ledger.  
**Date:** 2026-11-07.

## Canonical owner

Black-hole research should reuse the current Mycelix-DeSci append-only scientific event kernel in:

- `mycelix-desci/src/core/src/scientific_events.rs`
- `mycelix-desci/src/core/src/scientific_authority_audit.rs`
- `mycelix-desci/src/core/src/scientific_governance.rs`

The canonical model already separates:
- `ResearchObject` and `AtomicClaim`;
- content-addressed `EvidenceArtifact`;
- typed `Attestation` and `EvidenceOutcome`;
- signed `ScientificEventEnvelope` / `SignedScientificEvent`;
- append-only `ScientificEventLog`;
- deterministic rebuildable `ClaimProjection`;
- versioned `EvidenceAssessment` and `EvidenceProfile`;
- receipt-time `ScientificAuthorityReceipt`.

The older mutable `DesciClaim` and E0–E4 APIs are explicitly compatibility-only. Do not add a second BlackHoleEvidencePackage event ledger, duplicate generic claim model, or a black-hole-specific confidence / truth score.

## Mapping the black-hole workflow to canonical objects

### Research objects

Use existing `ResearchObjectType` values:
- `Dataset`: GWOSC strain, SXS waveform, EHT calibrated visibility/image/polarization product, numerical-relativity slice or horizon data.
- `Software`: pinned source tool, reference solver, waveform model, horizon finder, inference engine.
- `Workflow`: complete analysis / benchmark / injection-recovery execution.
- `Model`: GR / alternative gravity model, waveform approximant, surrogate, discrepancy model, ringdown basis.
- `Manuscript`: paper, preprint, supplement or technical report.

The persistent identifier should use a DOI, arXiv identifier, SXS simulation identifier, GWOSC event/product identifier, immutable Git commit, or another stable external identifier.

### Atomic claims

Keep each scientific proposition narrow. Examples:
1. “This pipeline extracts the declared non-QNM residual from GW250114 under the pinned preprocessing and filter configuration.”
2. “The residual frequency is consistent with a specified light-ring predictor over the held-out SXS domain.”
3. “The residual frequency tracks Kerr horizon angular velocity across a declared spin/mass-ratio domain.”
4. “The inferred remnant mass/spin posterior remains calibrated after waveform discrepancy marginalization.”
5. “This numerical-relativity apparent-horizon surface satisfies the declared MOTS expansion conditions on this Cauchy slice.”

Do not combine “a residual can be extracted”, “it is a horizon signal”, and “it tests the area theorem” into one atomic claim. Those statements have different evidentiary burdens.

The claim scope should include the physical domain, waveform modes, frequency/time window, source population, detector/noise assumptions, model family and the exact meaning of the proposition.

### Evidence artifacts

Attach existing immutable manifests and data references through `EvidenceArtifact`:
- Symtropy benchmark manifests (direct-wave spin sweep, light-ring references, waveform surrogate certification, waveform discrepancy calibration, horizon object contract).
- GWOSC/SXS/EHT source metadata manifests.
- exact code revision / build / command records;
- inference configurations and posterior samples;
- plots and residual diagnostics;
- numerical-relativity resolution/extraction/convergence reports;
- independent verification output.

Use `content_hash` over the exact canonical artifact bytes, `media_type` for the manifest/result type, `locator` for a stable URI or content-addressed location, `license` for reuse terms, and `availability` for public/controlled/embargoed/unavailable status.

Large waveforms and raw simulation arrays normally remain in authoritative archives or suitable content-addressed storage. The event log should attach hashes and stable locators, not copy large scientific datasets into event records.

### Attestations

Use existing `AttestationKind` and `EvidenceOutcome`:
- `ComputationalReproduction { outcome: Supports | DoesNotSupport | Inconclusive }` for rerunning a pinned analysis or simulation.
- `IndependentReplication { outcome: ... }` for a separately implemented or independently performed reproduction.
- `Review` for scientific or methodological review.
- `Critique` for a documented alternative interpretation / counteranalysis.
- `ConflictDisclosure` for relevant conflicts.
- `Correction` or the correction event for amended evidence.

Computational reproduction and independent replication require evidence IDs and a versioned protocol reference under the current validator. Use the exact protocol/benchmark manifest version, not a mutable branch name alone.

A published counteranalysis is not an error to overwrite. It is a distinct claim with its own evidence artifacts and critique / non-supporting attestations. Mycelix must preserve disagreement and subsequent corrections as event history.

### Authority and receipts

Use the canonical signed event and receipt path. Preserve:
- authenticated actor and acting organization;
- event signer public key;
- exact credential-registry revision used to resolve the actor;
- authorization action and policy version;
- authority snapshot hash;
- receipt time, event hash, stream ID and sequence.

Do not infer that an artifact is scientifically correct because its hash is valid, its submitter is reputable, or an organization voted for it. Signatures and authority receipts establish attribution and authorization, not physical truth.

## Required scientific provenance inside attached manifests

The Symtropy/Symthaea manifests should include, where applicable:
- exact source paper/version and DOI/arXiv plus the passage/table/figure used;
- data release, event/simulation ID and file-level digest;
- source repository, immutable commit SHA and license;
- model family/version, parameter domain, modes, units and coordinate/sign conventions;
- preprocessing, PSD, calibration, alignment, frequency band, time window and priors;
- training/validation/held-out complete NR simulation IDs;
- numerical resolution, extrapolation/CCE and convergence information;
- discrepancy model revision, covariance/basis assumptions and calibration IDs;
- inference implementation, random seeds, posterior samples and output hashes;
- coverage confidence intervals, proper scores, bias, sharpness, OOD/abstention and negative-control outcomes;
- verifier code/version, exact command and independent reproduction result.

The manifest is the detailed scientific record; the canonical event links it by hash and provenance.

## Black-hole-specific claim boundaries

- A hash proves artifact identity/integrity, not scientific validity.
- A published paper is not the same as an independent reproduction.
- A simulated waveform is not observed strain.
- A reconstructed EHT image is not a direct photograph of an event horizon.
- A MOTS/apparent horizon on a named slice is not a reconstructed global event horizon.
- A filtered direct-wave residual does not by itself establish a horizon-frequency or area-law interpretation.
- A waveform mismatch score does not establish posterior calibration.
- A calibrated posterior does not prove a physical theory.

## First integration slice

1. Create one ResearchObject for the GW250114 source dataset and one for the direct-wave analysis workflow.
2. Create separate AtomicClaims for residual extraction and for the contested horizon-property interpretation.
3. Attach the pinned Nature analysis manifest, Zenodo source manifest, qnm_filter revision/configuration, and the Kankani–McWilliams / Dyer et al. counteranalysis manifests as EvidenceArtifacts.
4. Record a computational reproduction attestation with explicit outcome and protocol reference after the pipeline is actually rerun.
5. Record independent replication / critique attestations separately.
6. Rebuild the ClaimProjection from the signed event stream and confirm the EvidenceProfile / EvidenceAssessment represents support and contestation without deleting either.
7. Add the same mapping for the waveform-discrepancy injection campaign once it has actual results; the current Symtropy manifest is protocol-only.

## Related work

- Symtropy: [PR #4053](https://github.com/Luminous-Dynamics/symtropy/pull/4053) and [BH-HORIZON-SEMANTICS-001 #4071](https://github.com/Luminous-Dynamics/symtropy/issues/4071).
- Symthaea: [BH-WAVEFORM-DISCREPANCY-001 #10944](https://github.com/Luminous-Dynamics/symthaea/issues/10944) and [GW250114 replication #10841](https://github.com/Luminous-Dynamics/symthaea/issues/10841).
- Mycelix issue: [#5903](https://github.com/Luminous-Dynamics/mycelix/issues/5903).
- Sol Atlas: [#586](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/586).
