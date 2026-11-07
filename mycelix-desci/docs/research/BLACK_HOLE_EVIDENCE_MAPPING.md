# Black-hole research evidence adapter for Mycelix-DeSci

**Status:** integration design mapped to the existing Mycelix-DeSci scientific-event kernel.  
**Date:** 2026-11-07.  
**Scope:** evidence/provenance contract for Symtropy, Symthaea, Sol Atlas, and external black-hole research workflows.

## Decision: reuse the existing scientific-event kernel

Do not create a parallel BlackHoleEvidencePackage ledger, new consensus model, or second maturity/confidence score. The current Mycelix-DeSci implementation already provides the required durable primitives:

- `ResearchObject` and `AtomicClaim` separate a research object from individual propositions.
- `EvidenceArtifact` carries a content hash, media type, locator, license, and availability.
- `Attestation` links an assessment to exact evidence IDs and an optional protocol reference.
- `AttestationKind` distinguishes review, computational reproduction, independent replication, critique, conflict disclosure, and correction.
- `EvidenceOutcome` distinguishes `Supports`, `DoesNotSupport`, and `Inconclusive`.
- `ScientificEventEnvelope` is versioned, canonical, signed, sequenced, hash-chained, and idempotency-aware.
- `ClaimProjection` rebuilds the immutable stream and derives `EvidenceMaturity` / contestation from qualifying artifacts and attestations.

The authoritative implementation is in:
- `mycelix-desci/src/core/src/scientific_events.rs`
- `mycelix-desci/src/core/src/lib.rs`
- legacy claim types in `mycelix-desci/src/types/src/claims.rs`

The append-only event kernel is the canonical path for new research claims. The older mutable `DesciClaim` model remains for compatibility; do not build the black-hole adapter on that mutable path.

## Claim decomposition for the GW250114 / direct-wave programme

Do not create one monolithic claim called “the direct wave measures the horizon.” Split the scientific propositions so one can be reproduced while another remains disputed.

| Claim family | Example atomic proposition | Evidence needed |
|---|---|---|
| Data identity | “This is the exact GWOSC/GWTC-5.0 strain release and detector segment used.” | Source release, data-quality segment, calibration version, content digest |
| Extraction | “Under filter configuration X, the declared post-peak residual is extracted from this signal.” | Raw strain, conditioning, QNM basis/removal set, code commit, residual artifact |
| Residual characterization | “The residual's instantaneous frequency/damping follows the declared estimate over interval T.” | Posterior samples, start-time scan, model and error budget |
| Horizon association | “The residual frequency/damping tracks (2\Omega_H) / (kappa) over the declared parameter domain.” | Joint remnant posterior, competing predictors, broad held-out SXS/CCE sweep |
| Source-region attribution | “A measurable fraction of this residual is sourced inside the prograde light ring.” | Source-region model, inside/outside decomposition, NR / Teukolsky comparison |
| Area-law implication | “The inferred horizon parameters provide a valid independent test of the area law.” | Validated source association, independent remnant area, calibrated uncertainties and alternatives |

These are independent claims. Reproducing a filtered residual does not automatically reproduce its physical interpretation.

The current scientific disagreement must be represented as linked evidence rather than a binary governance vote:
- Nature 2026-06-24 direct-wave / horizon interpretation, DOI `10.1038/s41586-026-10696-0`.
- Kankani & McWilliams, arXiv `2607.02380`, SXS spin-sweep counteranalysis.
- Kankani & McWilliams, PRD `10.1103/kjjd-nk8f`, BOB / peak-News frequency analysis.
- Dyer, Chung & Moore, PRL accepted 2026-10-27, DOI `10.1103/xftt-2n6t`.
- Cheung, PRD accepted 2026-10-06, DOI `10.1103/l1mc-6hsz`, light-ring source-region estimate.
- Shi et al., arXiv `2610.30031`, QNM-filter approximation and vacuum-perturbation controls.
- Suzuki et al., PRD `10.1103/6x5z-q9ml`, orthonormalized-QNM analysis.

## Canonical object and artifact layout

### ResearchObject

Create distinct research objects for:
- each public dataset/release (GWOSC/GWTC event strain, SXS waveform catalog release, CCE Bondi-News release, EHT product);
- each software/model (qnm_filter, BOB model, QNM basis, Symtropy solver revision, waveform surrogate);
- each protocol/analysis workflow (GW250114 residual extraction, SXS spin sweep, geodesic benchmark, surrogate certification);
- each manuscript/preprint/revision.

Use the existing `ResearchObjectType` variants (`Dataset`, `Software`, `Workflow`, `Protocol`, `Model`, `Manuscript`). Preserve DOI, version, source URL, and license in the persistent identifier and attached manifest artifact where the object type lacks dedicated metadata.

### EvidenceArtifact

Attach exact bytes or a durable content-addressed reference for:
- original public input data or a source-archive locator and file-level hash manifest;
- calibration / data-quality metadata;
- software source archive or exact repository commit;
- dependency lockfile and execution environment manifest;
- model/parameter/prior/configuration manifest;
- posterior samples and residual/waveform output;
- independent verifier output and machine-readable benchmark report;
- null-control and failed/inconclusive run artifacts.

Use the existing fields as intended:
- `content_hash`: BLAKE3 digest of the exact artifact bytes;
- `media_type`: e.g. `application/json`, `application/x-hdf5`, `application/vnd.ligo.lw`, or a registered project media type;
- `locator`: stable archive URI, content-addressed object URI, or versioned repository URL;
- `license`: source-declared license or explicit unknown/restricted value;
- `availability`: `Public`, `Controlled`, `Embargoed`, or `Unavailable`.

Large strain arrays, SXS HDF5 files, and EHT products remain in their source archives or suitable object storage. The event log stores artifact identity and locator, not bulk scientific arrays.

A separate JSON manifest artifact should carry rich domain metadata not represented by `EvidenceArtifact`: units, coordinate chart, frame, polarization convention, source release, detector, time interval, calibration, priors, model family, numerical method, error budgets, and digest list.

### Attestation

Use existing attestation variants:
- `Review`: a domain scientist reviewed a specified claim and evidence set.
- `ComputationalReproduction { outcome }`: same declared protocol/code/data lineage was executed and yielded `Supports`, `DoesNotSupport`, or `Inconclusive`.
- `IndependentReplication { outcome }`: an independent implementation/team executed the preregistered protocol.
- `Critique`: identifies a substantive limitation, alternative explanation, or methodological concern.
- `ConflictDisclosure`: discloses relevant conflicts of interest.
- `Correction`: use the versioned correction event path; never silently replace a previous result.

For reproduction and replication, `protocol_reference` is mandatory in the current API. Point it to a preregistration or immutable protocol artifact hash / URI. Each attestation references exact evidence artifact IDs. A replication outcome is not a free-form confidence scalar.

## Event sequence

For one narrow proposition, the event stream should look like:

1. `ClaimProposed`: create a ResearchObject and AtomicClaim with explicit scope and claim text.
2. `EvidenceAttached`: attach the source paper, source data manifest, code revision, and preregistered protocol as separate artifacts.
3. `AttestationRecorded`: record review / computational reproduction / independent replication / critique with exact evidence IDs and protocol reference.
4. Additional `EvidenceAttached` and `AttestationRecorded` events: preserve alternative models, failed controls, and subsequent revisions.
5. `ClaimSuperseded` or `ClaimRetracted` only when scientifically warranted, with an explicit reason and a linked successor claim where applicable.

Every signed event retains actor, acting organization, event time, sequence, previous hash, canonical codec and schema version. The event hash proves event integrity and ordering, not the scientific truth of the statement.

## Evidence maturity and contestation

Use the existing `EvidenceMaturity` projection:
- `Proposed`
- `ArtifactBacked`
- `Reviewed`
- `ComputationallyReproduced`
- `IndependentlyReplicated`
- `Contested`
- `Superseded`
- `Retracted`

The projection intentionally keeps `contested` separate from maturity. A claim can have a computational reproduction and still carry active critiques or non-supporting results. This is exactly the desired behavior for the direct-wave/horizon association.

Do not use:
- token-weighted scientific truth;
- MATL reputation as a substitute for evidence;
- aggregate support counts as a likelihood;
- cryptographic hashes as proof of model validity;
- consensus status as a reason to delete contradictory evidence.

MATL or governance can help route reviewers and fund replication. It must not decide whether an astrophysical prediction is correct.

## Cross-system contract

| Producer / consumer | Required artifact or event |
|---|---|
| Symtropy | Model/solver version, metric/curvature/geodesic fixture manifest, output digest, numerical error diagnostics |
| Symthaea | Claim decomposition, hypothesis/model-comparison manifest, priors, predictions, posterior samples, null-control results |
| Sol Atlas | Source observation manifest, visualization projection version, layer type (measured/reconstructed/inferred/simulated/predicted), evidence IDs |
| Mycelix-DeSci | Canonical ResearchObject, AtomicClaim, EvidenceArtifact and Attestation event streams |

The five current Symtropy benchmark manifests should be attached as protocol/configuration artifacts:
- analytic reference cases;
- direct-wave SXS spin sweep and source-region comparisons;
- geodesic reference cases;
- waveform surrogate baselines;
- waveform surrogate certification v1.

Their current status is specification-only; no execution receipts should be created until runs actually occur.

## Acceptance tests for the adapter

1. A direct-wave extraction claim can be artifact-backed without being marked independently replicated.
2. A later critique or `DoesNotSupport` reproduction remains visible and marks active contestation without deleting the first extraction result.
3. A separate horizon-association claim can be contested while the residual-extraction claim remains independently reproduced.
4. An independent replication requires a protocol reference and evidence IDs.
5. Duplicate evidence IDs, invalid protocol references, broken event hashes, stale sequence numbers, and replayed idempotency keys fail closed under the existing kernel.
6. Source release, dataset version, code SHA, configuration, outputs and verifier receipt remain traversable from the claim projection.
7. Superseded and retracted claims retain their full lineage.
8. No single “truth score” is derived from a mixture of reputation, artifact count, posterior probability and community consensus.

## Non-goals

This adapter does not create a new DHT schema, claim-confidence metric, evidence ledger, peer-review marketplace, or scientific adjudication algorithm. It composes the existing DeSci event protocol with the black-hole programme's exact source, model, computation and replication artifacts.
