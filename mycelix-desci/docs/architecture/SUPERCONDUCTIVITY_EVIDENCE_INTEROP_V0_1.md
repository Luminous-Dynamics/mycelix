# Superconductivity scientific-event interoperability profile v0.1

**Parent issue:** [#6426 SC-DESCI-INTEROP-001](https://github.com/Luminous-Dynamics/mycelix/issues/6426)  
**Status:** mapping proposal; no adapter code or scientific data ingestion is claimed.  
**Canonical owner:** Mycelix-DeSci append-only scientific-event kernel.  
**Domain owner:** Symthaea superconductivity evidence schemas and audit rules.

## 1. Design decision

Do not add a second mutable superconductivity claim ledger to Mycelix-DeSci.

The current canonical kernel already separates:
- research objects from atomic claims;
- evidence artifacts from attestations;
- immutable signed events from deterministic query projections;
- authenticated actor / organization identity from scientific content;
- evidence-profile dimensions from versioned explanatory assessment.

The superconductivity integration should preserve the existing event model and attach a versioned, content-addressed domain observation artifact. The core event continues to record who proposed the atomic claim, which evidence artifact was attached, who attested, and the append-only event chronology. Symthaea owns the scientific meaning and domain-specific validator.

## 2. Current canonical event surface

Inspected canonical source:
- `mycelix-desci/src/core/src/scientific_events.rs`
- current constants in source: protocol `mycelix-desci`, codec `mycelix-canonical-binary-v1`, schema version 3, minimum replay schema version 2;
- event payload variants include `ClaimProposed`, `EvidenceAttached`, `AttestationRecorded`, `AttestationCorrected`, `AttestationWithdrawn`, `ClaimSuperseded`, and `ClaimRetracted`.

The exact source revision and schema constants must be read at integration time. Do not assume the values above remain the live version after this document is authored.

### Mapping

| Superconductivity concept | Canonical Mycelix representation | Domain payload / restriction |
|---|---|---|
| Material, specimen, batch, process run, reel, conductor, device | ResearchObject, with stable domain identifier in persistent identifier / registry | Keep material identity and sample state in the domain artifact; do not treat title alone as identity |
| One atomic scientific assertion | AtomicClaim | One proposition per claim; scope binds specimen / state / conditions by an immutable domain-record ID |
| Raw data, paper, image, spectrum, calculation, process trace | EvidenceArtifact | ContentHash plus media type, locator, license and availability |
| Measurement / prediction / calculation details | Versioned superconductivity observation artifact | Includes property, authority class, evidence state, units, uncertainty, conditions, sample and source lineage |
| Review, computational reproduction, independent replication, critique, conflict disclosure | Attestation | Bind exact evidence artifact IDs and protocol reference where required |
| Correction / withdrawal | AttestationCorrected / AttestationWithdrawn | Append-only; never rewrite or erase earlier events |
| Claim supersession / retraction | ClaimSuperseded / ClaimRetracted | Preserve historical claim and reason |
| Actor, acting organization, event time, stream sequence, previous hash | ScientificEventEnvelope | Governed append path separately validates authority and receipt-time journal |
| Rebuilt view | ClaimProjection / EvidenceProfile / EvidenceAssessment | Deterministic and explainable, not a hidden physical-truth score |

## 3. Domain observation artifact

The v0.1 adapter should consume a versioned Symthaea observation-event record as opaque domain content, after validating it against the exact source schema version. A minimal mapping envelope should preserve:

- domain schema identifier and exact schema version;
- stable observation ID;
- stable material / specimen / batch / phase / process-run IDs;
- atomic property and value or interval;
- unit, uncertainty, measurement method and instrument/calibration identity;
- evidence authority: measured, calculated, predicted, inferred, source-reported, synthetic, disputed or unresolved;
- measurement state: measured, not detected, unmeasured, inferred, disputed, not applicable;
- exact temperature, pressure semantics, field magnitude/vector/orientation, strain, current criterion and geometry where applicable;
- source paper / dataset / software revision and source locator;
- raw artifact digest(s), rights / availability, and review state;
- parent / related observation IDs and same-specimen / same-batch join keys.

The domain schema should remain the primary machine-readable source. The prose claim is a human-readable proposition, not a replacement for the structured observation.

## 4. Superconductivity semantic constraints

These distinctions must survive import, projection, search and export:

- measured Tc onset, zero-resistance, diamagnetic / Meissner, and thermodynamic / bulk transition are separate properties / criteria;
- calculated, predicted, literature-reported and directly measured values are different authority classes;
- synthesis pressure, measurement pressure, operating pressure, chemical pressure and unknown pressure are distinct;
- non-detection is not a zero value; missing report is not evidence of a non-superconductor;
- same-specimen / same-batch joins are required for a combined experimental claim;
- bulk sample values cannot be promoted to tape / cable / device performance;
- layer Jc and conductor engineering Je are not interchangeable;
- microscopic xi0, GL xi, penetration depth estimators and superfluid stiffness remain distinct;
- measured critical fields and clean-limit calculated Hc/Hc1/Hc2 remain distinct;
- intended synthesis protocol, issued setpoints, actual sensor execution and realized phase outcome remain distinct;
- a signed assertion is authenticated under its declared authority; it is not thereby experimentally correct.

No single claim may compose the best Tc, strongest magnetic response and best structure result from different specimens into one "bulk confirmed" statement.

## 5. Hash and artifact interoperability

The present Mycelix `ContentHash` implementation uses BLAKE3. Symthaea and external scientific artifacts may carry SHA-256 or other digests.

Do not silently reinterpret one algorithm as another:
- retain each source digest with its explicit algorithm identifier and exact hashed-byte scope;
- compute Mycelix's ContentHash over the exact artifact bytes only when those bytes are lawfully available;
- if a restricted artifact cannot be read, store its declared external digest, source locator and rights / availability state as domain metadata, and do not claim Mycelix independently verified the bytes;
- never treat a digest as a source truth, calibration, signer authorization or data-use permission.

Large artifacts remain outside inline event JSON. The event attaches a content-addressed EvidenceArtifact locator and the domain manifest references source records / evidence bundles.

## 6. Evidence maturity is not superconductivity classification

The current `EvidenceProfile` and `EvidenceAssessment` report artifact, review, computational reproduction, independent replication, critique, contestation, correction and withdrawal dimensions.

They do not encode whether a superconducting state is physically established under a particular measurement protocol. Therefore:
- Mycelix's maturity label must not be rendered as "superconductor confirmed";
- scientific assessment must continue to use the Symthaea observation authority and evidence rules;
- independent replication attestation counts only under the current qualified actor / organization independence policy;
- a review is not a measurement and a computational reproduction is not an experimental replication;
- active non-supporting evidence and critiques remain visible alongside supportive evidence;
- a retracted or superseded claim remains queryable as historical record.

## 7. Hash-chain and authority flow

A minimal read-only integration flow is:

1. Validate the source observation against its exact Symthaea schema version.
2. Resolve the domain observation ID to the exact content-addressed artifact and rights state.
3. Create one research object for the relevant material / sample / dataset / protocol subject as appropriate.
4. Propose one atomic claim per scientific proposition, with scope referencing the stable domain observation ID.
5. Attach the observation manifest and underlying evidence artifacts using EvidenceAttached events.
6. Record reviews, computational reproductions, independent replications, critiques, corrections and withdrawals as distinct typed attestations/events.
7. Rebuild the projection from the signed append-only event stream.
8. Compare the rebuilt projection with the adapter's expected domain mapping; preserve conflicts, unknowns and unmapped fields.
9. Export a read-only view with source rights, authority, condition and same-specimen caveats intact.

Actor signatures, governed credentials, organization roles, receipt-time authority evidence, idempotency and append authorization remain the current Mycelix kernel's responsibility. The adapter must not bypass the governed append path or add a parallel scientific write authority.

## 8. Required negative controls

- a calculated Tc cannot answer a query requiring a measured Tc;
- a Tc onset claim cannot be rendered as a bulk thermodynamic transition;
- unknown pressure cannot be defaulted to ambient;
- same formula but different specimens cannot satisfy a same-specimen evidence join;
- different process batches / polymorphs cannot be merged by nominal formula alone;
- a not-detected observation is not converted to value zero;
- a missing observation is not converted to a non-superconductor label;
- a valid signature over unsupported content remains an authenticated assertion, not a validated result;
- a source digest without rights / availability does not authorize public redistribution;
- an attestation correction or withdrawal adds a new event and preserves the earlier revision;
- replaying an event is idempotent under the canonical stream rules; changing a payload under the same event / idempotency identity is rejected;
- reputation, review count or evidence maturity does not override contradictory scientific observations.

## 9. Implementation order

1. Pin the live Symthaea observation-event schema and Mycelix scientific-event source revision.
2. Produce a lossless mapping table and explicit refusal / unmapped-field list.
3. Build a read-only projection over synthetic fixtures before any canonical write adapter.
4. Add schema round-trip and semantic negative-control tests.
5. Add exact event / artifact / credential / receipt digests to the replay report.
6. Only then consider governed publication of domain attestations; do not create new actor authority inside the adapter.

## Claim ceiling

This document is an interoperability design mapping. It does not implement an adapter, ingest third-party data, verify a physical measurement, classify a new superconductor, or prove reproducibility. Mycelix establishes signed event integrity and governed actor authority within its declared protocol; Symthaea's scientific rules and the underlying experiments remain the authorities for superconductivity claims.
