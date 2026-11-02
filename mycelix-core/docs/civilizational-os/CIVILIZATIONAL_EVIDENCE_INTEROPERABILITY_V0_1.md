# Civilizational Evidence Interoperability Crosswalk v0.1

**Status:** architecture and implementation mapping proposal.  
**Purpose:** adapt existing Mycelix claim, identity, attribution, moderation and Chronicle concepts to the Civilizational Systems Science interchange contract without creating a second evidence ledger.  
**Maturity warning:** Mycelix's root README describes the ecosystem as pre-alpha and says that most clusters have unit tests but substantially fewer have real multi-agent tests or deployed interfaces. This crosswalk is not evidence that all referenced hApps are operational.

## 1. Design decision

Do not create a parallel `CivilizationalClaim` DHT or a new generic trust ledger.

Instead:
- retain the existing knowledge claim as the claim identity and discussion/moderation object;
- retain existing E/N/M epistemic classification as a **separate evaluative axis**;
- attach typed evidence/source/derivation references through existing knowledge interfaces and Chronicle provenance;
- use the existing identity/agent-key model for attribution of authors and verifiers;
- use Chronicle's designed archive/provenance/access-control envelope as the long-term memory integration target, after verifying its actual zome maturity;
- use Attribution's `DependencyIdentity`, `UsageReceipt` and `UsageAttestation` patterns only for what they represent (identity, declared usage and privacy-preserving usage evidence), not as proof that a scientific claim is true;
- represent rights, custodianship, dispute, correction, supersession and retraction as explicit records/status transitions. Do not erase the historical chain.

## 2. Existing substrate crosswalk

The following paths were inspected on the repository default branch during this audit. They are existing types or design artifacts, not all independently verified as deployed.

| Interchange concern | Existing Mycelix surface | Reuse and limits |
|---|---|---|
| Claim identity/content | `mycelix-workspace/crates/mycelix-claim-types/src/lib.rs::UnifiedClaim`; `mycelix-knowledge/client/src/index.ts::Claim` | Reuse claim id, content, author, type, version, status, topics/tags. Additional subject/predicate/object and temporal/spatial scope need a versioned extension or metadata schema. |
| Epistemic evaluation | `EpistemicClassification` (Empirical, Normative, Materiality) | Keep E/N/M as a multidimensional classification. Do **not** overload it to mean observed/modelled/scenario/speculative; add a separate epistemic evidence class. The composite score must not be interpreted as truth or civilizational worth. |
| Evidence and sources | `mycelix-knowledge/client/src/index.ts::Evidence` and `Source`; evidence relations `Supports`, `Contradicts`, `DerivedFrom`, `Generalizes`, `Specializes`, `PartOf`, `Causes` | These provide a useful evidence/source/link surface. Current Source fields (URI, title, author, publishedAt, reliability) do not fully bind exact snapshot digest, licence, consent, method, valid time or transformations. Extend additively, preserve old readers. |
| Shared claim types | `mycelix-workspace/sdk/src/epistemic/claims.rs::EpistemicClaim` | Has evidence strings, issuer, timestamp and arbitrary JSON metadata. It is a compatibility surface, not a fully typed, content-addressed evidence contract. Current `should_retain` logic should be tested for clock rollback and unsigned-time edge cases before use as archive policy. |
| Long-term memory/provenance | `mycelix-core/docs/civilizational-os/happs/MYCELIX_CHRONICLE_DESIGN.md` | Design defines `ArchiveEntry`, `Provenance`, `SourceInfo`, `Transformation`, `AccessPolicy`, relations, citations and retention. This is the strongest conceptual match, but the file is a design document; confirm integrity/coordinator zomes, DHT validation and multi-agent maturity before treating it as operational. |
| Dependency identity | `mycelix-attribution/zomes/registry/integrity/src/lib.rs::DependencyIdentity` | Reuse source/software/technology identity patterns where appropriate; do not coerce an archaeological source or historical event into a dependency. |
| Usage lineage | `mycelix-attribution/zomes/usage/integrity/src/lib.rs::UsageReceipt` | Shows immutable, authored usage receipts and a receipt-to-identity relationship. It is evidence that a user declared use, not that the underlying source/claim is correct. |
| Privacy-preserving attestation | `UsageAttestation`, STARK prover/verifier, Ed25519 verifier metadata | Potentially useful for private proof of custodial access or resource-use events if the exact statement, input commitment, verifier identity, expiry and revocation semantics are bound. It currently concerns usage scale, not generic truth or scientific validation. |
| Disputes/retractions | `mycelix-knowledge/docs/governance/CLAIM_MODERATION.md`; `ClaimStatus` in knowledge client | Reuse challenge, contradictory evidence, review and retracted status. Preserve evidence and source lineage; do not delete history merely because a claim is superseded or disputed. |
| Rights/access | Chronicle `AccessPolicy` design; Mycelix identity and membership/capability systems | Need actual DHT-level authorization and key-management tests. A client-side hidden field or public URL is not access control. |

## 3. Critical semantic separation: E/N/M is not evidence class

The existing E/N/M cube asks about:
- empirical evidential support;
- normative/value alignment;
- materiality/persistence.

The Civilizational Systems Science contract asks what kind of assertion is being represented:
- observed measurement;
- archaeological evidence;
- primary historical source;
- oral/community history;
- compiled secondary source;
- model inference;
- scenario assumption;
- speculative hypothesis;
- confirmed detection.

These are orthogonal. An oral history can be high-materiality and well-supported in its custodial context; a model inference can have high empirical calibration but remains an inference; a speculative hypothesis may be important while not being observed. Do not collapse these dimensions into a single scalar or assume the normative/materiality axes measure scientific truth.

Proposed additive field group (exact serialization to be reviewed against existing zome compatibility):

```text
EvidenceProfile {
  evidence_class,
  value_state,
  valid_time,
  observed_time,
  publication_time,
  spatial_scope,
  quantity_and_unit?,
  uncertainty?,
  source_refs[],
  derivation_refs[],
  method_ref?,
  rights_and_access[],
  limitations[],
  independent_verifier_refs[]
}
```

The current Mycelix claim remains the primary user-facing claim object. EvidenceProfile is typed metadata associated with that claim or a Chronicle archive entry, not a competing claim identity.

## 4. Minimum adapter behavior

A first adapter should be read/write conservative:

1. Read current claim identity, author and E/N/M fields unchanged.
2. Translate existing `Evidence` and `Source` references into the interchange envelope without inventing missing metadata. Unknown snapshot digest, licence or method stays unknown.
3. Store exact source digests, source-version snapshots and transformations as append-only provenance records where the underlying app supports it.
4. Represent `observed_measurement`, `model_inference`, `scenario_assumption` and `speculative_hypothesis` as separate typed evidence classes.
5. Preserve the time semantics separately: `valid_time`, `observed_time`, `published_time` and Holochain record timestamp are not interchangeable.
6. Attach spatial scope/CRS and quantitative unit to claims that describe locations or resource measures.
7. Link disputes, corrections, supersession and retractions rather than mutating source evidence.
8. Enforce community restrictions and custodial permissions before exposing any claim/source in public search or a Sol-Atlas layer.
9. Expose stable references for Symthaea, Symtropy and Sol-Atlas; consumers must not need to know Holochain internal entry encoding.
10. Reject unsupported conversion instead of silently coercing an unknown or missing field into zero, a default date, a public access level or a numeric trust score.

## 5. Integrity blocker found in the existing Attribution hApp

The usage integrity zome currently rejects updates to `UsageReceipt`, but its update handler for `UsageAttestation` returns `Valid` for every update. Its own source comment acknowledges that intended verifier-only mutation is enforced only in the coordinator and that a modified coordinator can bypass it.

This is a DHT integrity issue. It is tracked in [ATTRIBUTION-SEC-001, issue #5710](https://github.com/Luminous-Dynamics/mycelix/issues/5710).

Do not build civilizational evidence receipts on top of this mutation boundary until the validator binds the update to the original action author, checks immutable proof/identity fields, enforces one-way verification transitions and passes multi-agent Sweettest/Tryorama adversarial tests. A fixed-length public key and signature are not proof that the signature is cryptographically valid or bound to the attested content.

## 6. Acceptance gates

### Gate A — model compatibility
- Map claim/source/evidence/classification/status types without breaking existing serializations.
- Include explicit migration/version behavior and round-trip property tests.
- Demonstrate that E/N/M and evidence class can vary independently.

### Gate B — provenance integrity
- Exact source snapshot digest is immutable and transformations retain parent references.
- Corrections create new lineage records; no silent overwrite.
- Authorship, publisher, verifier, source custodian and current viewer are distinct roles.

### Gate C — rights and privacy
- A community-restricted record cannot be read through an alternate query, link traversal, archive or Sol-Atlas adapter.
- Withheld-by-custodian is not represented as missing data or public metadata.
- Revocation/correction propagates to descendants while retaining the audit trail.

### Gate D — DHT integrity
- Fix issue #5710 before depending on UsageAttestation updates.
- Verify direct DHT validation with at least two conductors and a malicious/modified coordinator path.
- Include restart/recovery and replay tests.

### Gate E — maturity evidence
- Report which components only compile, which have unit tests, which have multi-agent tests and which are deployed.
- Do not infer operational civilizational memory from the existence of Chronicle design documents or TypeScript client interfaces.
- No production claim until build, unit, multi-agent, rights/privacy and migration gates are independently reported.

## 7. Boundaries

- A signed claim proves who signed a given byte sequence, not whether the statement is true.
- E/N/M composite scores are not truth, worth or a ranking of peoples/cultures.
- Usage receipts are declarations of usage; ZK-STARK usage-scale proofs do not prove scientific accuracy.
- Chronicle retention must preserve important negative evidence, source lineage and custodial rights. Auto-pruning cannot be driven only by low access frequency.
- Mycelix is the provenance/governance substrate; Symthaea models learning; Symtropy runs validated dynamics; Sol-Atlas renders geography. None should duplicate the others' authority.
