# Regenerative Evidence Envelope Contract v0.1

## Purpose

Define the Mycelix-side envelope for evidence produced by Symthaea and other regenerative-asset subsystems.

The envelope is intentionally a **reference and provenance contract**, not a second physics or safety engine.

It answers:

> What exact evidence was produced, for which asset/component, under which configuration and lifecycle state, by which producer, and what authoritative result or resolution does it reference?

## Evidence envelope

A canonical evidence envelope should bind:

- schema_version
- producer_id
- producer_commit
- asset_id
- component_id
- evidence_kind
- evidence_id
- evidence_digest
- configuration_digest
- topology_epoch
- topology_digest
- qualification_digest
- observed_at_ms
- received_at_ms
- authority_reference
- verification_reference
- authority_statement_id
- authority_resolution_epoch
- authority_predecessor_statement_digest
- authority_effective_from_ms
- supersedes_reference
- revokes_reference
- transparency_receipt_reference
- verifier_id
- verifier_version
- cryptosuite
- verification_report_digest
- policy_fingerprint
- environment_fingerprint
- resolution_snapshot_fingerprint
- verification_outcome

The fields have distinct meanings and must not be collapsed into one generic score.

### Producer identity

producer_id identifies the software or device that generated the evidence.

producer_commit identifies the exact software revision claimed to have produced it.

A commit identity is provenance, not proof of execution. Runtime attestation remains a separate security concern.

### Evidence identity

evidence_id is the stable reference to the evidence record.

evidence_digest identifies the exact evidence content. Consumers must not treat two evidence records as equivalent merely because their IDs or timestamps are similar.

### Physical lifecycle identity

asset_id and component_id provide stable identity.

configuration_digest binds the evidence to the configuration under which it was generated.

topology_epoch and topology_digest bind evidence to a specific admitted lifecycle topology state.

An old evidence record therefore cannot become current merely because its observation timestamp remains within a generic freshness window.

### Qualification identity

qualification_digest identifies the deterministic local qualification context that admitted the evidence, such as sensor qualification, temporal corroboration, physics consistency, or recovery verification.

The envelope references that qualification result; it does not reinterpret the result.

### Authority references

authority_reference identifies an external authoritative statement, such as a topology attestation or lifecycle resolution.

verification_reference identifies the corresponding verification result.

Mycelix can enforce that references are present and internally coherent while preserving the boundary that authoritative cryptographic verification is performed by the configured trust layer.



## Concrete Symthaea verification binding

The first Symthaea observation-attestation implementation already exposes structured verifier evidence beyond a bare verification reference.

Where available, the Mycelix envelope should preserve:

- verifier_id: the verifier identity used for the decision;
- verifier_version: the stable report/verifier implementation version;
- cryptosuite: the proof suite used;
- verification_report_digest: the fingerprint of the complete verification report;
- policy_fingerprint: the exact policy inputs used by the verifier;
- environment_fingerprint: the verifier environment identity;
- resolution_snapshot_fingerprint: the durable key/verification-method snapshot consulted by the verifier;
- verification_outcome: the explicit terminal outcome.

These fields are evidence references, not a second verifier implementation.

This distinction is important because a bare statement such as "verified" loses the inputs that made the verification reproducible and auditable. The verifier report should therefore remain the source of detailed stage semantics, while the Mycelix envelope records stable references to that report.

The evidence chain becomes:

**Symthaea evidence → verification report → report fingerprint → Mycelix evidence envelope → durable provenance**

A future transport can selectively disclose the detailed report while retaining the digest and provenance references needed for audit.

## Lifecycle authority binding

The envelope must preserve the relationship between an evidence record and the authoritative lifecycle statement it relies upon.

Where an authority resolution participates in a monotonic resolution history, the envelope should retain:

- `authority_statement_id`: stable identity of the exact authority statement;
- `authority_resolution_epoch`: the authority decision sequence number;
- `authority_predecessor_statement_digest`: the exact prior authority statement digest when this statement supersedes a previous resolution;
- `authority_effective_from_ms`: the lifecycle time at which the resolved topology becomes effective.

A resolution may therefore be valid before it becomes current. Consumers must not infer currentness solely from statement validity or resolution timestamp.

Lifecycle relationships are explicit:

- `supersedes_reference`: points to the exact prior resolution being replaced by a later authoritative decision;
- `revokes_reference`: points to the exact resolution being invalidated by an authority revocation.

A revocation is not a topology replacement. A superseding resolution is not a revocation. Consumers must preserve both records and evaluate the relationship rather than collapsing them into a mutable current-state field.

`transparency_receipt_reference` is optional for offline operation and identifies a receipt proving registration of the relevant signed statement in a configured transparency service. Its presence does not itself establish that the issuer's claim is physically true.

This keeps the chain explicit:

**evidence → verifier result → authority statement → lifecycle relation → transparency evidence**

## Evidence kinds

The initial vocabulary should remain small and composable:

- SensorObservation
- TemporalCorroboration
- PhysicsConsistency
- TopologyAttestation
- TopologyResolution
- TopologyRevocation
- Intervention
- RecoveryVerification

Platform-specific systems may add kinds without changing the identity, configuration, lifecycle, or provenance rules.

## Lifecycle resolution references

When evidence was produced under a topology fork resolution, the envelope should reference both:

- the selected topology successor; and
- the authority resolution that made that successor current.

The competing topology branches remain historical evidence. They must not be deleted or rewritten when a resolution is recorded.

## Offline-first semantics

A vehicle or other asset must be able to generate and consume envelopes locally without Mycelix connectivity.

Connectivity is required for durable synchronization and authoritative exchange, not for local quarantine or restriction.

Offline replicas may produce concurrent lifecycle statements. Synchronization must preserve those statements until an authoritative resolution is available.

## Replay and rollback rules

A consumer should reject or quarantine an envelope when:

- the asset/component identity does not match the expected context;
- the configuration digest is incompatible;
- the topology epoch is older than the admitted current epoch;
- a topology digest does not belong to the referenced epoch;
- a claimed authority resolution is from an older resolution epoch;
- a claimed authority resolution has not yet reached its declared effective time;
- a supersession or revocation reference does not match the exact targeted authority statement;
- the evidence timestamp is outside the permitted freshness window;
- required authority or verification references are missing;
- a resolution selects a successor not present in the preserved branch set.

These are evidence-integrity failures, not diagnoses of the physical asset.

## Relationship to Symthaea

Symthaea remains responsible for deterministic local qualification and physical-model evidence.

Mycelix remains responsible for durable identity, provenance, authorization, synchronization, and multi-party lifecycle evidence.

Neither layer by itself establishes regulated operational permission.

## Transparency compatibility

The envelope is designed to map naturally onto append-only transparency systems.

RFC 9943 SCITT defines an append-only statement history with non-equivocation and replayability requirements, with receipts proving registration of signed statements in a verifiable data structure. The explicit authority statement, lifecycle relation, and optional receipt references above allow Mycelix to preserve that evidence chain without turning a mutable “current state” record into the source of truth. citeturn612028search0

A future Mycelix transport can bind envelopes to signed statements and transparency receipts without changing the core distinction between evidence, verification, authority, and physical truth.


The envelope is designed to map naturally onto append-only transparency systems.

RFC 9943 SCITT defines an append-only statement history with non-equivocation and replayability requirements, with receipts proving registration of signed statements in a verifiable data structure. The envelope therefore uses stable references and digests rather than mutable “current state” records.

A later Mycelix transport can bind envelopes to signed statements and transparency receipts without changing the semantic contract.

## Core invariant

**Evidence may establish what was observed, qualified, referenced, or resolved; it must never silently establish what the physical asset is.**

Physical truth, safety, repair success, and recovery remain separate claims requiring their own evidence boundaries.
