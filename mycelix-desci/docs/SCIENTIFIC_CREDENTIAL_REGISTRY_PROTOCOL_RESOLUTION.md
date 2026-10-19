# Scientific Credential Registry: Protocol Resolution Contract

**Status:** Proposed recovery/design contract; not an active protocol specification  
**Scope:** Issue [#2465](https://github.com/Luminous-Dynamics/mycelix/issues/2465) only  
**Observed base:** Mycelix `main` at `445a84abdaa64f3d05e2d5c51115e4d060b78ec7` (2026-10-12)  
**Authority rule:** This document does not claim to recover historical implementation bytes, certify an existing registry file, or authorize deployment of a replacement.

## 1. Decision required before implementation

The module declared by `mycelix-desci/src/core/src/lib.rs` as `scientific_credentials` is absent from the tracked tree. The same is true at PR #4953's inspected head, and a path-history query returns no commits for `mycelix-desci/src/core/src/scientific_credentials.rs`. The archived standalone `mycelix-desci` repository also does not contain a provider.

No authentic implementation has been recovered. The existing `SCIENTIFIC_CREDENTIAL_REGISTRY.md` describes intended behavior; it does not specify enough canonical bytes to reconstruct the implementation safely.

**Decision gate:** do not implement against guessed v1 bytes. First freeze the new protocol identifier, protocol/schema versions, canonical codec, and migration behavior as a new lineage. The exact identifier literals and byte grammar must be committed in the implementation RFC and golden vectors before the provider is wired to live consumers. Until then, this proposal is not a wire-format authority.

## 2. Consumer inventory and minimum API surface

The missing provider is consumed by more than one re-export.

| Consumer | Provider contract required |
|---|---|
| `src/lib.rs` | Publicly re-export registry/event types and their protocol, schema, codec, and file-size constants |
| `scientific_credential_governance.rs` | `ScientificCredentialRegistry`, `ScientificCredentialRegistryProjection`, `PreparedScientificCredentialAppend`, recorded events and append receipts; resolve administrators at server receipt time; prepare and preview governed mutations; apply only after durable commit |
| `postgres_credential_authority.rs` | Verify signed events, event hashes, record hashes, chain sequence/predecessor, accepted time, indexed SQL columns and serialized record; atomically commit registry and governance rows/outbox |
| `postgres_authority_epoch.rs` | Rebuild a historical registry projection from durable records under a trusted acceptance-service key set |
| `bin/commands/credential_registry.rs` | Create genesis, load a durable file, inspect events/projection, resolve an administrator, append a signed successor |
| `bin/commands/credential_governance.rs` and API handlers | Open the same registry authority used by threshold governance and expose its exact revision |

At minimum, existing call sites require these public domain types (the following names are observed imports/re-exports, not a claim that their missing definition has been recovered):

- `ScientificCredentialEventId`, `ScientificCredentialEnvelope`, `ScientificCredentialPayload`, `SignedScientificCredentialEvent`
- `RecordedScientificCredentialEvent`, `ScientificCredentialAppendReceipt`, `ScientificCredentialRegistrySummary`
- `ScientificCredentialRegistryProjection`, `ScientificCredentialRegistry`
- `PreparedScientificCredentialAppend`

Required operations observed in consumers include `ScientificCredentialEnvelope::genesis`, `::successor`, `SignedScientificCredentialEvent::sign`, `::verify`, `::event_hash`, recorded-record `::record_hash`, and registry operations `open_file`, `memory`, `events`, `event`, `append`, `projection`, `projection_at`, `resolve`, `synchronize`, `lock_mutation`, `prepare_governance_approved_at`, `preview_with_prepared`, and `apply_prepared`.

The implementation RFC must freeze signatures and error semantics for this surface. It must not add a second authority path or bypass threshold governance to make compilation pass.

## 3. Protocol contract that must be explicit

### 3.1 Signed event envelope

The canonical envelope must bind, at minimum, the event ID, sequence, previous record/event hash according to the chosen chain model, actor ID, event time, idempotency key (optional only where the consuming contract permits it), and payload. Any field that changes authorization, replay, ordering or projection must be covered by the signature.

The exact wire representation must define:
- a domain separator unique to this registry protocol and operation;
- integer widths, signedness, byte order and overflow behavior;
- string/byte length framing and UTF-8 requirements;
- option/enum discriminants and canonical variant field order;
- normalized time encoding including sub-second precision;
- the exact digest algorithm and the differences between event hash and accepted-record hash;
- rejection of unsupported protocol/schema/codec versions and non-canonical or trailing bytes.

Serde/JSON serialization is a storage envelope, not canonical signing bytes. Never hash pretty-printed JSON or rely on map iteration order for signed values.

### 3.2 Append-only accepted record

An accepted record binds the verified signed event to a server-observed `received_at`, acceptance-service public key and signature, and a deterministic record hash. Server receipt time is assigned only after acquiring the mutation serialization boundary. It must not be supplied or backdated by the caller.

Append must verify all of the following before making the record visible:
1. expected next sequence and exact previous head;
2. canonical signed bytes, signer key encoding, signature, protocol and schema;
3. genesis-only constraints or a valid successor;
4. authorized actor, role and active signing key at the server's linearization time;
5. lifetime uniqueness of a signing public key across all actor IDs, including expired, revoked and compromised key records;
6. idempotency/replay behavior, including rejection if an already-used event ID is presented with different signed bytes;
7. that the accepted resulting projection retains at least one active registry administrator with a recovery-eligible key;
8. valid acceptance-service signature, and the exact accepted timestamp, event hash and registry revision it binds.

Rejecting a write must leave the event log, head, projection, pending execution and outbox unchanged.

### 3.3 Projection and historical resolution

The projection must be deterministically rebuildable from the verified record prefix plus an explicit trusted acceptance-service key set. It must expose its event count and exact head hash, resolve actors by registry state at a requested time, preserve organizations and role/key lifecycle history, and fail closed on chain gaps, duplicate IDs, broken links, invalid receipts, unknown payloads or trust-key omissions.

A historical projection at time `t` must follow the documented acceptance-time convention exactly. Key rotation, revocation, role changes and organization-membership changes must not reinterpret why a previously accepted event was authorized.

### 3.4 Transactional prepare/apply boundary

The consumer API shows a prepare/preview/apply protocol for governance-approved mutations. Its required meaning is:

- **prepare:** validate against the locked current head and construct a candidate recorded event without publishing it;
- **preview:** build the candidate projection for governance transition validation without mutating durable/current state;
- **durable commit:** commit the credential row, governance execution row and signed outbox publication in one serializable database transaction;
- **apply:** update the in-memory projection only after durable commit succeeded;
- **reconcile:** on interruption, inspect exact persisted records and finish or reject the pending transition idempotently; never create a second event or guess whether the first committed.

File-backed and PostgreSQL implementations must share the same protocol validation and projection rules. Backend choice must not weaken signature, sequence, replay or authority invariants.

## 4. Persistence and compatibility boundary

Observed PostgreSQL storage uses `desci_credential_events` with sequence, event ID/hash, record hash, previous-record hash, actor ID, idempotency key, received time and serialized record JSON. SQL indexes/columns are checked against recomputed values after deserialization. The governance path also commits an event outbox record within the transaction.

The implementation must inventory real deployment files/backups before any migration. Do not truncate, reinitialize, silently rewrite, or bless an existing `scientific-credentials.json` file merely because the source provider is missing from Git.

If authentic v1 canonical bytes cannot be recovered, a new protocol/schema lineage does **not** imply backward compatibility. The migration plan must choose one of:
- a verifiable import adapter that checks old records using the authentic old verifier and preserves their exact bytes/hashes; or
- read-only archival of unverifiable legacy data plus explicit operator-authorized migration into the new protocol, with source hashes, conversion decisions, and a new genesis/authority boundary.

The second path may not claim continuity of old signatures or record hashes. No registry migration may silently grant roles, rebind keys, erase revocation history, or create an authority key from an untrusted profile.

## 5. Required adversarial and compatibility tests

The first implementation PR must include tests for all of these, with fixed fixtures and deterministic outcomes:

1. Golden canonical-byte and signature vectors for every event/payload variant and every record/receipt hash; reject modified field order, lengths, domain separator, version, truncation and trailing bytes.
2. Genesis accepted only at sequence zero with no predecessor; successor requires the precise prior record hash and next sequence.
3. Mutated payload, signer key, acceptance time, acceptance-service signature, predecessor, sequence and persisted index column are detected.
4. Same event ID + identical signed bytes is idempotent; same ID + different signed bytes is a hard conflict. Duplicate idempotency keys cannot create a second write.
5. One public key cannot be associated with two actor IDs—even if the first record is expired, revoked, compromised or no longer active.
6. Expired/revoked keys and historical role/organization changes are evaluated at server receipt time, not event-declared time or current wall time during replay.
7. Registry cannot remove/revoke its final recovery-eligible administrator key, including through threshold-governed proposals and key rotations.
8. Concurrent appends at one expected sequence have one winner. A stale prepared append cannot commit after head movement.
9. Failure injection before commit, during transaction, after commit but before in-memory apply, and during restart reconciliation cannot create partial or duplicate authority transitions.
10. File replay and PostgreSQL replay derive byte-identical projections and head hashes from the same accepted record sequence.
11. Empty, oversized, truncated, malformed and duplicate JSON file records fail closed; file replacement is atomic and durability/fsync behavior is tested.
12. Every historical fixture from any recovered prior implementation either verifies byte-for-byte or is classified as unsupported/unverified; it must never be silently interpreted under the new codec.

Qualification records must identify the exact commit, protocol constants, fixture hashes, lockfile/toolchain and the verifier version. `queued`, `skipped`, source inspection, and a test target that never ran are not passes.

## 6. Ordered implementation sequence

1. **Recovery freeze:** retain evidence that no authentic provider was recovered; check backups, deployment artifacts and trusted copies without treating consumer code as provider bytes.
2. **Protocol freeze:** review/freeze a fresh identifier and schema/codec lineage, canonical byte grammar, payload variants, signing/record-hash rules, and migration policy. Publish golden vectors before using the protocol in production.
3. **Pure core:** implement types, codec, signatures, hash chain and deterministic projection without filesystem or PostgreSQL side effects.
4. **In-memory invariants:** add actor-key lifetime uniqueness, temporal authorization, replay and final-admin recovery checks.
5. **File backend:** add bounded loading, full history verification and atomic durable replacement; qualify restart and corruption behavior.
6. **Postgres backend:** implement transactionally equivalent prepare/preview/commit/apply and verify denormalized columns/outbox from serialized signed records.
7. **Consumer integration:** re-enable governance, epoch reconstruction, CLI and API against the same provider without a parallel role-grant path.
8. **Exact-head qualification:** first prove the pre-fix module-resolution failure on an isolated baseline; then run the complete affected core crate and downstream consumers, formatting, protocol golden vectors, backend parity and restart/failure-injection suites. State failures precisely; no blanket pass claim.

## 7. Explicit non-claims

This document does not implement the module, prove what old deployments wrote, select production protocol bytes, or authorize deployment. The identifiers/byte grammar are not frozen here. It is a review contract to stop a compile-driven reconstruction from accidentally becoming the credential authority.
