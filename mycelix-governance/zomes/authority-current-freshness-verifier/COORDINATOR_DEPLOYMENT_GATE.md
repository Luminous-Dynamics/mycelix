# Current Freshness — Coordinator Deployment Gate v0.13

Status: **offline-rooted + crash-durable release-key-policy continuity, native hybrid release authentication, offline-rooted + crash-durable complete release-registry continuity, exact deployment composition and subject-bound stability exist as candidates; target/conductor provenance and atomic effect admission remain incomplete**

## Core deployment distinction

Operational authority identity, approved coordinator release identity, release-registry status and actually installed coordinator code are separate facts.

`same DNA != same coordinator implementation`.

Coordinator deployment evidence therefore binds exact `CellId` = DNA hash + cell agent public key and the complete coordinator WASM closure.

## Release-key trust chain

The release-authentication path is:

```text
independently delivered production key-policy root fingerprint
        ↓
#341 offline hybrid root threshold
        ↓
#347 crash-durable state-owned current root/head
        ↓
short-lived current CoordinatorReleaseKeyPolicy
        ↓
manifest-bound QualifiedCoordinatorReleaseSigningKey
        ↓
#326 strict Ed25519 AND ML-DSA-65 authentication
        ↓
QualifiedCoordinatorReleaseRequirement
```

Release signing keys remain leaves. They cannot authorize their own policy root/currentness.

#347 owns predecessor/root/head state under one secure locked path and persists state before #307 positive authority can escape.

Its theorem is local crash/restart continuity under normal owner-protected filesystem semantics, not same-UID arbitrary-write or full-machine rollback resistance.

## Complete release-registry trust chain

#355 replaces the detached status-service authority path with one independently rooted complete canonical registry snapshot:

```text
independently delivered registry-root fingerprint
        ↓
offline hybrid registry-root threshold
        ↓
separate hybrid registry-head threshold
        ↓
short-lived complete canonical registry snapshot
        ↓
local exact manifest-status lookup
        ↓
private #275 head/status compatibility receipts
        ↓
local #275 qualification
        ↓
QualifiedCurrentCoordinatorRelease
```

Release signing keys are not registry-root or registry-head keys.

The registry root commits root identity/version, release authority, registry id, separate root/head key sets and thresholds, and lifetime. Root/head key identities and actual hybrid key material are disjoint.

Initial qualification requires an out-of-band root pin plus the configured root threshold. Rotation requires the exact transition to satisfy both old and new root thresholds, version `old + 1`, fixed scope and non-decreasing threshold floors.

A bootstrap root-lineage digest survives rotation so a predecessor snapshot from another independently pinned root lineage cannot be grafted into the chain.

## Complete snapshot semantics

One registry snapshot commits exact:

- root version;
- registry id;
- release authority;
- release-policy digest/profile chain;
- registry generation;
- predecessor-head digest or explicit genesis;
- complete ordered release-status record set; and
- bounded validity window.

Its head uses #275's fixed `REGISTRY_HEAD_PROFILE`; v0.1 snapshot lifetime is at most 30 seconds and one snapshot contains at most 65,536 records.

Records are strictly ordered and unique by manifest digest. Every successor retains every prior record.

An unchanged status preserves exact effective time/reference. Only:

```text
Active -> Withdrawn
Active -> Superseded
```

with strictly later effective time are accepted. Terminal status resurrection/history rewriting and future-dated status records deny.

Currentness is therefore not inferred from DHT ordering, cache ordering, a `latest record` query or absence of a later record. Completeness is explicit authenticated authority semantics.

## Durable latest complete-registry state

`mycelix-authority-coordinator-release-registry-state` closes the ordinary restart/lost-update boundary above #355.

The central rule is:

`serialized registry state != caller authority`.

Positive registry/current-release authority comes only from `TrustedCoordinatorReleaseRegistryStore` loading its own state path while holding the exclusive store lock.

Normal operations accept no caller-supplied:

- previous trusted state;
- previous/current root;
- root pin;
- previous registry snapshot;
- previous registry generation/head digest;
- #275 registry-head proof; or
- #275 status-at-head proof.

### Canonical path ownership

At construction, a relative path is anchored to the current directory and the requested parent is canonicalized.

The store then retains only the canonical real parent + file name. Changing an ancestor symlink later cannot retarget that store instance.

The parent must be a real effective-user-owned directory, owner-writable and inaccessible to group/other users.

State/lock/temp files must be regular effective-user-owned exact-mode `0600` files opened with `O_NOFOLLOW | O_CLOEXEC`.

### First-use bootstrap

`bootstrap_from_out_of_band_pin` is the only state-adapter operation accepting a registry root pin.

It refuses an existing state path, delegates exact pin + root-threshold cryptography to #355, constructs state generation 1 and durably installs it before returning success.

The adapter does not prove how the production operator obtained the fingerprint; root-pin delivery/protection remains separate provisioning policy.

### State-owned root rotation

`rotate_root` loads the old root from trusted state rather than request input.

The candidate transition requires exact old + new root thresholds, exact version advance, fixed root/authority/registry scope and post-crypto liveness.

Threshold floors ratchet upward:

```text
root_floor_next = max(root_floor_current, new_root.root_threshold)
registry_head_floor_next = max(registry_head_floor_current, new_root.registry_head_threshold)
```

### Registry advancement is query-independent

`advance_snapshot(snapshot, signatures)` verifies and durably advances the complete registry without taking a release query.

A caller asking about release X therefore cannot choose the registry snapshot used to answer that question.

The persisted checkpoint contains the **full canonical snapshot**, not merely its head digest, so status derivation after restart still uses the exact complete state that was accepted previously.

### Persisted latest-snapshot lineage

Without a stored snapshot, genesis is accepted only at root version 1 / registry generation 1 / no predecessor.

An unchanged exact head may be reverified while live.

A changed successor must:

- use generation `current + 1`;
- commit the exact persisted current head digest;
- never decrease root version;
- retain registry id, release authority and release-policy chain; and
- preserve all complete-record/status monotonicity rules.

The predecessor is state-owned across restart.

### Root rotation suspends the old snapshot

An old-root snapshot may remain stored only as the lineage predecessor across a root rotation.

It cannot authorize a release under the new root. `qualify_current_release` requires the snapshot's root version to equal the exact current root version.

Current-release authority resumes only after a successor snapshot signed by the new registry-head role extends the persisted old head.

### Checkpoint lease containment

A persisted checkpoint must recompute the exact snapshot head and satisfy:

```text
snapshot.valid_from <= verified_at < snapshot.valid_until
verified_at < valid_until <= snapshot.valid_until
```

When it is under the current root, its horizon must also not exceed the current root horizon.

Persistence may preserve or shorten authenticated evidence lifetime; it may not widen it.

## Current-release query is state-owned

`qualify_current_release` accepts exactly one already-authenticated `QualifiedCoordinatorReleaseRequirement`.

It loads the current root + complete snapshot from trusted state, validates liveness/scope, performs exact local manifest lookup and denies absent, Withdrawn or Superseded records.

The caller supplies no status and no currentness receipt.

## Persistence precedes #275 positive authority

The live current-release causal order is:

```text
exclusive lock
→ load persisted root + complete snapshot
→ trusted-clock/root/snapshot/release checks
→ local exact Active lookup
→ advance trusted clock-floor state
→ temp fsync + atomic rename + parent-directory fsync
→ privately construct #275 head receipt
→ privately construct #275 status receipt
→ local #275 qualification
→ QualifiedCurrentCoordinatorRelease
```

If state replacement fails, no positive current-release authority escapes.

The evidence-shaped #275 receipts remain private compatibility projections derived from the same persisted complete checkpoint.

## Durable trusted host clock

The state persists `last_trusted_time_ms`.

Every accepted root/snapshot transition samples trusted host time after cryptographic verification. A successful current-release query also advances the clock floor before positive authority escapes.

Observed host time below the persisted floor fails closed.

## Filesystem durability contract

All bootstrap/rotation/snapshot/query state transitions share one exclusive `File::lock` transaction. Rust 1.89 is therefore the v0.1 minimum toolchain.

State parsing is bounded to 32 MiB.

Replacement is:

```text
same-directory create_new temp
→ write exact state
→ fsync temp
→ atomic rename
→ fsync containing directory
```

## Integrity claim is intentionally bounded

Each state carries a monotonically increasing state generation, predecessor-state digest, bootstrap root identity, current root identity, threshold floors, complete snapshot checkpoint, trusted clock floor and BLAKE3 self-digest.

The root digest and state digest are recomputed on load.

This detects corruption/non-self-consistent bytes under the owner-protected-filesystem model. The self-digest is **not keyed**.

Therefore:

```text
crash-durable locally monotone state
!= same-UID arbitrary-write resistance
!= full-machine rollback resistance
```

An attacker with arbitrary trusted-UID file-write authority can recompute an unkeyed state digest. Restoring an entire older machine image can also restore an older internally valid root/snapshot/clock floor together.

A stronger threat model needs an independent keyed/monotonic anchor such as TPM/hardware state, enterprise/device-management state or a separately trusted append-only witness.

## Native conductor observation

`mycelix-authority-coordinator-native-attestor` independently queries the loopback Holochain Admin API for one exact `CellId`, enumerates the complete installed coordinator set, extracts exact `WasmHash` values and owns a short observation lease.

It does not choose release policy, current release or target policy.

Its remaining problem is **live endpoint/conductor provenance**: loopback location by itself is not sufficient proof that the observed endpoint is the intended trusted conductor.

## Exact deployment composition

#290 keeps three domains separate:

```text
trusted exact target CellId selection
        +
QualifiedCurrentCoordinatorRelease
        +
ObservedCoordinatorDeployment
        ↓
#262 exact whole-set equality
        ↓
QualifiedCoordinatorDeploymentComposition
```

Missing, substituted, duplicate or unexpected installed coordinator code denies.

Pure `TargetCellSelection` still does not prove why that CellId is the trusted target.

## Subject-bound stability

#298 requires a strictly later second exact deployment observation, reruns the same deployment requirement and binds stability to one admission subject + attempt nonce.

It proves no detected coordinator-code change across that observed interval.

It is not an atomic exclusion mechanism: `UpdateCoordinators` can still race after the second observation and before an effect.

## Remaining independent live boundaries

The dominant unresolved boundaries are now:

- actual production delivery/protection of both initial root fingerprints;
- trusted target CellId selection provenance;
- real-conductor/local Admin API endpoint provenance and qualification;
- native ownership of admission subject/attempt nonce and pre/post observation bracketing;
- post-observation coordinator-update/effect atomicity;
- final lifecycle/executor/effect-safety authority; and
- stronger same-UID/full-machine rollback anchoring if required by the deployment threat model.

## Consumer rule

A future effect-capable consumer needs independently:

1. fresh operational authority currentness from the direct local authority verifier chain;
2. candidate coordinator release semantics;
3. production-rooted and durably current release-key policy;
4. manifest-bound authorized release signing key;
5. #326 strict hybrid release authentication;
6. production-rooted and durably latest complete registry state;
7. local exact persisted status derivation yielding #275 `QualifiedCurrentCoordinatorRelease`;
8. trusted exact target CellId selection;
9. qualified native pre-deployment conductor observation;
10. #290/#262 exact target/current-release/installed-code composition;
11. qualified native post observation + #298 subject/attempt stability;
12. explicit coordinator-update/effect atomicity;
13. lifecycle/executor/effect-safety authority; and
14. an effect path consuming only in-process positive qualifications, never caller-supplied serialized positive receipts.

## Provisioning state

The v0.13 durable-registry candidate does **not** satisfy the complete deployment/effect gate.

Until production root-pin provenance, target/conductor provenance, native admission ownership and update/effect atomicity are qualified:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no consumer may interpret release currentness or deployment stability as atomic external-effect authority; and
- external effects remain disabled.

## Highest-value next work

1. trusted target CellId selection provenance;
2. real-conductor/local-admin-endpoint qualification;
3. native pre/post admission orchestrator owning subject + attempt nonce;
4. explicit coordinator-update/effect atomicity boundary;
5. final lifecycle/executor/effect-safety binding; and
6. optional keyed/TPM/hardware/enterprise rollback anchor for the trusted state stores.
