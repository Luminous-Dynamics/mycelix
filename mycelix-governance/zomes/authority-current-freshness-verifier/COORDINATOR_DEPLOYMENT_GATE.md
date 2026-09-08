# Current Freshness — Coordinator Deployment Gate v0.12

Status: **offline-rooted + crash-durable release-key-policy continuity, native hybrid release authentication, offline-rooted complete release-registry snapshots, exact deployment composition and stability fencing exist as candidates; durable latest-registry state, remaining target/runtime provenance and atomic effect admission are incomplete**

## Core deployment distinction

Operational authority identity, approved coordinator release identity and actually installed coordinator code are separate facts.

`same DNA != same coordinator implementation`.

Coordinator deployment evidence therefore binds exact `CellId` = DNA hash + cell agent public key and the complete coordinator WASM closure.

## Release-key trust chain

The current release-key path is:

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
#326 strict Ed25519 AND ML-DSA-65 release authentication
        ↓
QualifiedCoordinatorReleaseRequirement
```

Release signing keys remain leaves. They cannot authorize their own policy root/currentness.

#347 normal operations load predecessor/root/head state from one configured secure path under an exclusive transaction lock; state replacement is fsynced and atomically installed before #307 positive authority can escape.

#347 protects ordinary restart/lost-update/torn-write continuity under the local host/filesystem trust model. It does not claim resistance to restoration of an entire old machine image.

## Complete release-registry theorem

`mycelix-authority-coordinator-release-registry` replaces the intended detached status-service path with one independently rooted complete canonical registry snapshot.

The registry trust chain is:

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

### Registry root

The registry root commits exact root identity/version, release authority, registry id, offline root key set/threshold, separate registry-head key set/threshold and lifetime.

Both roles use exact Ed25519 + ML-DSA-65 AND-composition. Key IDs and actual hybrid key material must be disjoint across root and registry-head roles.

Initial root qualification requires an out-of-band pinned canonical root digest/profile plus the configured root threshold. Pin provenance remains an external provisioning responsibility.

Root rotation requires the same transition to satisfy both old and new root thresholds, version exactly old + 1, fixed root/authority/registry scope and non-decreasing threshold floors.

A bootstrap `root_lineage_digest` survives authorized rotation. Registry snapshots from another independently pinned root lineage cannot be grafted into this lineage.

### Complete snapshot identity

A snapshot commits exact:

- current root version;
- registry id;
- release authority;
- one release-policy digest/profile chain;
- registry generation;
- exact predecessor-head digest or explicit genesis;
- complete ordered release-status record set; and
- bounded validity window.

The head digest uses #275's fixed `REGISTRY_HEAD_PROFILE`.

The v0.1 snapshot lifetime is at most 30 seconds.

### Canonical completeness

Records must be strictly ordered by manifest digest, making duplicate manifest entries impossible. v0.1 caps one snapshot at 65,536 records.

Every status record commits exact manifest digest/profile, status, effective time and status reference using #275's exact `STATUS_RECORD_PROFILE`.

A successor may add records, but every record from the previous qualified snapshot must remain present.

Therefore a previously Withdrawn/Superseded release cannot be made to disappear merely by omission in a later otherwise-valid snapshot.

A release not present in the complete qualified snapshot cannot qualify as current.

### Monotone status history

An unchanged status preserves exact effective time/reference.

A status transition is allowed only:

```text
Active -> Withdrawn
Active -> Superseded
```

with a strictly later effective time.

`Withdrawn` and `Superseded` are terminal in v0.1. Neither may become `Active` again, and terminal states cannot silently rewrite into each other.

Future-dated status records also deny: status-effective time must be no later than snapshot validity start.

### Registry predecessor lineage

Without a predecessor, only root version 1 / registry generation 1 / no predecessor digest qualifies.

A changed successor must advance generation by exactly one and commit the exact previous qualified head digest.

It must also remain in the same bootstrap root lineage, registry id, release authority and release-policy digest/profile chain.

Exact current-head revalidation remains allowed while the same immutable snapshot is live.

## #275 receipts are private compatibility objects

The complete qualified snapshot's `qualify_current_release` method performs the exact manifest lookup locally.

It privately creates:

- `VerifiedCurrentReleaseRegistryHeadProof`; and
- `VerifiedCoordinatorReleaseStatusAtHeadProof`

from the same already-qualified complete snapshot and immediately invokes local #275 `qualify_current_coordinator_release`.

The intended live path therefore accepts no caller-supplied #275 head/status receipt bytes as positive registry authority.

This removes a major oracle surface:

`detached status service != release currentness authority`.

## Completeness is not a DHT/latest heuristic

No currentness theorem is inferred from DHT ordering, cache order, `latest record`, or absence of later records.

The registry authority signs the complete canonical snapshot. An authenticated release must have an exact entry in it or qualification denies.

## Durable latest registry state is still missing

The registry theorem currently proves the predecessor represented by an in-process non-deserializable qualified snapshot.

It does not yet prove after restart that the supplied predecessor is the durably latest registry head ever accepted by the device.

The next native registry-state adapter should mirror #347's fail-closed continuity model and persist at minimum:

- bootstrap registry-root lineage digest;
- current registry root + digest/version;
- root/registry-head threshold floors;
- current release-policy chain identity;
- current complete snapshot generation/head digest/records;
- predecessor-state digest/state generation; and
- trusted host-clock floor.

Normal operations must load these from one configured secure path under an exclusive lock, reject caller-selected predecessors, atomically/fsync advance state, and persist before returning current-release positive authority.

Full-machine snapshot rollback remains a stronger separate TPM/hardware/enterprise/witness problem.

## Native conductor observation

`mycelix-authority-coordinator-native-attestor` independently queries the loopback Holochain Admin API for one exact `CellId`, enumerates the complete installed coordinator set, extracts exact `WasmHash` values and owns a short observation lease.

It does not choose release policy, current release or target policy.

## Exact deployment composition

#290 keeps three facts separate:

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

## Subject-bound stability

#298 requires a strictly later second exact deployment observation, reruns the same exact deployment requirement and binds the stability result to one admission subject + per-attempt nonce.

This proves no detected coordinator-code change across the observed interval.

It is not an atomic update exclusion mechanism. `UpdateCoordinators` can still race after the second observation.

## Remaining independent live boundaries

Major unresolved boundaries are now:

- actual production delivery/protection of both initial root fingerprints;
- durable latest complete release-registry state across restart;
- trusted target CellId selection provenance;
- real-conductor qualification/local-endpoint provenance for the native observer;
- native ownership of admission subject/attempt nonce and pre/post bracketing;
- post-observation coordinator-update/effect atomicity;
- final lifecycle/executor/effect-safety authority; and
- stronger full-machine rollback anchoring if required.

## Consumer rule

A future effect-capable consumer needs independently:

1. fresh operational authority currentness from the direct local verifier chain;
2. candidate coordinator release semantics;
3. production-rooted and durably current release-key policy;
4. manifest-bound authorized release signing key;
5. #326 strict hybrid release authentication;
6. production-rooted complete registry snapshot that is also durably latest;
7. local exact registry status derivation yielding #275 `QualifiedCurrentCoordinatorRelease`;
8. trusted exact target CellId selection;
9. native pre-deployment conductor observation;
10. #290/#262 exact target/current-release/installed-code composition;
11. native post observation + #298 subject/attempt stability;
12. explicit update-race/effect atomicity policy;
13. lifecycle/executor/effect-safety authority; and
14. an effect path consuming only in-process positive qualifications, never caller-supplied serialized positive receipts.

## Provisioning state

The v0.12 registry theorem does **not** satisfy the complete deployment/effect gate.

Until durable latest registry state, production root-pin provenance, target/conductor provenance, native bracketing and update-race/effect semantics are qualified:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no consumer may interpret a registry snapshot or deployment match as atomic external-effect authority; and
- external effects remain disabled.

## Highest-value next work

1. native crash-durable latest release-registry state adapter;
2. trusted target CellId selection provenance;
3. real-conductor/local-admin-endpoint qualification;
4. native pre/post admission orchestrator;
5. explicit coordinator-update/effect atomicity boundary;
6. final lifecycle/executor/effect-safety binding; and
7. optional TPM/hardware/enterprise rollback anchor for both key-policy and registry trusted state.
