# Current Freshness — Coordinator Deployment Gate v0.14

Status: **offline-rooted + crash-durable release-key-policy continuity, native hybrid release authentication, offline-rooted + crash-durable complete release-registry continuity, out-of-band pinned + crash-durable live target-CellId provenance, exact deployment composition and subject-bound stability exist as candidates; conductor-process identity and atomic effect admission remain incomplete**

## Core deployment distinction

Operational authority, approved release, release-registry status, exact target CellId, conductor process identity and actually installed coordinator code are separate facts.

`same DNA != same CellId != same conductor process != same coordinator implementation`.

A complete deployment admission therefore needs independently grounded evidence for each domain.

## Release authentication chain

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

Release signing keys are leaves; they cannot authorize their own policy root/currentness.

#347 persists root/head continuity before #307 positive key-policy authority escapes. Its guarantee is local crash/restart continuity under an owner-protected filesystem, not same-UID arbitrary-write or full-machine rollback resistance.

## Complete release-registry chain

```text
independently delivered registry-root fingerprint
        ↓
#355 offline hybrid registry-root threshold
        ↓
separate hybrid registry-head threshold
        ↓
complete canonical release-status snapshot
        ↓
#365 crash-durable state-owned latest snapshot
        ↓
local exact manifest-status lookup
        ↓
private #275 head/status compatibility receipts
        ↓
QualifiedCurrentCoordinatorRelease
```

The registry snapshot is complete authority data, not a latest-record heuristic. Records are canonical/unique, old records cannot disappear, and terminal status cannot resurrect.

#365 persists the full snapshot, root lineage, threshold floors, trusted clock floor and predecessor state before current-release authority escapes.

Root rotation suspends an old-root snapshot from authorizing releases until a successor snapshot under the new root extends the persisted predecessor head.

## Durable registry integrity claim remains bounded

The registry/key-policy state stores use canonical owner-controlled paths, exact `0600` regular files, `O_NOFOLLOW | O_CLOEXEC`, one exclusive `File::lock` transaction and same-directory temp-fsync/rename/directory-fsync replacement.

Their BLAKE3 self-digests are unkeyed. Therefore:

```text
crash-durable locally monotone state
!= same-UID arbitrary-write resistance
!= full-machine rollback resistance
```

A stronger deployment threat model needs a separately protected keyed/TPM/hardware/enterprise/append-only rollback anchor.

## Exact target CellId is now independently pinned

`mycelix-authority-coordinator-target-cell` closes #290's caller-supplied target provenance gap.

The first-use binding commits exact:

- raw 39-byte `DnaHash`;
- raw 39-byte `AgentPubKey`;
- loopback Holochain Admin `SocketAddr`;
- binding id; and
- provisioning reference.

The pair `DnaHash + AgentPubKey` is the exact Holochain `CellId`; DNA-only identity is insufficient.

Bootstrap requires the exact binding digest/profile to match an independently delivered out-of-band pin and refuses an already initialized state path.

Normal live qualification accepts **no target CellId and no Admin endpoint** from the caller.

## Target state is state-owned and crash durable

The target store resolves the requested parent once to a canonical real directory, then owns one fixed state/lock path.

Persisted state commits:

- exact target binding digest;
- monotone state generation;
- exact predecessor-state digest; and
- durable host-clock floor.

The same filesystem rules used by the registry/key-policy stores apply: owner-controlled directory, exact `0600` files, `O_NOFOLLOW`, exclusive lock, bounded parsing, same-directory temp, fsync, atomic rename and parent-directory fsync.

There is intentionally **no in-band retargeting API** in v0.1. A legitimate cell reinstall/replacement that changes exact `CellId` requires an explicit external decommission + fresh provisioning ceremony rather than silently inheriting old target authority.

## Live target presence is observed directly

The target adapter creates its own `AdminWebsocket` to the state-owned pinned loopback endpoint and calls Holochain `list_cell_ids()`.

The exact pinned DNA + agent pair must occur exactly once in the returned live-cell set.

Missing target denies. Duplicate exact target identity denies.

No app-name, role-name, cache, DHT, release-manifest or caller-selected heuristic chooses the target.

Observation time is sampled only after the conductor response and CellId extraction complete.

Positive target reuse is bounded to five seconds.

## Persistence precedes positive target authority

The live target causal order is:

```text
exclusive lock
→ load state-owned exact target + pinned endpoint
→ Admin list_cell_ids
→ require exact pinned CellId live exactly once
→ post-observation host clock
→ advance trusted clock-floor state
→ temp fsync + atomic rename + parent-directory fsync
→ privately construct #290 TargetCellSelection compatibility data
→ QualifiedTargetCellSelection
```

If persistence fails, no positive target capability escapes.

`QualifiedTargetCellSelection` is non-deserializable. The inner #290 `TargetCellSelection` remains deserializable compatibility data and must not be accepted directly as live provenance by an effect-capable consumer.

## Pinned endpoint still does not prove conductor process identity

Pinning the exact loopback socket removes caller endpoint selection, but loopback is not a cryptographic process identity.

A successful Admin request currently proves only that the adapter exchanged the expected Admin protocol with the process listening at the pinned local endpoint.

It does **not** yet prove:

- executable identity/hash/signature of the Holochain conductor;
- OS service identity owning the socket;
- process namespace/cgroup/systemd-unit identity;
- kernel/host integrity; or
- absence of a malicious local process impersonating the endpoint.

That is now the dominant remaining provenance boundary.

## Native coordinator-code observation remains separate

`mycelix-authority-coordinator-native-attestor` independently queries the exact target CellId through a loopback Holochain Admin endpoint, enumerates the complete `DnaDef.coordinator_zomes` set and preserves exact coordinator `WasmHash` identities.

It does not choose the approved release or perform the #262 equality match.

The intended next composition must ensure the native code observer and the target capability refer to the same independently qualified conductor/endpoint and exact CellId.

## Exact deployment composition

#290 currently joins:

```text
TargetCellSelection
        +
QualifiedCurrentCoordinatorRelease
        +
ObservedCoordinatorDeployment
        ↓
#262 exact whole-set equality
        ↓
QualifiedCoordinatorDeploymentComposition
```

Missing, substituted, duplicate or unexpected installed coordinator code denies through #262.

For live admission, the deserializable target input must be projected privately from `QualifiedTargetCellSelection`; caller-supplied #290 target data must not regain authority at the join.

## Subject-bound stability

#298 requires a strictly later second exact deployment observation, reconstructs the same target/current-release/deployment requirement and binds stability to one admission subject + attempt nonce.

This proves no detected coordinator-code change across the observed interval.

It is still not atomic exclusion. The target can stop, an app can be disabled, the conductor can be replaced, or `UpdateCoordinators` can race after the second observation and before an external effect.

## Remaining independent boundaries

The dominant unresolved boundaries are now:

- production delivery/protection of the initial key-policy, registry and target-binding fingerprints;
- real conductor-process / pinned Admin-endpoint identity;
- native consumption of `QualifiedTargetCellSelection` together with target-matched coordinator observation;
- native ownership of admission subject/attempt nonce and pre/post observation bracketing;
- target-liveness / coordinator-update / conductor-replacement atomicity with the effect;
- final lifecycle/executor/effect-safety authority; and
- stronger same-UID/full-machine rollback anchoring if required by deployment threat model.

## Consumer rule

A future effect-capable admission path needs independently:

1. fresh operational authority currentness from the direct local authority verifier chain;
2. candidate coordinator release semantics;
3. production-rooted and durably current release-key policy;
4. manifest-bound authorized release signing key;
5. #326 strict hybrid release authentication;
6. production-rooted and durably latest complete registry state;
7. local exact persisted Active status yielding `QualifiedCurrentCoordinatorRelease`;
8. independently pinned + durably state-owned + freshly live `QualifiedTargetCellSelection`;
9. independently qualified conductor-process/Admin-endpoint identity;
10. native exact coordinator-code observation for that same conductor + exact target;
11. #290/#262 exact target/current-release/installed-code composition using only local positive target provenance;
12. qualified native post observation + #298 subject/attempt stability;
13. explicit target/conductor/coordinator-update/effect atomicity;
14. lifecycle/executor/effect-safety authority; and
15. an effect path consuming only in-process positive qualifications, never caller-supplied serialized positive receipts.

## Provisioning state

The v0.14 target candidate still does **not** satisfy the complete deployment/effect gate.

Until production pin provenance, conductor-process identity, native admission ownership and update/effect atomicity are qualified:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no consumer may interpret target liveness, release currentness or deployment stability as atomic external-effect authority; and
- external effects remain disabled.

## Highest-value next work

1. conductor-process / pinned Admin-endpoint identity qualification;
2. native target-aware deployment composer consuming `QualifiedTargetCellSelection` rather than caller target data;
3. native pre/post admission orchestrator owning subject + attempt nonce;
4. target-liveness + coordinator-update + conductor-replacement atomicity boundary;
5. final lifecycle/executor/effect-safety binding; and
6. optional keyed/TPM/hardware/enterprise rollback anchors for trusted state stores.
