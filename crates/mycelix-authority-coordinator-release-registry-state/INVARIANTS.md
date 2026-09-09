# Authority Coordinator Release Registry State v0.1 — Normative Invariants

Status: **native crash-durable latest complete-registry continuity under an owner-protected Unix filesystem; same-UID arbitrary rewrite, full-machine rollback, production root-pin delivery, runtime admission and effects remain separate theorems**

## 1. Serialized state is data, not caller authority

`TrustedCoordinatorReleaseRegistryState` is serializable/deserializable because it is persisted to disk. Its bytes are not a positive capability.

Normal live authority comes only from `TrustedCoordinatorReleaseRegistryStore` loading the configured store-owned path while holding the exclusive store lock.

No normal public operation accepts caller-supplied previous trusted state, previous root, root pin, previous registry snapshot, registry generation, previous head digest, #275 head proof or #275 status proof.

## 2. The state path resolves to one canonical real parent

At store construction, relative paths are anchored to the current directory and the requested parent is canonicalized.

The store retains only the canonical real parent + requested file name. An ancestor symlink changed after store construction therefore cannot retarget that store instance.

The canonical parent must be an effective-user-owned real directory, owner-writable, and inaccessible to group/other users.

## 3. Bootstrap is a distinct first-use ceremony

`bootstrap_from_out_of_band_pin` is the only state-adapter operation accepting `CoordinatorReleaseRegistryRootPin`.

It refuses any pre-existing state path, delegates exact root-pin + hybrid threshold verification to #355, constructs state generation 1, and durably persists it before reporting success.

The adapter does not prove how the operator obtained the production root fingerprint. Root-pin delivery/protection remains an independent provisioning theorem.

## 4. The previous root is state-owned

`rotate_root` obtains the old root only from persisted trusted state.

A caller may supply only the candidate new root and detached old/new threshold signatures.

Rotation requires exact root scope, version `old + 1`, old-root threshold verification, new-root threshold verification, post-crypto liveness, and durable state advancement.

## 5. Threshold floors ratchet upward

The durable state carries root and registry-head threshold floors.

On rotation:

`root_floor_next = max(root_floor_current, new_root.root_threshold)`

`registry_head_floor_next = max(registry_head_floor_current, new_root.registry_head_threshold)`

A later in-band root may not lower a threshold previously strengthened by a trusted root.

## 6. Registry advancement is independent of release queries

`advance_snapshot(snapshot, signatures)` verifies and durably installs one candidate complete registry snapshot without taking an authenticated release as input.

A release query cannot choose which snapshot becomes current.

## 7. The complete snapshot is persisted, not merely its head digest

`TrustedRegistrySnapshotCheckpoint` contains the exact complete canonical `CoordinatorReleaseRegistrySnapshot` plus its head digest, signer-set identity and verified/reuse window.

After restart, local status lookup therefore uses the exact full registry state previously accepted by the adapter rather than reconstructing status from a cache, DHT query or detached status service.

## 8. Checkpoint horizons may not exceed signed snapshot horizons

A checkpoint must recompute the exact registry-head digest and satisfy:

- `snapshot.valid_from <= verified_at < snapshot.valid_until`;
- `verified_at < valid_until`; and
- `valid_until <= snapshot.valid_until`.

When the checkpoint is under the current root version, its reuse horizon must also not exceed the current root horizon.

Persistence may preserve or shorten a verified lease; it may never widen it.

## 9. Persisted latest-snapshot lineage is monotone

With no current snapshot, genesis is accepted only at root version 1 / registry generation 1 / no predecessor.

An unchanged exact head may be reverified while live.

A changed successor must:

- use registry generation `current + 1`;
- commit the exact persisted current head digest;
- never decrease root version;
- retain the exact registry id, release authority and release-policy chain; and
- satisfy complete-record monotonicity.

The predecessor comes from state-owned storage, never caller input.

## 10. Complete-record monotonicity survives restart

Every persisted successor must retain every record from the previous complete snapshot.

An unchanged status preserves exact effective time/reference.

Only `Active -> Withdrawn` and `Active -> Superseded` with strictly later effective time are accepted.

Terminal statuses cannot resurrect or rewrite history.

## 11. Root rotation immediately suspends old-snapshot release authority

The old complete snapshot may remain persisted as the lineage predecessor across an authorized root rotation, but it cannot authorize a release after the root version changes.

`qualify_current_release` requires the persisted complete snapshot to be under the exact current root version.

A newly signed successor snapshot under the new root must extend the old persisted head before current release authority resumes.

## 12. Release lookup is state-owned

`qualify_current_release` accepts exactly one already-authenticated `QualifiedCoordinatorReleaseRequirement`.

It accepts no snapshot, status value, head proof or status proof.

The method loads the persisted latest complete snapshot, performs exact manifest lookup locally and denies missing, Withdrawn or Superseded records.

## 13. #275 evidence-shaped receipts remain private compatibility projections

After local status lookup, the adapter privately constructs both `VerifiedCurrentReleaseRegistryHeadProof` and `VerifiedCoordinatorReleaseStatusAtHeadProof` from the same persisted complete checkpoint.

Those receipts are immediately consumed through local #275 `qualify_current_coordinator_release` and are never returned as public positive authority.

## 14. Trusted host time has a durable floor

Every accepted root/snapshot transition advances `last_trusted_time_ms` after cryptographic verification.

A successful current-release query also advances and persists the clock floor before positive current-release authority escapes.

Observed host time below the persisted floor fails closed.

This can trade availability for safety after severe backward clock correction.

## 15. Persistence precedes positive current-release authority

The current-release causal order is:

`lock -> load persisted root/snapshot -> check clock/root/snapshot/release -> local Active lookup -> construct next clock-floor state -> fsync/rename/fsync state -> private #275 receipts -> local #275 qualification -> QualifiedCurrentCoordinatorRelease`.

If durable state replacement fails, no positive `QualifiedCurrentCoordinatorRelease` escapes.

## 16. Read / verify / advance is one exclusive transaction

Bootstrap, root rotation, snapshot advancement, current-release qualification and diagnostics share one store lock.

Normal operations cannot interleave read/verify/write phases through separate caller-managed transactions.

Rust 1.89 is the minimum toolchain because v0.1 uses the stabilized `std::fs::File::lock` API.

## 17. Filesystem inputs fail closed

State and lock paths reject symlink final components.

State, lock and temporary files must be regular effective-user-owned files with exact mode `0600` and are opened with `O_NOFOLLOW | O_CLOEXEC`.

State parsing is bounded to 32 MiB.

## 18. State replacement is crash-durable at the filesystem boundary

Replacement is:

`same-directory create_new temp -> write -> temp fsync -> atomic rename -> containing-directory fsync`.

A failed replacement returns an error; the temporary file is best-effort removed.

## 19. State identity is self-consistent and parent linked

Every state commits protocol/profile, monotonically increasing state generation, predecessor-state digest, bootstrap root identity, current root identity, threshold floors, complete snapshot checkpoint and trusted clock floor.

The current root digest and state self-digest are recomputed on load.

This detects accidental corruption and non-self-consistent bytes under the trusted local-filesystem model.

## 20. The self-digest is not a keyed anti-tamper proof

`state_digest` is unkeyed BLAKE3. An attacker able to arbitrarily rewrite files as the trusted effective UID can also recompute it.

Likewise, the persisted checkpoint stores the result of prior signature verification rather than enough historical transition evidence to re-prove the entire root/snapshot lineage after arbitrary trusted-file modification.

Therefore:

`self-consistent persisted state != cryptographically authenticated persisted state against same-UID rewrite`.

The v0.1 theorem explicitly assumes the owner-protected filesystem prevents untrusted modification.

## 21. Local durability is not full-machine anti-rollback

Restoring an entire older filesystem/machine image can restore an older internally valid root, snapshot, state generation and clock floor together.

Full-image rollback resistance requires an independent monotonic anchor such as TPM/hardware state, enterprise/device-management state or a separately trusted append-only witness.

No such property is claimed here.

## 22. No Holochain or external effects

This crate performs native cryptographic verification and local trusted-state persistence only.

It contains no Holochain zome calls, AdminWebsocket operations, coordinator updates, lifecycle admission or external-effect execution.

## 23. Provisioning remains blocked

Before effect-capable provisioning, the stack still requires at minimum:

- production delivery/protection of the release-key and release-registry root fingerprints;
- trusted target CellId selection provenance;
- real-conductor/local Admin API provenance and qualification;
- native admission subject/attempt ownership and pre/post bracketing;
- coordinator-update/effect atomicity;
- final lifecycle/executor/effect-safety composition; and
- a stronger independent rollback/tamper anchor if the deployment threat model includes same-UID trusted-state rewrite or whole-machine snapshot rollback.
