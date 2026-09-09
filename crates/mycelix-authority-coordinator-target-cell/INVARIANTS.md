# Authority Coordinator Target Cell v0.1 — Normative Invariants

Status: **native, crash-durable exact target-CellId selection candidate; conductor-process identity, coordinator-code observation, update atomicity and effect admission remain separate**

## 1. Target identity is pinned out of band

The target is the exact Holochain `CellId` pair:

`DnaHash + AgentPubKey`.

The first-use `TargetCellBinding` also commits the exact loopback Holochain Admin endpoint and a provisioning reference.

`bootstrap_from_out_of_band_pin` accepts the binding only when its canonical binding digest/profile exactly matches the independently delivered `TargetCellBindingPin`.

Pure digest equality does not prove how the operator obtained the pin. Production pin delivery/protection remains an external provisioning theorem.

## 2. Normal live qualification accepts neither target nor endpoint

`TrustedTargetCellStore::qualify_live_target()` has no target/CellId, agent key, DNA hash, endpoint, binding, pin, timestamp or validity argument.

It loads the exact binding from its store-owned state path while holding the exclusive lock.

A caller therefore cannot retarget one admission attempt by choosing another CellId or another Admin socket.

## 3. The Admin endpoint is part of target binding

The pinned endpoint must be loopback with a non-zero port through `LocalAdminEndpoint` validation.

The endpoint is committed by the binding digest and persisted with the exact CellId.

Loopback narrows network exposure but is not proof of conductor-process identity. A malicious local process capable of impersonating the pinned socket remains outside this theorem.

## 4. Live presence is observed directly

The adapter creates its own `AdminWebsocket` to the state-owned endpoint and calls `list_cell_ids()`.

Holochain defines `ListCellIds` as the IDs of all live cells currently running in the conductor.

The exact pinned DNA hash + exact pinned agent public key must occur exactly once in the observed set. Missing or duplicate target identity denies.

The live observer does not infer target identity from app names, role names, DNA-only identity, cache order, DHT records, release metadata or caller input.

## 5. Observation time follows conductor evidence production

The adapter samples `observed_at_ms` only after the `list_cell_ids()` response is received and exact raw CellId bytes are extracted.

The public API accepts no caller clock or validity horizon.

A persisted trusted-clock floor is checked before observation and rechecked against the post-observation timestamp.

## 6. Target reuse is short lived

Positive target qualification has a fixed v0.1 reuse horizon:

`valid_until_ms = observed_at_ms + 5 seconds`.

This is a bounded target-presence lease, not a claim that the target cannot stop immediately afterward.

## 7. Persistence precedes positive target authority

The exact causal order is:

`exclusive lock -> load state-owned binding -> Admin list_cell_ids -> exact pinned CellId presence -> post-observation clock -> advance trusted state -> temp fsync -> atomic rename -> directory fsync -> construct TargetCellSelection -> construct QualifiedTargetCellSelection`.

If durable state advancement fails, no positive target capability escapes.

## 8. Positive target qualification is non-deserializable

`QualifiedTargetCellSelection` derives `Serialize` but not `Deserialize`.

The inner legacy #290 `TargetCellSelection` remains deserializable compatibility data, but the intended native live path constructs it privately only after persisted target qualification.

A consumer must take the outer non-deserializable qualification as the provenance boundary; caller-supplied `TargetCellSelection` remains untrusted data.

## 9. The exact #290 selection identity is preserved

The privately constructed compatibility selection uses #290's exact protocol/profile and exact pinned raw 39-byte DNA + agent identities.

Its `selection_ref` is derived locally from the binding digest, newly persisted state digest, pinned endpoint and observation time.

No release authority may choose the installation-specific agent key.

## 10. State path resolves to one canonical real parent

At store construction a relative requested path is anchored to the current directory and the parent directory is canonicalized.

The store retains the canonical real parent plus fixed state/lock filenames. A later ancestor-symlink change cannot retarget an already-created store instance.

The parent must be a real effective-user-owned directory, owner writable and inaccessible to group/other users.

## 11. Read / verify / advance is one exclusive transaction

Bootstrap, live qualification and diagnostics use the same fixed lock file and `File::lock()` exclusive transaction boundary.

State and lock files must be regular effective-user-owned exact-mode `0600` files opened with `O_NOFOLLOW | O_CLOEXEC`.

State parsing is bounded to 1 MiB.

## 12. State replacement is crash durable at the filesystem boundary

Replacement is:

`same-directory create_new temp -> write -> fsync temp -> atomic rename -> fsync containing directory`.

No positive target qualification may be returned before this completes.

## 13. Trusted host time has a durable floor

Every positive live-target observation advances `last_trusted_time_ms` in persisted state.

An observed host time below that floor denies.

This detects ordinary local clock rollback across restart under the protected-filesystem model; it is not a hardware trusted-time theorem.

## 14. State identity is self-consistent and parent linked

Persisted state commits exact protocol/profile, monotone state generation, predecessor-state digest, binding digest/profile and trusted-clock floor.

Generation 1 has no predecessor. Every later generation commits the exact previous state digest.

The BLAKE3 state self-digest detects accidental/non-self-consistent corruption under the owner-protected-filesystem model.

## 15. The self-digest is not a keyed anti-tamper proof

The state digest is unkeyed.

An attacker with arbitrary write authority as the trusted UID can rewrite the state and recompute it. This theorem does not advertise same-UID arbitrary-write resistance.

A stronger threat model needs a separately protected keyed/monotonic anchor.

## 16. Local durability is not full-machine anti-rollback

Restoring an entire older machine/filesystem image can restore an older internally valid target binding, state generation and clock floor together.

TPM/hardware monotonic state, enterprise/device-management state or a separately trusted append-only witness remains a future stronger rollback theorem.

## 17. There is no in-band retargeting in v0.1

After first-use bootstrap, this crate exposes no API that changes the pinned DNA hash, agent public key or Admin endpoint.

A legitimate replacement/reinstallation that changes exact CellId requires an explicit external decommission + new provisioning ceremony. Silent continuity from old CellId to new CellId is intentionally denied.

## 18. Target provenance remains separate from release and code observation

This crate does not authenticate a coordinator release, inspect coordinator WASM, run #262 exact code-set matching or create #290 deployment composition.

The trust domains remain:

`pinned exact target identity != target live-presence observation != approved current release != installed coordinator-code observation`.

## 19. No conductor-process integrity claim

A successful `list_cell_ids()` call proves that this adapter exchanged the Admin protocol with the process at the pinned loopback endpoint and observed the pinned CellId in that response.

It does not prove the identity/hash/signature of the conductor executable, socket ownership by an expected service, OS/kernel integrity or absence of a malicious local endpoint impersonator.

That is now the next native provenance boundary.

## 20. No update atomicity or effect authority

Target qualification says only that the exact pinned target was observed live within a short lease.

It does not prevent cell/app disable, coordinator update or process replacement immediately afterward, and grants no lifecycle/executor/effect-safety/external-effect authority.

## 21. Provisioning remains blocked

Before effect-capable provisioning the stack still requires at minimum:

- conductor process / Admin endpoint identity qualification;
- native use of this non-deserializable target capability in deployment composition;
- native coordinator-code observation for that same exact target;
- pre/post admission orchestration;
- coordinator-update / target-liveness race atomicity; and
- final lifecycle/executor/effect-safety binding.
