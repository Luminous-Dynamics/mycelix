# Authority Coordinator Release Key-Policy Trusted State v0.1 — Normative Invariants

Status: **native crash-durable local continuity candidate above #341; whole-machine snapshot rollback and production root-pin delivery remain outside this theorem**

## 1. Serialized state is data, not authority

`TrustedCoordinatorReleaseKeyPolicyState` is serializable/deserializable because it is stored on disk.

Deserializing those bytes does **not** create a positive authority capability. The live trust boundary is the configured `TrustedCoordinatorReleaseKeyPolicyStore`, which loads state itself from its fixed secure path while holding its exclusive transaction lock.

No normal live method accepts a caller-supplied `TrustedCoordinatorReleaseKeyPolicyState`, prior root, prior root pin, prior policy head, prior state generation or prior state digest.

## 2. Bootstrap is a separate first-use ceremony

`bootstrap_from_out_of_band_pin` is the only live method that accepts `CoordinatorReleaseKeyPolicyRootPin`.

It is first-use only and refuses to overwrite any existing state path, including a symlink.

The pin must come from an independently authenticated operator/provisioning channel. This crate verifies exact pin/root equality and #341 root-threshold crypto but cannot prove how the operator obtained the fingerprint.

Normal root/head verification never accepts a new root pin.

## 3. The previous root is state-owned

`rotate_root` accepts a candidate new root plus old/new transition signatures. It does not accept the old root.

The old root is loaded from persisted trusted state under the exclusive lock. This prevents a request from selecting an alternate historical root as the authority for its own rotation.

## 4. Read / verify / advance is one exclusive transaction

Bootstrap, root rotation, policy-head verification and diagnostics take the same exclusive state lock.

A transition must execute:

`lock -> load trusted state -> verify exact transition -> construct next state -> durably replace state -> return`.

Concurrent callers cannot independently read the same generation and both commit divergent successors through this adapter.

## 5. Filesystem inputs fail closed

v0.1 is Unix-only.

The configured immediate parent directory must be:

- a real directory, not a symlink;
- owned by the effective user;
- owner-writable; and
- inaccessible to group/other users.

The state file and lock file must be:

- regular files;
- opened with `O_NOFOLLOW` and `O_CLOEXEC`;
- owned by the effective user; and
- mode `0600` exactly.

A symlink, wrong owner, broad mode or non-regular file denies.

This is a local-filesystem trust contract, not a sandbox against a fully compromised same-UID process or root administrator.

## 6. State replacement is crash-durable at the filesystem boundary

A next state is written to a unique same-directory file opened with `create_new`, then:

1. exact bytes are written;
2. the temporary file is `sync_all`'d;
3. the temporary file is renamed over the trusted-state path; and
4. the containing directory is `sync_all`'d.

The exclusive transaction lock remains held throughout.

This prevents ordinary torn-write and lost-update failures under filesystem guarantees compatible with atomic same-directory rename and fsync semantics.

## 7. State size is bounded

The trusted-state file is capped at 4 MiB.

Loading uses a bounded read even after metadata length inspection, so a file that grows concurrently cannot force an unbounded read through this parser.

## 8. State identity is self-consistent and parent linked

Every state commits exact:

- protocol/profile;
- monotone state generation;
- predecessor-state digest after genesis;
- bootstrap root digest/profile;
- current root digest/profile;
- root and policy-head threshold floors;
- current policy-head checkpoint, if any; and
- trusted clock floor.

The current root digest is recomputed from embedded root metadata on every load.

The BLAKE3 state digest detects corruption and inconsistent reconstruction. It is **not** a cryptographic anti-rollback anchor against an attacker who can rewrite the entire file and recompute its unkeyed digest.

## 9. State generation never moves backward through the adapter

Genesis state is generation 1 with no predecessor-state digest.

Every persisted transition uses checked `state_generation + 1` and commits the exact previous state's digest.

The previous-state digest is lineage metadata. Because old files are replaced rather than retained in an independently anchored log, it is not by itself proof against restoration of a complete older state image.

## 10. Trusted clock has a durable floor

Each successful transition persists `last_trusted_time_ms`.

Any later host time below that stored value fails closed before root/head currentness can qualify.

A backward NTP adjustment can therefore cause availability loss. v0.1 chooses security over silently accepting rollback in the currentness clock.

The trusted clock floor is still stored in the same local state file and is not a hardware monotonic clock.

## 11. Root rotation remains dual-threshold and exact

The state-owned current root and candidate new root must satisfy the #341 scope/version theorem:

- both roots live at verification start and after crypto;
- new root version = old root version + 1;
- root id unchanged;
- release authority unchanged;
- policy id unchanged;
- old root threshold verifies the exact transition; and
- new root threshold verifies the exact same transition.

Both threshold roles use strict Ed25519 AND ML-DSA-65 verification over the exact #341 rotation message.

## 12. Threshold floors ratchet upward

The bootstrap root and policy-head thresholds establish initial local floors.

After an authorized rotation:

`root_floor_next = max(root_floor_current, new_root.root_threshold)`

and

`policy_head_floor_next = max(policy_head_floor_current, new_root.policy_head_threshold)`.

A later in-band rotation cannot reduce a threshold that an earlier trusted root strengthened.

## 13. Policy-head currentness is rooted in persisted latest state

A candidate head must bind the exact currently persisted root version and exact semantic policy digest/profile/generation.

Lineage is accepted only as:

- the exact currently persisted head, for fresh revalidation; or
- exact generation `current + 1` committing the persisted current head digest.

Once a head exists, an older signed head, skipped generation, alternate parent or generation restart denies even after creating a fresh `TrustedCoordinatorReleaseKeyPolicyStore` instance.

## 14. v0.1 genesis is deliberately fail-closed across root rotation

A predecessor-free policy head is accepted only when the persisted root is version 1 and the policy generation is 1.

Therefore operators must establish the initial generation-1 policy head before rotating the root.

This is stricter than necessary once durable state can explicitly prove that no policy head has ever existed. A successor may add an explicit persisted lineage-initialization marker and migration theorem; v0.1 does not infer absence-of-history from a rotated root.

## 15. Exact-head revalidation is allowed

The same exact signed head may be reverified while still live so multiple candidate manifests can consume one short-lived current policy generation.

Revalidation still performs threshold crypto, post-crypto clock/liveness checks and a durable state advancement. It does not trust a cached serialized positive receipt.

## 16. Currentness crypto remains fail-closed hybrid AND

The adapter independently verifies the state-owned root/head transitions using the same exact v0.1 wire contract as #341:

- Ed25519 public key: 32 bytes;
- Ed25519 signature: 64 bytes;
- ML-DSA-65 public key: 1952 bytes;
- ML-DSA-65 signature: 3309 bytes.

Threshold evidence contains exactly the configured number of unique authorized live signers. Every participating signer must pass both algorithms over identical deterministic bytes.

No OR/fallback/classical-only/PQ-only path exists.

## 17. Qualification time follows crypto

The adapter samples host time before verification only to establish a start/liveness boundary.

After threshold crypto completes it samples time again, requires the persisted clock floor, and rechecks root, policy, head and signer liveness before constructing the next checkpoint.

Evidence expiring during verification fails closed.

## 18. Currentness lease is monotone

The private #307 currentness horizon is bounded by:

`min(current root, semantic policy, signed head, participating policy-head signer horizons)`.

It must also fit within #307's maximum currentness reuse window.

The adapter never widens the #341/#307 evidence lease.

## 19. Persistence precedes positive authority escape

For the manifest-key-policy live path the causal order is strictly:

`load persisted latest state -> verify root/policy/head/manifest -> hybrid threshold crypto -> post-crypto clock -> construct next checkpoint -> fsync+rename+directory-fsync next state -> privately construct #307 receipt -> local #307 qualification -> return non-deserializable positive capability`.

If persistence fails, no `QualifiedCoordinatorReleaseManifestKeyPolicy` is returned.

## 20. #307 evidence-shaped receipt stays private

`VerifiedCurrentCoordinatorReleaseKeyPolicyProof` is created only after successful durable state advancement and is immediately consumed by local #307 qualification.

No public state-adapter API returns it.

## 21. Manifest mismatch cannot advance trusted policy state

Before any head checkpoint is persisted, the candidate manifest is validated and must already commit the exact semantic policy digest/profile and release authority being qualified.

A manifest/policy mismatch fails before state advancement, preventing invalid candidate manifests from consuming or advancing trusted currentness state.

## 22. No release-signing-key circularity

The trusted-state crate depends on semantic release/key-policy/root types, not on #269 authenticated release, #275 current release or #326 release-authentication positive objects.

Release signing keys remain leaves below independently rooted policy currentness.

## 23. Local durable state is not a hardware rollback anchor

The theorem proved here is:

`normal host/filesystem protection + exclusive transaction + crash-durable replacement + persisted monotonic checks`.

It is **not**:

`proof against restoring an entire older machine/filesystem snapshot`.

Full-image rollback resistance requires an independent monotonic anchor such as TPM/hardware-backed state, enterprise/device-management monotonic state, or another separately trusted append-only witness. No such property is claimed here.

## 24. No Holochain or external effects

This crate contains no Holochain client/zome calls, no coordinator update call, no lifecycle execution and no external-effect path.

Persisting this local trust checkpoint is internal verifier state; it does not authorize arbitrary external effects.

## 25. Provisioning remains blocked

Before effect-capable provisioning the stack still requires at minimum:

- real production delivery/protection of the out-of-band root pin;
- current release-registry head/completeness provenance;
- exact release status-at-head provenance;
- trusted target CellId selection;
- real-conductor qualification of coordinator observations;
- native ownership of admission subject/nonce and pre/post observation bracketing;
- coordinator-update race / atomic effect admission semantics;
- final lifecycle/executor/effect-safety admission; and
- hardware/enterprise monotonic anchoring if full-machine rollback resistance is a requirement.
