# Current Freshness — Coordinator Deployment Gate v0.11

Status: **offline-rooted + crash-durable local release-key-policy continuity, native hybrid release authentication, release currentness semantics, exact deployment composition and stability fencing exist as candidates; remaining registry/target/runtime provenance and atomic effect admission are incomplete**

## Core deployment distinction

The operational currentness theorem binds local `DnaHash`, but coordinator WASM can change without changing that hash. Therefore:

`same DNA != same coordinator implementation`.

Coordinator deployment evidence must bind exact `CellId` = DNA hash + cell agent public key and the complete coordinator WASM closure. Coordinator self-report is never sufficient.

## Native conductor observation candidate

`mycelix-authority-coordinator-native-attestor` queries the loopback Holochain Admin API for one exact `CellId`, enumerates the complete installed coordinator set, extracts exact `WasmHash` values, samples time after observation/extraction and owns a five-second reuse cap.

It does not choose an approved release, target policy or deployment match.

## Candidate release semantics

`mycelix-authority-coordinator-release` defines the DNA/code-scoped release manifest. It commits the complete approved coordinator `WasmHash` closure plus DNA bundle/source/lock/toolchain/build-recipe/SBOM identities and release authority/policy identity.

It exists before signature authentication and does not choose an installation-specific agent.

## Release-key trust is non-circular

The intended key-policy chain is now:

```text
independently delivered production root fingerprint
        ↓
first-use pinned offline hybrid root
        ↓
crash-durable state-owned current root
        ↓
separate hybrid policy-head threshold
        ↓
durably latest short-lived policy head
        ↓
semantic CoordinatorReleaseKeyPolicy
        ↓
manifest-bound QualifiedCoordinatorReleaseSigningKey
        ↓
#326 hybrid release authentication
```

Release signing keys remain leaves. They cannot create or rotate their policy root and cannot declare their own key policy current.

## #341 offline-rooted policy-currentness theorem

`mycelix-authority-coordinator-release-key-policy-root` defines the pure cryptographic root/currentness theorem.

The root commits exact root identity/version, release authority, policy id, offline root key set/threshold, distinct policy-head key set/threshold and root lifetime.

Root and policy-head roles use disjoint key IDs and disjoint actual Ed25519 + ML-DSA-65 material.

Initial bootstrap requires exact out-of-band pin equality plus the configured offline root threshold. The pin establishes identity only; root lifetime remains bounded by root metadata.

Root rotation requires one exact transition to satisfy both old and new root thresholds, version exactly old + 1, fixed trust scope and non-decreasing bootstrap threshold floors.

Policy heads are signed by the separate policy-head role, bind exact root/policy generation/predecessor/lifetime and are capped at 30 seconds. Positive root/head objects are non-deserializable.

## Native durable trusted-state candidate

`mycelix-authority-coordinator-release-key-policy-state` now closes the ordinary restart/lost-update boundary above #341.

The central distinction is:

`serialized trusted-state bytes != positive authority`.

Positive authority comes only from a configured `TrustedCoordinatorReleaseKeyPolicyStore` loading its own fixed state path under an exclusive transaction lock.

Normal operations do **not** accept caller-supplied:

- previous trusted state;
- previous root;
- root pin;
- previous policy head;
- previous state generation; or
- previous state digest.

### First-use bootstrap

`bootstrap_from_out_of_band_pin` is the only state-adapter operation accepting a root pin.

It is first-use only, refuses to overwrite any existing state path and delegates exact root-pin/threshold cryptography to #341 before durably creating generation-1 state.

The adapter still cannot prove how the production operator received the fingerprint. Root-pin delivery remains an independent provisioning responsibility.

### State-owned root rotation

`rotate_root` obtains the old root from persisted state rather than request input.

The candidate new root still requires exact old+new hybrid thresholds and the #341 version/scope theorem.

The durable adapter strengthens the pure theorem by ratcheting threshold floors upward:

```text
root_floor_next = max(root_floor_current, new_root.root_threshold)
policy_head_floor_next = max(policy_head_floor_current, new_root.policy_head_threshold)
```

A future root cannot lower a threshold that an earlier trusted root strengthened.

### Persisted latest policy head

The state stores the latest qualified policy-head checkpoint.

A later head is accepted only as:

- exact-current head revalidation while still live; or
- exact generation `current + 1` committing the persisted current head digest.

Older signed heads, alternate parents, skipped generations and generation restart deny after process restart because the predecessor is loaded from the state-owned path.

v0.1 deliberately remains stricter at genesis: generation 1 without a predecessor is accepted only while root version is 1. Operators therefore establish the initial policy head before the first root rotation. A later version may relax this only with an explicit persisted lineage-initialization/migration theorem.

## Filesystem continuity contract

v0.1 is Unix-only.

The immediate trusted-state directory must be a real owner-controlled directory with no group/other access. State and lock files must be regular, effective-user-owned, exact mode `0600`, and opened with `O_NOFOLLOW`/`O_CLOEXEC`.

All read/verify/advance transactions share one exclusive file lock.

State replacement is:

```text
same-directory create_new temporary file
→ write exact state
→ fsync temporary file
→ atomic rename over state path
→ fsync containing directory
```

The parser also enforces a 4 MiB maximum with a bounded read.

## Persisted state identity and clock floor

Every state commits exact protocol/profile, monotonically increasing state generation, predecessor-state digest, bootstrap root identity, current root identity, threshold floors, current head checkpoint and last trusted host time.

The current root digest and state self-digest are recomputed on load.

The persisted trusted time is a local clock floor: if host time later moves below the last accepted value, currentness qualification fails closed.

This can trade availability for security after severe backward clock adjustment.

## Persistence precedes positive authority

For release-key-policy currentness the causal order is now:

```text
exclusive lock
→ load persisted latest root/head
→ verify manifest + semantic policy + head lineage
→ strict policy-head Ed25519 AND ML-DSA-65 threshold crypto
→ post-crypto clock/liveness checks
→ construct next checkpoint
→ fsync + rename + directory-fsync trusted state
→ privately construct #307 currentness receipt
→ local #307 qualification
→ non-deserializable manifest-bound policy capability
```

If durable state advancement fails, no positive #307 capability escapes.

A manifest that does not already bind the exact semantic policy digest/profile/release authority fails before any state advancement.

## Local continuity is not full-image rollback resistance

The state self-digest and predecessor digest provide integrity/lineage checks under the trusted local-filesystem model. They are not an independent anti-rollback anchor against an attacker who can restore or rewrite the entire trusted state and recompute its unkeyed digest.

Therefore:

`crash-durable local monotonic state != hardware-monotonic rollback proof`.

Restoring an entire older machine/filesystem image can still restore an older internally valid state. Stronger resistance requires an independent monotonic anchor such as TPM/hardware-backed state, enterprise/device-management state, or a separately trusted append-only witness.

No such stronger property is claimed in v0.11.

## #307 currentness receipt remains private

After durable head advancement, the adapter privately constructs `VerifiedCurrentCoordinatorReleaseKeyPolicyProof` and immediately consumes it through #307.

The public successful result is only non-deserializable `QualifiedCoordinatorReleaseManifestKeyPolicy`; exact key selection then returns non-deserializable `QualifiedCoordinatorReleaseSigningKey`.

## Native hybrid release authentication candidate

`mycelix-authority-coordinator-release-hybrid-verifier` consumes only:

```text
candidate CoordinatorReleaseManifest
+ manifest-bound QualifiedCoordinatorReleaseSigningKey
+ detached hybrid signature bytes
```

It accepts no caller-selected trusted public key or positive proof metadata.

The v0.1 release signature contract is strict Ed25519 AND ML-DSA-65 over identical deterministic manifest/authority/policy/key-id bytes. The exact wires remain Ed25519 64-byte signature and ML-DSA-65 3309-byte signature, with the key wires fixed upstream at 32 and 1952 bytes respectively.

After both crypto checks the verifier samples host time, derives verifier-owned proof metadata, privately constructs #269 signature evidence and immediately consumes it locally. The public success is only non-deserializable `QualifiedCoordinatorReleaseRequirement`.

The RustCrypto hybrid construction remains experimental/unaudited; this gate does not upgrade that assurance status.

## Release currentness follows authentication

#275 requires an independently verified current release-registry head plus exact status of the authenticated #269 release at that same head.

Only `Active` qualifies. `Withdrawn`, `Superseded`, old-head `Active`, wrong policy or alternate profiles deny.

Thus:

`candidate release != rooted key-policy currentness != authorized key != release authentication != release currentness`.

## Exact deployment composition

#290 keeps target CellId selection, current authenticated release and conductor observation separate, specializes the release to the independently selected target agent and delegates exact whole-set coordinator equality to #262.

Missing, substituted, duplicate or unexpected coordinator code denies.

## Subject-bound pre/post stability

#298 requires a strictly later second exact deployment observation inside the first composition's evidence window, reconstructs the same release/target requirement, reruns #262 and binds the result to one admission subject + per-attempt nonce.

This proves no detected coordinator-code change across the observed interval. It is not a mutex/transaction/atomicity guarantee: `UpdateCoordinators` can still race after the post observation.

## Remaining independent live origins

The major unresolved boundaries are now:

- actual production delivery/protection of the initial root fingerprint;
- current release-registry head provenance/completeness;
- exact release status-at-head provenance;
- trusted target CellId selection provenance;
- real-conductor qualification of the native observer;
- native ownership of admission subject/attempt nonce and pre/post bracketing;
- post-observation coordinator-update race / effect atomicity; and
- hardware/enterprise monotonic anchoring if full-machine rollback resistance is required.

In particular:

`durable local key-policy continuity != release-registry currentness`.

`exact deployment stability != atomic effect admission`.

## Consumer rule

A future lifecycle/effect consumer needs independently:

1. fresh operational authority currentness from a direct local verifier call;
2. trusted exact target CellId selection;
3. candidate coordinator release semantics;
4. production-rooted and durably current release-key policy;
5. manifest-bound non-deserializable qualified hybrid signing key;
6. native strict hybrid release authentication;
7. independently proven current release-registry head/completeness and exact `Active` status-at-head;
8. native pre-deployment observation for the exact CellId;
9. exact target/current-release/observation composition through #290/#262;
10. native post observation and subject/attempt-bound stability through #298;
11. explicit coordinator-update race/atomicity policy;
12. final lifecycle/executor/effect-safety authority; and
13. an effect path consuming only in-process positive qualifications, never caller-supplied serialized positive receipts.

## Provisioning state

The v0.11 trusted-state candidate materially closes ordinary restart/rollback injection and crash-durability gaps, but it still does not satisfy the complete deployment/effect gate.

Until production root-pin delivery, release-registry provenance, target selection, real-conductor observation qualification, native pre/post orchestration and post-observation atomicity are independently qualified and bound into final effect admission:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may treat coordinator deployment evidence as atomic execution authority; and
- external effects remain disabled.

## Qualification still required

Highest-value remaining work after this tranche is:

1. current release-registry head/completeness verifier;
2. exact release status-at-head verifier;
3. trusted target CellId selection provenance;
4. real-conductor qualification of the native observer;
5. native pre/post admission orchestration;
6. explicit post-observation coordinator-update race / atomic effect boundary;
7. production root-pin ceremony/delivery integration;
8. optional TPM/hardware/enterprise monotonic rollback anchor; and
9. final lifecycle/executor/effect-safety binding with adversarial rollback/fork/wrong-key/forged-signature/wrong-cell/stale/extra/withdrawn/replayed-attempt/local-endpoint-impersonation/update-race tests.
