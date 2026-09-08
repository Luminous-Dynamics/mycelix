# Current Freshness — Coordinator Deployment Gate v0.10

Status: **offline-rooted release-key-policy currentness + native hybrid release authentication + release currentness + exact deployment composition + stability fence implemented as candidates; durable root/head state, remaining live provenance and atomic effect admission remain incomplete**

## Core deployment distinction

The v0.11 current-freshness theorem binds local `DnaHash`, but coordinator WASM can change without changing that hash. Therefore:

`same DNA != same coordinator implementation`.

Coordinator deployment evidence binds exact `CellId` = DNA hash + cell agent public key and the complete coordinator WASM closure. Coordinator self-reported code identity is never sufficient.

## Native conductor observation candidate

`mycelix-authority-coordinator-native-attestor` queries a loopback Holochain Admin API for one exact `CellId`, enumerates the complete installed coordinator set, extracts exact `WasmHash` values, samples time only after observation/extraction and owns a fixed five-second reuse cap.

It does not choose an approved release, target policy or deployment match.

## Candidate release semantics

`mycelix-authority-coordinator-release` defines the DNA/code-scoped release manifest. It commits the complete approved coordinator `WasmHash` closure plus DNA bundle/source/lock/toolchain/build-recipe/SBOM identities and release authority/policy identity.

It exists before signature authentication and does not choose an installation-specific agent.

## Release signing keys are not their own trust root

The release key-policy chain is now explicitly independent of the release signing keys it authorizes.

The intended trust order is:

```text
out-of-band production root fingerprint
        ↓
offline hybrid root threshold
        ↓
separate hybrid policy-head threshold
        ↓
short-lived parent-linked current policy head
        ↓
semantic CoordinatorReleaseKeyPolicy
        ↓
manifest-bound QualifiedCoordinatorReleaseSigningKey
        ↓
#326 hybrid release authentication
```

Release signing keys have no authority to create/rotate the offline root or declare their own key policy current.

## Offline-rooted key-policy currentness candidate

`mycelix-authority-coordinator-release-key-policy-root` defines a minimal TUF-like root/currentness theorem specialized to coordinator release-key policy.

The root commits exact:

- root identity/version;
- release authority;
- release-key-policy id;
- offline root hybrid-key set + threshold;
- distinct policy-head hybrid-key set + threshold; and
- root validity window.

Both roles use exact Ed25519 + ML-DSA-65 AND-composition. Root and policy-head key IDs/material must be disjoint.

### Bootstrap

The initial root must exactly match an out-of-band pinned canonical root digest/profile and must satisfy its configured offline root threshold.

The pin establishes identity only. It does not widen the root lease.

The pure theorem cannot prove where that fingerprint came from. A future native bootstrap must source it from independently managed trusted state, never ordinary caller/RPC input.

### Root rotation

One exact transition must satisfy both:

- the current/old root threshold; and
- the candidate/new root threshold.

In-band rotation must advance root version by exactly one, preserve root id/release authority/policy id, and may not reduce the bootstrap-pinned root or policy-head threshold floors.

### Current policy heads

Policy heads are signed by the root's **separate policy-head role**, never by release signing keys.

A head binds exact root version, semantic policy digest/profile, policy generation, predecessor digest and lifetime. The v0.1 head lifetime is at most 30 seconds.

A predecessor-free head is legal only at root version 1 / policy generation 1. Later heads must increase generation by exactly one and commit the exact qualified predecessor head digest. Rotating the root cannot restart policy generation at 1.

The positive root/head objects are non-deserializable.

## Durable rollback/fork protection is still missing

The pure root/head theorem proves only the lineage represented by its in-process non-deserializable predecessor object. It does **not** prove that a supplied predecessor remains the durably latest trusted device state after reboot, snapshot restore or local-state rollback.

A production native trusted-state adapter is still required. It must at minimum:

- bootstrap once from an independently supplied root fingerprint;
- reject caller-selected root/pin input during normal operation;
- persist exact root/head identities and generations;
- serialize read/verify/advance under an exclusive lock;
- atomically replace trusted state and fsync file + containing directory;
- reject root/head rollback or fork substitution; and
- treat severe trusted-clock rollback as a security fault.

No TPM/hardware monotonic counter is claimed yet.

## #307 currentness receipt is now a private compatibility projection

A cryptographically qualified current policy head privately constructs `VerifiedCurrentCoordinatorReleaseKeyPolicyProof` and immediately consumes it through #307 `qualify_manifest_key_policy`.

The intended live path therefore does not accept caller-supplied current-policy receipt bytes as positive authority.

The #307 positive result remains manifest-bound and non-deserializable, and exact key selection produces a non-deserializable `QualifiedCoordinatorReleaseSigningKey`.

## Native hybrid release authentication candidate

`mycelix-authority-coordinator-release-hybrid-verifier` consumes only:

```text
candidate CoordinatorReleaseManifest
+ manifest-bound QualifiedCoordinatorReleaseSigningKey
+ detached hybrid signature bytes
```

It accepts no caller-selected trusted public key or positive proof metadata.

The verifier pins:

- `ed25519-dalek = 2.2.0`;
- `ml-dsa = 0.1.1`.

Exact signature wires are Ed25519 64 bytes + ML-DSA-65 3309 bytes. Both components verify the same deterministic manifest/authority/policy/key-id message. Ed25519 uses strict verification; no classical-only/PQ-only/fallback path exists.

The existing Luminous hybrid construction remains experimental/unaudited; this gate does not upgrade its assurance status.

After both crypto checks succeed, the verifier samples host time, derives its own signature reference, privately constructs #269 signature evidence, and immediately consumes it through local #269 qualification. The public successful result is only non-deserializable `QualifiedCoordinatorReleaseRequirement`.

## Release currentness follows authentication

#275 requires an independently verified exact current release-registry head plus exact status of the authenticated #269 release at that same head.

Only `Active` qualifies. `Withdrawn`, `Superseded`, old-head `Active`, wrong policy or alternate v0.1 profiles deny.

Thus:

`candidate release != key-policy currentness != authorized key != signature authentication != release currentness`.

## Exact target/release/observation composition

#290 keeps target CellId selection, current authenticated release and conductor observation separate, specializes the release to the independently selected target agent and delegates exact whole-set coordinator equality to #262.

Missing, substituted, duplicate or unexpected installed coordinator code denies.

## Subject-bound pre/post stability

#298 requires a strictly later second exact deployment observation inside the first composition's evidence window, reconstructs the same release/target requirement, reruns #262 and binds the result to one admission subject + per-attempt nonce.

This proves no detected coordinator-code change across the observed interval. It is **not** a mutex/transaction/atomicity guarantee; `UpdateCoordinators` can still race after the post observation.

## Remaining independent live origins

The following remain unresolved or only candidate-qualified:

- production out-of-band root-pin provenance;
- durable monotonic root/head trusted state;
- release-registry current-head provenance/completeness;
- exact release status-at-head provenance;
- target CellId selection provenance;
- real-conductor qualification of the native observer;
- native ownership of admission subject/attempt nonce and pre/post bracketing; and
- post-observation update-race/effect atomicity semantics.

In particular:

`offline-rooted crypto != durable latest-policy state`.

`real release crypto != release-registry currentness`.

`exact deployment stability != atomic effect admission`.

## Consumer rule

A future lifecycle/effect consumer needs independently:

1. fresh v0.11 operational currentness from a direct local verifier call;
2. exact stable/dynamic deployment evidence + lease;
3. trusted exact target CellId selection;
4. candidate coordinator release semantics;
5. production-rooted + durably current release-key policy;
6. manifest-bound non-deserializable qualified hybrid signing key;
7. native strict hybrid authentication returning only local #269 authenticated release;
8. independently proven #275 current release-registry head/completeness and exact status-at-head;
9. native pre-deployment observation for the exact CellId;
10. exact target/current-release/observation composition through #290/#262;
11. native post observation and subject/attempt-bound stability through #298;
12. explicit native coordinator-update race/atomicity policy;
13. later lifecycle/executor/effect-safety authority; and
14. an effect path consuming only in-process positive qualifications, never caller-supplied serialized positive receipts.

## Provisioning state

The v0.10 root/currentness theorem still does not satisfy the whole deployment gate.

Until production root-pin provenance, durable root/head trusted state, release-head/status provenance, real-conductor observer qualification, target-selection provenance, native pre/post orchestration and post-observation race semantics are qualified and bound into final lifecycle/effect admission:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may interpret deployment evidence as coordinator-code-attested atomic authority; and
- external effects remain disabled.

## Qualification still required

Highest-value remaining work is now:

1. native persistent trusted-state adapter for the pinned release-key-policy root + monotone policy head;
2. current release-registry head/completeness verifier;
3. exact release status-at-head verifier;
4. trusted target-cell selection provenance;
5. real-conductor qualification of the native observer;
6. native pre/post admission orchestration;
7. explicit post-observation update-race/atomic effect boundary; and
8. final lifecycle/executor/effect-safety binding with adversarial rollback/fork/wrong-key/forged-signature/wrong-cell/stale/extra/withdrawn/replayed-attempt/local-endpoint-impersonation/update-race tests.
