# Authority Coordinator Release Key-Policy Root v0.1 — Normative Invariants

Status: **pure/native cryptographic root + policy-head currentness theorem; out-of-band pin provenance and durable trusted-state persistence remain unimplemented**

## 1. Release signing keys are leaves, never the policy root

The coordinator release keys authorized by `CoordinatorReleaseKeyPolicy` have no authority over the trust root or policy-head role that authorizes them.

The trust chain is strictly:

`out-of-band root pin -> offline hybrid root threshold -> separate hybrid policy-head threshold -> current semantic release key policy -> manifest-bound release signing key -> #326 release signature authentication`.

Any dependency from this crate on #269 authenticated release, #275 current release, or #326 release authentication would recreate circular authority and is forbidden.

## 2. Initial root identity is pinned out of band

`CoordinatorReleaseKeyPolicyRootPin` identifies the exact canonical root digest/profile. `bootstrap_policy_root` requires exact equality before any positive root can exist.

Pure equality does **not** prove where the pin came from. A live deployment must obtain the initial pin from independently managed trusted state / an out-of-band ceremony. Caller/RPC-supplied root pins are not positive authority.

The pin establishes root identity only. It never widens root lifetime.

## 3. Root and policy-head roles are cryptographically separate

The root contains two distinct hybrid-key roles:

- offline root keys;
- policy-head currentness keys.

Key IDs and actual Ed25519+ML-DSA key material must be disjoint across those roles. A key may not simultaneously act as root and policy-head authority inside one root.

## 4. v0.1 uses one exact hybrid wire contract

Both roles use AND-composed:

`Ed25519 + ML-DSA-65`

with exact wire sizes:

- Ed25519 public key: 32 bytes;
- Ed25519 signature: 64 bytes;
- ML-DSA-65 public key: 1952 bytes;
- ML-DSA-65 signature: 3309 bytes.

Both signature components must verify over identical deterministic bytes. A future algorithm/wire change requires a versioned successor.

The RustCrypto construction remains experimental/unaudited; this theorem does not upgrade that assurance status.

## 5. Root bootstrap requires pin equality and exact threshold evidence

The candidate root must be live and match the out-of-band pinned digest/profile exactly.

The bootstrap message commits the exact canonical root identity. Exactly the configured root-threshold number of unique authorized live signers must be supplied, and every supplied signer must pass both Ed25519 and ML-DSA-65 verification.

`exact threshold evidence` is deliberate: extra signatures are not silently admitted into the evidence set and cannot alter signer identity or lease semantics. A future native adapter may canonicalize a larger valid signature collection into an exact threshold receipt before invoking this theorem.

The verification clock is sampled again after crypto and the root/signers must still be live.

## 6. Positive root is non-deserializable

`QualifiedCoordinatorReleaseKeyPolicyRoot` derives `Serialize` but not `Deserialize`.

It can arise only through bootstrap or authorized root rotation. Root-pin bytes or root metadata alone are not reusable positive authority.

## 7. Root rotation requires old + new threshold authorization

One exact root-rotation message commits both old and new root identities and versions.

Rotation must satisfy:

- current qualified root is live;
- new root version is exactly current + 1;
- root id, release authority and policy id do not change in band;
- old root threshold verifies the transition;
- new root threshold verifies the same transition;
- bootstrap root-threshold floor is not reduced;
- bootstrap policy-head-threshold floor is not reduced; and
- new root is live after cryptographic verification.

Compromise of only the old or only the newly introduced root keys is insufficient for rotation.

## 8. Root pin never becomes an infinite lease

A successfully pinned/self-threshold-authenticated root is reusable only until the root metadata's own `valid_until_ms`.

The out-of-band fingerprint is an identity anchor, not an authority-expiry source.

## 9. Policy heads are short lived

A `CoordinatorReleaseKeyPolicyHead` binds exact:

- root version;
- semantic policy digest/profile;
- policy generation;
- predecessor-head digest or explicit genesis;
- validity window.

The v0.1 head lifetime is capped at 30 seconds. This is a currentness/reuse bound, not proof of atomic immutability during the window.

## 10. Policy-head threshold is separate from root threshold

Policy heads are verified only against the root's dedicated `policy_head_keys` and `policy_head_threshold`.

Release signing keys are never considered policy-head signers.

Exactly the configured number of unique, authorized, live policy-head signers must supply valid Ed25519 AND ML-DSA-65 signatures.

## 11. Policy-head lineage is monotone and parent linked

A predecessor-free head is accepted only for root version 1, policy generation 1.

With a predecessor, generation must increase by exactly one and the new head must commit the exact predecessor-head digest. Policy id and release authority must remain identical.

A rotated root cannot restart policy generation at 1.

This proves in-process lineage over non-deserializable qualified predecessors. It does **not** by itself prove that a supplied predecessor is the durably latest device state after restart.

## 12. Durable rollback/fork protection remains a host theorem

The trusted current root/head state must eventually be persisted by a native adapter using authenticated, monotonic, crash-safe storage.

The intended live adapter must:

- bootstrap once from an independently supplied root fingerprint;
- reject caller-selected roots/pins during normal operation;
- persist exact qualified root/head identity and generation;
- serialize read/verify/advance transactions under an exclusive lock;
- atomically replace state and fsync file + containing directory;
- reject rollback to earlier root/head generations; and
- treat severe clock rollback as a security fault.

Hardware monotonic storage / TPM rollback resistance is not claimed by this pure theorem.

## 13. Current-policy evidence is privately projected into #307

`QualifiedCurrentCoordinatorReleaseKeyPolicyHead::qualify_manifest_key_policy` privately constructs #307's deserializable `VerifiedCurrentCoordinatorReleaseKeyPolicyProof` and immediately consumes it locally.

The intended live path does not accept caller-supplied `VerifiedCurrentCoordinatorReleaseKeyPolicyProof` bytes as positive authority.

## 14. Policy-head lease is monotone

The positive head lease is bounded by:

`min(qualified root, semantic policy, signed head, participating policy-head signer horizons)`.

No later #307 manifest/key-policy or signing-key qualification can widen that lease.

## 15. Root/currentness does not authenticate a release

A current key policy authorizes which key may authenticate one candidate release manifest. It does not itself prove a release signature, release registry status, installed coordinator code, deployment stability, lifecycle authority, effect safety or external-effect permission.

## 16. Provisioning remains blocked

Before effect-capable provisioning the stack still requires at minimum:

- out-of-band production root-pin provenance;
- native durable monotonic root/head state;
- qualified live #326 release signature authentication;
- live release registry-head/status provenance;
- trusted target-cell provenance;
- qualified conductor observations and composition;
- native pre/post admission orchestration;
- update-race/effect atomicity semantics; and
- final lifecycle/executor/effect-safety admission.
