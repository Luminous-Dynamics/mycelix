# Authority Coordinator Release Registry v0.1 — Normative Invariants

Status: **offline-rooted complete-snapshot + local status-derivation theorem; durable latest-snapshot state and production root-pin delivery remain unimplemented**

## 1. Registry authority is independent of release signing keys

The registry root and registry-head role are not coordinator release signing keys.

The trust order is:

`independent root pin -> offline registry root threshold -> separate registry-head threshold -> complete current snapshot -> local exact status lookup -> #275 current-release qualification`.

Release signing keys cannot mark themselves Active, Withdrawn or Superseded.

## 2. Initial registry root is pinned out of band

`CoordinatorReleaseRegistryRootPin` must exactly match the candidate root digest/profile before a positive root can exist.

The pin's delivery/provenance is outside this pure theorem. Ordinary caller/RPC bytes are not sufficient production root-pin provenance.

The pin establishes identity only and never widens root lifetime.

## 3. Root and registry-head roles are separate

The root contains distinct hybrid-key roles for:

- offline root rotation/bootstrap; and
- short-lived complete registry-head signing.

Key IDs and actual Ed25519+ML-DSA-65 key material must be disjoint across roles.

## 4. v0.1 has one exact hybrid wire contract

Both roles require Ed25519 **AND** ML-DSA-65 over identical deterministic bytes.

Exact wires:

- Ed25519 public key: 32 bytes;
- Ed25519 signature: 64 bytes;
- ML-DSA-65 public key: 1952 bytes;
- ML-DSA-65 signature: 3309 bytes.

Exactly the configured threshold number of unique authorized live signers is accepted.

The RustCrypto hybrid construction remains experimental/unaudited; this theorem does not upgrade that assurance status.

## 5. Positive root is non-deserializable

`QualifiedCoordinatorReleaseRegistryRoot` derives `Serialize` but not `Deserialize`.

A serialized candidate root or root pin is data, not reusable positive authority.

## 6. Root rotation requires old + new thresholds

The exact same rotation statement must satisfy both current/old and candidate/new root thresholds.

Rotation must advance root version by exactly one, preserve root id, release authority and registry id, and may not lower the bootstrap threshold floors.

Threshold floors ratchet upward when a trusted new root strengthens them.

## 7. Root lineage survives rotation

A qualified root carries `root_lineage_digest`, initialized to the pinned bootstrap root digest and preserved through authorized root rotations.

A registry snapshot from another independently pinned root lineage cannot become the predecessor of this lineage merely because registry/policy labels match.

## 8. One snapshot is the complete registry state for one release-policy chain

`CoordinatorReleaseRegistrySnapshot` commits exact:

- root version;
- registry id;
- release authority;
- release-policy digest/profile;
- registry generation;
- exact predecessor-head digest or genesis;
- the complete ordered release-status record set; and
- bounded validity window.

The signed head digest is the #275 `REGISTRY_HEAD_PROFILE` identity.

## 9. Snapshot records are canonical and unique

Records must be strictly increasing by manifest digest. Duplicate or out-of-order manifest digests deny.

The complete snapshot is capped at 65,536 records in v0.1.

Each record binds exact manifest digest/profile, status, status-effective time and status reference, and its digest uses #275's exact `STATUS_RECORD_PROFILE`.

## 10. Snapshot currentness is short lived

A registry snapshot lifetime is at most 30 seconds.

The snapshot and participating registry-head signers must still be live after cryptographic verification. The final positive lease is bounded by the minimum root, snapshot and participating-signer horizons.

A short lease limits reuse; it does not prove no newer head exists unless the latest predecessor state is itself trusted.

## 11. Genesis and successor lineage are explicit

Without a predecessor, only root version 1 / registry generation 1 / no predecessor digest qualifies.

A changed successor must:

- advance generation by exactly one;
- commit the exact previous qualified head digest;
- stay in the same root lineage;
- stay in the same registry id/release authority; and
- stay in the same release-policy digest/profile chain.

Exact current-head revalidation is allowed while the same immutable snapshot remains live.

## 12. Records cannot disappear

Every record present in generation N must remain present in generation N+1.

This specifically prevents a registry publisher from erasing a withdrawal/supersession by simply omitting the release from a later otherwise-valid snapshot.

A release absent from the complete qualified snapshot cannot qualify as current.

## 13. Status history is monotone

An unchanged status must preserve its exact effective time and status reference.

A status change is allowed only from `Active` to `Withdrawn` or `Superseded`, with a strictly later effective time.

`Withdrawn` and `Superseded` are terminal in v0.1. Neither may become `Active` again and terminal states do not silently rewrite into each other.

A successor profile cannot rewrite the old record's manifest identity or move status-effective time backward.

## 14. Future-dated status records deny

Every record's `status_effective_at_ms` must be no later than the snapshot's `valid_from_ms`.

A registry head cannot claim a status transition that becomes effective only after the head itself begins being treated as current.

## 15. Positive snapshot is non-deserializable

`QualifiedCurrentCoordinatorReleaseRegistrySnapshot` derives `Serialize` but not `Deserialize`.

Its authority comes from exact root lineage + complete-snapshot validation + registry-head threshold verification, not from serialized positive bytes.

## 16. Detached #275 status receipts are removed from the intended live path

`qualify_current_release` performs status lookup directly inside the complete non-deserializable snapshot.

It privately constructs both:

- `VerifiedCurrentReleaseRegistryHeadProof`; and
- `VerifiedCoordinatorReleaseStatusAtHeadProof`

then immediately consumes them through local #275 `qualify_current_coordinator_release`.

No public live API accepts caller-supplied #275 head/status receipt bytes as positive registry authority.

## 17. Completeness is authority semantics, not a DHT heuristic

The theorem does not infer currentness from `latest record`, cache order, absence of later DHT entries or network discovery heuristics.

The signed object is the complete canonical snapshot for its release-policy chain. An authenticated release must have an exact record in that snapshot or qualification denies.

## 18. Registry root/snapshot does not authenticate release code

The registry states lifecycle status for an already authenticated #269 release.

It does not prove the release signature, installed conductor code, target CellId provenance, operational authority, deployment stability or effect permission.

## 19. Durable latest-snapshot state remains separate

The pure theorem proves the predecessor chain supplied as an in-process non-deserializable positive snapshot.

After restart it cannot by itself prove that the supplied predecessor is the durably latest head ever accepted by the device. A later native trusted-state adapter must persist root lineage, current root, current snapshot generation/head digest/records, threshold floors and trusted clock under the same crash-durable fail-closed model used by #347.

Until that exists, a short-lived signed head is not equivalent to durable latest-head proof.

## 20. Full-machine rollback remains outside the pure theorem

Even a future crash-durable registry state file will not by itself resist restoration of an entire old machine image. Hardware/enterprise monotonic anchoring or an independently trusted append-only witness remains a separate stronger property if required.

## 21. No Holochain or external effects

This crate contains no Holochain client/zome calls, coordinator update operation, lifecycle executor or external-effect path.

## 22. Provisioning remains blocked

Before effect-capable provisioning the stack still requires at minimum:

- production root-pin delivery/protection for key-policy and release-registry roots;
- durable latest release-registry snapshot state;
- trusted target CellId provenance;
- qualified real-conductor observation;
- native ownership of admission subject/attempt and pre/post bracketing;
- coordinator-update/effect atomicity semantics; and
- final lifecycle/executor/effect-safety admission.
