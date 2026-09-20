# MYC-ZKP-GH-TUF-002R — `sigstore-tuf` Rust Backend Profile v1

Status: **backend selection/design contract / no trusted-root or authentication authority**

Issue: #2490  
Parent theorem: #2479 / `ZKP_OFFLINE_TUF_DERIVED_TRUST_ROOT_CONTRACT.md`

## 1. Purpose

This contract freezes the first Rust backend profile for deriving a candidate Sigstore trusted root under the stronger offline TUF theorem.

Initial backend candidate:

```text
crate: sigstore-tuf
version: 0.11.0
crates.io checksum:
eedac50883a917b7b434db22e2e6e853ace8c00f4a9c27f53e1e9c87e6d89fe4
upstream repository: sigstore/sigstore-rust
reviewed workspace release commit:
ef17cacdbd357befea4c1c768ef02ed9bf52672c
```

At that upstream release commit, the workspace version is `0.11.0` and `crates/sigstore-tuf/Cargo.toml` uses `version.workspace = true`.

These identities define a **candidate backend lineage**. They do not by themselves establish Mycelix source closure or dependency qualification.

```text
crate name + version + checksum + upstream commit
!= qualified vendored/source closure
```

QUAL-001D / #1335 or a successor must independently bind the exact dependency/source closure used by a production build.

## 2. Why `sigstore-tuf`

The reviewed 0.11.0 source provides the exact primitives required by #2479:

- TUF/securesystemslib-compatible canonical JSON;
- GitHub `tuf-on-ci` interoperability;
- declared TUF key IDs treated as producer-chosen identifiers;
- self-signed bootstrap-root verification;
- sequential dual-threshold root rotation;
- timestamp threshold/rollback/expiry checks;
- snapshot hash/length/version/rollback/expiry checks;
- targets and delegated-target threshold/integrity/expiry checks;
- transport-free `TrustedMetadataSet`;
- pluggable `Repository` transport;
- caller-supplied `jiff::Timestamp` for expiry evaluation;
- per-role download bounds;
- exact target hash/length verification.

The upstream crate documentation explicitly identifies a GitHub-TUF incompatibility in `tough` caused by recomputing key IDs from complete key objects containing producer-specific fields. This backend profile therefore does not use `tough` unless that incompatibility is independently resolved in a future profile.

## 3. Dependency profile

The first implementation candidate should request the crate without its network transport features:

```toml
sigstore-tuf = { version = "=0.11.0", default-features = false }
```

The production authority path must contain no `HttpRepository` construction and no reqwest/TLS fallback supplied by application code.

A build that enables network-fetch features belongs to a different backend profile even if the crate version is identical.

```text
same crate version
+ different Cargo feature set
!= same verifier backend identity
```

The exact Cargo feature/dependency closure must be captured by QUAL-001D or its successor.

## 4. Two-state architecture

Mycelix separates TUF verification state from application admission state.

```text
TUF verified state
!= Mycelix admitted rollback checkpoint
```

### 4.1 Ephemeral TUF verification state

`sigstore_tuf::TrustedMetadataSet` and `Updater` hold the state of one verification attempt.

This state is allowed to evolve role by role according to TUF semantics.

### 4.2 Mycelix rollback/admission state

`MycelixTufRollbackCheckpointV1` is a separate persistent authority input.

It advances only after the complete Mycelix derivation succeeds.

A verified root or timestamp cached/staged during an attempt does not by itself advance the Mycelix checkpoint.

## 5. Do not attach persistent `FileStore` on the authority path

`sigstore-tuf` intentionally supports write-through caching: after a metadata object is verified, the updater may write it to an attached `MetadataStore`, and a later run seeds trusted lower-role state from that store.

That is valid standard TUF client behavior.

The initial Mycelix production profile deliberately does **not** attach a persistent store to `Updater`.

Reasons:

1. application admission remains distinct from incremental verified cache state;
2. a failed later phase cannot be confused with a committed Mycelix checkpoint;
3. retained acquisition bytes and trusted prior state remain separate types;
4. checkpoint persistence can receive its own atomicity/rollback qualification.

`FileStore` may still be used by non-authoritative developer/interoperability profiles.

## 6. Untrusted retained repository

Implement:

```rust
RetainedTufRepositoryV1
```

as a Mycelix-owned implementation of `sigstore_tuf::Repository`.

It adapts exact retained acquisition bytes to the upstream transport interface.

Its authority is:

```text
UntrustedAcquisitionTransportOnly
```

It never becomes a `MetadataStore` and never claims that its bytes have already been verified.

## 7. Retained repository manifest

The repository adapter receives a closed manifest such as:

```text
RetainedTufObjectV1 {
    logical_name,
    object_kind,
    byte_len,
    sha256,
    acquisition_ref,
}
```

Allowed object kinds are explicit, for example:

```text
RootMetadata
TimestampMetadata
SnapshotMetadata
TargetsMetadata
DelegatedTargetsMetadata
Target
```

The manifest must be unique by admitted logical identity.

Reject:

- duplicate logical names;
- absolute/path-traversing logical names;
- unmanifested files;
- manifest/byte digest mismatch;
- manifest/byte length mismatch;
- role/type mismatch;
- unsupported names or encodings under the selected TUF profile.

## 8. Bounded reads

Every `Repository` fetch receives an upstream `max_length`.

The adapter must enforce this bound while producing the returned byte vector.

```text
retained object length > max_length
-> reject before returning bytes
```

Do not allocate/read arbitrary-size files and only check the bound afterward.

The adapter additionally enforces Mycelix absolute profile maxima. The effective limit is the minimum of the upstream request bound and the Mycelix profile bound.

## 9. No path authority

Filesystem paths are transport details only.

Preferred v1 execution materializes or opens retained objects under the qualified offline handoff profile from #2319/#2323/#2482.

The repository adapter binds exact bytes/digests; it does not infer trust from:

- pathname;
- owner name;
- mode bit;
- extension;
- containing directory name.

Path/race-resistant open semantics remain a separate handoff theorem.

## 10. Bootstrap construction

Construct the updater as:

```text
exact admitted RetainedTufRepositoryV1
+ exact admitted bootstrap-root bytes
+ exact UpdaterConfigV1
-> Updater
```

`Updater::new()` self-verifies the pinned bootstrap root to its own root threshold.

The bootstrap root identity is also independently bound in the Mycelix profile and checkpoint.

Self-signature correctness does not replace out-of-band bootstrap-root admission.

## 11. Exact updater limits

Do not use upstream defaults as an implicit long-term security contract.

Define and version a Mycelix profile:

```text
MycelixSigstoreTufUpdaterConfigV1
```

that freezes:

- root maximum bytes;
- timestamp maximum bytes;
- snapshot maximum bytes;
- targets/delegated-target maximum bytes;
- target maximum bytes;
- maximum root rotations per refresh;
- maximum delegation traversal steps/depth according to the upstream semantic field.

Changing any limit changes backend/profile identity.

## 12. Trusted-time adaptation

The upstream API accepts one `jiff::Timestamp` for expiry checks.

Mycelix #2192/#2346 yields a trusted interval rather than a caller wall-clock instant.

Conservative v1 rule:

```text
upstream_expiry_now = TrustedCurrentTimeIntervalV1.upper_bound
```

A metadata object accepted as unexpired at the upper bound is unexpired throughout the trusted interval.

The authority path MUST NOT call:

```rust
jiff::Timestamp::now()
```

or derive `now` from host `SystemTime`.

If the trusted upper bound cannot be represented exactly/admissibly as a `jiff::Timestamp`, fail closed.

## 13. Refresh sequence

The adapter invokes the upstream workflow in its specified order:

```text
root chain
-> root expiry check
-> timestamp
-> snapshot
-> top-level targets
```

The exact 0.11.0 upstream implementation routes every candidate through `TrustedMetadataSet` before considering it trusted.

No Mycelix application checkpoint mutation occurs during this sequence.

## 14. Root rotation

The reviewed upstream state machine requires each new root version to be exactly the previous version plus one and verifies the new root against both the old root authority and the new root's own authority.

Mycelix records:

- bootstrap root version/digest;
- final trusted root version/digest;
- exact retained intermediate root byte identities requested by the repository adapter.

A skipped root version or missing intermediate root must fail rather than jump directly to a later root.

## 15. Timestamp state

After refresh, `Updater::trusted().timestamp()` must exist for a positive candidate.

The checkpoint candidate records at least:

- timestamp version;
- exact retained timestamp SHA-256/length;
- timestamp expiry value;
- pinned snapshot version/commitment information needed by the checkpoint profile.

The exact retained bytes are recorded from the repository adapter's closed request/evidence trace; the parsed version/expiry are taken from the verified `TrustedMetadataSet` state.

## 16. Snapshot state

`Updater::trusted().snapshot()` must exist.

Record:

- snapshot version;
- exact retained snapshot SHA-256/length;
- snapshot expiry;
- commitment to the metadata pins relevant to the selected target lineage.

The Mycelix checkpoint comparison independently rejects snapshot rollback relative to the prior admitted checkpoint even if the current updater was intentionally created from the older bootstrap root.

## 17. Targets state

`Updater::trusted().targets()` must exist after refresh.

For top-level and any delegated roles used to resolve `trusted_root.json`, record:

- role name;
- verified version;
- exact retained metadata SHA-256/length;
- delegator relationship;
- terminating/path-match semantics relevant to the chosen traversal;
- target pin used for the selected target.

The custom repository records the exact metadata objects requested during traversal, while the upstream trusted state supplies the verified role payloads.

## 18. Target derivation

Resolve the exact admitted target name:

```text
trusted_root.json
```

using upstream `get_targetinfo` / `get_target` semantics under the same trusted-time upper bound.

The selected target bytes must pass upstream pinned length/hash verification.

Mycelix additionally records:

- selected target logical path;
- exact target byte length;
- raw SHA-256;
- role/delegation lineage commitment;
- trust-domain profile.

A successful target download/lookup establishes only a verified TUF target candidate.

## 19. No network fallback

The production offline backend is complete only when the binary/dependency profile contains no network fetch implementation reachable from the authority path.

Tests must prove missing retained metadata/targets return an explicit error rather than:

- HTTP fetch;
- GitHub API access;
- DNS lookup;
- mirror fallback;
- embedded-target fallback not explicitly admitted by the profile.

## 20. Verified candidate capability

Positive backend verification yields an opaque object conceptually:

```rust
pub struct SigstoreTufVerifiedRootCandidateV1 {
    // private
}
```

Authority:

```text
TufVerificationOnly
```

It binds or retains:

- backend profile ID;
- exact source/dependency identity;
- bootstrap-root identity;
- trusted-time evidence commitment;
- final root state;
- timestamp state;
- snapshot state;
- targets/delegation lineage;
- selected trusted-root target bytes/digest;
- repository request/evidence trace;
- updater-config identity.

It is not deserializable into the sealed capability and has no public constructor.

## 21. Mycelix rollback checkpoint

The prior admitted checkpoint is an independent input:

```text
Option<MycelixTufRollbackCheckpointV1>
```

The candidate is checked against it only after complete TUF verification and target derivation.

At minimum reject:

- final root version below prior root version;
- timestamp version below prior timestamp version;
- snapshot version below prior snapshot version;
- selected trust-domain change;
- bootstrap/profile/backend substitution not authorized by an explicit migration theorem;
- selected target lineage inconsistent with the new verified snapshot/targets state.

Equal versions may be permitted only when the exact policy defines a no-update/current-state case and currentness is independently established.

## 22. Atomic Mycelix checkpoint promotion

A successful TUF candidate does not mutate the durable checkpoint itself.

Use an explicit promotion boundary:

```text
verified candidate
+ prior checkpoint
+ checkpoint policy
+ durable compare-and-swap / append-only persistence theorem
    -> new MycelixTufRollbackCheckpointV1
```

Failure after TUF verification but before durable promotion leaves the prior checkpoint authoritative.

Failure during a new verification attempt cannot make a partially updated staging/cache state authoritative.

## 23. Why upstream `FileStore` is not the checkpoint

The exact upstream updater writes verified metadata through role by role when a store is attached and later seeds trusted state from that store.

That behavior is correct for a TUF client cache.

It does not express Mycelix's additional proposition:

> all backend/profile/trust-domain/time/target/persistence checks for this application-level root admission completed successfully as one admitted derivation.

Therefore:

```text
verified FileStore contents
!= MycelixTufRollbackCheckpointV1
```

## 24. Source identity

Initial reviewed upstream source candidate:

```text
repo: sigstore/sigstore-rust
commit: ef17cacdbd357befea4c1c768ef02ed9bf52672c
commit message: chore: release v0.11.0 (#150)
workspace tree: 965160b7b635b2a4848dfac49446a082225d1b0f
workspace Cargo.toml blob:
5509e97d1b1c977f00b5ffdca78c11a75b2496b2
sigstore-tuf Cargo.toml blob:
c079e51966706dfe39f86037c266ed17aa9d5b25
registry version: 0.11.0
registry checksum:
eedac50883a917b7b434db22e2e6e853ace8c00f4a9c27f53e1e9c87e6d89fe4
```

The release commit is GitHub-signature verified.

A later source-closure gate must still reconcile the exact crates.io archive/vendor bytes with the reviewed source and dependency closure.

## 25. Upstream source invariants relied upon by v1

The adapter qualification must pin and independently test the upstream semantics it relies on, including:

- root version increments are sequential;
- root transition requires old-root and new-root thresholds;
- timestamp rollback and pinned-snapshot rollback are rejected;
- timestamp expiry is checked;
- snapshot length/hash/version is bound by timestamp;
- snapshot metadata rollback/removal protections hold;
- snapshot expiry is checked;
- targets length/hash/version is bound by snapshot;
- targets signatures use correct root/delegator authority;
- delegated roles cannot reuse top-level role names;
- target length/hash verification occurs before returning target authority;
- repository fetch bounds are honored;
- no attached store means no upstream persistent-cache authority path.

A dependency update must requalify these assumptions.

## 26. Differential/conformance oracles

Do not qualify the backend only against itself.

Use independent fixtures/oracles from at least two of:

- Python TUF / `securesystemslib`;
- `go-tuf` / `sigstore-go`;
- GitHub's `tuf-on-ci` repository fixtures;
- official TUF specification vectors;
- current Public Good/GitHub TUF metadata snapshots captured as evidence.

The purpose is semantic compatibility, not a winner/implementation ranking.

## 27. Qualification tranches

### TUF-RUST-A — source/dependency closure

Prove exact crate archive, reviewed upstream revision, Cargo feature set, transitive dependency closure and Rust toolchain identity.

### TUF-RUST-B — repository transport

Prove closed retained-object lookup, no network, path/manifest rejection and per-fetch resource bounds.

### TUF-RUST-C — root chain

Prove bootstrap self-signature, threshold transitions, sequential rotation, bounds and trust-domain separation.

### TUF-RUST-D — lower roles

Prove timestamp/snapshot/targets/delegation signature, version, expiry, integrity and mix-and-match behavior against independent fixtures.

### TUF-RUST-E — trusted time

Prove the trusted interval upper bound is the only authority `now`; wall-clock perturbation cannot affect outcome.

### TUF-RUST-F — target derivation

Prove exact `trusted_root.json` selection and target hash/length binding.

### TUF-RUST-G — rollback checkpoint

Prove previous admitted state rejects older valid metadata and failed attempts never advance the Mycelix checkpoint.

### TUF-RUST-H — authority ceiling

Prove the candidate cannot deserialize/default/convert into `OfflineTufDerivedTrustedRootV1`, attestation verification, receipt authentication or production authority.

## 28. Required failure corpus

At minimum include:

1. malformed bootstrap root;
2. valid but wrong trust-domain bootstrap root;
3. root threshold failure;
4. root version skip;
5. root rollback;
6. excessive root-rotation chain;
7. timestamp signature failure;
8. timestamp version rollback;
9. snapshot version rollback via timestamp;
10. timestamp expiry at trusted interval upper bound;
11. snapshot digest mismatch;
12. snapshot length mismatch;
13. snapshot version mismatch;
14. snapshot metadata rollback/removal;
15. targets threshold failure;
16. targets digest/length/version mismatch;
17. delegated role threshold failure;
18. delegated role top-level-name collision;
19. delegation path substitution;
20. target digest mismatch;
21. target length mismatch;
22. missing retained metadata with no network fallback;
23. oversized retained object;
24. duplicate/unmanifested object;
25. host-clock changes with fixed trusted-time evidence;
26. Public Good/private profile substitution;
27. older valid metadata after newer Mycelix checkpoint;
28. failure after partial TUF verification leaves durable checkpoint unchanged;
29. source/checksum/feature drift changes backend identity;
30. candidate-to-authentication authority conversion attempt.

## 29. Migration rule

Any of the following requires a new backend/profile revision and fresh qualification:

- `sigstore-tuf` version change;
- crates.io checksum change;
- reviewed upstream source revision change;
- Cargo feature-set change;
- relevant transitive cryptography/parser dependency change;
- updater-limit change;
- bootstrap-root profile change;
- trust-domain change;
- trusted-time adaptation change;
- checkpoint semantics change.

## 30. Final theorem

```text
exact source/dependency closure
+ exact retained offline repository
+ admitted bootstrap root
+ exact updater limits
+ trusted-current-time upper bound
+ sigstore-tuf 0.11.0 verified metadata workflow
+ exact trusted_root.json target verification
    -> SigstoreTufVerifiedRootCandidateV1

SigstoreTufVerifiedRootCandidateV1
+ admitted prior rollback checkpoint
+ separately qualified atomic checkpoint promotion
    -> eligible input for OfflineTufDerivedTrustedRootV1
```

Neither transition establishes attestation verification, trusted-builder identity, authenticated qualification receipts, production admission or application authority.

## 31. Nonclaims

This document adds no Rust dependency to Mycelix, implements no repository adapter, performs no TUF verification, promotes no rollback checkpoint and claims no qualification PASS.

The implementation remains blocked on source/dependency closure, trusted-time implementation, rollback-checkpoint persistence and fresh independent execution qualification.