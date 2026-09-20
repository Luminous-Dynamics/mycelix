# MYC-ZKP-GH-TUF-003R — Replayable TUF Rollback Checkpoint v1

Status: **design contract / no durable persistence or trusted-root authority**

Issue: #2495  
Parent backend profile: #2490 / `ZKP_SIGSTORE_TUF_RUST_BACKEND_PROFILE.md`

## 1. Purpose

This contract defines the cross-run trusted-state boundary needed by the offline TUF backend.

The design preserves TUF's own recovery and rollback semantics rather than replacing them with simplified application arithmetic.

```text
prior admitted checkpoint
        ↓ exact previously verified metadata bytes
bounded ephemeral seed/staging store
        ↓
sigstore-tuf re-verification from pinned bootstrap root
        ↓
new untrusted retained repository
        ↓
complete refresh + target derivation
        ↓
checkpoint candidate
        ↓ separate durable promotion theorem
new admitted checkpoint
```

Still:

```text
checkpoint replay
!= durable promotion
!= trusted-root authority
!= attestation verification
!= receipt authentication
!= production/application authority
```

## 2. Why replay exact trusted state

TUF rollback protection is stateful.

A client needs knowledge of previously trusted metadata in order to distinguish an older but still correctly signed repository view from the newest state it has already accepted.

Version numbers alone are insufficient because:

- signatures must be re-verifiable;
- role keys/thresholds may rotate;
- snapshot/timestamp integrity relationships matter;
- exact target/delegation state matters;
- root recovery may invalidate prior lower-role trust.

Therefore:

```text
previous role versions
!= replayable TUF client state
```

The checkpoint must preserve or content-address the exact bytes required to reconstruct prior trusted state.

## 3. TUF-native lower-role reset semantics

The reviewed `sigstore-tuf` 0.11.0 `TrustedMetadataSet::update_root` replaces the trusted root and clears timestamp, snapshot and targets state after an admitted new root.

That is security-significant.

A root transition can rotate/recover lower-role authorities. An application-level rule such as:

```text
new_timestamp_version >= old_timestamp_version forever
```

can therefore be stricter than the TUF state machine in a way that incorrectly blocks legitimate recovery.

The Mycelix profile MUST let the verified TUF state machine determine whether prior lower-role state remains admissible after a root transition.

```text
root-authority transition
may begin a new lower-role rollback epoch
```

This is not permission to ignore rollback. It means the rollback floor is defined by the trusted metadata state *after* applying the admitted root transition, not by a global integer comparison detached from authority changes.

## 4. No-root-transition semantics

When the trusted root does not change, previously admitted timestamp/snapshot/targets state remains the rollback floor.

The prior checkpoint is replayed into the upstream state machine so that:

- older timestamp versions reject;
- timestamp-pinned snapshot rollback rejects;
- snapshot metadata rollback/removal rejects under the same authority epoch;
- stale/equal-version handling follows the TUF client workflow;
- expiry is evaluated using the new trusted-time input while rollback knowledge remains retained.

Mycelix must not duplicate these rules with a second parser.

## 5. Checkpoint types

The first profile should define three distinct types:

```rust
pub struct MycelixTufRollbackCheckpointV1 { /* durable admitted state */ }
pub struct CheckpointStagingStoreV1 { /* process-local replay + writes */ }
pub struct MycelixTufCheckpointCandidateV1 { /* complete but unpromoted successor */ }
```

Only the persistence layer may create a durable successor checkpoint.

Neither staging nor candidate types are trusted-root capabilities.

## 6. Durable checkpoint identity

`MycelixTufRollbackCheckpointV1` binds at minimum:

- schema version;
- profile ID;
- monotonic checkpoint sequence;
- predecessor checkpoint digest when non-genesis;
- trust-domain ID;
- TUF backend profile/source/dependency identity;
- updater-config identity;
- bootstrap-root identity;
- exact root-history state needed for replay;
- current timestamp state when present;
- current snapshot state when present;
- current top-level targets state when present;
- delegated-target evidence used for trusted-root derivation;
- selected trusted-root target digest/length;
- trusted-time evidence commitment used during admission;
- complete derivation-evidence commitment.

A presentation JSON form may exist, but canonical binary identity is separately defined and domain-separated.

## 7. Genesis checkpoint

A profile may begin with no prior durable checkpoint.

Genesis still requires an admitted bootstrap root through #2479/#2490.

```text
None prior checkpoint
!= trust-on-any-first-use
```

The exact bootstrap root is the root of TUF trust. The first successful derivation can create checkpoint sequence 0 only through the durable promotion theorem.

## 8. Exact replay bytes

The checkpoint needs exact bytes for every metadata object that must be replayed through `sigstore-tuf`.

The first bounded profile should retain directly or content-address at least:

- every admitted `root_history/<N>.root.json` from bootstrap successor through current root;
- current `root.json`;
- current `timestamp.json` when admitted;
- current `snapshot.json` when admitted;
- current `targets.json` when admitted;
- delegated-target metadata actually used in the derivation evidence.

Every object carries:

```text
logical identity
byte length
raw SHA-256
role/version metadata derived only after verification
```

The checkpoint digest commits to the complete ordered object manifest.

## 9. Root-history completeness

A current root blob alone is not sufficient for replay from an older pinned bootstrap.

For each version after the bootstrap through the checkpoint's current root, the checkpoint must either retain the exact sequential root bytes or reference them through a separately qualified immutable object store.

Missing an intermediate root fails checkpoint replay.

```text
bootstrap vN + final root vN+K
without admitted intermediate roots
!= replayable root chain
```

## 10. Ephemeral seed/staging store

`CheckpointStagingStoreV1` implements `sigstore_tuf::MetadataStore` but is not itself durable authority.

Conceptually:

```text
immutable_seed: prior admitted checkpoint bytes
staged_writes: current-attempt upstream-verified bytes
```

Read rule:

```text
if staged_writes contains name:
    return staged current-attempt value
else:
    return immutable admitted seed value
```

Write rule:

```text
store(name, bytes)
    -> stage in process-local attempt state only
```

No write operation modifies the durable checkpoint.

## 11. Why use an ephemeral store

The exact upstream updater supports `MetadataStore` specifically so a later run can seed trusted root/lower-role state and re-verify it.

Using an ephemeral seed store gives Mycelix:

- TUF-native rollback behavior across attempts;
- TUF-native root recovery behavior;
- no persistent write-through authority;
- complete discard-on-failure semantics;
- a closed place to capture verified current-attempt writes.

Thus:

```text
upstream store API
+ process-local staging
= useful verified-state bridge

upstream store API
!= durable Mycelix authority
```

## 12. Do not use ambient `FileStore` as admitted state

A shared filesystem `FileStore` is useful for ordinary clients, developer tooling and interoperability testing.

The production v1 checkpoint path does not treat ambient cache contents as durable Mycelix authority because:

- it has per-file rather than cross-refresh atomicity;
- another client/process may update it;
- it does not encode Mycelix profile/trust-domain/time/checkpoint semantics;
- it represents verified TUF cache state, not successful application admission.

A persistent upstream store may become an optimization in a future profile only after its relationship to the durable checkpoint is independently qualified.

## 13. Seed construction boundary

The staging store cannot be created from arbitrary caller-provided bytes labelled "verified".

Construction must consume either:

```text
None
```

for genesis, or an already admitted:

```text
MycelixTufRollbackCheckpointV1
```

The constructor verifies every checkpoint object commitment/resource bound before exposing it to the upstream store interface.

A deserialized checkpoint presentation object is not equivalent to the sealed admitted checkpoint capability.

## 14. Logical-name allowlist

Seed/staged metadata names are restricted to the admitted upstream cache namespace.

Examples:

```text
root.json
root_history/<positive-version>.root.json
timestamp.json
snapshot.json
targets.json
<encoded-delegated-role>.json
```

Reject:

- absolute names;
- `..` traversal;
- empty names;
- aliases that normalize to another admitted name;
- delegated-role collisions with top-level roles;
- names outside profile bounds.

The logical namespace is data identity, not host filesystem path authority.

## 15. Bounded staging resources

Freeze explicit limits for:

- maximum seed entries;
- maximum staged entries;
- maximum root-history entries;
- maximum delegated roles;
- per-role bytes;
- aggregate seed bytes;
- aggregate staged bytes;
- logical-name bytes;
- request/write trace entries.

All size arithmetic is checked.

An oversized checkpoint fails before constructing the upstream updater.

## 16. Panic discipline

The authority path should not rely on `Mutex::lock().unwrap()` semantics.

A Mycelix-owned staging store should use an interior-mutation implementation whose poison/error behavior is explicit and fail-closed.

A panic or poisoned state cannot silently become an empty cache, because that could remove a rollback floor.

If staging state becomes unavailable or internally inconsistent:

```text
-> abort current attempt
-> preserve prior durable checkpoint
```

## 17. Seed root replay

The upstream updater is constructed from the pinned bootstrap root, then attached to the staging store.

`Updater::with_store` walks:

```text
root_history/<bootstrap+1>.root.json
root_history/<bootstrap+2>.root.json
...
```

through `TrustedMetadataSet::update_root` until the staged checkpoint history ends or a replay fails.

The adapter then verifies that the resulting trusted root identity equals the checkpoint's claimed current root before continuing.

A malformed/tampered/missing history rejects replay.

## 18. Seed lower-role replay

During `refresh(now)`, upstream `seed_lower_from_store` attempts to re-admit:

```text
timestamp.json
snapshot.json
targets.json
```

through the current trusted root and integrity chain.

The adapter does not mark a checkpoint valid merely because those names exist in the staging store.

If checkpoint corruption causes a lower role not to replay when the checkpoint claims it should be present, checkpoint validation fails rather than silently accepting a weaker rollback state.

## 19. Root transition during new refresh

After prior state replay, the untrusted retained repository may contain newer root metadata.

If upstream `update_root` accepts a new root:

```text
old lower-role trusted state
-> cleared by upstream state machine
```

Subsequent lower-role metadata is validated under the newly trusted root.

The Mycelix adapter MUST NOT reapply the prior lower-role numeric floors outside this state machine unless a separately proven rule requires it.

## 20. Recovery fixture requirement

Qualification must include at least one independent valid recovery/key-rotation fixture where:

- a new root is properly dual-threshold authorized;
- lower-role keys are rotated/recovered;
- the old lower-role state is no longer trusted;
- the new lower-role state is valid under the new root;
- an intentionally naïve global lower-role version-floor rule would give a different answer.

The fixture establishes that the adapter follows TUF authority semantics rather than accidental implementation folklore.

## 21. No-root-change rollback fixture

Qualification must separately prove the opposite case:

- prior root is unchanged;
- prior timestamp/snapshot/targets replay successfully;
- repository supplies older-but-validly-signed lower metadata;
- upstream state rejects rollback.

Both cases are needed.

## 22. Staged write evidence

The Mycelix store records a bounded evidence trace for every `store(name, bytes)` call after upstream verification.

Each event commits to:

```text
operation sequence
logical name
byte length
raw SHA-256
seed/staged origin
backend attempt ID/profile
```

The trace proves what the upstream verifier attempted to persist into its TUF client state.

It does not independently prove the upstream verification was correct; backend qualification provides that theorem.

## 23. Repository request evidence

The separate `RetainedTufRepositoryV1` records every metadata/target request:

```text
operation sequence
metadata vs target
logical name/path
upstream max_length
returned object digest/length or absence
```

The final checkpoint candidate binds both repository-request and staging-write traces.

This makes the acquisition→verification→verified-state flow auditable without merging the two stores.

## 24. Delegated-role checkpoint evidence

Upstream `seed_lower_from_store` currently seeds only timestamp, snapshot and top-level targets.

Delegated-role bytes used for the selected `trusted_root.json` path remain checkpoint evidence even if they are not automatically replayed into the next updater.

Rollback protection for their metadata versions is still anchored by the trusted snapshot's pins when the snapshot is retained.

On the next target resolution, delegated-role bytes are re-fetched from the retained repository and re-verified against the trusted snapshot/delegation authority.

A future backend may seed delegated roles directly only after an upstream/profile-specific theorem defines that behavior.

## 25. Candidate construction

A `MycelixTufCheckpointCandidateV1` may be constructed only after all of:

- bootstrap/prior checkpoint replay succeeds;
- full upstream refresh succeeds;
- exact `trusted_root.json` target derivation succeeds;
- Public Good trust-domain/profile checks succeed;
- trusted-time policy succeeds;
- source/dependency/backend identity checks succeed;
- retained repository trace closes;
- staging-write trace closes;
- resulting verified metadata set is internally complete for the selected profile.

The candidate is immutable and non-durable.

## 26. Candidate does not mutate predecessor

The predecessor checkpoint remains the sole durable authority until promotion succeeds.

```text
candidate creation
!= checkpoint promotion
```

The candidate carries:

- expected predecessor sequence/digest;
- proposed successor sequence/digest;
- exact new replay state;
- exact derivation evidence digest.

## 27. Sequence rule

Genesis candidate proposes sequence `0`.

For non-genesis:

```text
candidate.sequence = predecessor.sequence + 1
candidate.predecessor_digest = predecessor.digest
```

Sequence is an application checkpoint sequence, not a TUF role version.

This avoids conflating two unrelated version domains.

## 28. Durable promotion theorem

Durable promotion is out of scope for this contract but its required atomic interface is frozen:

```text
promote(
    expected_current_checkpoint_digest,
    candidate
) -> new admitted checkpoint | conflict | storage failure
```

The storage backend must compare the durable current state with the expected predecessor and atomically advance only on exact match.

No last-writer-wins semantics.

## 29. Concurrent attempts

If attempts A and B both derive candidates from checkpoint N:

```text
A promotes N -> N+1
B then attempts promote(expected=N)
    -> conflict
```

B must restart verification from the newly current checkpoint if it still wishes to derive authority.

The two candidates may remain diagnostics but cannot both become authoritative successors.

## 30. Failed attempt semantics

Any failure before durable promotion discards staging/candidate state.

Examples:

```text
root verification fails
lower metadata fails
target verification fails
trusted time fails
trust-domain check fails
candidate creation fails
storage promotion fails
```

Result:

```text
prior durable checkpoint unchanged
```

This is required even if upstream staging already contains newly verified metadata.

## 31. Checkpoint replay self-test

Before a new durable checkpoint candidate is eligible for promotion, the backend should perform a replay self-test from its proposed checkpoint representation:

```text
candidate replay bytes
+ pinned bootstrap
-> reconstruct same verified current root/lower state
-> reproduce selected trusted-root target identity
```

This detects incomplete checkpoint serialization/manifest construction before durability.

The replay self-test uses a fresh ephemeral store/state machine instance.

## 32. Canonical checkpoint digest

Use a versioned domain-separated binary encoding, not generic JSON object serialization.

Suggested domain:

```text
MYCELIX_TUF_ROLLBACK_CHECKPOINT_V1\0
```

Bind fixed-order scalar fields plus sorted/typed object manifests and raw-byte commitments.

Any embedded raw bytes are length-prefixed.

Unknown checkpoint versions fail closed.

## 33. Authority surfaces

Suggested scopes:

```text
MycelixTufRollbackCheckpointV1
    -> ReplayableRollbackStateOnly

CheckpointStagingStoreV1
    -> EphemeralVerifiedStateOnly

MycelixTufCheckpointCandidateV1
    -> CheckpointCandidateOnly
```

All return false for:

```text
establishes_trusted_root_authority()
establishes_attestation_verification()
establishes_receipt_authentication()
grants_production_authority()
grants_application_authority()
```

`OfflineTufDerivedTrustedRootV1` remains a separate downstream capability.

## 34. Qualification tranches

### CKPT-A — canonical checkpoint representation

Prove deterministic bounded encoding/digest, unknown-version rejection and exact byte/object commitments.

### CKPT-B — seed-store replay

Prove exact root history and lower-role state are re-verified through the pinned upstream state machine.

### CKPT-C — unchanged-root rollback

Prove older lower metadata cannot displace the retained rollback floor.

### CKPT-D — root recovery semantics

Prove a valid root-authority transition can clear/re-authorize lower state without being blocked by naïve global lower-role version arithmetic.

### CKPT-E — failure discard

Prove all staged writes disappear after every pre-promotion failure class while predecessor remains unchanged.

### CKPT-F — candidate replay self-test

Prove candidate serialization reproduces the same TUF trusted state and selected target.

### CKPT-G — concurrency semantics

Prove stale-predecessor candidate promotion is rejected by the later persistence layer.

### CKPT-H — authority ceiling

Prove staging/candidate/checkpoint objects cannot convert into trusted-root/authentication/production authority.

## 35. Required adversarial corpus

At minimum:

1. tampered bootstrap-root commitment;
2. missing intermediate root history;
3. tampered intermediate root;
4. checkpoint current-root mismatch after replay;
5. tampered timestamp bytes;
6. tampered snapshot bytes;
7. tampered targets bytes;
8. missing checkpoint lower role claimed present;
9. unchanged-root timestamp rollback;
10. unchanged-root snapshot rollback;
11. valid root recovery/key rotation fixture;
12. invalid root transition attempting to reset rollback state;
13. unverified repository object inserted into seed state;
14. unsafe logical-name alias/traversal;
15. entry-count/per-object/aggregate-memory overflow;
16. poisoned/internal store state fails rather than dropping rollback floor;
17. target derivation failure after staged writes;
18. trusted-time failure after staged writes;
19. failed candidate replay self-test;
20. predecessor mismatch during promotion;
21. concurrent stale candidate conflict;
22. checkpoint sequence overflow;
23. canonical digest mutation sensitivity;
24. presentation JSON reordering does not alter identity;
25. checkpoint/candidate authority conversion attempt.

## 36. Relationship

```text
#2479/#2482  OfflineTufDerivedTrustedRootV1 theorem
#2490/#2493  sigstore-tuf backend profile
#2495        replayable checkpoint + staging-store theorem
future       durable checkpoint storage/CAS theorem
```

## 37. Final theorem

```text
admitted prior MycelixTufRollbackCheckpointV1
+ exact checkpoint replay bytes
+ ephemeral CheckpointStagingStoreV1
+ pinned bootstrap root
+ sigstore-tuf verified replay/refresh
+ complete trusted_root.json derivation
+ Mycelix profile/currentness/trust-domain checks
+ candidate replay self-test
    -> MycelixTufCheckpointCandidateV1

MycelixTufCheckpointCandidateV1
+ separately qualified atomic durable promotion
    -> next MycelixTufRollbackCheckpointV1
```

Neither result alone creates `OfflineTufDerivedTrustedRootV1`, attestation verification, receipt authentication, production admission or application authority.

## 38. Nonclaims

This contract implements no staging store, no checkpoint type, no persistence backend and no TUF verification. It freezes semantics only.

The initial implementation remains dependent on #2490 source/dependency closure and fresh independent execution qualification.