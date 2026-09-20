# MYC-ZKP-GH-TUF-001R — Offline TUF-Derived Trusted Root v1

Status: **design contract / no attestation or authentication authority**

Issue: #2479  
Parent deployment profile: #2319 / `ZKP_NIXOS_OFFLINE_VERIFIER_PROFILE.md`

## 1. Purpose

This contract strengthens the production trust-root boundary for Mycelix qualification verification.

The networked acquisition phase is reduced to an **untrusted byte courier**. It may fetch candidate bytes, but it does not select or mint the root of trust consumed by the offline verifier.

The offline side derives the admitted Sigstore trusted-root material from a pinned TUF bootstrap root plus a verified retained metadata chain.

```text
network transport bytes
        ↓
untrusted retained TUF metadata + targets
        ↓
pinned TUF bootstrap root
+ trusted-current-time interval
+ rollback state
        ↓
offline TUF verification
        ↓
exact target hash/length verification
        ↓
OfflineTufDerivedTrustedRootV1
```

Still:

```text
OfflineTufDerivedTrustedRootV1
!= attestation verified
!= trusted signer/builder
!= authenticated qualification receipt
!= ProductionAdmissionV1
!= application authority
```

## 2. Why retained `trusted_root.jsonl` alone is insufficient

A retained root file is useful reproducibility evidence, but a networked producer of that file still influences the trust material supplied to the verifier.

The stronger profile therefore distinguishes:

```text
FetchedTrustedRootBytesV1
!= OfflineTufDerivedTrustedRootV1
```

A JSON document does not become a root capability because:

- its syntax is valid;
- a network fetch succeeded;
- its URL looked like a known Sigstore service;
- an online tool printed it;
- its digest matches a caller-provided digest.

The admitted root must be derived from the TUF trust chain itself.

## 3. Relationship to GitHub CLI

GitHub CLI currently supports TUF-backed trusted-root acquisition and allows a caller to provide a custom TUF mirror/root for `gh attestation trusted-root`. It also supports offline attestation verification through retained bundle bytes plus `--custom-trusted-root`.

This contract does not require `gh` to implement the offline TUF theorem.

Two implementation families are permitted after separate qualification:

### 3.1 Dedicated TUF derivation + offline `gh`

```text
qualified offline TUF verifier
    -> exact trusted_root.jsonl bytes
    -> offline gh attestation verify --custom-trusted-root ...
```

### 3.2 Native Rust Sigstore/TUF verifier

A later Rust backend may verify both the TUF chain and Sigstore bundle directly.

The authority theorem is the same regardless of backend.

## 4. Acquisition role: bytes only

The networked acquisition service may obtain candidate bytes for:

- GitHub attestation bundles;
- TUF root metadata required for admitted root rotation;
- timestamp metadata;
- snapshot metadata;
- targets metadata;
- admitted delegated-target metadata;
- the target object containing Sigstore trusted-root material;
- a fresh trusted-time anchor required by the currentness policy.

The acquisition output authority is:

```text
UntrustedAcquisitionTransportOnly
```

It must machine-report false for:

```text
establishes_tuf_trust()
establishes_trusted_root()
establishes_attestation_verification()
establishes_receipt_authentication()
grants_production_authority()
grants_application_authority()
```

The fetcher must not own the final trusted-root capability constructor.

## 5. Bootstrap root

The offline TUF verifier begins from an exact bootstrap root admitted through a separate out-of-band profile.

`TufBootstrapRootV1` must bind at minimum:

- trust-domain/profile ID;
- repository/mirror profile ID;
- exact root metadata bytes or canonical digest;
- root version;
- root-role key IDs;
- root-role threshold;
- supported TUF specification/profile version;
- the out-of-band acquisition/admission reference.

A URL alone is not bootstrap-root identity.

```text
well-known TUF URL
!= trusted bootstrap root
```

## 6. Trust-domain separation

Public Sigstore and GitHub-private Sigstore trust are different domains.

The Mycelix public-repository production profile must bind a specific Public Good TUF profile and must not accept a valid root from another domain merely because its target format is compatible.

```text
valid GitHub-private TUF root
!= valid Public Good root for public Mycelix profile
```

The trust-domain identity must survive every derived object and receipt.

## 7. Required retained metadata set

`RetainedTufRepositorySnapshotV1` should bind exact bytes, lengths and SHA-256 digests for every admitted metadata/target object.

At minimum:

- bootstrap/current root chain objects used in the update;
- timestamp metadata;
- snapshot metadata;
- targets metadata;
- every delegated-target metadata object actually traversed;
- the selected trusted-root target bytes.

Unknown, unused or unreferenced files in the handoff directory do not silently participate in trust.

## 8. Root update theorem

Root rotation/update must follow the admitted TUF root-update semantics.

The verifier must reject at least:

- insufficient signatures from the currently trusted root role;
- insufficient signatures from the candidate new root role where required by the selected TUF profile;
- invalid key IDs or thresholds;
- unsupported specification/profile versions;
- version rollback;
- forbidden version jumps when the selected client profile requires sequential root updates;
- root bytes that do not match the retained evidence digest.

Successful root rotation establishes only a new TUF root state. It does not yet establish current timestamp/snapshot/targets state.

## 9. Timestamp theorem

The timestamp role is the first freshness/rollback-sensitive online metadata boundary.

The verifier must establish:

```text
trusted timestamp-role keys/threshold
+ exact timestamp bytes
+ valid signatures
+ admitted version progression
+ trusted-current-time interval
+ metadata expiry interval
    -> AdmittedTufTimestampV1
```

A timestamp is rejected if the trusted current-time interval cannot establish that the metadata is unexpired under the selected policy.

Host wall-clock time alone is not authority.

## 10. Snapshot theorem

The snapshot metadata must be selected through the admitted timestamp metadata and checked for every required hash/length/version relationship defined by the profile.

The verifier rejects:

- snapshot rollback;
- version mismatch;
- hash mismatch;
- length mismatch;
- invalid snapshot signatures;
- expiry failure;
- metadata mixing from incompatible snapshots.

This prevents a mirror/fetcher from assembling a view from mutually inconsistent repository states.

## 11. Targets and delegation theorem

The verifier must establish the exact targets/delegation chain used to select the trusted-root target.

It must reject:

- invalid target-role signatures;
- unsupported or unexpected delegation;
- path selection outside the admitted target profile;
- metadata rollback;
- metadata expiry failure;
- target hash mismatch;
- target length mismatch;
- ambiguous target selection.

No target is trusted merely because its filename resembles `trusted_root.jsonl`.

## 12. Trusted-current-time dependency

TUF metadata expirations require a time theorem stronger than caller-supplied `SystemTime`.

This profile consumes #2192/#2346:

```text
TrustedCurrentTimeIntervalV1
```

The expiry evaluator must account for the interval rather than silently collapsing it to one guessed instant.

Conservative v1 rule:

```text
if the trusted-time interval overlaps or exceeds metadata expiry
-> currentness not established
```

A profile may later define more precise interval semantics, but it must remain fail-closed.

## 13. Historical Sigstore timestamps are not current time

A transparency-log or RFC3161 timestamp already present in an old attestation bundle may establish chronology for that signing event.

It does not establish verifier current time.

```text
historical signing timestamp
!= TrustedCurrentTimeIntervalV1
```

A fresh time anchor must be bound to the current acquisition/verification attempt when the policy requires live currentness.

## 14. Rollback state is persistent authority input

A stateless TUF signature verifier is insufficient for a current-root claim.

`TufRollbackStateV1` must retain the highest admitted versions/checkpoints needed by the selected profile, including at minimum the version state required to reject older timestamp/snapshot/root metadata after a newer state has been accepted.

The state must itself have an integrity and persistence theorem.

```text
signed + unexpired old metadata
!= current metadata after newer state was accepted
```

## 15. Rollback-state storage

The first production implementation must choose one explicit persistence profile, for example:

- append-only authenticated local journal;
- Xenia-signed monotonic checkpoint;
- TPM-backed monotonic/sealed state;
- separately qualified replicated Mycelix evidence state;
- another fail-closed monotonic persistence mechanism.

Plain mutable JSON with no rollback protection cannot establish `TufRollbackStateV1`.

The storage theorem remains separate from TUF signature verification.

## 16. Handoff byte identity

The fetcher-to-offline-verifier handoff remains evidence-only under #2319/#2323.

The TUF verifier must consume exact retained byte identities, not filenames as authority.

At minimum bind:

- relative logical object identity;
- length;
- raw SHA-256;
- acquisition manifest identity;
- handoff profile/version.

Path strings remain transport metadata.

## 17. Path/TOCTOU ceiling

Read-only file permissions and prior digest checks do not by themselves prove that a path cannot be substituted between checks and use.

This contract therefore keeps the stronger file-identity theorem separate.

Future admitted profiles may use:

- `openat2`-style directory-bounded no-symlink opening with retained file descriptors;
- fs-verity-backed immutable files;
- sealed `memfd` objects transferred through a qualified IPC boundary;
- content-addressed Nix-store import with separately qualified import semantics;
- another equivalent theorem.

None is inferred from a pathname or chmod bit.

## 18. Derived trusted-root object

The positive result should be an opaque type similar to:

```rust
pub struct OfflineTufDerivedTrustedRootV1 {
    // private fields
}
```

It should retain or commit to:

- profile/version;
- trust-domain ID;
- bootstrap-root identity;
- root-update lineage commitment;
- admitted timestamp identity/version;
- admitted snapshot identity/version;
- targets/delegation lineage commitment;
- exact trusted-root target digest/length;
- trusted-time evidence reference/digest;
- rollback-state before/after commitments;
- exact TUF verifier implementation/profile identity.

It must not implement `Default` and should not be deserializable into the authority-bearing capability.

A serializable evidence summary may exist separately.

## 19. Authority scope

Suggested machine-readable authority:

```text
TrustedRootDerivationOnly
```

Methods must return false for:

```text
establishes_attestation_verification()
establishes_trusted_builder()
establishes_receipt_authentication()
grants_production_authority()
grants_application_authority()
```

## 20. Offline attestation composition

Only after root derivation may the offline attestation verifier consume the resulting exact trusted-root material.

```text
OfflineTufDerivedTrustedRootV1
+ exact retained attestation bundle
+ exact canonical qualification subject
+ qualified offline verifier execution
    -> cryptographic attestation-verification candidate
```

Signer workflow trust, strict predicate binding, currentness and authenticated-receipt minting remain separate downstream theorems.

## 21. No credential inheritance into offline verification

The production offline verifier should require no GitHub API token once all required bundle/TUF bytes are retained locally.

The NixOS profile should therefore reject inherited `GH_TOKEN`, `GITHUB_TOKEN`, user GitHub config credentials and equivalent ambient network-auth material in the offline verifier service.

This materially reduces the impact of a verifier descendant even before stronger containment is considered.

## 22. Acquisition credential boundary

If the networked fetcher needs GitHub credentials, those credentials belong only to the acquisition service.

Preferred deployment properties:

- credential delivered through a bounded service credential mechanism rather than a broad inherited environment;
- fetcher has no cgroup-delegation authority;
- fetcher cannot mint verifier/authentication capability types;
- fetcher exits before offline verification begins where the profile permits;
- no credential bytes cross the immutable handoff.

## 23. Network isolation

The TUF verifier and attestation verifier run under the offline network boundary from #2319/#2323.

A verification implementation that silently fetches missing TUF metadata or trust material violates this profile.

Missing retained material must fail as missing evidence, not trigger online fallback.

## 24. Evidence digest

A future `OfflineTufDerivationEvidenceDigestV1` should be domain-separated and bind the exact semantic inputs, including:

- bootstrap root;
- retained metadata/target byte commitments;
- trusted-time commitment;
- rollback-state before/after commitments;
- selected target identity;
- TUF verifier implementation/profile;
- trust-domain profile;
- derived target digest.

JSON presentation ordering must not define evidence identity.

## 25. Qualification gates

### TUF-A — parser/canonical structure

Reject malformed metadata, duplicate/unknown authority-critical fields according to the selected TUF implementation profile, unsupported versions and oversized inputs.

### TUF-B — root rotation

Exercise valid and invalid root transitions, threshold failures, key rotation, rollback and forbidden version jumps.

### TUF-C — timestamp freshness

Exercise signature, rollback, expiry and trusted-time interval boundaries.

### TUF-D — snapshot consistency

Exercise snapshot hash/length/version/expiry and mixed-metadata attacks.

### TUF-E — targets/delegations

Exercise target selection, delegated roles, threshold failures, path ambiguity and target hash/length substitution.

### TUF-F — rollback persistence

Prove an older but cryptographically valid repository state is rejected after a newer state has been admitted.

### TUF-G — trust-domain separation

Prove Public Good and GitHub-private roots cannot satisfy one another's profiles.

### TUF-H — offline/no-fallback

Prove complete verification succeeds with network denied and missing retained metadata fails rather than refetching.

### TUF-I — authority ceiling

Prove the derived-root capability cannot convert into attestation/authentication/production authority.

## 26. Required adversarial corpus

At minimum reject:

1. wrong bootstrap root;
2. root-role threshold failure;
3. unauthorized root key addition/removal;
4. root rollback;
5. invalid sequential root update;
6. expired timestamp;
7. timestamp rollback;
8. timestamp evaluated with untrusted wall clock;
9. snapshot digest mismatch;
10. snapshot size mismatch;
11. snapshot rollback/version mismatch;
12. targets signature failure;
13. delegated-target signature failure;
14. target path substitution;
15. trusted-root target digest mismatch;
16. trusted-root target length mismatch;
17. private-instance root under Public Good profile;
18. valid historical metadata after newer rollback state exists;
19. missing metadata causing online fallback;
20. fetcher-produced root bytes being treated as authority without TUF derivation;
21. derived root being treated as attestation verification;
22. derived root being treated as receipt authentication.

## 27. Relationship to existing Mycelix layers

```text
#2069/#2071  GitHub verifier/trust-root modes
#2096/#2134  retained-root structural classification
#2192/#2346  trusted current-time theorem
#2295/#2317  race-safe process containment
#2319/#2323  NixOS offline verifier deployment
#2479        offline TUF-derived root authority
```

This contract strengthens the production path without invalidating the simpler retained-root profile. The simpler profile remains useful for interoperability/testing with a lower authority ceiling.

## 28. Final theorem

```text
pinned admitted TUF bootstrap root
+ exact retained TUF repository bytes
+ trusted-current-time interval
+ persistent rollback state
+ offline TUF verification
+ exact target hash/length
+ Public Good trust-domain policy
    -> OfflineTufDerivedTrustedRootV1

OfflineTufDerivedTrustedRootV1
    != attestation verified
    != trusted builder
    != authenticated qualification receipt
    != production admission
    != application authority
```

## 29. Nonclaims

This document implements no TUF parser, no cryptographic verifier, no trusted-time backend, no rollback-state store, no immutable-file backend, no Sigstore verifier and no qualification PASS.

The first implementation must independently choose and qualify those adapters rather than inheriting authority from this design document.