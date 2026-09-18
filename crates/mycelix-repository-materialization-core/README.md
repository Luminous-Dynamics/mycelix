# Repository Materialization Transaction V1

Status: candidate pure protocol. This document defines semantic publication identity and recovery classification only. It grants no repository write authority and does not qualify any particular publisher, artifact, product, or workflow.

## Why this exists

The repository has now demonstrated the same boundary in several independent evidence lines: candidate derivation may succeed while publication is delayed, a push can return an ambiguous transport result, two safe publishers can construct physically different Git commits for the same exact mutation, and an artifact can expire without making the underlying semantic candidate false.

Those are transaction-state questions, not product-semantics questions.

The protocol therefore separates:

```text
candidate semantics
  -> exact artifact binding
  -> semantic publication descriptor
  -> bounded compare-and-swap attempt
  -> independent remote observation
  -> typed publication outcome
```

from both candidate execution and repository mutation machinery.

## Semantic publication identity

V1 defines:

```text
SemanticPublicationIdV1 = SHA-256(canonical_preimage_v1)
```

The pure Rust core emits the canonical preimage but deliberately does not implement SHA-256 itself. An enclosing verifier must compute the digest with a qualified cryptographic implementation. This prevents a convenience hashing implementation from silently becoming the authority boundary.

The preimage commits to:

- protocol domain and version;
- repository `owner/name`;
- exact `refs/heads/*` target;
- exact expected parent object ID, including Git object-hash algorithm;
- bounded mutation class;
- a 32-byte policy commitment;
- the sorted exact mutation set;
- for each replacement: path, Git mode, native Git object-ID algorithm/digest, **and independent SHA-256 of the exact file/symlink bytes**;
- for each deletion: the exact deleted path.

It deliberately does **not** commit author, committer, timestamps, commit message, or a preferred physical commit ID.

Therefore:

```text
same exact parent + policy + semantic mutation
with different physical commit metadata
    => same SemanticPublicationIdV1
```

while changing the parent, target ref, mutation policy, path, mode, deletion/replacement state, native object ID, or independent content SHA-256 necessarily changes the canonical preimage.

The independent content commitment is intentional. A Git SHA-1 blob/object ID remains useful repository plumbing, but V1 does not ask SHA-1 alone to carry semantic content identity.

## Frozen V1 vector

The reference vector is:

```text
repository       = Luminous-Dynamics/mycelix
target_ref       = refs/heads/fix/example
parent           = Git SHA-1, 20 bytes of 0x11
mutation_class   = lock-only
policy           = 32 bytes of 0x22
mutation path    = Cargo.lock
mode             = 100644
Git object       = Git SHA-1, 20 bytes of 0x33
content SHA-256  = 32 bytes of 0x44
```

Canonical preimage length: `260` bytes.

Canonical preimage hex:

```text
6d7963656c69782f7265706f7369746f72792d6d6174657269616c697a6174696f6e2f73656d616e7469632d7075626c69636174696f6e2f7631000001000000194c756d696e6f75732d44796e616d6963732f6d7963656c697800000016726566732f68656164732f6669782f6578616d706c6501141111111111111111111111111111111111111111000000096c6f636b2d6f6e6c792222222222222222222222222222222222222222222222222222222222222222000000010000000a436172676f2e6c6f636b01000081a4011433333333333333333333333333333333333333334444444444444444444444444444444444444444444444444444444444444444
```

Independent SHA-256 of those exact bytes:

```text
fbcd1412678368c9e561bedeb0b7ce0d49bae7542e44db20d933429b70308267
```

## Publication equivalence

A remote commit is semantically equivalent to a candidate only when all of the following are positively established:

```text
remote parent count == 1
remote parent       == candidate expected parent
remote mutation set == candidate normalized mutation set
```

The mutation set includes paths, replacement/deletion state, Git modes, native object algorithms/digests, and independent SHA-256 content commitments.

An observer therefore cannot establish equivalence by looking only at `git diff --name-only` or Git blob IDs. It must authenticate the exact remote bytes (or an equivalent independently trustworthy content proof) for each replacement and bind their SHA-256 values into the observation.

If the remote commit ID also equals the candidate's preferred physical commit ID, the observation is `ExactPhysicalPublication`. Otherwise it is `SemanticallyEquivalentPublication`.

A matching tree reached through a different parent is **not** equivalent under V1. Neither is a commit that changes an additional path, even if all expected paths are present.

## Attempt and recovery states

The mutation adapter must classify its local attempt as one of:

```text
NotAttempted
ConfirmedApplied
RejectedBeforeMutation
UnknownAfterAttempt
```

A generic non-zero `git push`, connection reset, timeout, runner cancellation, or lost response must be treated as `UnknownAfterAttempt` unless the adapter has positive evidence that the remote ref mutation was rejected before it could occur.

The independent observer supplies one of:

```text
ObservationUnavailable
AtExpectedParent
Commit(exact authenticated remote facts)
```

The resolver produces:

```text
Published
PublishedRecovered
PublishedEquivalent
PublishedRecoveredEquivalent
DefinitelyNotPublished
Conflict
IndeterminatePublication
CandidateUnavailable
```

Critical rules:

- `push_error != DefinitelyNotPublished`;
- `UnknownAfterAttempt + AtExpectedParent = IndeterminatePublication`;
- exact/equivalent publication after an ambiguous attempt is recoverable only by positive remote proof;
- an extra path, wrong parent, wrong mode, wrong native object, wrong independent content hash, or wrong deletion/replacement state is `Conflict`;
- a missing/expired candidate before any attempt is `CandidateUnavailable`, not a product failure;
- `ConfirmedApplied` without observable post-state remains `IndeterminatePublication` under this protocol;
- queue starvation is a liveness failure, not semantic candidate invalidity.

`DefinitelyNotPublished` is transaction-scoped: it means this materialization transaction did not mutate the target, not that no independent actor could ever have changed the repository.

## Authority separation

The intended adapter architecture is:

```text
Deriver
  may execute candidate/toolchains
  must not publish

Artifact verifier
  may validate immutable candidate bytes and manifest identity
  must not publish

Publisher
  may construct the exact already-qualified mutation and perform bounded CAS
  must not execute candidate code/toolchains

Observer/resolver
  may inspect authenticated remote state and classify the outcome
  must not mutate
```

A future adapter may combine processes only if it preserves the same effective authority separation and can prove that combination does not widen capabilities.

## Relationship to integration materializers

This protocol is intentionally a sibling of the existing integration materializer stack, not an extension of its WASM ABI.

The integration materializer stack answers:

```text
Which exact qualified WASM may turn a canonical integration command into provider payload bytes?
```

This protocol answers:

```text
Did an already-qualified repository mutation become the exact intended semantic publication, and what can we prove after an ambiguous attempt?
```

They share content binding, bounded authority, fail-closed qualification, and non-laundering principles, but their authority domains remain separate.

## Claim ceiling

V1 does **not** establish:

- that any candidate product is semantically correct;
- that an artifact was honestly or completely derived;
- that GitHub, Git, a runner, or a network observation is trustworthy;
- that a policy commitment is legitimate;
- that a publisher is authorized to mutate a repository;
- that a compare-and-swap operation actually occurred;
- that a parent Git SHA-1 object ID is an independently collision-resistant semantic commitment;
- that SHA-256 was correctly computed by the Rust core;
- that a publication is merge-worthy or release-worthy.

The parent identity is deliberately still an exact repository object ID in V1. A later profile may add an independent parent-state commitment if a concrete evidence line demonstrates that need. V1 avoids inventing a parent-state canonicalization before we have a qualified derivation for it.

## Intended first adapter

The first adapter should be the MYC-CONST-001 R2 recovery pattern because it already has independent derived-artifact evidence, an exact raw lock SHA-256 plus Git blob ID, and a concrete semantic-equivalence race between two possible publishers. The adapter should consume the existing exact artifact and remote observations; it should not rederive the constitutional product.
