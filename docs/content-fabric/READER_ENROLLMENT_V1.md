# Mycelix Content Fabric — Reader Enrollment v0.1

Status: CF-07C1 executable draft.

## Purpose

CF-07C1 defines the explicit application-identity directory required before Content Fabric can safely serve authenticated principal/group audiences.

The central rule is:

> Cryptographic peer identity may be evidence for an enrollment lookup, but it must not silently become the Mycelix application principal.

## Authority separation

```text
hybrid public-key pair
        |
        v
CF-07C1 enrollment lookup
        |
        v
explicit PartyIdV1 + groups

NOT:

public key hash -> PartyIdV1
```

This preserves separate authority for:

1. cryptographic peer authentication;
2. application identity enrollment;
3. live connection ownership;
4. remote content exposure policy; and
5. Nix software trust.

No one layer substitutes for another.

## Credential profile v1

CF-07C1 v1 commits the exact current Xenia standard hybrid peer identity:

- Ed25519 RFC 8032 verifying key: 32 bytes;
- ML-DSA-65 FIPS 204 verifying key: 1952 bytes.

The credential commitment uses explicit suite labels and exact bytes under:

```text
content-fabric/xenia-reader-credential@1
```

Length validation is representation validation only. CF-07C1 does not parse or verify the cryptographic keys and does not prove possession. A live Xenia adapter must supply that proof.

## Enrollment object

`ReaderEnrollmentV1` binds:

```text
XeniaHybridReaderCredentialV1
        +
non-zero PartyIdV1
        +
canonical group set
        |
        v
stable enrollment ID
```

Groups are:

- non-zero;
- sorted; and
- duplicate-free.

Changing credential, principal, or group membership changes the enrollment commitment.

Stored or wire enrollment IDs are not trusted directly: `ReaderEnrollmentV1::from_stable_id(...)` reconstructs the canonical enrollment and requires its recomputed commitment to equal the claimed ID.

## Registry snapshot

`ReaderEnrollmentRegistryV1` is immutable after construction.

It is indexed by exact hybrid credential commitment and has its own deterministic stable ID derived from the complete set of enrollment IDs in credential-ID order.

The same registry contents therefore produce the same registry ID regardless of input order.

Persisted or wire registry IDs are also recomputed. `reconstruct_reader_enrollment_registry_v1(...)` canonicalizes the complete supplied enrollment set, derives the registry ID, and rejects any mismatched claimed snapshot ID.

An empty registry is valid and means no hybrid credential is enrolled.

## Hybrid downgrade resistance

Matching only one key is forbidden.

In particular:

```text
enrolled Ed25519 + enrolled ML-DSA-65 -> match
same Ed25519 + different ML-DSA-65    -> no match
same ML-DSA-65 + different Ed25519    -> no match
```

This mirrors Xenia's existing operator-policy rule that both verified keys must belong to the same enrollment record for hybrid authentication to retain its meaning.

## Key rotation

A principal may intentionally have more than one enrolled hybrid credential in one snapshot.

This supports controlled overlap during key rotation:

```text
credential A -> Party P
credential B -> Party P
```

The reverse ambiguity is refused:

```text
same credential -> Party P
same credential -> Party Q
        x
```

The duplicate exact credential fails registry construction.

## Stable-ID collision hardening

Registry lookup first derives the credential stable ID, then verifies the stored raw public-key bytes still equal the supplied raw public-key bytes before returning an enrollment.

This means a merely colliding stable commitment is not treated as an exact credential match.

## No request authority in CF-07C1

CF-07C1 intentionally does not depend on `mycelix-nix-exposure` and does not construct `RemoteReaderV1`.

It also does not depend on Xenia Rust crates.

That split prevents an enrollment directory from being mistaken for authentication.

The later live adapter should perform:

```text
sealed Xenia authenticated handshake evidence
        +
exact CF-07C1 enrollment match
        +
live connection generation ownership
        |
        v
authenticated RemoteReaderV1
        |
        v
CF-07A RemoteServingSnapshotV1 lookup
```

The adapter must not accept caller-supplied headers, raw PartyId assertions, bare key fingerprints, or stale handshake evidence as substitutes for live authenticated connection state.

## Tests required for v0.1

The executable model pins:

- credential commitment changes when either signature key changes;
- wrong ML-DSA-65 representation length fails;
- claimed credential commitment mismatch fails;
- zero principals fail;
- zero groups fail;
- groups are canonicalized;
- group ordering/duplicates do not change enrollment identity;
- enrollment commitment changes with principal/groups;
- claimed enrollment IDs are recomputed and verified;
- same Ed25519 plus different ML-DSA-65 does not match;
- same ML-DSA-65 plus different Ed25519 does not match;
- duplicate credential enrollment fails;
- multiple credentials may map to one principal;
- registry commitment is input-order independent;
- claimed registry IDs are recomputed from the complete canonical snapshot; and
- empty registry is a valid deny-by-absence snapshot.

## Deferred

- Xenia crate dependency/integration;
- sealed handshake evidence consumption;
- same-generation live transport ownership;
- authenticated HTTP routing;
- enrollment provenance/signatures;
- dynamic enrollment refresh;
- enrollment revocation/retirement ceremony;
- persistent secure-time coupling;
- TLS/DNS/listener policy; and
- Nix signing/trusted-key policy.
