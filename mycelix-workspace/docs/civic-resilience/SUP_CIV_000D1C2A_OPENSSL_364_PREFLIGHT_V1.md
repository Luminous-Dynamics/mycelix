# SUP-CIV-000D1C2A — OpenSSL 3.6.4 ML-KEM/CMS qualification preflight v1

Status: external-toolchain qualification only  
Tracks: #2476 / #2417  
Canonical transcript parent: SUP-CIV-000D1B exact head `1152251e1b8d09bf69cc2497164a0fc1d58da320`

## Purpose

Freeze one reproducible external reference toolchain capable of materializing the later RFC 9629 / RFC 9936 recipient-wrap evidence without perturbing Mycelix's Rust/Cargo dependency graph.

This tranche does **not** freeze a KEMRecipientInfo vector. It proves only that the exact pinned toolchain exposes the primitive and CMS functionality needed to create and independently inspect the future C2B corpus.

## Exact external source

```text
OpenSSL 3.6.4
tag: openssl-3.6.4
asset: openssl-3.6.4.tar.gz
SHA-256: 9bffaa1ad1e07b354c21bd3324ec02fa15579f45a7d0494b3e74bc449b7333ef
```

The source is downloaded from the official OpenSSL GitHub release and built into an isolated temporary prefix.

```text
ExternalToolchainBuild != MycelixRuntimeDependency
```

No Cargo manifest, lockfile, Nix expression, product crate or Holochain runtime is changed by this tranche.

## Required capabilities

The exact OpenSSL build must demonstrate:

1. `OpenSSL 3.6.4` version identity;
2. ML-KEM-768 key generation;
3. ML-KEM encapsulation and decapsulation;
4. CMS `-recip_kdf` support;
5. CMS `-recip_ukm` support;
6. HKDF availability;
7. AES-256 key-wrap availability;
8. deterministic ML-KEM-768 test key generation from one fixed 64-byte `hexseed`;
9. deterministic ML-KEM-768 test encapsulation from one fixed 32-byte `hexikme`;
10. byte-identical repeated test key/public-key output;
11. byte-identical repeated KEM ciphertext/shared secret for the same test fixtures;
12. successful decapsulation to the exact encapsulated shared secret;
13. ML-KEM-768 ciphertext length = 1088 bytes;
14. shared-secret length = 32 bytes.

`hexseed` and `hexikme` are test-vector controls only. They must never become production key-generation interfaces or entropy evidence.

## Deterministic fixture boundary

The preflight uses public deterministic fixture material:

```text
key seed = bytes 00..3f        (64 bytes)
encapsulation IKM = bytes 00..1f (32 bytes)
```

These values are intentionally non-secret.

```text
DeterministicTestVector != ProductionEntropy
FixturePrivateKey != DeployablePrivateKey
```

The generated private key, KEM ciphertext and shared secret are temporary qualification artifacts and are not committed as product secrets.

## CMS boundary

OpenSSL 3.6 added the KEMRecipientInfo-related CLI controls:

```text
-recip_kdf
-recip_ukm
```

C2A proves only that the exact pinned build exposes them.

It does not establish that any generated CMS object is RFC 9629-correct, that Mycelix `RecipientWrapContextV1` has been mapped correctly into UKM, or that the resulting DER is canonical for our profile.

```text
CmsCliSupportsKEMRecipientInfoOptions
!= RFC9629VectorQualified
```

## C2B continuation

Only after C2A qualifies should C2B freeze the actual recipient-wrap corpus:

```text
RecipientWrapContextV1 bytes
        -> exact UKM
ML-KEM-768 deterministic fixture
        -> KEM ciphertext / shared secret
RFC 9629 CMSORIforKEMOtherInfo DER
        -> HKDF-SHA256 KEK
32-byte payload DEK
        -> AES-256-KW wrapped DEK
all fields
        -> exact DER KEMRecipientInfo
```

and then independently prove decapsulation/KDF/unwrap plus hostile context/DER/recipient-identity cases.

## Required non-equivalences

```text
ToolchainAvailable != ProtocolQualified
DeterministicTestVector != ProductionEntropy
MLKEMRoundTrip != RecipientAuthorized
MLKEMSharedSecret != PayloadDEK
CmsOptionPresent != KEMRecipientInfoCorrect
OpenSSLVersionPinned != LibraryBugFree
OpenSSLBuildPass != MycelixRuntimeIntegrated
C2APass != C2BPass
```

## Qualification claim ceiling

A PASS may establish only that the exact OpenSSL 3.6.4 source identified by the frozen SHA-256 builds in the qualification environment and exhibits the required ML-KEM/CMS primitive-tooling behavior for the frozen deterministic fixtures.

It does not establish RFC 9629/9936 artifact correctness, AES-KW context binding, recipient authorization, production entropy/key custody, Mycelix runtime integration, protected storage, endpoint security, legal compliance, municipal authority, Johannesburg readiness or deployment readiness.
