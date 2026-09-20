# SUP-CIV-000D0 — Protected-Storage Reuse Ownership Census v1

Status: semantic ownership contract only. No runtime cryptography, storage selection, or Civic authority is created here.

Tracked by #2306. Parent storage-profile problem: #2161. Public/protected data-plane prerequisite: #2153 / draft #2155.

## Exact audited source cuts

- Mycelix: `a85369699099d4c7524e502e531735eed4ab36f4`
- Xenia Peer: `af4fcefc6d4cc7c3f74a3ca48f26abcd97d1e930`
- Xenia Wire: `057dd03043b8863d8f8280abf333331cc9ddf317`

Repository names are not timeless qualification identities. A later implementation must bind exact reviewed successors if any primitive materially changes.

## Ownership classes

`Owns` means the audited source directly establishes the named primitive/property under its own claim ceiling.

`CandidateAdapter` means reusable machinery exists, but protected-case semantics require a separate qualified adapter/composition theorem.

`ExplicitlyDoesNotOwn` means the property must not be inferred from the named primitive.

## Xenia Handshake

The audited `xenia-handshake` source establishes ML-KEM-768 session key establishment, mandatory Ed25519 + ML-DSA-65 transcript authentication, HKDF-SHA256 root/key derivation, and transcript-bound session/lane key scheduling.

It therefore owns session/peer cryptographic establishment under its exact profile. It does not own durable case-key lifecycle, multi-recipient wrapping, case-purpose authorization, retention, erasure, or storage replication.

```text
AuthenticatedSessionKey != DurableCaseStorageKey
SecurePeer != AuthorizedCaseRecipient
TranscriptAuthenticated != CaseAccessAuthorized
```

## Xenia Wire

The audited `xenia-wire` source exposes ChaCha20-Poly1305 sealed wire envelopes, replay-window semantics, epoch rotation, and optional handshake integration.

Those are wire/session properties. They may supply reusable AEAD machinery to a later protected-envelope adapter, but a wire envelope is not a durable case envelope and wire epoch rotation is not recipient/key lifecycle for protected records.

```text
WireEnvelopeAEAD != ProtectedCaseEnvelope
WireEpochRotation != CaseRecipientRotation
```

## Xenia Secure File

The audited `xenia-secure-file` crate establishes hardened local filesystem handling for secret files: owner-only creation, protected parent directory, descriptor-relative/O_NOFOLLOW access, atomic publish/replace, and anti-race behavior.

That is valuable for local key custody or local cache material. It does not establish multi-party sharing, remote recipient authorization, replication, storage availability, or policy-purpose legitimacy.

```text
SecureLocalFile != MultiPartyProtectedStore
FilesystemOwnerOnly != CasePurposeAuthorization
```

## Mycelix crypto availability

The presence of AEAD, KEM, DKG, signatures, or other cryptographic dependencies in Mycelix is not a protected-storage theorem. Existing primitives should be reused where exact semantics match, but recipient binding, durable key lifecycle, metadata exposure, recovery/escrow, current authorization, and storage topology remain separately qualified obligations.

```text
CryptoPrimitiveAvailable != ProtectedStorageProfileQualified
```

## Holochain substrate

Holochain mechanisms remain separate primitives:

- private entry content: author-local content confidentiality primitive;
- capability grant: callable-function access primitive;
- restricted/clone DNA: participant/network isolation primitive.

None is automatically a multi-party per-record protected database.

```text
PrivateEntry != MultiPartyProtectedDatabase
CapabilityGrant != DataAtRestEncryption
RestrictedDNA != PerRecordLeastPrivilege
```

## Reuse-before-build rule

Civic/Support must not invent a new cipher or KEM stack merely because protected payloads need encryption. Future implementation should preferentially compose existing reviewed primitives into a new project-neutral protected-envelope theorem.

The preferred decomposition is:

```text
SUP-CIV-000D0  exact reuse/ownership census
SUP-CIV-000D1  project-neutral protected envelope + key lifecycle
SUP-CIV-000D2  storage topology/profile selection
SUP-CIV-000D3  executable composition + accountable access
```

The envelope theorem and storage topology theorem are independent: either may change without silently rewriting the other's authority.

## Required anti-laundering laws

```text
SecureTransport != ProtectedAtRestStorage
SessionKey != DurableCaseDEK
AuthenticatedPeer != AuthorizedCaseRecipient
HasDecryptionKey != CurrentPolicyAuthorization
CapabilityGrant != DataAtRestEncryption
PrivateEntry != MultiPartyProtectedDatabase
RestrictedDNA != PerRecordLeastPrivilege
LocalSecureFile != DistributedProtectedStore
AEADAvailable != KeyLifecycleQualified
KeyRotation != KnowledgeRevocation
WireEnvelopeAEAD != ProtectedCaseEnvelope
WireEpochRotation != CaseRecipientRotation
FilesystemOwnerOnly != CasePurposeAuthorization
CryptoPrimitiveAvailable != ProtectedStorageProfileQualified
```

## Runtime consequences

A future strong profile must fail closed if:

- a session key is offered as a durable case DEK without an explicit adapter theorem;
- an authenticated peer lacks current case authorization;
- storage becomes unavailable and the only fallback is public plaintext;
- cryptographic success is used to bypass purpose/access-accountability checks;
- recipient/key rotation is represented as retroactive knowledge erasure;
- a local secure-file property is presented as remote/multi-party authorization evidence.

## Claim ceiling

A PASS for this contract can establish only the closed ownership/non-ownership classification for the exact audited source cuts, plus the required decomposition. It does not establish protected case storage, key rotation correctness, recipient authorization, metadata privacy, legal compliance, anonymity, retroactive revocation, municipal authority, Johannesburg readiness, or deployment readiness.
