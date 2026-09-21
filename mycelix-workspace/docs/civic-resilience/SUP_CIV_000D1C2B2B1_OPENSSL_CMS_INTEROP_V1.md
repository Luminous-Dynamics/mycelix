# SUP-CIV-000D1C2B2B1 — Pinned OpenSSL 3.6.4 CMS ML-KEM Interoperability v1

Status: external-toolchain interoperability evidence only. No real recipient authorization, production PKI trust, production private-key custody, or protected-read authority is established.

Structural parent: repaired SUP-CIV-000D1C2B2B0 r2 / draft #2576 / exact head `c3599c9c73e3cec5da6c1352227981859563f61c`.

Tracking issue: #2571.

## Purpose

Close the external CMS interoperability boundary for the frozen ML-KEM-768 recipient profile before any independent Rust implementation is allowed to claim compatibility.

The positive theorem is intentionally narrow:

```text
exact B2B0 certificate
+ deterministic test private key whose SPKI matches that certificate
+ exact D1B UKM
+ ML-KEM-768
+ HKDF-SHA256
+ AES-256-WRAP
+ AES-256-GCM content encryption
+ pinned OpenSSL 3.6.4
-> test CMS round trip succeeds
```

This still does not create authorization.

## Exact qualification toolchain

The subject builds the official OpenSSL release from source in an isolated temporary prefix:

```text
OpenSSL 3.6.4
openssl-3.6.4.tar.gz
SHA-256 = 9bffaa1ad1e07b354c21bd3324ec02fa15579f45a7d0494b3e74bc449b7333ef
```

No Mycelix Cargo, Nix or runtime dependency is changed.

## Exact recipient-key binding

The test-only ML-KEM-768 private key is regenerated from:

```text
hexseed = bytes 00..3f
```

Its public `SubjectPublicKeyInfo` must match both the frozen C2A1 public fixture and the public key extracted from the repaired B2B0 certificate:

```text
SHA-256 = c23e23dd3d485a9256cda09358a4a286e00b373db10761eadf99f710649ca31c
```

```text
GeneratedPrivateKeySPKIMatch != ProductionPrivateKeyCustody
```

No private key is committed to the repository; it exists only inside the isolated qualification directory and is removed on exit.

## Exact CMS profile

The encryption command is required to bind the already-loaded recipient certificate to:

```text
content cipher = AES-256-GCM
recipient ID   = subjectKeyIdentifier (`-keyid`)
KEM            = ML-KEM-768
KDF            = HKDF-SHA256
UKM            = exact D1B RecipientWrapContextV1 / wrap-r1
wrap           = AES-256-WRAP
```

The exact UKM is 165 bytes with SHA-256:

```text
4a6bbfc2baa973f7bf098f730fb1554d677a0c814ba178269571f1044cf8b53d
```

The OpenSSL 3.6 command-line implementation attaches `-recip_kdf` and `-recip_ukm` to the current recipient, which is why the recipient certificate appears before those options in the frozen harness.

## Frozen plaintext

The exact plaintext fixture is:

```text
SUP-CIV B2B1 CMS ML-KEM interoperability fixture v1\n
```

with:

```text
length  = 52
SHA-256 = d54dc56071885aacd704e4e15ff0325649ade754a98d8a206fd40348d8c7d0c8
Git blob = 679d8083e7c6ef96427f633f43a02ba035fe4a30
```

The randomized CMS ciphertext itself is deliberately **not** assigned a normative digest.

```text
RandomizedCMSBytes != ProtocolIdentity
```

## Positive evidence

The executable harness must prove:

1. exact OpenSSL version `3.6.4`;
2. exact source tarball digest;
3. `-recip_kdf` and `-recip_ukm` are present in the exact build;
4. exact B2B0 certificate digest;
5. deterministic recipient private-key SPKI equals the B2B0/C2A1 SPKI;
6. CMS encryption succeeds with the frozen profile;
7. serialized CMS contains ML-KEM-768, HKDF-SHA256 and AES-256-WRAP AlgorithmIdentifier bytes;
8. serialized CMS contains the frozen recipient SKI and exact D1B UKM;
9. the exact D1B UKM occurs once in the serialized CMS;
10. CMS decrypt using the matching certificate/key succeeds;
11. recovered plaintext is byte-identical to the frozen fixture.

## Negative evidence

### Wrong ML-KEM private key

A second deterministic key generated from test seed bytes `0x40..0x7f` must fail to decrypt the CMS object.

```text
WrongPrivateKey -> REFUSE
```

### Post-encryption UKM mutation

The harness locates the exact 165-byte D1B UKM inside the finished CMS DER, requires exactly one occurrence, flips one byte without changing ASN.1 lengths, and requires decryption to fail with the otherwise correct recipient certificate/private key.

```text
SameCiphertext
+ SameRecipientKey
+ MutatedUKM
-> REFUSE
```

This proves that the application context is load-bearing in the KDF/wrap path rather than decorative metadata.

```text
UKMMutationRejected != AllCMSMutationsRejected
```

## Relation to B2A/B2B0

B2A owns the exact standalone `KEMRecipientInfo` DER syntax vector.

B2B0 r2 owns the exact ML-KEM X.509 certificate / SKI-RID binding fixture.

B2B1 owns only OpenSSL 3.6.4 behavioral interoperability of the frozen test profile.

```text
B2APass != B2B1Pass
B2B0Pass != B2B1Pass
PinnedOpenSSLInterop != IndependentRustInterop
```

## Required non-equivalences

```text
CMSRoundTripPass != RecipientAuthorized
CertificateKeyMatch != CurrentAuthorization
CMSDecryptSucceeded != ProtectedReadAuthorized
TestPrivateKeyPossession != ProductionKeyCustody
UKMMutationRejected != AllCMSMutationsRejected
PinnedOpenSSLInterop != IndependentRustInterop
B2B1Pass != D3ProtectedReadQualified
```

## Continuation

After this external interoperability theorem is independently qualified, the next crypto implementation should be an independent Rust envelope/CMS-adapter line that reproduces the already-frozen semantics and vectors rather than treating OpenSSL as the production Mycelix runtime.

## Claim ceiling

A PASS may establish only end-to-end interoperability of the exact frozen test certificate/key/context profile under the exact pinned OpenSSL 3.6.4 CMS implementation, including the two frozen negative cases. It does not establish real recipient authorization/currentness, production PKI/key custody, independent Rust correctness, protected-storage access authority, legal compliance, municipal legitimacy or deployment readiness.
