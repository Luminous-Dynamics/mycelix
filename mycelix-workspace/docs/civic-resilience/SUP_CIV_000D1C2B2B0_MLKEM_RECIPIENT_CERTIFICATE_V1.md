# SUP-CIV-000D1C2B2B0 — ML-KEM Recipient Certificate / RID Binding Fixture v1

Status: exact X.509 fixture and syntax/binding evidence only. No recipient authorization, production PKI trust, private-key custody, or CMS interoperability is established.

Structural parent: SUP-CIV-000D1C2B2A / draft #2565 / exact head `27852e1eadc831fc3957758769031ef8855cd62f`.

Tracking issue: #2567.

## Purpose

Freeze one exact ML-KEM-768 X.509 recipient certificate and one exact test CA certificate so the B2A `RecipientIdentifier` is bound to the same public key already frozen by C2A1 before full CMS interoperability is attempted.

```text
RIDMatchesCertificate != RecipientAuthorized
CertificateChainValid != CurrentAuthorization
SPKIMatchesC2A1 != PrivateKeyCustodyProven
```

## Exact recipient certificate

The immutable recipient DER fixture is:

```text
length  = 1517
SHA-256 = 6639c042c6e263aab522652b744f4620ce22d4749156cfc21ed657146fb3804b
Git blob = 1fa21ea9b37c8dfc68298d1bdf2b6ec3d15b85e4
```

Frozen identity metadata:

```text
subject = CN=SUP-CIV ML-KEM-768 Recipient Fixture
issuer  = CN=SUP-CIV C2B2B Test CA
serial  = 0x2329 (9001)
notBefore = Sep 21 08:38:22 2026 GMT
notAfter  = Sep 18 08:38:22 2036 GMT
```

These dates are fixture bytes, not a production certificate-lifetime recommendation.

## Exact test CA

```text
length  = 343
SHA-256 = 4a1a2154e1936fa7ff888ad369dacde6eabd65dea3e3a451eee669e03cf36e0b
Git blob = 7d10e90b1a9a4ea26531712b84ef8c87befa0e66
```

The test CA exists only so the recipient fixture has a verifiable certificate signature/chain during qualification.

```text
TestCAValidatesFixture != ProductionTrustAnchor
```

No CA private key is committed by this tranche.

## Exact ML-KEM public-key binding

The recipient certificate's extracted `SubjectPublicKeyInfo` must hash to exactly:

```text
c23e23dd3d485a9256cda09358a4a286e00b373db10761eadf99f710649ca31c
```

That is the same deterministic ML-KEM-768 public DER already frozen in the C2A1 primitive-vector line.

The certificate's public-key algorithm must be ML-KEM-768. The verifier independently asks OpenSSL to parse the extracted SPKI and recognize ML-KEM-768 by name or OID.

## Exact RID / Subject Key Identifier binding

The certificate Subject Key Identifier is exactly:

```text
f61e8bc9b896925256cce487facf27a108eb5b04
```

That is the exact B2A test RID.

```text
TestRID == FixtureSKI
```

means only that the frozen syntax identifier selects this frozen certificate fixture. It does not establish that a real actor is currently authorized to receive any protected datum.

## RFC 9935 certificate profile boundary

The recipient fixture requires:

```text
basicConstraints = critical, CA:FALSE
keyUsage          = critical, Key Encipherment only
```

The qualifier refuses extra key-usage bits.

This keeps the test certificate aligned with the ML-KEM X.509 profile rather than treating a generic certificate carrying an ML-KEM SPKI as sufficient.

## Executable verifier

The stdlib-only Python verifier uses OpenSSL as an independent X.509/SPKI parser to prove:

1. exact recipient/CA DER lengths and SHA-256 values;
2. recipient chain validation to the exact frozen test CA;
3. exact subject/issuer/serial/notBefore/notAfter metadata;
4. ML-KEM-768 SPKI algorithm recognition;
5. extracted SPKI DER SHA-256 equals the C2A1 public-key fixture;
6. SKI exactly equals the B2A test RID;
7. `keyUsage` is critical and contains only `Key Encipherment`;
8. `basicConstraints` is critical and exactly `CA:FALSE`.

The host OpenSSL is qualification tooling only and creates no Mycelix runtime dependency.

## No private-key fixture in this tranche

B2B0 deliberately commits no recipient private key.

The later CMS-interoperability tranche may regenerate the deterministic test-only ML-KEM-768 private key from the existing frozen `hexseed = bytes 00..3f` under the pinned OpenSSL 3.6.4 toolchain, then require its derived SPKI to match this certificate before use.

```text
CertificateContainsPublicKey != PrivateKeyCustodyProven
```

## Boundary to B2B1 CMS interoperability

B2B0 does not establish:

- `OtherRecipientInfo` encoding;
- OpenSSL CMS KEM recipient processing;
- ML-KEM encapsulation/decapsulation inside CMS;
- KDF/AES-KW execution inside CMS;
- CEK recovery;
- plaintext recovery;
- recipient authorization/currentness;
- production PKI trust.

Those remain B2B1.

## Required non-equivalences

```text
CertificateFixtureValid != CMSKEMRecipientInteroperable
RIDMatchesCertificate != RecipientAuthorized
CertificateChainValid != CurrentAuthorization
SPKIMatchesC2A1 != PrivateKeyCustodyProven
TestCAValidatesFixture != ProductionTrustAnchor
B2B0Pass != B2B1Pass
```

## Claim ceiling

A PASS may establish only the exact certificate/CA fixture bytes, X.509 profile, signature-chain validation, SPKI equality to the frozen deterministic ML-KEM public-key fixture, and RID/SKI equality. It does not establish recipient authorization/currentness, production certificate issuance/trust, private-key custody, CMS interoperability, protected-storage access authority, legal compliance, municipal legitimacy or deployment readiness.
