# Government-ID Claim-Shape Conventions

The [`mycelix-identity/crates/eidas-zkp/`](../../mycelix-identity/crates/eidas-zkp/) crate currently provides experimental VC-shaped data structures and simple JSON claim filtering with caller-supplied proof metadata. It does not implement an interoperable W3C Data Integrity cryptosuite, generate or verify DASTARK/ML-DSA proofs, or establish eIDAS/EUDI Wallet conformance.

This document defines **claim-key conventions only**. The conventions can help issuers and verifiers agree on data shape, but they are not a credential schema registration, issuer trust decision, signature check, zero-knowledge proof, or certification. A `ProvenClaim` description is not proof that a predicate holds.

---

## Passport (ICAO 9303)

Credential `type` MUST include `"PassportCredential"` alongside `"VerifiableCredential"` and `"EidasCredential"`.

Claim shape:

```json
{
  "documentType": "P",
  "issuingCountry": "USA",
  "documentNumber": "123456789",
  "givenName": "Alice",
  "familyName": "Smith",
  "nationality": "USA",
  "dateOfBirth": "1990-03-15",
  "sex": "F",
  "dateOfIssue": "2022-05-10",
  "dateOfExpiry": "2032-05-10",
  "placeOfBirth": "San Francisco, CA, USA",
  "portraitImage": "data:image/jpeg;base64,...",
  "mrzLine1": "P<USASMITH<<ALICE<<<<<<<<<<<<<<<<<<<<<<<<<<<",
  "mrzLine2": "1234567890USA9003154F3205102<<<<<<<<<<<<<<00"
}
```

**Future selective-disclosure design note:** a verifier may need a predicate such as "holder is at least 18" rather than the document number. The current `ProvenClaim` helper only stores descriptive metadata and caller-supplied proof bytes; it does not prove the predicate. Do not use it as a security control until a proof system, verifier, and negative test vectors are implemented.

---

## Mobile Driver's License (mDL, ISO 18013-5)

Credential `type` MUST include `"MobileDriversLicense"`.

Claim shape (per ISO 18013-5 data elements):

```json
{
  "family_name": "Smith",
  "given_name": "Alice",
  "birth_date": "1990-03-15",
  "issue_date": "2022-05-10",
  "expiry_date": "2027-05-10",
  "issuing_country": "USA",
  "issuing_authority": "California DMV",
  "document_number": "DL123456789",
  "portrait": "data:image/jpeg;base64,...",
  "driving_privileges": [
    {
      "vehicle_category_code": "C",
      "issue_date": "2010-03-15",
      "expiry_date": "2027-05-10"
    }
  ],
  "un_distinguishing_sign": "USA",
  "administrative_number": "AN12345",
  "sex": 2,
  "height": 165,
  "weight": 60,
  "eye_colour": "brown",
  "hair_colour": "brown",
  "birth_place": "San Francisco, CA",
  "resident_address": "1 Main St, San Francisco, CA",
  "portrait_capture_date": "2022-05-10",
  "age_over_18": true,
  "age_over_21": true
}
```

**Future integration note:** mDL supports standardized data elements and selective-disclosure mechanisms in specific wallet protocols. Mycelix's current `ProvenClaim` structure is metadata only. A production integration needs a specified mDL presentation flow, cryptographic verifier, verifier policy, and conformance tests.

---

## US SSN-equivalent (`schema.org/PersonalIdentification`)

**Warning**: SSN is extraordinarily sensitive. Recommend against embedding the raw number in any credential. Preferred pattern is an "SSN-derived identity attestation" where a trusted intermediary (SSA-approved KYC provider) attests "this holder is a uniquely-identified US person" without transmitting the SSN.

Credential `type`: `"SsnDerivedAttestation"` (custom; not a passport or mDL).

Claim shape:

```json
{
  "attestationType": "ssn_derived",
  "attestingAuthority": "did:web:ssa-approved-kyc.example",
  "attestationDate": "2026-04-18",
  "holderHash": "b3sum256:7a8b9c...",
  "isUsPerson": true,
  "usTaxResident": true,
  "ssnSuffixLastFour": null,
  "expiresAt": "2027-04-18"
}
```

Never:
- `"ssn": "123-45-6789"` (plaintext SSN) — rejected by CLI at import time.
- `"ssnFull": ...` — rejected.

The `holderHash` is a BLAKE3 hash of the SSN with a per-attestor salt, suitable for deduplication ("same person across attestations") without revealing the SSN.

---

## SA ID Number (SA-specific)

South African green-bar-coded or smart-card ID. Credential `type`: `"SaIdCredential"`.

```json
{
  "id_number": "9003155800087",
  "given_name": "Alice",
  "family_name": "Smith",
  "date_of_birth": "1990-03-15",
  "sex": "F",
  "citizenship": "ZA",
  "place_of_issue": "Johannesburg",
  "date_of_issue": "2014-03-20"
}
```

First 6 digits of `id_number` encode DOB; digits 7-10 encode sex; digit 11 encodes citizenship (0=SA, 1=permanent resident). Verifiers can derive `date_of_birth`, `sex`, and `is_sa_citizen` via `ProvenClaim` without revealing the full number.

---

## EU eIDAS 2.0 (EUDI Wallet)

This section lists example claim-key names inspired by PID-style data. The current `eidas-zkp` crate does not implement the EUDI Wallet PID issuance/presentation profile or its conformance requirements.

### Separate implemented proof primitive: jurisdiction containment

The [Winterfell-backed jurisdiction proof module](../../crates/mycelix-zkp-core/src/circuits/jurisdiction_proof.rs) implements a scoped STARK range-proof workflow for showing that a private coordinate is within a selected published bounding box, subject to the proof's public inputs and policy checks. It does **not** prove the location was truthfully measured or attested. The corresponding [location-attestation adapters](../../crates/mycelix-zkp-core/src/location_attestation.rs) are separate: the current T1 adapter does not verify device signatures itself, and T2/T3/T4 adapters return `NotImplemented`. A deployment must verify the attestation source and freshness independently before relying on the jurisdiction statement.

The cross-DID zome in this repository is a nonce/replay coordination service; it stores submitted proof bytes without cryptographically verifying them.

```json
{
  "family_name": "Smith",
  "given_name": "Alice",
  "birth_date": "1990-03-15",
  "birth_place": "Berlin, Germany",
  "nationality": "DE",
  "resident_address": "...",
  "gender": 2,
  "age_over_18": true
}
```

---

## Required `ProvenClaim` types (reused from `eidas-zkp`)

| `proof_type` field | What it proves | Use case |
|---|---|---|
| `"range"` | value ∈ [min, max] | `age >= 21`, `latitude ∈ [42.3, 42.4]` |
| `"equality"` | value == constant | `nationality == "USA"` (when issuer-certified) |
| `"membership"` | value ∈ published set | `issuing_country ∈ {"USA", "CAN", "MEX"}` |

Verifier UIs SHOULD present `description` text ("age ≥ 21", "US / Canadian / Mexican citizen") rather than raw proof bytes.

---

## Forbidden claim keys

The `legal-did` zome validation layer rejects credentials containing:

- Any field named `private_key`, `secret_key`, `seed`, `mnemonic`, `recovery_phrase`
- Raw SSN fields (`ssn`, `social_security_number`, `socialSecurityNumber`) — must go through attestation pattern above
- Biometric raw template data (`fingerprint_raw`, `iris_raw`, `face_template_raw`) — rejected even if base64 — only hashes of biometrics
- Signed statements that could be replayed as general-purpose signatures (protection against credential-based identity hijack)

---

## Issuer DID conventions

See `zomes/issuer-trust-tier/` for the three-tier scheme:

| Tier | Example | Who sets it |
|------|---------|-------------|
| Sovereign | `did:web:state.gov`, `did:web:home.affairs.gov.za`, `did:web:gov.uk` | User (never canonical) |
| RegulatedIntermediary | `did:web:jumio.com`, `did:web:onfido.com` | User |
| Peer | anything else | Default |

Verifiers declare which tier they require. No tier influences Mycelix governance weight.
