# Mycelix Identity — Interoperability and Assurance Roadmap

**Status:** Engineering assessment and staged plan, 2 November 2026  
**Scope reviewed:** Public `mycelix-identity` source tree on `main`, including the VC coordinator, VC integrity types, the `eidas-zkp` crate, its local tests, and the repository's internal security-status notes.  
**Review type:** Source/documentation inspection, not an independent audit, full build, conductor run, cryptographic review, or standards-conformance test.

## 1. Executive summary

Mycelix Identity has a substantial foundation: Holochain identity and credential zomes, schema and revocation flows, Ed25519 signing, algorithm-tagged signature abstractions, derived-credential/Merkle structures, and privacy-preserving proof research.

The next step is not to claim a new identity standard. It is to make the existing system interoperable with established standards while keeping Mycelix's additional trust, recovery, consent, and coordination features explicit.

The current source uses W3C VC Data Model 2.0-shaped fields, but source inspection found important gaps between a familiar JSON shape and an interoperable Verifiable Credential implementation:

1. The current credential signing function hashes a **custom subset** of fields. The inspected hash covers credential ID, issuer DID, subject DID, `validFrom`, serialized claims, and the Mycelix schema ID. It does not cover every security-relevant serialized field, including the context, type list, `validUntil`, credential schema reference, credential status, or proof metadata.
2. The Ed25519 path verifies the Mycelix-specific hash, not a W3C Data Integrity proof created using an independently specified and implemented cryptosuite.
3. The WASM verifier's hybrid path verifies the Ed25519 component only. The pure ML-DSA / SLH-DSA path currently accepts a recognized signature length structurally; it does **not** verify the post-quantum signature cryptographically in WASM.
4. The `mycelix-eidas-zkp` crate currently defines credential/presentation data structures and helper functions that accept proof values supplied by the caller. Its tests use placeholder proof strings and test serialization / claim filtering. They do not establish STARK-proof generation, STARK-proof verification, EUDI Wallet conformance, or eIDAS compliance.
5. The credential status structure contains Bitstring Status List-shaped fields, but the inspected issue path sets `statusListCredential` to `None` and uses the same illustrative index value. This is not, by itself, a published, interoperable W3C Bitstring Status List.

These findings are implementation-specific. They do not imply that all Mycelix identity functions are defective; they identify the highest-priority gaps between current code and external interoperability claims.

## 2. Existing implementation — accurately scoped

| Capability | Evidence in current source | Claim boundary |
|---|---|---|
| Local DID-like identity | `did:mycelix:<AgentPubKey>` is used by identity zomes | A custom method format is not a published method specification or proof of external DID resolution |
| Credential data model | VC-style fields including `@context`, `id`, `type`, `issuer`, `validFrom`, `validUntil`, `credentialSubject`, `credentialSchema`, `credentialStatus`, and `proof` | Field names and a context URI do not prove full Data Model or Data Integrity conformance |
| Schema validation | The verifier can retrieve a local schema and check required fields; `strict_schema` changes how missing/unavailable schemas are handled | The inspected code does not establish full JSON Schema draft validation or remote schema interoperability |
| Native Ed25519 | Credential content is hashed and signed through the Holochain signing API; the verifier checks the resulting Ed25519 signature | The signed payload is a Mycelix-specific subset hash, not the complete VC under a W3C cryptosuite's defined transformation/canonicalization |
| Hybrid Ed25519 + ML-DSA | A tagged envelope can contain both components | Current WASM verification checks the Ed25519 component only; the PQC component is deferred off-chain |
| Pure ML-DSA / SLH-DSA | Tagged signature algorithms and off-chain signing path are represented | Current WASM verifier performs structural length checking rather than cryptographic verification |
| Revocation | A separate revocation zome is queried; unknown or unreachable status is handled fail-closed by the credential-verification path | This is not the same as a published and interoperable W3C Bitstring Status List resource |
| Derived credentials | Holder ownership, a hash of the original credential, selected-claim subset, holder signature, and custom Merkle paths are represented | This is a Mycelix-specific derivation design; independent verification and cross-implementation interoperability need dedicated tests |
| eIDAS / DASTARK research | `eidas-zkp` defines VC/VP-shaped structures, DASTARK / Dilithium identifiers, and selective-disclosure metadata | The crate's current test fixtures use placeholder proof strings; serialization and filtering are not cryptographic conformance evidence |

## 3. Priority 0 — close the signature-coverage gap

**This is the highest-priority engineering item.** Before expanding standards support, ensure that a valid signature binds every security-relevant field that a verifier consumes.

### 3.1 Complete payload binding

The current `compute_credential_hash` uses a custom concatenation of selected values. Replace that with one explicitly selected, versioned signing profile whose transformation is fully specified and independently testable.

Candidate paths:

- **W3C VC Data Integrity:** implement a published cryptosuite using its specified transformation and canonicalization; do not substitute an informal JSON serialization or a custom hash and call it Data Integrity.
- **VC-JOSE / COSE:** where deployment constraints favor compact token or CBOR structures, use the relevant W3C securing-credentials specifications and standardized JOSE/COSE algorithms.
- **Mycelix-native internal profile:** retain a versioned internal profile only for explicitly Mycelix-only use, with a distinct identifier and no external-conformance claim.

The profile must bind, as applicable:

- `@context`, `id`, `type`, issuer, and subject;
- `validFrom` and `validUntil`;
- credential schema identifier, schema type, schema version/digest;
- credential status identifier and status parameters;
- proof purpose, verification method, cryptosuite, algorithm, challenge, and domain;
- any Mycelix extension that can affect interpretation, authorization, or verifier behavior.

A field may be excluded only when the chosen standard explicitly defines it as outside the signed representation. The exclusion must be documented and covered by tests.

### 3.2 Required mutation tests

For every supported suite, produce a known-good credential and then mutate one field at a time. Every mutation that is supposed to be covered by the signature MUST fail verification.

Required negative cases:

1. Change `issuer`.
2. Change `credentialSubject.id`.
3. Change one claim value and then add, remove, or rename a claim.
4. Change `validFrom` or `validUntil`.
5. Change `@context`.
6. Add, remove, or reorder a credential type where order/semantics matter.
7. Change `credentialSchema.id`, type, version, or digest.
8. Change `credentialStatus.id`, type, status purpose, list index, or list credential.
9. Change proof purpose, verification method, cryptosuite, algorithm, challenge, or domain.
10. Substitute a different signature while retaining the original metadata.
11. Substitute a different schema record with the same human-readable label.
12. Use an expired, revoked, suspended, or unresolvable issuer/key/status reference.

Required positive cases:

- valid credential round-trip through an independent serializer and verifier;
- valid credential with optional fields omitted according to the profile;
- a verifier on a second implementation accepts the same credential;
- key rotation and previously issued credentials follow explicit continuity and revocation policy.

## 4. Priority 0 — make cryptographic verification truthful

The current verifier has three different assurance behaviors which must not collapse into one `valid: bool` result.

| Signature form | Current observed WASM behavior | Required treatment |
|---|---|---|
| Ed25519 | Verifies the signature over the Mycelix-specific content hash | Retain only as an internal profile until complete payload binding and standard cryptosuite support are implemented |
| Hybrid Ed25519 + ML-DSA | Verifies Ed25519 component; does not verify ML-DSA component in WASM | Report `PARTIAL` / `INCONCLUSIVE` unless both required components are verified by trusted, bound verifiers |
| Pure ML-DSA / SLH-DSA | Checks recognized tagged algorithm and expected signature length | Must **not** return cryptographically verified / valid. Return `UNSUPPORTED` or `INCONCLUSIVE`, or require a separately authenticated off-chain verification attestation bound to the exact credential digest and key identity |

A structural parse is not signature verification. A detached off-chain result is not trusted merely because a client supplies a Boolean. If external verification is introduced, its attestation must bind the exact credential digest, algorithm/profile version, issuer verification key, verifier implementation and revision, verification time, and policy result. The verifier attestation itself needs authentication and revocation policy.

NIST FIPS 204 (ML-DSA) and FIPS 205 (SLH-DSA) standardize algorithms; they do not standardize a W3C VC cryptosuite, key discovery method, JSON transformation, or Mycelix credential profile. Keep algorithm standardization and credential-format interoperability as separate acceptance gates.

## 5. Priority 1 — publish the DID method and resolver boundary

The current local identifier format is:

```text
did:mycelix:<Holochain AgentPubKey>
```

To interoperate with external DID consumers, publish a method specification that defines:

- method-specific identifier syntax and normalization;
- DID Document generation and resolution behavior;
- verification-method identifiers and supported key types;
- authentication and assertion-method relationships;
- key rotation, deactivation, recovery, and historical resolution;
- service endpoint semantics and endpoint trust;
- privacy and correlation considerations;
- error cases, caching, and document-version semantics.

Build an independent resolver implementation and test it against pinned DID Core test vectors and third-party resolver expectations. Do not equate extracting an AgentPubKey from the DID string with resolving a DID Document or validating the verification relationship named by a credential's `verificationMethod`.

Key rotation tests must prove that a credential's verification method resolves to the correct historical or current key under an explicit policy. Deactivation and recovery must not silently cause old credentials to verify against an unintended replacement key.

## 6. Priority 1 — use a real credential status mechanism

Choose one of two clearly separated models:

1. **W3C Bitstring Status List:** issue and publish a valid status-list credential, assign unique indices, encode and expose the bitstring, and verify the list credential and status entry using an independent implementation.
2. **Mycelix-native status registry:** retain Holochain revocation queries as a native mechanism, publish a precise method specification and verifier API, and do not label it a W3C Bitstring Status List.

Test revoked, suspended, active, stale, missing, unavailable, malformed, and conflicting status responses. Unknown or stale status MUST NOT be converted into active status in a high-assurance policy.

A credential's `credentialStatus` reference must identify an actual resource with a documented resolution and verification method. A `statusListIndex` of `0` with no `statusListCredential` is not a published status-list implementation.

## 7. Priority 1 — distinguish derived credentials from interoperable selective disclosure

The current Mycelix derived-credential/Merkle scheme can be retained as a research or internal profile, but it needs a complete verifier and profile specification.

Required properties:

- Merkle root is committed by the signed original credential or an authenticated companion commitment.
- Leaf serialization, key ordering, hash function, tree shape, odd-node behavior, and proof encoding are specified.
- Every disclosed claim has a path that validates to the committed root.
- Holder authorization is bound to the exact original credential digest and selected claims.
- Expiry and revocation of the original credential affect derived presentations as defined by policy.
- A verifier can independently validate a proof without retrieving undisclosed original claims.
- Negative cases include claim substitution, omitted or reordered paths, changed root, wrong original credential, replay, expired derived result, and revoked original.

For wallet interoperability, choose a supported external profile such as a tested W3C VC Data Integrity suite, VC-JOSE/COSE, or an interoperable selective-disclosure credential format. Keep experimental DASTARK as a separate research cryptosuite until its algorithms, transcript, canonicalization, proof generation, verification, and test vectors have been specified and independently reviewed.

The current `eidas-zkp` helper's filtering of JSON claims and attachment of caller-supplied proof strings must not be described as proof generation or cryptographic verification.

## 8. Priority 2 — OpenID issuance and presentation interoperability

Once the underlying credential and DID semantics are stable:

- implement OpenID4VCI issuer flows for credential offer, authorization, token, credential request, proof-of-possession, and error handling;
- implement OpenID4VP verifier / presentation flows including nonce, audience, client identifier, response mode, and replay protections;
- select and document the required profile: W3C VC, SD-JWT VC, or another supported credential format;
- implement wallet/holder behavior only to the extent it is in scope;
- keep Holochain transport and storage as implementation choices, not proprietary wire requirements.

Use the OpenID Foundation's published conformance tests for the relevant implementation role and profile. As of 7 August 2026, OpenID4VP 1.0 and OpenID4VCI 1.0 conformance suites for HAIP 1.0 were reported complete and open for self-certification. In September 2026, the Foundation announced first implementers had self-certified. A successful local Holochain test is not a substitute for those suites.

## 9. Benchmark and interoperability matrix

| Test family | Baseline / external tool | Required evidence |
|---|---|---|
| VC Data Model serialization | W3C VC 2.0 examples and JSON-LD processing implementations | Parse/serialize round-trip, context validation, unknown-extension handling |
| Data Integrity | Independent W3C Data Integrity implementation and cryptosuite vectors | Signature creation/verification interop, complete field tampering rejection |
| DID resolution | Independent DID resolver / DID Core vectors | Resolution success/error, key rotation, deactivation, method-specific edge cases |
| OpenID4VCI | OpenID Foundation conformance suite | Issuer role test report and negative-case coverage |
| OpenID4VP | OpenID Foundation conformance suite | Verifier role test report, nonce/audience/replay handling |
| Status / revocation | W3C Bitstring Status List reference implementation or specified Mycelix-native API | Issued, active, revoked, suspended, stale, unavailable, and malformed cases |
| PQC signatures | NIST FIPS 204 / 205 test vectors plus independent implementation | Actual verification for every claimed algorithm; hybrid must verify both components |
| Selective disclosure | Independent verifier and cryptographic test vectors for the selected profile | Undisclosed claim privacy, proof validity, mutation rejection, linkability analysis |
| Recovery | Adversarial multi-agent Holochain tests | Threshold correctness, malicious trustee, compromised trustee, replay, stale key, timeout and recovery races |

Report p50/p95 latency, verifier time, proof size, issuance cost, storage/network overhead, revocation propagation, failure/false-accept counts, and resource use. Compare only equivalent credential formats, cryptosuites, security levels, and hardware.

**Hard safety gates:**

- No unverified cryptographic signature is reported as valid.
- Every signed semantic field is covered by the selected proof profile.
- Unknown, revoked, expired, malformed, or unavailable high-assurance evidence is not treated as valid.
- A verifier accepts credentials produced by an independent implementation.
- A third-party implementation accepts Mycelix credentials where interoperability is claimed.
- Every public claim names the exact version, profile, implementation role, and conformance report.

## 10. Promotion gates

| Stage | Acceptance criteria | Public wording allowed |
|---|---|---|
| A0 — Internal model | Data shapes and internal interfaces exist | "VC-shaped internal records" |
| A1 — Correct internal crypto | Full signature coverage, real verification, tamper tests, strict failure states | "Internally verified Mycelix credential profile" |
| A2 — Standards mapping | Published DID method and selected VC proof profile | "Maps to / implements the documented standards profile" (list incomplete parts) |
| A3 — Independent interop | External resolver/verifier pairwise tests pass | "Interoperability tested with named implementation/version" |
| A4 — Conformance suite | Applicable OpenID/W3C test suites pass and reports are public | "Conformance tested for named role/profile/version" |
| A5 — External security review | Independent review of DID, key lifecycle, signatures, status, recovery, privacy and deployments | "Independently reviewed for the stated scope" |

No stage implies eIDAS, EUDI Wallet, GDPR, HIPAA, or general regulatory compliance unless the appropriate scheme-specific evaluation has separately been completed.

## 11. Immediate implementation tickets

1. **P0:** Add mutation tests for every field consumed by credential verification; fix signature coverage before widening format support.
2. **P0:** Remove structural acceptance of pure PQC signatures from any path that returns `valid: true`; expose explicit verification assurance and unsupported/inconclusive results.
3. **P0:** Make hybrid verification require both signature components or return partial/inconclusive.
4. **P1:** Implement a published cryptosuite or choose a standard proof format and independent reference library.
5. **P1:** Define the `did:mycelix` method and a resolver with key-rotation/deactivation vectors.
6. **P1:** Implement or clearly separate W3C Bitstring Status List from the Mycelix-native status registry.
7. **P1:** Write a full verifier for the Mycelix derived-credential Merkle profile and independently test it.
8. **P2:** Build OpenID4VCI issuer and OpenID4VP verifier adapters, then run the OpenID Foundation suites.
9. **P2:** Publish a compatibility matrix covering W3C VC 2.0, DID resolution, proof suites, revocation, and supported wallet/verifier implementations.

## 12. Standards references

- [W3C Verifiable Credentials Data Model 2.0](https://www.w3.org/TR/vc-data-model-2.0/) — W3C Recommendation, 15 May 2025.
- [W3C Verifiable Credential Data Integrity 1.0](https://www.w3.org/TR/vc-data-integrity/) — W3C Recommendation, 15 May 2025.
- [W3C Data Integrity EdDSA Cryptosuites v1.0](https://www.w3.org/TR/vc-di-eddsa/) — select and implement an actual standard cryptosuite rather than using the generic `DataIntegrityProof` type as a claim of conformance.
- [W3C Bitstring Status List v1.0](https://www.w3.org/TR/vc-bitstring-status-list/) — status publication and verification.
- [W3C DID Core](https://www.w3.org/TR/did-core/) — DID Document and resolution semantics.
- [OpenID4VCI 1.0](https://openid.net/specs/openid-4-verifiable-credential-issuance-1_0.html) and [OpenID4VP 1.0](https://openid.net/specs/openid-4-verifiable-presentations-1_0.html) — credential issuance and presentation protocols.
- [OpenID4VP / OpenID4VCI HAIP conformance suites](https://openid.net/openid4vp-and-openid4vci-conformance-tests-are-complete-and-open-for-self-certification/) — announced complete and available for self-certification on 7 August 2026.
- [NIST FIPS 204: ML-DSA](https://csrc.nist.gov/pubs/fips/204/final) and [NIST FIPS 205: SLH-DSA](https://csrc.nist.gov/pubs/fips/205/final) — standardized post-quantum signature algorithms, not credential data models or W3C cryptosuites.
