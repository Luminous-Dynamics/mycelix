# Maritime Commons canonical request bytes v0.1

**Status:** Proposed decision for review; not normative or merged
**Parent:** [#5956 — canonical request encoding and exact numeric semantics](https://github.com/Luminous-Dynamics/mycelix/issues/5956)
**Composes with:** Stay Protocol #5918, semantic verifier #5921, global profiles #5945, monetary profile #5948, profile verifier #5953/#5954
**Proposed profile:** MC-JCS-1
**Evidence grade:** protocol research and synthetic vectors; no executable conformance claim

## 1. Decision requested

Adopt a deliberately restricted profile of the JSON Canonicalization Scheme (JCS, RFC 8785) as the single canonical semantic-byte representation for the current JSON Stay Protocol v0.1 request/idempotency domain.

This is a recommendation, not a claim that the protocol is already frozen. It becomes normative only after at least two independently implemented encoders agree byte-for-byte and digest-for-digest on the frozen corpus, including adversarial inputs.

Do not define a second canonical encoding for the same v0.1 request. A future deterministic-CBOR wire profile may be evaluated as a separately versioned protocol profile, with its own negotiation and digest domain. It must never be treated as an alternate byte representation of the same v0.1 idempotency identity.

## 2. Why JCS is the better v0.1 fit

The current Stay Protocol and provider-facing adapters are JSON-oriented. JCS preserves JSON as the exchange model while supplying a standard, cross-language canonicalization algorithm for signing and hashing. RFC 8785 is an IETF RFC, but it is Informational rather than Internet Standards Track; that status must be disclosed rather than overstated.

RFC 8949 CBOR is an Internet Standards Track format, and its core deterministic-encoding requirements are attractive for compact typed binary messages and exact integer representation. However, choosing CBOR now would add a second protocol representation beside the current JSON contract and would still require a Maritime Commons application profile for map ordering, number forms, decimal quantities, timestamps, tags, duplicate keys, and JSON-to-CBOR mapping. Deterministic serialization alone does not define those application semantics.

For v0.1, JCS has the smaller migration surface and better alignment with existing JSON interfaces. This is a narrow decision for the current wire contract, not a claim that JSON is universally superior to CBOR.

## 3. MC-JCS-1 input restrictions

MC-JCS-1 is a strict subset of RFC 8785. A compliant implementation MUST:

1. Parse the original UTF-8 JSON bytes, not a lossy intermediate map. Reject malformed UTF-8, malformed JSON, trailing data, and invalid Unicode including unpaired surrogate escapes.
2. Reject duplicate object member names at every nesting level, after escape decoding. For example, the spellings a and Unicode-escaped a denote the same member name and MUST be rejected when both occur in one object.
3. Sort object property names recursively by unsigned UTF-16 code units, as required by RFC 8785. UTF-8 byte order, Unicode scalar-value order, locale collation, and insertion order are not substitutes.
4. Preserve JSON string code points exactly. Do not apply NFC, NFD, NFKC, case folding, transliteration, or locale-dependent transformations during canonicalization. Composed and decomposed strings remain distinct. Any schema-level normalization must be separately named and versioned and must preserve the source value.
5. Preserve array order. Do not sort arrays unless the field's domain schema explicitly defines the array as an unordered set and a separate normalization step is specified.
6. Reject JSON fractional and exponent-form numeric tokens for MC-JCS-1. JSON numeric values are restricted to integer tokens in the exact range -(2^53-1) through +(2^53-1). This avoids cross-runtime integer rounding and excludes binary floating-point values from protocol arithmetic.
7. Represent money minor units, large integers, and exact decimal quantities as strings with field-specific grammars. For example, amount_minor is a base-10 integer string bound to its pinned ISO 4217 metadata artifact and exponent. A string is not interchangeable with a JSON number.
8. Follow RFC 8785 string escaping and number serialization for accepted values. In particular, a JSON negative-zero numeric value canonicalizes to JCS representation 0; this behavior is included in the conformance vectors.
9. Preserve the distinction between an absent member and a member whose value is null. Do not silently inject defaults during canonicalization.
10. Preserve unknown extension members in the canonical input. An adapter may reject unsupported semantics, but it MUST NOT silently drop, rewrite, or reinterpret an extension before the canonical digest is computed.

A parser that materializes JSON into a last-key-wins map before checking duplicates is not conformant. A generic serializer that merely sorts keys is not, by that fact alone, an MC-JCS-1 implementation.

## 4. Canonical request envelope

The canonical semantic input is a schema-validated request envelope containing, at minimum:

- protocol identifier and protocol/schema version;
- operation kind;
- provider reference and provider idempotency scope;
- scheme-qualified subject/resource references;
- the exact global, industry, and jurisdiction profile-bundle digest governing interpretation;
- the operation payload.

The envelope must bind every value that changes the meaning, authority, resource, or effect of the request. It must exclude transport-only data such as HTTP header ordering, retry counters, connection identifiers, trace spans, and gateway receive time unless a specific field is explicitly part of the domain request.

The provider adapter first performs a versioned, reviewed source-to-contract mapping. That mapping must preserve source semantics and report loss. Canonicalization is applied only after the typed contract has been validated; it must not conceal a lossy or ambiguous adapter mapping.

## 5. Digest construction and evidence separation

For MC-JCS-1, define the semantic request preimage exactly as:

ASCII bytes MARITIME-COMMONS, followed by one NUL byte, ASCII bytes JCS-1, followed by one NUL byte, followed by the UTF-8 bytes of the JCS serialization of the complete canonical request envelope.

The semantic request digest is SHA-256 over that preimage, rendered as lowercase hexadecimal with the sha256: prefix.

Keep three identities distinct:

- **Raw-source digest:** SHA-256 over the exact source bytes received from a provider or adapter. This preserves the source artifact, including whitespace and lexical spelling.
- **Canonical semantic digest:** SHA-256 over the domain-separated MC-JCS-1 preimage. This identifies the validated semantic request envelope.
- **Effect/receipt identity:** the provider's own durable idempotency/effect receipt, bound to the provider, operation, resource, idempotency scope, canonical semantic digest, and provider receipt reference.

Whitespace-only changes may alter the raw-source digest while leaving the canonical semantic digest unchanged. Changing the provider, operation, subject, resource, profile-bundle digest, or payload MUST change the semantic digest.

The raw-source digest is evidence about received bytes; it is not a substitute for semantic request identity. The semantic digest is not proof that a provider performed an effect. A signature over either digest proves only the signing key's attestation to the bound bytes, subject to independent credential, recognition, authorization, and currentness checks.

## 6. Idempotency and failure semantics

Within a provider's explicitly declared durable idempotency scope:

- same scope + same idempotency key + same canonical semantic digest: return or reference the original logical effect/receipt; do not repeat the provider effect;
- same scope + same idempotency key + different canonical semantic digest: return Conflict; do not reinterpret the key as a new request;
- provider outcome cannot be established after timeout, partition, or crash: preserve Unknown / NeedsReconciliation; do not claim success and do not blindly repeat an effect outside the provider's proven idempotency boundary.

This does not create a global exactly-once guarantee. Maritime Commons coordinates evidence and reconciliation; the provider retains authority to commit its own resource and the provider's actual durability boundary governs duplicate-effect protection.

## 7. Why not a dual JSON/CBOR identity?

Accepting two different encodings as equivalent canonical request identities creates a new translation surface at the exact point used for idempotency and signatures. It invites disagreements about absent/null values, numeric strings, decimal tags, timestamp normalization, unknown extensions, and map-key ordering.

If compact CBOR transport is later justified, define a new named profile with an exact RFC 8949 core-deterministic encoding rule, a complete application data model, explicit JSON adapter behavior, independent vectors, and a new domain-separated identity. Do not allow a caller to choose an encoding per retry while retaining the same v0.1 idempotency namespace.

## 8. Required implementation and conformance gates

Before MC-JCS-1 can be called frozen:

- a standalone Rust implementation must reject duplicate keys before map collapse, enforce the safe-integer/no-float profile, implement UTF-16 key ordering, and emit exact RFC 8785 strings;
- an independently maintained implementation in another language must produce identical bytes and digests; this should be a small test oracle, not a production runtime dependency;
- every positive vector must match exact canonical UTF-8 bytes and SHA-256 output;
- every negative vector must fail closed with a stable error class;
- permutation, Unicode, duplicate-key, numeric-boundary, timestamp, extension, and domain-separation negative controls must be included;
- the corpus and both implementations must be pinned by exact commit and artifact digest in the result;
- exact-head CI must pass, but CI alone is not independent interoperability evidence;
- the provider effect/idempotency boundary must be measured separately from canonicalization correctness.

A canonicalization PASS proves only that the declared input was accepted and encoded according to MC-JCS-1. It does not prove adapter semantic equivalence, issuer authenticity, local recognition, resource availability, authorization, provider commitment, payment finality, legal compliance, or field safety.

## 9. Existing-code warning

The repository contains helpers named canonical JSON that use ordinary map sorting and standard JSON serializers. Those helpers are useful within their declared local scope, but they must not be silently reused as MC-JCS-1 without proving RFC 8785's UTF-16 property ordering, duplicate-key rejection before map materialization, Unicode preservation, numeric restrictions, escaping, and exact cross-language outputs.

Likewise, an enum naming a deterministic-CBOR profile is not evidence that a complete deterministic CBOR encoder/decoder exists. Maritime Commons should depend on an implemented, tested codec contract, not a label.

## 10. References

- RFC 8785 — JSON Canonicalization Scheme: https://www.rfc-editor.org/rfc/rfc8785.html
- RFC 8949 — Concise Binary Object Representation (CBOR), especially Section 4.2: https://www.rfc-editor.org/rfc/rfc8949.html
- RFC 7493 — The I-JSON Message Format: https://www.rfc-editor.org/rfc/rfc7493.html
- Stay Protocol v0.1 and semantic fixtures: #5918 / #5919 / #5922
- Global profiles and standards dependency lock: #5945 / #5946
- Monetary evidence profile: #5948 / #5949
- Effects-disabled profile verifier: #5953 / #5954

## 11. Open review questions

1. Is MC-JCS-1 acceptable as the v0.1 JSON digest profile, with its Informational RFC status explicitly acknowledged?
2. Is every effect-relevant context field included in the canonical envelope, especially provider idempotency scope and profile-bundle digest?
3. Should the no-float restriction be a protocol-wide requirement, or should any future numeric domain require an explicitly named, separately tested numeric profile?
4. Are the test-only independent implementation and the negative-control corpus sufficient for a first freeze?
5. Does a future CBOR transport justify a separately versioned profile, or is JSON adequate for the foreseeable adapter surface?