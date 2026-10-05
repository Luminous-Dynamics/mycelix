# SYM-CIVIC-018 — exact CBOR duplicate-map / encoding boundary

Status: research/qualification boundary; not production-qualified.

Parent subject: c0ff69b37b2ba768894b55178370ce0bff7adeb0.

## Purpose

This seam binds the 018 qualification to exact CBOR bytes rather than JSON objects. An ordinary JSON round-trip cannot faithfully represent duplicate CBOR map labels or non-preferred CBOR encodings.

## Standards basis

RFC 8949 requires a CBOR-based protocol to define duplicate-map-key handling, distinguishes well-formed from valid data, and defines core deterministic encoding requirements including preferred serialization, no indefinite-length items, and deterministic map-key ordering. Invalid UTF-8 is a CBOR validity error that an application must handle explicitly. RFC 9052 requires COSE header labels to be unique within each header map, requires rejection of repeated labels as malformed, and restricts the protected/unprotected header encoding used by COSE. ARP-04 further makes deterministic CBOR a wire-level requirement for the artefacts tested by this research seam.

## Boundary

The fixture contains 30 exact byte strings:

- 4 canonical positive controls;
- 8 COSE/ARP message-policy rejects, including duplicate labels and protected-header requirements;
- 14 deterministic-CBOR encoding rejects, including preferred-serialization, map ordering, indefinite-length deviations, and non-shortest floating-point encodings;
- 4 parse/validity failures, including truncation, trailing bytes under the one-item framing contract, invalid UTF-8, and reserved simple-value encoding.

The manifest stores only case identity, family, and exact hexadecimal bytes. It contains no expected-verdict or oracle-verdict fields.

The qualifier computes a SHA-256 corpus digest over the ordered case identifiers and exact bytes at runtime. It also runs two separately structured byte interpreters: a primary parsed-node implementation and a reference wire scanner. Any parser exception or primary/reference disagreement becomes CBOR_ENCODING_UNRESOLVED.

## Duplicate-map policy

Duplicate labels inside either the protected or unprotected COSE header map are rejected as malformed. The synthetic ARP profile used here also rejects the same label appearing in both buckets because that creates an ambiguity the profile deliberately does not permit. This cross-bucket rule is an application/profile policy, not a claim that RFC 9052 uses MUST language for every possible application.

## Determinism policy

For this research seam, an accepted message must be representable in the RFC 8949 core deterministic form. Non-preferred integer/length encodings, non-shortest finite/infinity floating-point encodings, indefinite-length items, and incorrect map-key ordering therefore receive CBOR_ENCODING_REJECT rather than being silently normalized. NaN encodings are outside this synthetic accepted profile and are rejected rather than assigned a guessed canonical payload.

The qualifier does not claim to implement every CBOR semantic type or every production COSE validation rule. Its purpose is the exact-byte failure boundary immediately in front of the 018 trust-chain verifier.

## Failure precedence

CBOR_PARSE_ERROR means the exact bytes are not acceptable at the CBOR well-formedness/validity boundary.

CBOR_ENCODING_REJECT means the bytes can be parsed but violate the deterministic encoding profile.

CBOR_MESSAGE_REJECT means the encoding is processable but the synthetic COSE/ARP message contract is violated.

CBOR_ENCODING_UNRESOLVED is reserved for implementation faults or primary/reference disagreement.

No one category is silently collapsed into another.

## Nonclaims

This is a synthetic research qualifier. It does not establish a production CBOR library, COSE implementation, cryptographic signature verification, HTTP transport, Web-PKI, SCITT Receipt verification, or deployment behavior.

019 remains behind this entire 018 encoding fence.