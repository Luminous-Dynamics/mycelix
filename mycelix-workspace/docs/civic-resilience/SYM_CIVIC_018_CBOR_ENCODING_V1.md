# SYM-CIVIC-018 — exact CBOR floating-point / parser-resource boundary

Status: research/qualification boundary; not production-qualified.

Parent subject: ec2eb36f58ea9dede58e4071228d3fa5d42ffffc.

## Purpose

This seam binds the 018 qualification to exact CBOR bytes rather than JSON objects. An ordinary JSON round-trip cannot faithfully represent duplicate CBOR map labels or non-preferred CBOR encodings.

## Standards basis

RFC 8949 requires a CBOR-based protocol to define duplicate-map-key handling, distinguishes well-formed from valid data, and defines core deterministic encoding requirements including preferred serialization, no indefinite-length items, and deterministic map-key ordering. Invalid UTF-8 is a CBOR validity error that an application must handle explicitly. RFC 9052 requires COSE header labels to be unique within each header map, requires rejection of repeated labels as malformed, and restricts the protected/unprotected header encoding used by COSE. ARP-04 further makes deterministic CBOR a wire-level requirement for the artefacts tested by this research seam.

## Boundary

The fixture contains 37 exact message byte strings plus 2 exact deferred-depth composition vectors:

- 8 sufficient controls, including canonical finite/infinite/negative-zero values plus canonical binary32 and binary64 values that cannot be represented exactly by shorter formats;
- 8 COSE/ARP message-policy rejects, including duplicate labels and protected-header requirements;
- 16 deterministic-CBOR encoding rejects, including preferred-serialization, map ordering, indefinite-length deviations, non-shortest floating-point encodings, and the synthetic NaN exclusion;
- 5 parse/validity/resource failures, including truncation, trailing bytes under the one-item framing contract, invalid UTF-8, reserved simple-value encoding, and an explicit recursion-depth ceiling.

The manifest stores only case identity, family, and exact hexadecimal bytes. It contains no expected-verdict or oracle-verdict fields. The separate depth-vector list is also exact-byte-only and is not assigned a COSE message verdict.

The qualifier computes a SHA-256 corpus digest over the ordered case identifiers and exact bytes at runtime. It also runs two separately structured byte interpreters: a primary parsed-node implementation and a reference wire scanner. Any parser exception or primary/reference disagreement becomes CBOR_ENCODING_UNRESOLVED.

## Duplicate-map policy

Duplicate labels inside either the protected or unprotected COSE header map are rejected as malformed. The synthetic ARP profile used here also rejects the same label appearing in both buckets because that creates an ambiguity the profile deliberately does not permit. This cross-bucket rule is an application/profile policy, not a claim that RFC 9052 uses MUST language for every possible application.

## Determinism policy

For this research seam, an accepted message must be representable in the RFC 8949 core deterministic form. Non-preferred integer/length encodings, indefinite-length items, incorrect map-key ordering, and non-shortest floating-point encodings therefore receive CBOR_ENCODING_REJECT rather than being silently normalized. Finite values and infinities use the shortest representation that preserves the value; negative zero is retained with its sign; NaN is outside this synthetic accepted profile and is rejected as an encoding-policy violation. An explicit parser recursion ceiling of 32 nested levels converts deeper hostile structure into CBOR_PARSE_ERROR rather than allowing unbounded recursion. Protected-header reparsing inherits the containing protected-bstr depth instead of restarting at zero.

The qualifier does not claim to implement every CBOR semantic type or every production COSE validation rule. Its purpose is the exact-byte failure boundary immediately in front of the 018 trust-chain verifier, including deterministic floating-point representation and bounded parser depth.

## Failure precedence

CBOR_PARSE_ERROR means the exact bytes are not acceptable at the CBOR well-formedness/validity boundary.

CBOR_ENCODING_REJECT means the bytes can be parsed but violate the deterministic encoding profile.

CBOR_MESSAGE_REJECT means the encoding is processable but the synthetic COSE/ARP message contract is violated.

CBOR_ENCODING_UNRESOLVED is reserved for implementation faults or primary/reference disagreement.

No one category is silently collapsed into another.

## Nonclaims

This is a synthetic research qualifier. It does not establish a production CBOR library, COSE implementation, cryptographic signature verification, HTTP transport, Web-PKI, SCITT Receipt verification, or deployment behavior.

019 remains behind this entire 018 encoding fence.
## Deferred protected-header depth regression

The two depth vectors are a dedicated resource-composition probe, not additional COSE message cases.

D-01 wraps the protected bstr in 30 unary CBOR array levels. Its protected map then contains a nested array. Parsing the protected bstr must continue with the bstr's inherited depth, so the nested child crosses MAX_DEPTH=32 and the probe must report "exceeded".

D-02 contains the same protected map without the outer wrapper and must report "within limit". The pair therefore distinguishes additive parser depth from a reset-to-zero deferred parse.

Both primary and reference implementations must agree on D-01 = exceeded and D-02 = within limit.

## Repair notes

The floating-point helper now consumes the complete CBOR float item, including its initial byte. Consequently binary16, binary32, and binary64 are 3, 5, and 9 bytes at the node boundary. This aligns the primary implementation with the wire-item representation already used by the reference implementation.

Only byte-string nodes carry deferred-depth metadata because byte strings are the only nodes that trigger a second CBOR parse in this qualifier. Protected-map parsing starts at containing-bstr-depth + 1 in both implementations.
