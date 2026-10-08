# SYM-CIVIC-018 — exact CBOR floating-point / parser-resource boundary v17

Status: research/qualification boundary; not production-qualified.

Parent subject: 1196889eb2a849ddc359d8d05edf063aa9537699.

## Purpose

This seam binds the 018 qualification to exact CBOR bytes rather than JSON objects. An ordinary JSON round-trip cannot faithfully represent duplicate CBOR map labels or non-preferred CBOR encodings.

## Standards basis

RFC 8949 requires a CBOR-based protocol to define duplicate-map-key handling, distinguishes well-formed from valid data, and defines core deterministic encoding requirements including preferred serialization, no indefinite-length items, and deterministic map-key ordering. Invalid UTF-8 is a CBOR validity error that an application must handle explicitly. RFC 9052 requires COSE header labels to be unique within each header map, requires rejection of repeated labels as malformed, and restricts the protected/unprotected header encoding used by COSE. ARP-04 further makes deterministic CBOR a wire-level requirement for the artefacts tested by this research seam.

## Boundary

The fixture contains 51 exact message byte strings plus 4 exact deferred-depth composition vectors and 12 exact resource-boundary vectors.

The 51-message machine census is:
- 9 sufficient controls;
- 13 COSE/ARP application-policy rejects;
- 17 deterministic-CBOR encoding rejects;
- 12 parse/validity/resource failures;
- 0 unresolved.

The 4 depth vectors and 12 resource vectors are separate exact-byte probes and are not assigned COSE message verdicts. The resource probes cover definite strings/arrays/maps exactly at the configured boundary and indefinite strings/arrays/maps at both the boundary and one item/byte beyond it.

The manifest stores only case identity, family, and exact hexadecimal bytes. It contains no expected-verdict or oracle-verdict fields. In addition, a separate 12-vector resource corpus exercises exact configured limits and indefinite-length over-limit boundaries without changing the message-verdict census. The separate depth-vector list is also exact-byte-only and is not assigned a COSE message verdict.

The qualifier computes SHA-256 corpus digests over the ordered message, depth, and resource vector IDs, semantic families, and exact bytes at runtime. It also runs two separately structured byte interpreters: a primary parsed-node implementation and a reference wire scanner. Any message parser exception or primary/reference disagreement becomes CBOR_ENCODING_UNRESOLVED; resource probes additionally distinguish within-limit from explicit resource-bound exceptions.

## Duplicate-map policy

Duplicate labels inside either the protected or unprotected COSE header map are rejected as malformed at the COSE/application layer. Duplicate CBOR map keys are deliberately not treated as a deterministic-serialization failure by the core deterministic predicate here: RFC 8949 separates deterministic map ordering from map-key validity, and duplicate keys are surfaced by the explicit duplicate-label/validity checks instead. The synthetic ARP profile used here also rejects the same label appearing in both buckets because that creates an ambiguity the profile deliberately does not permit. This cross-bucket rule is an application/profile policy, not a claim that RFC 9052 uses MUST language for every possible application.

## Determinism policy

For this research seam, an accepted message must be representable in the RFC 8949 core deterministic form. Non-preferred integer/length encodings, indefinite-length items, incorrect map-key ordering, and non-shortest floating-point encodings therefore receive CBOR_ENCODING_REJECT rather than being silently normalized. Finite values and infinities use the shortest representation that preserves the value; negative zero is retained with its sign. NaNs first undergo the RFC 8949 shortest-preserving test; a deterministically encoded NaN then reaches the synthetic COSE application policy and is rejected there, while a non-shortest NaN is rejected at the deterministic encoding layer. Deterministic floating-point COSE header labels are rejected independently at the application layer because COSE labels are restricted to integers or text strings. An explicit parser recursion ceiling of 32 nested levels converts deeper hostile structure into CBOR_PARSE_ERROR rather than allowing unbounded recursion. Protected-header reparsing inherits the containing protected-bstr depth instead of restarting at zero.

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

The four depth vectors are a dedicated resource-composition probe, not additional COSE message cases.

D-01 wraps the protected bstr in 30 unary CBOR array levels. Its protected map then contains a nested array. Parsing the protected bstr must continue with the bstr's inherited depth, so the nested child crosses MAX_DEPTH=32 and the probe must report "exceeded".

D-02 contains the same protected map without the outer wrapper and must report "within limit". D-03 places the minimal protected map at the exact allowed depth boundary (32), while D-04 moves it one structural level deeper (33) and must report "exceeded". Together these vectors distinguish additive parser depth, exact boundary semantics, and reset-to-zero deferred parsing.

D-03/D-04 use a complete six-byte protected map ("46 a2 01 26 04 41 01") so the exact-depth pair tests depth rather than a malformed bstr-length condition. D-03 has 30 outer unary arrays and therefore parses the protected map's children at depth 32; D-04 adds one outer array and therefore crosses the 32-level ceiling.

Both primary and reference implementations must agree on D-01 = exceeded, D-02 = within limit, D-03 = within limit, and D-04 = exceeded.

## Repair notes

The floating-point helper now consumes the complete CBOR float item, including its initial byte. Consequently binary16, binary32, and binary64 are 3, 5, and 9 bytes at the node boundary. This aligns the primary implementation with the wire-item representation already used by the reference implementation.

Only byte-string nodes carry deferred-depth metadata because byte strings are the only nodes that trigger a second CBOR parse in this qualifier. Protected-map parsing starts at containing-bstr-depth + 1 in both implementations.


## NaN-layering regression

The successor deliberately distinguishes:
- shortest-preserving NaN -> deterministic CBOR, then synthetic COSE application rejection;
- non-shortest NaN -> deterministic encoding rejection;
- deterministic float-valued header label -> COSE application rejection.

This prevents a canonical NaN from being misrepresented as a deterministic-CBOR failure and preserves the RFC 8949 well-formed/valid/deterministic hierarchy alongside the RFC 9052 application layer.

## Canonical NaN application-policy closure

Hosted qualification of the v17 successor exposed the intended NaN-layering transition directly: C-34 is a shortest-preserving canonical NaN in a protected-header value, so deterministic CBOR admission succeeds but the synthetic COSE application policy rejects the message. The active census therefore moves one case from sufficient to application reject (9 / 13) without adding a manifest case. A separate alternate witness exercises the same policy through an unprotected-header value, proving the recursive NaN policy is applied symmetrically across both header buckets.

## Successor lineage

The parent is the actually GREEN exact-CBOR head `1196889eb2a849ddc359d8d05edf063aa9537699`, established by hosted run `37467707466` / job `112282815715`. This successor is exactly one commit above that parent with the same four-file research surface. 019 remains fenced until this successor independently qualifies.


## Reserved simple-value admission

RFC 8949 explicitly identifies `f8 00`, `f8 01`, `f8 18`, and `f8 1f` as non-well-formed two-byte simple-value encodings. All four are now represented in the machine corpus and must terminate in the parser/admission layer rather than reaching COSE application semantics.

## COSE label-type enforcement

The v6 successor now machine-checks the COSE structural rule that header labels are restricted to integers or text strings in both independent interpreters. The float-valued header-label vectors therefore reach an explicit application-layer rejection in both paths rather than relying on the final required-label check to reject them indirectly.

## Resource budget

The v7 research profile adds explicit synthetic resource ceilings in both interpreters:
- maximum byte/text string content: 1024 bytes;
- maximum definite or indefinite array items: 64;
- maximum definite or indefinite map pairs: 64.

These are admission/resource bounds, not claims that arbitrary CBOR implementations are safe under all memory or CPU attacks. The corpus includes orthogonal oversized byte-string, text-string, array, and map vectors.

## Bound resource policy

The synthetic resource policy is an explicit manifest object and is checked byte-for-byte by the qualifier before execution. The dedicated resource corpus also proves that these limits are applied consistently to definite and indefinite forms:
```
max_depth = 32
max_string_bytes = 1024
max_container_items = 64
```

Both interpreters implement the declared values. This prevents documentation/manifest/code policy drift from being silently accepted by the qualification harness.


## Exact resource-boundary probes

R-01 and R-04 exercise definite byte/text strings exactly at 1024 bytes. R-02/R-03 and R-05/R-06 exercise indefinite byte/text strings at exactly 1024 and 1025 bytes using one-byte chunks. R-07 exercises a definite array at 64 items; R-08/R-09 exercise an indefinite array at 64 and 65 items. R-10 exercises a definite map at 64 pairs; R-11/R-12 exercise an indefinite map at 64 and 65 pairs.

The exact-limit probes must complete within the resource profile. The one-over-limit probes must terminate through the explicit ResourceExceeded path in both interpreters. These vectors are deliberately separate from the message census so resource-bound exceptions cannot masquerade as COSE message policy outcomes.


## Canonical simple-value regression

The v10 successor adds an exact simple-value witness for the primary/reference equivalence boundary. RFC 8949 encodes simple values 0 through 23 directly in the additional-information bits; only values 32 through 255 use the f8 one-byte extension. The primary canonicalizer now preserves the direct single-byte form for simple values 0–23. C-48 is the deterministic COSE-shaped message carrying simple(0) as an unprotected header value and must remain CBOR_ENCODING_SUFFICIENT.


## Optimization fail-closed

The qualifier uses executable assertions for corpus geometry, manifest structure, oracle agreement, census, and boundary expectations. v11 therefore refuses execution whenever Python is optimized (-O or -OO) so those assertions cannot be silently disabled. The workflow independently verifies both optimized modes terminate non-successfully before accepting the ordinary exact qualification run.


## Probe-resolution regression

v12 adds executable synthetic-fault witnesses for the depth and resource probes. A generic Fault or unexpected exception must produce *_PROBE_UNRESOLVED rather than a false WITHIN_LIMIT result. This is asserted directly in the exact qualifier, in addition to the normal 64/65 boundary vectors.


## C-48 correction and direct simple-value invariants

Hosted v12 exposed a corpus-construction defect in C-48: the original witness omitted the fourth COSE_Sign1 array element and therefore classified as a parse failure. The vector is now a complete four-element message with simple(0)=e0 in the unprotected header. v13 also asserts the canonicalizer directly preserves e0, f3, and f7 for simple values 0, 19, and 23. The intended message census is restored to 10 sufficient / 10 message rejects / 17 encoding rejects / 12 parse/resource failures / 0 unresolved.


## Required common-parameter value typing

RFC 9052 defines the common alg header parameter (label 1) as int / tstr and kid (label 4) as bstr. v14 adds C-49/C-50/C-51 to prove these value-type rules reach the application-reject layer in both independent interpreters: alg as canonical float, alg as bstr, and kid as text string. Required labels remain present, unique, and deterministically encoded, so the new failures isolate value typing rather than label or encoding defects.


## C-49/C-50 corpus correction

The v14 hosted run showed C-49 and C-50 were themselves malformed protected-header byte strings: their declared bstr lengths excluded the final kid value byte. v15 corrects those exact lengths so the vectors are complete COSE_Sign1 messages and can isolate common-header value typing rather than parser truncation.


## C-51 corpus correction

Independent structural decoding of the v15 candidates found one final construction error in C-51: its protected byte string declared 7 bytes while the embedded map occupied 6 bytes. v16 corrects the protected length to 6 so C-51 is a complete four-element COSE_Sign1 message and isolates the kid text-vs-bstr application type rule.


## v16 record synchronization

The machine corpus now contains 51 message cases. Historical v12/v13/v14/v15 sections remain diagnostic, while the active machine contract is the v16 manifest/script pair and current four-file exact tree. The current expected message census is 9 sufficient / 13 message rejects / 17 encoding rejects / 12 parse/resource failures / 0 unresolved.

## Reserved simple-value witness completion

The machine corpus includes all four RFC 8949 two-byte simple-value non-preferred witnesses: `f8 00`, `f8 01`, `f8 18`, and `f8 1f`. They are represented by C-41/C-42/C-43 plus the earlier C-24 witness; all terminate at the CBOR parsing/admission boundary.

## C-12 corpus isolation correction

The prior v16 candidate accidentally reused protected labels 4 and 1 in the reversed unprotected-map fixture, so the synthetic cross-bucket label policy fired before deterministic map-order validation. C-12 now uses disjoint unprotected labels 3 and 2 in reversed wire order (`a2 03 00 02 00`), isolating the intended encoding-order failure.


## Five-bucket census closure

The hosted census failure also exposed a record-integrity gap: an all-zero `CBOR_ENCODING_UNRESOLVED` bucket was absent from the derived dictionary rather than represented explicitly. The active qualifier initializes all five verdict buckets to zero before classification, so the asserted zero-unresolved state is itself machine-visible.

## Manifest identity closure

The machine qualifier now requires the manifest top-level schema to be exact and rejects duplicate exact byte strings independently within the message, depth, and resource corpora. Contiguous IDs alone are therefore insufficient to silently duplicate or replace a frozen vector.

## Global frozen-byte identity

The active qualifier rejects duplicate JSON object member names and duplicate vector families, binds the exact ordered semantic family lists, requires every vector hex field to be canonical lowercase hexadecimal, to round-trip exactly to the stored bytes, and to be globally unique across message, depth, and resource corpora. This closes textual-alias and cross-corpus duplication paths.

## Parent-gate shell normalization

The parent qualification gate now uses a directly quoted Python check rather than an embedded heredoc, while preserving the pinned run/job identity checks.

## Authoritative PR-run boundary

The qualification workflow is intentionally authoritative only on pull-request events. A branch push and the corresponding pull-request synchronization can otherwise create two identical exact-head qualification runs, which adds queue pressure without adding independent evidence. The workflow therefore removes the push trigger and uses a pull-request-number concurrency group with cancellation so only the latest synchronized head remains authoritative.

## Hosted event identity binding

The hosted job now binds the workflow event to the exact checked-out Git object before the qualification logic runs. It requires a pull_request event, the canonical repository ID/name 1176351975 / Luminous-Dynamics/mycelix, the PR head repository to be that same repository, the event head SHA to equal git rev-parse HEAD, and the event base SHA to equal the qualified GREEN parent. This closes the event-payload to checkout-identity seam instead of relying only on subsequent Git topology checks.

## Deterministic-before-application precedence

The primary and reference classifiers now evaluate deterministic CBOR encoding immediately after protected-map parsing and before COSE application checks. This makes the documented failure precedence executable: a message that simultaneously contains an application defect and a non-preferred encoding is classified as CBOR_ENCODING_REJECT rather than being hidden behind CBOR_MESSAGE_REJECT. A synthetic transformed duplicate-label witness asserts this precedence without adding another frozen manifest case.


## Bit-exact reference float oracle

The reference interpreter no longer uses Python float conversion, struct, or math to decide floating-point determinism or NaN presence. It decodes the IEEE-754 sign/exponent/fraction fields with integer operations and tests exact representability in half/single/double formats as powers-of-two times an integer significand. This covers normal and subnormal boundaries, signed zero, infinities, and NaN payload shortening while keeping the reference numerically independent from the primary implementation's host-float conversions.