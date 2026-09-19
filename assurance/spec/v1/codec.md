# MYCELIX-ASSURE/V1 Canonical CBOR Profile

Status: ASSURE-002B stacked codec candidate.

This profile defines ASSURE-V1 L0 canonical syntax. It does not define graph
validity, evidence admissibility, claim status, effective authority, or political
legitimacy.

Normative terms MUST, MUST NOT, SHALL, SHALL NOT, and MAY are requirements.

## Input boundary

CODEC-001. An L0 decoder MUST reject input larger than the active DecodeLimits
before attempting value materialization.

CODEC-002. Input MUST contain exactly one well-formed CBOR data item. Empty input,
truncated input, malformed UTF-8 text, and trailing CBOR data detected before an
active resource bound is exceeded MUST be rejected as D001_MALFORMED_ENCODING.
Once traversal exceeds an active resource bound, CODEC-012 resource-limit
precedence applies; the decoder MUST NOT continue beyond that bound merely to
discover a deeper syntactic or profile failure.

CODEC-003. ASSURE-V1 L0 uses the RFC 8949 Section 4.2.1 core deterministic
encoding requirements: preferred shortest argument encodings, definite lengths,
and bytewise-lexicographic ordering of deterministically encoded map keys.

CODEC-004. Canonicality MUST be checked by deterministic re-encoding of the
accepted data model and byte-for-byte equality with the original input.
A well-formed but non-canonical input MUST be rejected as
D002_NON_CANONICAL_ENCODING.

## Admitted generic data model

CODEC-005. L0 admits only CBOR integers, byte strings, text strings, booleans,
arrays, and maps.

CODEC-006. Floating-point values, semantic tags, null, undefined, and generic
simple values MUST NOT be admitted by L0 and MUST be rejected as
D005_FORBIDDEN_VALUE_KIND, subject to CODEC-012 resource-limit precedence.

CODEC-007. Every map key MUST be a non-negative integer representable as u64.
Other map-key kinds MUST be rejected as D006_INVALID_MAP_KEY, subject to
CODEC-012 resource-limit precedence.

CODEC-008. Duplicate map keys MUST be rejected as D007_DUPLICATE_MAP_KEY,
subject to CODEC-012 resource-limit precedence. Forwarding, preserving, or
selecting one value from duplicate keys is forbidden.

CODEC-009. L0 text semantics are the exact UTF-8 scalar sequence carried on the
wire. The codec MUST NOT perform Unicode normalization, case folding, trimming,
or locale-dependent transformation.

CODEC-010. Indefinite-length items are not canonical in ASSURE-V1. If otherwise
well-formed and representable within the active resource bounds, they MUST fail
the byte-for-byte canonicality check rather than being silently normalized and
accepted.

## Resource bounds

CODEC-011. DecodeLimits are verifier resource policy, not constitutional meaning.

CODEC-012. L0 MUST enforce explicit bounds for input bytes, nesting depth, total
decoded items, array length, map length, byte-string length, and text-string
length. Exceeding any active bound MUST be rejected as
D004_RESOURCE_LIMIT_EXCEEDED. Resource-bound failure takes precedence over any
syntactic or profile classification that would require traversing beyond the
exceeded bound. The input-byte bound MUST be checked before value materialization
as required by CODEC-001. Structural nesting MUST be represented by a bounded
limit and, where the underlying parser supports a recursion limit, that parser
limit MUST be configured from the active nesting policy; a parser recursion-limit
failure caused by that bound MUST be reported as D004_RESOURCE_LIMIT_EXCEEDED for
nesting depth. ASSURE-V1 does not claim that the remaining application-level
DecodeLimits are pre-allocation limits: an implementation MAY enforce them after
syntactic parsing and MUST NOT describe those limits as total heap-allocation
ceilings unless a separately qualified preflight mechanism establishes that
stronger property.

CODEC-013. The decoder MUST NOT expose the third-party parser's dynamic value type
as part of the public assurance API.

## Positive-assurance prohibition

CODEC-014. Successful L0 decoding establishes only that the exact bytes satisfy
this canonical syntax profile under the active resource limits.

CODEC-015. Successful L0 decoding MUST NOT establish graph validity, evidence
truth, claim establishment, effective authority, constitutional legitimacy, or
political legitimacy.
