# LEX-NET-032 — Canonical Interpretation and Parser-Differential Resistance v1 R2

Status: executable research contract. This document has no legal effect and does not claim external-standard conformance.

## Governing theorem

`signature-valid bytes != parser agreement != canonical-data agreement != semantic agreement != local recognition`

A deterministic parser profile must also bound the resources it will spend on hostile foreign input. Parser agreement without resource bounds is not a sufficient ingress contract.

## Relationship to the qualified spine

This tranche is a child of qualified LEX-NET-018 head `94d9869fd82c8708af3e08b95306586ef0d7b4fb`.

R1 head `083890a2cf967cef2fe8294585f3eb32e3a44658` remains frozen historical pre-execution evidence. R2 does not rewrite it.

The intended integrated semantic order is:

`raw bytes -> origin verification -> canonical interpretation -> semantic translation -> quarantine -> local recognition`

## First executable profile

The executable profile remains:

`lex-net-json-interpretation-v1`

It is a synthetic JSON profile used to qualify interpretation semantics. It **does not claim RFC 8785, I-JSON, CBOR, COSE, XML, SCITT, or any other external conformance**.

### Representation rules

1. Source bytes are strict UTF-8.
2. The top-level value must be a JSON object.
3. Duplicate object keys are rejected at every nesting level.
4. Floating-point / decimal JSON numbers and non-finite values are rejected.
5. Integers are limited to signed 64-bit range.
6. Strings are preserved exactly; no Unicode normalization or case folding is applied to source values.
7. Object property order is representation-only; synthetic canonical output sorts keys lexicographically by decoded string.
8. Array order is preserved exactly.
9. A top-level `critical` member, if present, must be an array of unique strings. v1 supports no critical extensions.
10. Unknown non-critical fields are preserved.
11. Canonical output is the frozen internal Python JSON representation described by the manifest; this is not an external canonicalization claim.
12. Source and canonical projections are SHA-256 commitment-bound.
13. Caller-supplied projection commitments must match independently computed commitments.
14. Re-interpretation under another profile/version is a new interpretation event.
15. Parser disagreement is never resolved by majority count, newest implementation, or AI preference.

### Frozen resource limits

These are **profile-specific acceptance limits**, not claims about universal JSON validity:

- maximum source bytes: `65536`;
- maximum structural nesting depth: `32`;
- maximum members/items in any one object/array: `256`;
- maximum UTF-8 bytes in any decoded string value: `16384`;
- maximum UTF-8 bytes in any decoded object key: `1024`.

Ordering is normative:

1. source-size is checked before UTF-8 decoding and general JSON parsing;
2. structural depth is lexically checked, respecting JSON string/escape boundaries, before general JSON parsing;
3. duplicate-key handling remains strict during parse;
4. decoded container cardinality and string/key limits are checked before canonicalization.

`ResourceLimitExceeded` is a first-class non-positive disposition. Exceeding a frozen bound is not repaired, truncated, sampled, or delegated to AI.

RFC 8259 permits parser implementations to bound accepted text size, nesting depth, numeric range/precision, and string length. These exact numbers are LEX-NET profile choices and are not RFC 8259 conformance claims.

## Interpretation result

A positive interpretation result binds source bytes, exact profile/version, canonical projection, commitments, deterministic disposition/reason, and explicit non-authority flags.

`InterpretationReceipt != TranslationReceipt != RecognitionReceipt != AuthorityGrant != external effect`

This tranche does not mint a reusable authority-bearing token.

## Reference dispositions

- `InterpretationEstablished`
- `MalformedEncoding`
- `DuplicateKeyRejected`
- `CriticalFieldUnsupported`
- `NumericDomainViolation`
- `UnicodeViolation`
- `ResourceLimitExceeded`
- `CanonicalizationMismatch`
- `ParserDivergenceObserved`
- `InterpretationIndeterminate`
- `ProfileUnsupported`

## Golden/adversarial properties

The frozen corpus proves duplicate-key rejection, malformed/invalid UTF-8 rejection, numeric bounds, Unicode non-normalization, array-order preservation, critical-extension handling, projection-commitment binding, and each frozen resource boundary.

Resource tests include both accepted-at-limit and rejected-over-limit cases for source size, nesting, container cardinality, string length, and key length.

## Non-escalation

Every positive reference result states:

`grants_local_authority = false`

`grants_external_effect_authority = false`

Canonicalization does not establish factual truth.

Canonicalization does not establish local recognition.

Resource-limit compliance does not establish safety beyond the frozen profile.

## Nonclaims

A PASS does not establish factual truth, immunity from all parser/resource-exhaustion bugs, external-standard conformance, legal validity, local recognition, authority, production security, or external-effect authority. It establishes only the frozen LEX-NET-032 R2 interpretation/resource-bound contract and modeled protections.
