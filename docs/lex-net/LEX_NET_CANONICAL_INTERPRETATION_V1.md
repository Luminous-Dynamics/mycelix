# LEX-NET-032 — Canonical Interpretation and Parser-Differential Resistance v1

Status: executable research contract. This document has no legal effect and does not claim external-standard conformance.

## Governing theorem

`signature-valid bytes != parser agreement != canonical-data agreement != semantic agreement != local recognition`

Cryptographic verification can establish that particular bytes were signed or committed. It does not, by itself, establish that every implementation will decode those bytes into the same machine meaning. LEX-NET therefore treats interpretation as an explicit, commitment-bound evidence stage.

## Relationship to the qualified spine

This tranche is a child of qualified LEX-NET-018 head `94d9869fd82c8708af3e08b95306586ef0d7b4fb`.

The intended semantic order is:

`raw bytes -> origin verification -> canonical interpretation -> semantic translation -> quarantine -> local recognition`

LEX-NET-032 does not retroactively rewrite LEX-NET-017 or LEX-NET-018 evidence. It freezes the interpretation contract that future integrated profiles must compose before relying on translation/quarantine outputs.

## First executable profile

The first executable profile is deliberately narrow:

`lex-net-json-interpretation-v1`

It is a synthetic JSON profile used to qualify interpretation semantics. It references established JSON/canonicalization concerns but **does not claim RFC 8785, I-JSON, W3C, CBOR, COSE, XML, SCITT, or any other external conformance**.

Rules:

1. Source bytes are decoded as strict UTF-8.
2. The top-level value must be a JSON object.
3. Duplicate object keys at any nesting level are rejected. No first-wins or last-wins behavior is permitted.
4. Floating-point / decimal JSON numbers, NaN, Infinity, and non-finite values are not accepted by this v1 profile.
5. Integers must be within signed 64-bit range.
6. Strings are preserved exactly after UTF-8 decoding. Unicode normalization and case folding are not applied to source values.
7. Object property order is representation-only. Canonical output sorts object keys lexicographically by decoded string.
8. Array order is preserved exactly and remains semantically meaningful unless a future field profile explicitly says otherwise.
9. A top-level `critical` member, if present, must be an array of unique strings. v1 supports no critical extensions, so any non-empty `critical` array is rejected as `CriticalFieldUnsupported`.
10. Unknown non-critical fields are preserved rather than silently dropped.
11. Canonical JSON bytes for this **synthetic profile** are UTF-8 `json.dumps(..., sort_keys=True, separators=(',', ':'), ensure_ascii=False)` over the accepted integer/string/bool/null/list/object domain. This is an internal qualification representation, not an RFC 8785 claim.
12. The canonical projection is SHA-256 commitment-bound.
13. If a caller presents a claimed canonical-projection commitment, it must equal the independently computed commitment or the result is `CanonicalizationMismatch`.
14. Re-interpreting source bytes under another profile/version creates a different interpretation event; a prior receipt is immutable.
15. Parser disagreement is never resolved by majority count, newest implementation, or AI preference.

## Interpretation result

A positive interpretation result binds:

- source byte SHA-256 commitment;
- exact interpretation profile ID/version;
- canonical-byte SHA-256 commitment;
- decoded projection SHA-256 commitment (equal to the canonical-byte commitment in this first synthetic profile);
- deterministic disposition and reason;
- explicit non-authority flags.

The positive result is evidence that this frozen interpreter established one bounded machine interpretation under this profile.

It is not evidence that the source claim is factually true, legally valid, locally recognized, authorized, or safe to execute.

## Reference dispositions

- `InterpretationEstablished`
- `MalformedEncoding`
- `DuplicateKeyRejected`
- `CriticalFieldUnsupported`
- `NumericDomainViolation`
- `UnicodeViolation`
- `CanonicalizationMismatch`
- `ParserDivergenceObserved`
- `InterpretationIndeterminate`
- `ProfileUnsupported`

Not every disposition needs to be produced by every fixture. The frozen corpus must exercise all dispositions that this first executable profile can deterministically establish.

## Golden/adversarial properties

The qualification corpus must prove at least:

- duplicate `authority` keys are rejected even though an ordinary last-wins parser can decode them;
- invalid UTF-8 fails before JSON semantics are inferred;
- non-object top-level JSON is rejected;
- floats/decimals and signed-64 overflow fail closed;
- an unsupported critical extension fails closed;
- object whitespace/property order canonicalize to one stable projection;
- composed and decomposed Unicode identity strings are not implicitly normalized into one identity;
- arrays with different order produce different commitments;
- unknown non-critical fields survive interpretation;
- an unsupported interpretation profile cannot be treated as v1;
- a caller-supplied projection commitment mismatch fails rather than being trusted;
- malformed JSON is not AI-repaired into source evidence.

## Typed receipt boundary

The intended future type is `InterpretationReceipt`, but this tranche does not mint a reusable authority-bearing token.

`InterpretationReceipt != TranslationReceipt != RecognitionReceipt != AuthorityGrant != external effect`

Any integrated typed implementation must keep those classes distinct.

## Non-escalation

Every positive reference result states:

`grants_local_authority = false`

`grants_external_effect_authority = false`

Canonicalization does not establish factual truth.

Canonicalization does not establish schema correctness beyond this frozen profile.

Canonicalization does not establish semantic equivalence to another schema.

Canonicalization does not establish legal validity.

Canonicalization does not establish local recognition.

Canonicalization does not establish authorization.

## Nonclaims

A PASS does not establish:

- factual truth of any claim;
- immunity from all parser/canonicalization vulnerabilities;
- external JSON/JCS/I-JSON/CBOR/XML/COSE/SCITT conformance;
- legal validity or enforceability;
- jurisdiction or applicable law;
- local recognition;
- identity equivalence;
- authority;
- production security;
- external-effect authority.

It establishes only the frozen LEX-NET-032 interpretation contract and the modeled parser-differential protections.
