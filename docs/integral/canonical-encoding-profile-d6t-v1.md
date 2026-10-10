# Integral D6T — Cross-Language Canonical Encoding Profile

Status: **ReferenceModelOnly / D6S-CANON-1 frozen as a reference profile**

## Purpose

D6T freezes the byte-level representation used by D6S commitments.

The goal is not merely deterministic serialization inside one Rust implementation. The goal is:

```
same qualified semantic object
+ same D6S-CANON-1 profile
= identical canonical bytes
= identical commitment
```

This is required before D6S receipts can be treated as interoperable commitments across Rust, WASM, Holochain, Symthaea, Xenia, or independent verification implementations.

## Current reference implementation

D6S-CANON-1 is now a D6S-specific canonical JSON subset rather than a claim of RFC 8785/JCS compatibility.

The profile intentionally avoids floating-point canonicalization: current D6S schema numerics are integral, so non-integral JSON numbers are rejected rather than inheriting runtime-specific number formatting.

The profile uses recursive UTF-16 code-unit ordering for object properties, preserves array order, preserves Unicode scalar values without normalization, defines deterministic JSON escaping, and emits exact UTF-8 bytes.

No cross-language interoperability claim is made until an independent implementation reproduces the frozen vectors.

## D6S-CANON-1 frozen reference rules

The current reference profile deliberately defines a **D6S-specific canonical JSON subset** rather than claiming RFC 8785/JCS compatibility.

1. **Objects:** property names are sorted recursively by their UTF-16 code-unit sequences. Locale, insertion order, and runtime map order are irrelevant.
2. **Arrays:** element order is preserved exactly; arrays are never sorted by the canonicalizer.
3. **Strings:** UTF-8 Unicode scalar values are preserved without Unicode normalization. JSON quoting uses deterministic escaping: \b, \t, \n, \f, \r for the five short control escapes, \uXXXX for remaining U+0000..U+001F controls, and escaping only quote/backslash otherwise.
4. **Numbers:** only signed 64-bit and unsigned 64-bit integers are admitted. Decimal floating-point values, exponent forms, NaN, infinity, and integers outside the frozen i64/u64 domain are rejected. At raw parser boundaries, negative zero is rejected rather than normalized. This is intentional: all current D6S numeric schema fields are integral, and the profile does not attempt to freeze cross-runtime floating-point semantics.
5. **Null/booleans:** null, true, and false use those exact lowercase spellings.
6. **UTF-8:** canonical output is the exact UTF-8 byte sequence of the resulting canonical JSON text.
7. **Duplicate properties:** typed Rust values cannot represent duplicate object properties; any future parser-facing implementation MUST reject duplicate names rather than last-write-wins coercion.
8. **Domain separation:** commitments hash the frozen D6S domain prefix, an explicit object-domain label, a zero-byte separator, and the canonical bytes. Frozen object labels are `environment`, `derivation-profile`, `qualified-projection`, `canonical-receipt`, and `d6p-context-set`.

9. **Self-commitment:** the receipt_commitment field is cleared before receipt commitment calculation, preventing recursive self-hashing.
10. **Schema/profile versioning:** the canonicalization version is part of the committed projection/receipt material and MUST change for any incompatible encoding change.

This profile is intentionally narrower than general-purpose JSON canonicalization. It is therefore easier to implement independently, but it MUST NOT be described as JCS-compatible.


The frozen profile MUST specify:

- object property ordering;
- array ordering semantics;
- string escaping;
- Unicode handling and normalization policy;
- integer and numeric representation;
- rejection of NaN, infinity, duplicate object properties, and other ambiguous values;
- exact UTF-8 output;
- treatment of optional/null fields;
- schema/version tagging;
- hash domain separation;
- receipt self-commitment construction.

The profile MUST be independent of programming-language map iteration order.

## Reference implementation boundary

The Rust implementation in canonical_derivation_receipt.rs is a reference implementation of D6S-CANON-1. Its current golden vectors cover empty/nested values, ordering, UTF-16 ordering, controls, integer rejection, domain separation, and material-field mutation. Independent implementations should reproduce these vectors before interoperability is claimed.

## Security boundary

Canonicalization establishes a reproducible byte representation.

It does **not** establish:

- truth;
- causality;
- authority;
- observer independence;
- currentness;
- authorization;
- actuation safety.

A perfectly canonicalized false, historical, unauthorized, or semantically insufficient object remains false/historical/unauthorized/insufficient.

## Negative requirements

A conforming implementation MUST NOT:

- normalize Unicode merely for convenience if the frozen profile forbids it;
- rely on locale-specific ordering;
- depend on serializer insertion order;
- treat hash equality as semantic equality without schema/profile validation;
- accept ambiguous duplicate-field encodings;
- silently coerce unsupported numeric values;
- omit a semantically material field from the commitment.

## Golden vectors

D6T should maintain vectors covering:

- empty objects and arrays;
- nested objects;
- reordered properties;
- Unicode and escaped strings;
- control characters;
- i64/u64 boundary numeric values;
- negative zero rejection;
- duplicate-property rejection;
- optional fields;
- every D6S receipt field;
- exact D6P receipt bindings;
- altered semantic environment;
- altered derivation profile;
- altered input commitments.

If a future D6S profile adopts RFC 8785/JCS, its current errata and number semantics MUST be incorporated into that profile rather than copied from an older summary.

## Exit gate

D6T is complete only when:

1. the profile is versioned and normative;
2. Rust implementation vectors are frozen;
3. an independent implementation reproduces the same bytes;
4. every semantically material field has a mutation vector;
5. malformed/ambiguous inputs are rejected;
6. commitment/domain-separation labels are frozen;
7. no D6S claim ceiling is increased by canonicalization.

Claim ceiling: **ReferenceModelOnly**.
