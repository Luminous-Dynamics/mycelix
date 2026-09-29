# Integral D6T — Cross-Language Canonical Encoding Profile

Status: **ReferenceModelOnly / design freeze pending**

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

D6S currently uses `serde_json` over ordered Rust collections followed by SHA-256.

That implementation is intentionally **not** described as RFC 8785/JCS-compatible.

RFC 8785 defines canonical JSON using I-JSON constraints, deterministic property sorting, specified primitive serialization, and UTF-8 output. It also notes that Unicode normalization is not applied by JCS, so implementations must preserve string data consistently. citeturn0search0turn0search1

D6T must therefore either:

1. adopt a verified JCS profile; or
2. define a distinct D6S canonical encoding profile with equally explicit interoperability rules.

No interoperability claim should be made until this decision is frozen and golden vectors pass.

## D6S-CANON-1 requirements

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
- large and boundary numeric values;
- negative zero;
- duplicate properties;
- optional fields;
- every D6S receipt field;
- exact D6P receipt bindings;
- altered semantic environment;
- altered derivation profile;
- altered input commitments.

RFC 8785 currently has verified errata, including guidance concerning negative zero; any JCS adoption should account for the errata rather than relying on an unqualified summary of the original text. citeturn0search2

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
