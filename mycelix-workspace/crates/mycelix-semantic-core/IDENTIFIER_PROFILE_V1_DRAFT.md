# MYC-SEM-001D — Protocol Identifier Profile v1 (draft contract)

Status: draft adversarial contract only. This document does not change runtime behavior and MUST NOT be treated as qualified until MYC-SEM-001C-r2 is executable-qualified and a later implementation subject passes its own exact qualifier.

## Purpose

`BoundedSemanticTextV1` is allowed to carry bounded Unicode text. `SemanticIdV1`, however, participates in profile IDs, domain IDs, semantic subject IDs, and later commitment roots. Protocol identity needs a byte-obvious spelling that does not inherit Unicode normalization, bidi, zero-width, homoglyph, or percent-decoding ambiguity.

The V1 split is therefore:

```text
bounded semantic text
= bounded Unicode text

semantic protocol identifier
= bounded canonical ASCII token
```

This is a syntax theorem only. It does not establish namespace ownership, subject existence, identifier authenticity, URI/DID validity, dereferenceability, equivalence, currentness, or authority.

## Literal character repertoire

A V1 semantic protocol identifier is 1..=256 bytes and every stored byte MUST be US-ASCII.

Allowed literal bytes are the RFC 3986 `unreserved` and `reserved` repertoires:

```text
ALPHA / DIGIT
- . _ ~
: / ? # [ ] @
! $ & ' ( ) * + , ; =
```

Raw `%` is not a literal byte. It is admitted only as the introducer of a valid percent triplet.

The following raw bytes are therefore not admitted:

```text
SP and all ASCII controls
"  \\  <  >  {  }  |  ^  `
all non-ASCII UTF-8 bytes
```

## Percent triplet policy

A percent triplet is:

```text
% HEXUPPER HEXUPPER
```

where `HEXUPPER` is `0-9` or `A-F`.

Lower-case hexadecimal is rejected so each admitted escape has one spelling.

Uppercase syntax alone is not sufficient. The decoded byte sequence is also checked **for admission only**:

1. Percent-encoded ASCII `unreserved` bytes are rejected. They have a literal canonical spelling and RFC 3986 treats the encoded and literal forms as equivalent. Thus `%41` is rejected while `A` is admitted.
2. Percent-encoded ASCII controls, SP, DEL, and ASCII bytes outside RFC 3986 `reserved` are rejected. This prevents `%00`, `%0A`, `%20`, `%5C`, quotes, angle brackets, and similar bytes from bypassing the raw-byte firewall.
3. Percent-encoded ASCII `reserved` bytes remain allowed because RFC 3986 explicitly distinguishes a reserved delimiter from its encoded form. `%2F` and `/` therefore remain different admitted byte strings unless an importing scheme profile proves a relationship.
4. `%25` is allowed as the only representation of a literal percent data byte.
5. Every maximal contiguous percent-triplet run MUST decode as valid UTF-8. Arbitrary binary octets such as `%FF`, truncated UTF-8 such as `%C3`, and malformed sequences such as `%C3%28` are rejected.

The validation decode does **not** replace the stored token. The exact ASCII input remains the committed identifier bytes.

Examples:

```text
%E2%82%AC  -> admitted ASCII token; validation decode is U+20AC
%2F        -> admitted encoded reserved byte
%25        -> admitted encoded percent data byte

%41        -> rejected; canonical unreserved spelling is A
%00        -> rejected; decoded NUL
%5C        -> rejected; decoded backslash is outside the URI repertoire
%FF        -> rejected; not valid UTF-8
```

## No generic decoder law

After admission, `SemanticIdV1` is an opaque protocol identity token. Generic kernel code MUST NOT percent-decode, URI-normalize, case-fold, resolve dot segments, apply IDNA, normalize Unicode, or reinterpret it.

If a system needs URI, DID, IDNA, Git, Holochain, or another namespace's semantics, that typed adapter/profile operates **before** admission and supplies the exact ASCII identifier that Mycelix will commit.

This prevents a later generic decoder from silently changing identity after a commitment has already been formed.

## External identifiers

Adapters MAY admit identifiers originating from URI, DID, IDNA, URN, Holochain, Git, or other namespaces only after applying that namespace's own required syntax/canonicalization policy.

Examples:

- internationalized host names can be transformed by an IDNA-aware adapter before admission;
- non-ASCII URI data can be UTF-8 percent-encoded before admission;
- DID adapters can validate DID/DID-URL ABNF and DID-method-specific requirements before admission;
- an opaque binary identifier can use an explicit multibase/base32/base64url representation when its source protocol defines one.

The kernel then commits the exact admitted ASCII bytes.

## Equivalence firewall

The kernel intentionally does not assert:

```text
%2F == /
HTTP://example == http://example
/path/../x == /x
xn--... == a Unicode label
did method spelling A == spelling B
```

Even where an external scheme defines an equivalence, that equivalence belongs to the importing adapter/profile and must be represented explicitly before semantic identity is frozen.

Therefore:

```text
syntax-safe identifier
!= scheme-valid identifier
!= canonical identifier under external scheme
!= equivalent identifier under external scheme
```

## Compatibility requirement

MYC-SEM-001D MUST NOT change the corrected MYC-SEM-001C-r2 frozen vectors because all current vector identifiers are already within this profile:

```text
environment = 283f04d533916528a7054f9afe8958526cd3fdd94e29d9e990828af18dd12343
subject     = 5a88314b454bf23478c91af6adeff8a03bef0b8d4bd2f0c031b27321a50eb323
```

Any implementation that changes those two commitments is not a compatible 001D hardening and requires a new outer commitment profile/version.

## Deserialization rule

Wire deserialization MUST re-run the same identifier validator used by constructors. A derived transparent deserializer that can bypass the stricter `SemanticIdV1` validation is not acceptable.

## Qualification requirements

The later executable 001D subject must prove at minimum:

1. every valid corpus entry is admitted byte-for-byte;
2. every invalid corpus entry is rejected;
3. serde/wire decoding cannot bypass validation;
4. encoded unreserved aliases and percent-encoded control/unsafe ASCII are refused;
5. encoded non-ASCII runs must be valid UTF-8;
6. corrected 001C-r2 environment and subject commitments remain byte-identical;
7. no Unicode normalization library, URI parser, DID resolver, IDNA runtime, network, Holochain, clock, evidence, or authority dependency enters semantic-core;
8. fmt/check/test/clippy pass under the frozen toolchain/resolution;
9. an independent implementation of the grammar reaches the same corpus verdicts.

## Reference rationale

Unicode UTS #39 defines ASCII-Only as the strongest identifier restriction level and documents confusable/mixed-script risks. RFC 3986 defines the portable ASCII `reserved`, `unreserved`, and percent-encoding repertoire, says percent-encoded unreserved octets are equivalent to their literal forms, says encoded reserved characters can change interpretation, and warns that encoded control bytes such as `%00` need special handling. The WHATWG URL Standard likewise treats percent-decoded byte validity and UTF-8 handling as security-relevant.

Mycelix uses those facts only to justify a conservative protocol-byte boundary. It does not import URI, URL, or DID semantics into the kernel.
