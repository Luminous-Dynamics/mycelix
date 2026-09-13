# Mycelix Administrative Procedure Policy Identity Profile v1

Profile identifier:

`mycelix-administrative-procedure-policy-v1-blake3-framed-semantic`

Domain separator bytes (UTF-8, **not** length-framed):

`mycelix/administrative-procedure/policy/v1`

Hash algorithm: BLAKE3-256.

This document is normative for cross-language implementations. The Rust crate is one implementation of this profile, not the definition of the profile.

## Security / semantic boundary

The profile identifies the **semantic procedural obligations** carried by one ADMIN-002 `ProceduralCompletenessPolicy`.

It does not identify:

- where the policy was retrieved from;
- whether the policy is authoritative;
- whether it is the currently effective policy;
- whether it is legally valid;
- whether an institution adopted it;
- whether an actor has authority under it; or
- whether any external effect is permitted.

The identity claim itself (`policy_digest`, `policy_digest_profile`) is excluded from its own preimage.

`policy_ref` is also excluded because it is a provenance/locator and may legitimately differ across registries or mirrors while the semantic content remains identical.

## Primitive encodings

All strings are UTF-8 bytes.

Every framed field is encoded as:

`u64_le(byte_length) || field_bytes`

where `u64_le` is an unsigned 64-bit integer in little-endian byte order.

Integer semantic values are first encoded as fixed-width unsigned 64-bit little-endian bytes and **then framed** like every other field.

Enum discriminators are one byte and **then framed**.

Counts are unsigned 64-bit little-endian values and **then framed**.

No delimiter characters, NUL terminators, locale-specific encodings, JSON serialization, Rust serialization, or Unicode normalization step is implied by this profile.

ADMIN-002 validation occurs before canonicalization. Inputs that ADMIN-002 rejects are not alternate canonical representations.

## Set canonicalization

`required_notice_recipients` and `required_response_recipients` are semantic sets.

For each set:

1. every `PrincipalId` must already satisfy ADMIN-002 / institutional structural validation;
2. duplicate principal identities are invalid and must fail before hashing;
3. sort the exact UTF-8 principal strings by raw byte lexicographic order;
4. frame the set count; then
5. frame each sorted principal string in order.

Input vector order therefore has no effect on identity.

## Enum codes

### ResponseModeRequirement

- `None` = `0x00`
- `Written` = `0x01`
- `Hearing` = `0x02`
- `WrittenOrHearing` = `0x03`

### ReasonsRequirement

- `NotRequiredByProfile` = `0x00`
- `AtLeastOne` = `0x01`

Unknown enum values are not v1 values and must fail rather than being hashed under an invented code.

## Canonical preimage

Let `F(x)` mean `u64_le(len(x)) || x`.

Let `U64(n)` mean the eight-byte unsigned little-endian encoding of `n`.

Let `B(n)` mean the single byte whose value is `n`.

Let `N` be the sorted required-notice principal strings and `R` the sorted required-response principal strings.

The exact BLAKE3 input is:

1. raw domain separator bytes, unframed;
2. `F(UTF8(profile_identifier))`;
3. `F(UTF8(policy.protocol_version))`;
4. `F(UTF8(policy.procedure_profile))`;
5. `F(U64(len(N)))`;
6. for each `n` in `N`: `F(UTF8(n))`;
7. `F(U64(len(R)))`;
8. for each `r` in `R`: `F(UTF8(r))`;
9. `F(B(response_mode_code))`;
10. `F(U64(policy.min_response_window_ms))`;
11. `F(B(reasons_code))`.

The 32 output bytes of BLAKE3 are the canonical `Digest32` value. Hex rendering, when used for diagnostics or vectors, is lowercase hexadecimal with exactly 64 characters; hexadecimal text is not itself the committed value.

## Pseudocode

```text
h = BLAKE3()
h.update(UTF8("mycelix/administrative-procedure/policy/v1"))

frame(h, UTF8(PROFILE_ID))
frame(h, UTF8(protocol_version))
frame(h, UTF8(procedure_profile))

notice = sort_byte_lex(required_notice_recipients)
assert_no_duplicates(notice)
frame(h, U64_LE(len(notice)))
for principal in notice:
    frame(h, UTF8(principal))

response = sort_byte_lex(required_response_recipients)
assert_no_duplicates(response)
frame(h, U64_LE(len(response)))
for principal in response:
    frame(h, UTF8(principal))

frame(h, BYTE(response_mode_code))
frame(h, U64_LE(min_response_window_ms))
frame(h, BYTE(reasons_code))

return h.finalize_256()
```

`frame` is:

```text
frame(h, bytes):
    h.update(U64_LE(len(bytes)))
    h.update(bytes)
```

## Versioning rule

Any change to any of the following requires a **new profile identifier and domain separator** rather than silently changing v1:

- field set or field order;
- primitive encoding;
- set-ordering rule;
- enum discriminator assignment;
- framing convention;
- hash algorithm;
- string encoding;
- semantic inclusion/exclusion rule.

A v1 implementation must never reinterpret a future profile as v1.

## Relationship to currentness

A matching v1 digest proves only:

`these semantic obligations -> this canonical content identity`

A separate authoritative currentness theorem must establish, at minimum:

`institution + jurisdiction + rulebook/procedure profile + canonical policy identity + effective interval + provider generation/currentness evidence + provider proof`.

Therefore:

`canonical identity != governing policy`.
