# Mycelix Administrative Review Policy Identity Profile v1

Profile identifier:

`mycelix-administrative-review-policy-v1-blake3-framed-semantic`

Domain separator bytes (UTF-8, **not** length-framed):

`mycelix/administrative-review/policy/v1`

Hash algorithm: BLAKE3-256.

This document is normative for cross-language implementations. The Rust crate is one implementation of this profile, not the definition of the profile.

## Security / semantic boundary

The profile identifies the semantic content of one ADMIN-003 `AdministrativeReviewPolicy`.

It does not prove:

- policy-record authenticity;
- institutional adoption;
- policy currentness;
- reviewer competence;
- stay or remedy authority;
- absence of a challenge or appeal;
- administrative or judicial finality; or
- external-effect authority.

The identity claim itself (`policy_digest`, `policy_digest_profile`) is excluded from its own preimage.

`policy_ref` is also excluded because it is provenance/locator metadata. The same semantic policy may be mirrored or migrated without changing its identity.

## Precondition

ADMIN-003 structural validation MUST succeed before canonicalization, except that the circular identity-claim fields MAY be replaced for validation with:

- any non-zero 32-byte `policy_digest`; and
- this exact profile identifier as `policy_digest_profile`.

Inputs rejected by ADMIN-003 are not alternate canonical encodings.

## Primitive encodings

All strings are UTF-8 bytes exactly as accepted by ADMIN-003. No Unicode normalization is added by this profile.

Every framed field is encoded as:

`u64_le(byte_length) || field_bytes`

where `u64_le` is an unsigned 64-bit integer in little-endian byte order.

Unsigned 64-bit semantic integers are first encoded as exactly eight little-endian bytes and are then framed.

Boolean values use one byte:

- false = `0x00`
- true = `0x01`

The boolean byte is then framed.

Counts are unsigned 64-bit little-endian values and are then framed.

No delimiter characters, NUL terminators, JSON encoding, Rust serialization, locale encoding, or platform-native integer encoding is implied.

## Optional jurisdiction encoding

For each optional jurisdiction:

- `None`: frame the one-byte discriminator `0x00` and append no jurisdiction value;
- `Some(jurisdiction)`: frame `0x01`, then frame the exact UTF-8 jurisdiction identifier.

The presence discriminator is semantically significant.

## Rulebook encoding

Each `RulebookRef` is encoded as three consecutive framed fields:

1. exact UTF-8 rulebook ID;
2. exact UTF-8 rulebook version; and
3. the exact 32 raw digest bytes.

The digest is binary. Hex text is not placed in the preimage.

## Set canonicalization

`accepted_review_roles` and `allowed_remedy_types` are semantic sets under this profile.

For each set:

1. ADMIN-003 validation must already have rejected duplicate or malformed members;
2. sort the exact UTF-8 member bytes by raw byte lexicographic order;
3. frame the member count as `U64_LE(count)`; then
4. frame each sorted member in order.

Input vector order therefore has no effect on identity.

## Canonical preimage

Let `F(x)` mean `u64_le(len(x)) || x`.

Let `U64(n)` mean the eight-byte unsigned little-endian encoding of `n`.

Let `B(n)` mean the single byte whose value is `n`.

Let `OJ(x)` encode an optional jurisdiction as specified above.

Let `RB(r)` mean:

`F(UTF8(r.id)) || F(UTF8(r.version)) || F(r.digest_raw_32_bytes)`.

Let `Roles` be the sorted accepted review-role strings and `Remedies` the sorted allowed remedy-type strings.

The exact BLAKE3 input is:

1. raw domain separator bytes, unframed;
2. `F(UTF8(profile_identifier))`;
3. `F(UTF8(policy.protocol_version))`;
4. `F(UTF8(policy.procedure_profile))`;
5. `F(UTF8(policy.source_institution))`;
6. `OJ(policy.source_jurisdiction)`;
7. `RB(policy.source_rulebook)`;
8. `F(UTF8(policy.review_forum))`;
9. `OJ(policy.review_jurisdiction)`;
10. `RB(policy.review_rulebook)`;
11. `F(UTF8(policy.review_capability))`;
12. `F(UTF8(policy.stay_capability))`;
13. `F(UTF8(policy.remedy_capability))`;
14. `F(U64(len(Roles)))`;
15. for each role in `Roles`: `F(UTF8(role))`;
16. `F(U64(policy.challenge_window_ms))`;
17. `F(U64(policy.appeal_window_ms))`;
18. `F(U64(policy.finality_delay_ms))`;
19. `F(B(policy.require_independent_reviewer ? 1 : 0))`;
20. `F(U64(len(Remedies)))`;
21. for each remedy in `Remedies`: `F(UTF8(remedy))`.

The 32 BLAKE3 output bytes are the canonical `Digest32` value.

When rendered diagnostically as hexadecimal, use exactly 64 lowercase hexadecimal characters. Hex rendering is not itself the committed value.

## Pseudocode

```text
h = BLAKE3()
h.update(UTF8("mycelix/administrative-review/policy/v1"))

frame(h, UTF8(PROFILE_ID))
frame(h, UTF8(protocol_version))
frame(h, UTF8(procedure_profile))

frame(h, UTF8(source_institution))
frame_optional_id(h, source_jurisdiction)
frame_rulebook(h, source_rulebook)

frame(h, UTF8(review_forum))
frame_optional_id(h, review_jurisdiction)
frame_rulebook(h, review_rulebook)

frame(h, UTF8(review_capability))
frame(h, UTF8(stay_capability))
frame(h, UTF8(remedy_capability))

roles = sort_byte_lex(accepted_review_roles)
assert_no_duplicates(roles)
frame(h, U64_LE(len(roles)))
for role in roles:
    frame(h, UTF8(role))

frame(h, U64_LE(challenge_window_ms))
frame(h, U64_LE(appeal_window_ms))
frame(h, U64_LE(finality_delay_ms))
frame(h, BYTE(1 if require_independent_reviewer else 0))

remedies = sort_byte_lex(allowed_remedy_types)
assert_no_duplicates(remedies)
frame(h, U64_LE(len(remedies)))
for remedy in remedies:
    frame(h, UTF8(remedy))

return h.finalize_256()
```

## Versioning rule

Any change to any of the following requires a **new profile identifier and domain separator** instead of silently changing v1:

- field inclusion or field order;
- primitive encoding;
- optional-value encoding;
- rulebook encoding;
- set-ordering rule;
- boolean discriminator assignment;
- framing convention;
- hash algorithm;
- string encoding; or
- semantic inclusion/exclusion rules.

A v1 implementation must never reinterpret a future profile as v1.

## Deliberate v1 challenge-clock meaning

This identity commits `challenge_window_ms`, but ADMIN-003 v0.1 interprets that window from `decision.decided_at_ms`.

This profile therefore does **not** mean “challenge window after service.” If a later administrative profile introduces service-aware challenge clocks, it must use an explicit new semantic field/profile rather than silently changing the meaning of v1.

## Relationship to later proof domains

A matching v1 digest proves only:

`these review semantics -> this canonical content identity`.

Later layers must separately establish:

`canonical identity -> authentic record -> institutional adoption -> current governing review policy -> exact review/finality consequence`.

Therefore:

`canonical review-policy identity != governing review authority`.
