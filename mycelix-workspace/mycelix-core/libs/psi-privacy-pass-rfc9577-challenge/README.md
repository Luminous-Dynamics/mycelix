# PSI-002B3A1 — Exact RFC 9577 TokenChallenge Binding

Status: **source candidate / not compile-qualified / no token-verification theorem**

This crate closes one narrow boundary above corrected B3A r2:

```text
exact QueryCreditPolicyV1
+ exact derived PrivacyPassChallengeBindingV1
+ RFC 9577 default TokenChallenge encoding
    -> exact TokenChallenge bytes + SHA-256 digest
```

It does not parse or verify a Privacy Pass token.

## RFC wire format

The default RFC 9577 challenge is encoded as:

```text
uint16 token_type                         network byte order
opaque issuer_name<1..2^16-1>           uint16 length + bytes
opaque redemption_context<0..32>        uint8 length + bytes
opaque origin_info<0..2^16-1>           uint16 length + bytes
```

The B3A profile always uses one Origin and a 32-byte policy-derived redemption context.

## Conservative server-name profile

This first adapter intentionally accepts a narrower server-name alphabet than every possible RFC 3986 authority form. Names must be non-empty ASCII and may contain only letters, digits, `.`, `-`, `:`, `[` and `]`. Userinfo, path, query, fragment, comma and whitespace are therefore rejected.

A later profile may widen this only with its own parser/tests.

## RFC Appendix A canary

The source corpus independently rebuilds RFC 9577 Appendix A vector 1:

```text
token_type         0x0002
issuer              issuer.example
redemption_context  476ac2c935f458e9b2d7af32dacfbd22dd6023ef5887a789f1abe004e79bb5bb
origin              origin.example
wire length         67
SHA-256             8e1d5518ec82964255526efd8f9db88205a8ddd3ffb1db298fcc3ad36c42388f
```

The digest is computed from constructed bytes; it is not used as the construction input.

## B3A join

`encode_b3a_token_challenge_v1` re-derives the challenge from the supplied `QueryCreditPolicyV1` and requires exact equality with the supplied semantic challenge before producing RFC bytes.

```text
borrowed/mutated challenge
!= RFC challenge for this policy
```

## Authority ceiling

`Rfc9577EncodedTokenChallengeV1` may establish exact wire construction under this profile after source qualification. It always reports false for:

```text
token_challenge_digest_cryptographically_bound
token_cryptographically_verified
token_nonce_cryptographically_bound
atomic_single_use_established
query_credit_granted
application_authority_granted
```

## Qualification boundary

```text
source exists
!= source compiles
!= RFC vector passes
!= Privacy Pass backend qualified
!= token verified
```

A separate exact-source qualifier is required.
