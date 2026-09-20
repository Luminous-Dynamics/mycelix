# PSI-002B3A r2 — RFC 9577/9578 Structural Query-Credit Core

Status: **source constructed / structural only / no Privacy Pass cryptography / no query credit granted**

This replaces the unqualified r1 subject before execution.

The r1 model carried the whole token digest but did not expose the RFC token nonce as its own replay subject. RFC 9577 double-spend prevention is nonce based. R2 therefore distinguishes three identities:

```text
token_sha256
    = exact token artifact identity

token_nonce_sha256
    = SHA-256 of the token's RFC 32-byte nonce
    = intended replay subject input

token_challenge_digest_sha256
    = exact 32-byte challenge digest carried by the token, rendered as hex
    = intended future RFC challenge-verification join
```

All three values are raw observations here. A concrete RFC backend must extract and cryptographically bind them from one verified token.

## Stable token profiles

```text
0x0001 = private VOPRF(P-384, SHA-384)
0x0002 = public Blind RSA 2048 / SHA-384
```

Their exact versioned semantic IDs are part of policy identity rather than Rust enum debug output.

## Query-credit policy

`QueryCreditPolicyV1` binds:

- service domain;
- Privacy Pass issuer name;
- issuer-configuration digest;
- exact RFC 9578 token type;
- exact token-key ID;
- budget epoch;
- maximum identifiers per credit;
- mandatory atomic single-use replay policy.

Changing any field changes policy identity.

## Redemption context vs RFC challenge digest

The Mycelix 32-byte redemption context is derived from:

```text
service domain
+ budget epoch
+ exact query-credit policy commitment
```

`PrivacyPassChallengeBindingV1` is a semantic representation of the expected RFC 9577 challenge fields. It is **not** the RFC TLS-wire encoding.

Therefore:

```text
Mycelix challenge binding matches
!= RFC TokenChallenge bytes reproduced
!= token.challenge_digest verified
```

A later RFC adapter owns that exact wire theorem.

## No caller-controlled spend state

There is deliberately no `unspent`, `spent`, or `consumed` authority field in the raw observation.

## Strongest structural result

A fully matching observation may produce only:

```text
ReadyForBackendVerification
```

and still reports false for:

```text
token_cryptographically_verified
token_nonce_cryptographically_bound
challenge_digest_cryptographically_bound
token_unspent_verified
token_atomically_consumed
query_credit_granted
anonymous_rate_limit_established
enumeration_resistance_established
application_authority_granted
```

## Source corpus

The committed eleven-case source corpus covers:

- exact RFC 9578 token type codes;
- policy/epoch/budget changes changing redemption context;
- exact one-Origin service scoping;
- structural compatibility remaining non-authoritative;
- token-type substitution;
- token-key substitution;
- cross-service challenge substitution;
- old budget-epoch replay;
- policy-binding reuse after budget changes;
- identifier-budget enforcement;
- nonce and challenge-digest identities remaining separately preserved from whole-token identity.

## Next boundary

The corrected spend-key layer must use `token_nonce_sha256` as the replay identity input. `token_sha256` may remain audit/artifact evidence but must not partition the replay key.

A later concrete RFC verifier must prove that the observed token type, key ID, nonce digest, challenge digest and whole-token digest all came from the same successfully verified RFC token.

## Qualification boundary

```text
source exists
!= source compiles
!= tests pass
!= nonce bound to verified token
!= challenge digest bound to RFC bytes
!= Privacy Pass backend qualified
!= query credit granted
```

A fresh exact-source qualifier is required. No evidence transfers from superseded #2450/#2452.