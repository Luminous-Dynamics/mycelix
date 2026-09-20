# PSI-002B3A — RFC 9577/9578 Structural Query-Credit Core

Status: **source constructed / structural only / no Privacy Pass cryptography / no query credit granted**

This crate freezes the Mycelix policy and challenge bindings required before a stable RFC 9577/9578 Privacy Pass token can ever authorize one private-contact-discovery query.

It intentionally implements no token issuance, token cryptography, replay database, atomic compare-and-set, network service, Holochain integration, or application authorization.

## Stable token profiles

The first profile recognizes the two RFC 9578 token types:

```text
0x0001 = private VOPRF(P-384, SHA-384)
0x0002 = public Blind RSA 2048 / SHA-384
```

Their exact versioned semantic IDs are part of policy identity rather than Rust enum debug output.

## Query-credit policy

`QueryCreditPolicyV1` binds:

- contact-discovery service domain;
- Privacy Pass issuer name;
- issuer-configuration digest;
- exact RFC 9578 token type;
- exact token-key ID digest;
- budget epoch;
- maximum identifiers per credit;
- mandatory atomic single-use replay policy.

Changing any of those fields changes the policy commitment.

## Redemption context

The 32-byte redemption context is derived from a domain-separated commitment over:

```text
service domain
+ budget epoch
+ exact query-credit policy commitment
```

This is a Mycelix policy binding. A concrete RFC adapter must independently prove that the actual RFC 9577 challenge bytes and token challenge digest encode the same intended context.

## Single-origin profile

The first profile fixes exactly one service Origin in `origin_info`.

Cross-service redemption therefore fails structurally rather than depending on application convention.

## No caller-controlled spend state

There is deliberately no field equivalent to:

```text
unspent = true
consumed = true
```

in the raw redemption observation.

One-time spend state must come from a later atomic backend boundary. Deserializing or constructing an observation can never manufacture it.

## Strongest structural result

A perfectly matching observation may produce only:

```text
ReadyForBackendVerification
```

while still reporting:

```text
token_cryptographically_verified = false
token_unspent_verified = false
token_atomically_consumed = false
query_credit_granted = false
anonymous_rate_limit_established = false
enumeration_resistance_established = false
application_authority_granted = false
```

## Source corpus

The committed ten-case source corpus covers:

- exact RFC 9578 token type codes;
- budget/epoch changes changing redemption context;
- single-origin challenge scoping;
- structural compatibility remaining non-authoritative;
- token-type substitution;
- token-key substitution;
- cross-service challenge substitution;
- old budget-epoch replay;
- policy binding reuse after budget changes;
- identifier-budget enforcement.

## Next boundary

PSI-002B3B must consume this structural result together with a concrete Privacy Pass backend result and an atomic single-use replay store.

Only a successful atomic transition from unseen to consumed may mint a scoped query-credit positive type.

```text
valid token
!= unspent token

unspent check
!= atomic consumption

atomic consumption
!= anonymous rate limiting

query credit
!= PSI privacy
```

## Qualification boundary

```text
source exists
!= source compiles
!= tests pass
!= RFC backend qualified
!= token valid
!= token unspent
!= query credit granted
```

A separate exact-source qualifier is required.