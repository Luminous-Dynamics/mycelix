# PSI-002B3B1 r2 — Nonce-Bound Process-Local Atomic Spend Reference

Status: **source constructed / process-local reference only / no durable replay theorem**

This replaces the unqualified r1 spend subject before execution.

R1 keyed replay state by the whole-token SHA-256. RFC 9577 double-spend prevention is defined around the token nonce. R2 therefore uses the separately observed `token_nonce_sha256` as the replay-subject input and retains `token_sha256` only as artifact/audit evidence.

## Governing theorem

```text
B3A r2 structural redemption compatible
+ canonical nonce-bound spend key
+ one process-local linearizable store
+ atomic unseen -> consumed transition
    -> ProcessLocalConsumedQueryTokenV1
```

But:

```text
process-local consumption
!= nonce cryptographically bound to token
!= Privacy Pass token verified
!= durable replay protection
!= query credit granted
```

## Canonical spend identity

The spend-key commitment binds:

- exact B3A r2 policy commitment;
- exact semantic challenge-binding commitment;
- `token_nonce_sha256`;
- service domain;
- issuer name;
- exact RFC token type code + versioned semantic ID;
- token-key ID;
- budget epoch.

It deliberately does **not** bind `token_sha256`.

Therefore:

```text
same nonce + different token artifact digest
-> same spend key
```

The whole-token digest and token challenge digest remain retained evidence fields in the positive type for later exact-subject joins.

## Process-local store

`ProcessLocalAtomicSpendStoreV1` uses one `Mutex<BTreeSet<String>>` over canonical spend-key commitments.

The exact theorem is limited to one process:

```text
first exact key wins
sequential replay fails
concurrent same-key race has exactly one winner
```

## Positive ceiling

`ProcessLocalConsumedQueryTokenV1` is private-construction and Serialize-only.

It may report only:

```text
process_local_atomic_single_use_established = true
```

and reports false for token/nonce/challenge cryptographic binding, durable/multi-process/crash-safe replay protection, query credit, anonymous rate limiting, enumeration resistance and application authority.

## Source corpus

Nine tests cover:

- first spend + sequential replay;
- 16-way concurrent race;
- same nonce with changed token artifact preserving replay identity;
- changed nonce changing replay identity;
- service / epoch / token-key scope changes;
- non-ready B3A structure rejection;
- cross-policy challenge borrowing rejection;
- distinct nonces each consuming once;
- serialized-positive authority ceiling.

## Qualification boundary

A separate exact-source qualifier is required. No evidence transfers from superseded #2456/#2457.