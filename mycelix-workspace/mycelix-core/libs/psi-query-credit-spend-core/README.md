# PSI-002B3B1 — Process-Local Atomic Spend Reference

Status: **source constructed / process-local reference only / no durable replay theorem**

This crate isolates the atomic-spend problem from Privacy Pass token cryptography.

It consumes the exact PSI-002B3A structural policy/challenge/observation inputs, recomputes B3A structural compatibility internally, derives one canonical spend key, and provides a deliberately named process-local atomic store.

## Governing boundary

```text
B3A structural redemption compatible
+ process-local mutex-protected unseen -> consumed transition
    -> ProcessLocalConsumedQueryTokenV1
```

But:

```text
ProcessLocalConsumedQueryTokenV1
!= token cryptographically verified
!= durable replay protection
!= multi-process replay protection
!= crash-safe replay protection
!= query credit granted
```

## Canonical spend key

`QueryCreditSpendKeyV1` is private-construction and binds:

- exact B3A policy commitment;
- exact challenge-binding commitment;
- exact token SHA-256;
- service domain;
- issuer name;
- exact RFC token type identity;
- exact token-key ID;
- budget epoch.

The replay key does not depend on requested identifier count. Redeeming the same token once therefore blocks every later attempt to reuse it with a different query size.

## Reference atomic store

`ProcessLocalAtomicSpendStoreV1` uses one Rust `Mutex<BTreeSet<...>>` around spend-key commitments.

Insertion into that set is linearizable inside one process:

- first insertion succeeds;
- all later insertions of the exact spend key fail as replay;
- concurrent attempts produce exactly one winner.

The type name is intentional. It is not a production durable replay store.

## Positive type

`ProcessLocalConsumedQueryTokenV1` has private fields and implements `Serialize` only.

It reports:

```text
process_local_atomic_single_use_established = true
```

and false for:

```text
durable_single_use_established
multi_process_single_use_established
crash_safe_single_use_established
privacy_pass_token_cryptographically_verified
query_credit_granted
anonymous_rate_limit_established
enumeration_resistance_established
application_authority_granted
```

## Source corpus

The committed eight-case corpus covers:

- first exact spend succeeds and sequential replay fails;
- concurrent racing spends yield exactly one success;
- token digest changes spend identity;
- service / epoch / token-key changes spend identity;
- non-ready B3A structure is rejected;
- B3A challenge cannot be borrowed across another policy;
- distinct tokens may each be consumed once;
- the serialized positive retains the strict authority ceiling.

## Next boundary

A later production adapter must provide durable/crash-safe/multi-process replay authority and must itself be qualified.

Only a later join of:

```text
qualified RFC token verification
+ exact B3A structural subject
+ qualified durable atomic spend positive
```

may mint a real `ConsumedQueryCreditUnderProfileV1`.

## Qualification boundary

```text
source exists
!= source compiles
!= tests pass
!= process-local concurrency theorem qualified
!= durable replay resistance
!= query credit granted
```

A separate exact-source qualifier is required.