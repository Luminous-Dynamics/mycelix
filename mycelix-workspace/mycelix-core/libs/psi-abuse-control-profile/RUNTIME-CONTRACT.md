# PSI-002B Runtime Admission Contract v0.1

This document is normative for any future runtime claiming to enforce the structural PSI-002B profile. It does **not** establish that such a runtime exists or is correct.

## Admission transaction key

Every decision must bind the tuple:

```text
psi_subject
service_domain
capability_namespace
capability_identity
epoch_domain
request_commitment
```

Changing any member selects a different accounting record. Raw queried identifiers are never members of this tuple.

## Atomic compare-and-consume

For one request with `n` blinded elements, one serializable admission transaction must evaluate all of the following against the same committed state:

1. capability is valid and not revoked;
2. epoch is server-authorized and current for the capability/service policy;
3. request commitment has not already been consumed in this capability epoch;
4. `requests_used + 1 <= max_requests_per_epoch`;
5. `n <= max_blinded_elements_per_request`;
6. `elements_used + n <= max_blinded_elements_per_epoch`;
7. `concurrent_reserved + 1 <= max_concurrent_requests`.

If every predicate succeeds, the same transaction must atomically:

- mark the request commitment consumed;
- increment request count;
- increment blinded-element count by `n`;
- reserve one concurrency slot;
- bind the reservation to the request commitment.

If any predicate fails, none of those state changes may become visible.

A check-then-update sequence split across independent transactions does not satisfy this contract.

## Revocation ordering

Revocation status must be read inside the same serializable transaction that consumes budget. A successful earlier revocation read followed by a later budget mutation is insufficient because revocation may race the mutation.

## Replay scope

A request commitment is single-use within the exact `(psi_subject, service, capability, epoch)` scope. Reusing the same commitment after a failed admission is allowed only when the failed transaction made no durable state mutation. Reusing a commitment after successful admission must be rejected even if the previous request later aborts.

## Epoch rollover

Epoch identifiers are policy-authorized values, not client-selected budget-reset tokens.

Each new epoch gets a fresh ledger namespace. Counters are not copied forward as positive allowance, and an expired/closed epoch cannot be reopened merely by replaying its identifier. The mechanism establishing current epoch authority is a separate subsystem and must be independently qualified.

## Concurrency release

Concurrency reservations must be released idempotently using the request commitment/reservation identity. Crash recovery must not create additional capacity. A lease/recovery design may be used, but it requires separate evidence.

## Distributed enforcement

If more than one admission node can accept the same capability/epoch, the durable store must provide a consistency model strong enough to preserve the compare-and-consume invariants across those nodes. Local per-process counters do not satisfy this contract.

## Privacy boundary

The admission transaction may consume only blinded-request metadata and externally supplied policy facts. It must not receive raw queried identifiers, derived dictionary guesses, or a raw-contact-set commitment.

## Claim ceiling

Conformance to this document would still not establish:

```text
enumeration resistance
Sybil resistance
client anonymity
transport privacy
privacy-preserving accounting
real-data admission
production admission
application authority
```
