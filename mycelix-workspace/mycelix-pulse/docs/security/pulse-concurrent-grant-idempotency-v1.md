# Pulse concurrent grant idempotency — finite model v1

**Profile:** PULSE-GRANT-CONCURRENCY-001  
**Status:** Model authored; exact TLC result pending  
**Related:** #5936, #5775, #5771

## Why this model exists

The previous grant-idempotency model covers a sequential lost-ack retry. It does not model two coordinators that both read the same idempotency key as absent before either writes. This is the classic read-then-create race:

```text
Worker A: read key -> absent -------- create grant A
Worker B: read key -> absent -------- create grant B
```

Both operations may be individually correct in isolation while the combined history creates two active host grants for one logical authorization request.

## Safe protocol abstracted by the model

1. Both workers may observe absence.
2. An atomic per-key reservation allows exactly one worker to move the logical request from absent to reserved.
3. The reservation binds the key to a canonical request digest.
4. Only the reservation owner may create the one host grant.
5. A second identical request waits and reconciles to the exact same grant identity.
6. A different digest under the same key returns conflict.
7. A lost acknowledgement does not create a new grant; retry reconciles the original.
8. After revocation, retries return a revoked disposition; they do not create a replacement grant under the old logical key.

The finite state model uses two workers, two candidate grant identities and two request digests. It does not assume that a Holochain source-chain commit and a conductor CapGrant admin operation are one transaction.

## Files

- `PulseConcurrentGrantIdempotency.tla` — state machine and invariants.
- `PulseConcurrentGrantIdempotency.cfg` — intended safe protocol, identical request digests.
- `PulseConcurrentGrantIdempotency-race-mutant.cfg` — no atomic reservation.
- `PulseConcurrentGrantIdempotency-digest-mutant.cfg` — same key accepts a different request digest.
- `PulseConcurrentGrantIdempotency-resurrection-mutant.cfg` — retry reissues authority after revocation.
- `.github/workflows/pulse-concurrent-grant-idempotency.yml` — pinned TLC runner and invariant-specific mutation checks.

## Required model-check behavior

| Config | Expected result | Invariant |
|---|---|---|
| Safe | State queue exhausted, no violation | All listed safety invariants |
| Race mutant | Counterexample | `AtMostOneGrantPerKey` |
| Digest mutant | Counterexample | `ConflictingRequestNeverSucceeds` |
| Resurrection mutant | Counterexample | `NoResurrectionAfterRevocation` |

The workflow rejects a generic TLC parser error as a successful mutant test: it requires a non-zero TLC exit and the exact intended invariant name in that mutant's log.

## Invariants and claim ceiling

The core properties are: at most one grant identity per logical key; reservation before creation; exact request-digest binding; canonical grant existence; conflicting digest never succeeds; revocation is monotonic; every acknowledged caller receives the canonical grant identity; and no grant exists while the logical request is still absent.

A passing run will establish only that this finite model obeys the encoded invariants for two workers/two grants/two digest values and the pinned TLC build. It does not prove that Holochain serializes concurrent zome calls, that a source-chain update and a host grant are atomic, or that crash recovery in a real conductor behaves as modeled.

## Runtime experiment that must follow

Use two concurrent authenticated grant requests against the same cell. Record exact CapGrant ActionHashes, reservation action identity, canonical request digest, logical operation ID and separate attempt IDs. Test same-key/same-digest, same-key/different-digest, lost acknowledgement, concurrent revoke and restart between every durable boundary. Assert action deltas and the set of active host grants directly; never infer host grant state from a DHT projection.

No raw CapSecret or raw idempotency token belongs in the public application record or ordinary telemetry. Store only a domain-separated digest/fingerprint and exact grant ActionHash where needed.
