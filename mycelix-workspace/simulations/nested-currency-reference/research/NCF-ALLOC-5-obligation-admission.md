# NCF-ALLOC-5 — Canonical obligation evidence and principal-scoped admission

**Status:** sequential Python oracle (nine tests pass) plus a typed Rust reference module with ten test definitions added; Rust has not been compiled or run. Neither implementation is cryptographic verification, concurrent/durable admission, distributed/Holochain behavior, or production settlement.

## Why the paradigm must change

NCF-ALLOC-3 demonstrated that lower-median caps can reduce influence from a minority's inflated claims when shard IDs are fixed. NCF-ALLOC-4 then split the same 4,000 units of real work across 1–32 IDs and showed those arithmetic caps cease to help once identity slots multiply. More elaborate demand caps cannot solve a missing identity and duplicate-work policy.

NCF-ALLOC-5 shifts the unit of account from **reported shard demand** to **unique eligible obligations owned by a canonical principal**. It models a sequential admission oracle with:

- A unique obligation_id bound to principal, instrument and amount. Cross-shard or cross-epoch exact retries return the original receipt and do not add demand again.
- Same-ID changed amount, principal, or instrument is rejected without changing the book.
- The accepted payload also includes a canonical obligation-payload digest; reusing an ID for different underlying work is rejected even when amount/principal/instrument match.
- Principal/instrument caps aggregate distinct obligations across all shard aliases.
- Missing caps, malformed identifiers, malformed verifier receipt digests, and verifier-rejected evidence fail closed without consuming the ID.
- A rejected cap attempt does not burn the obligation ID; a corrected amount can be retried. The model also computes a canonical digest for the underlying eligible-work payload, so the same work submitted under different request IDs or shard aliases is treated as one obligation. Rebinding an accepted request ID to another payload, or reusing the same work digest with a different principal/instrument/amount, is rejected without mutation. This still assumes a trusted verifier derives that digest from real eligible work; the digest itself is not proof of work.

The model requires a canonical payload digest, but computes that digest from caller-supplied test content; digest binding is not proof of the truth of that content. The reference model's verifier_receipt_ok flag is an assumption supplied to the model. Hashes are deterministic test receipt digests, not signatures. No cryptographic principal resolution, verifier, database transaction, or distributed linearizability is implemented here. The typed Rust module places a crate-internal VerifiedObligation marker between the future verifier adapter and admission, while explicitly leaving authentication out of scope. Exact request-ID retries now bind the original verifier version and verifier receipt digest as well as principal, instrument, amount and canonical work digest; changing either proof field under the same ID is rejected without mutation. A new Rust regression covers both substitutions. A different request ID for the same canonical work digest remains an idempotent alias and returns the original canonical receipt.

## Rust reference module

The dependency-free `src/obligation.rs` now models the same sequential policy in Rust. `VerifiedObligation` has private fields and a crate-internal constructor reserved for a future verifier adapter; callers must not construct that marker until a trusted verifier has authenticated the principal, payload and receipt. This type boundary is not itself cryptographic authentication, and the repository does not yet implement that verifier or a production call path. The model stores accepted obligations in memory, and it has no settlement/release operation or cross-process durability.

Ten Rust unit-test definitions cover exact replay across shard/epoch aliases, amount/principal/instrument/payload conflicts, principal-cap aggregation, missing cap, malformed/zero input, checked overflow, zero cap, and independent principal/instrument scopes. They require an exact-head Rust 1.99.1 receipt before being described as passing.

## Local test and deterministic result

The three Python suites now contain **24 tests total**: 10 NCF-ALLOC-3, 5 NCF-ALLOC-4 and 9 NCF-ALLOC-5; all 24 passed in the recorded local run. Separately, Rust source contains 21 allocation tests and 10 obligation-admission test definitions; neither group has an execution pass recorded. The NCF-ALLOC-5 deterministic scenarios replay one 4,000-unit underlying obligation through 1, 4, 8, 16 or 32 shard aliases; every case records one unique accepted obligation, 4,000 outstanding units, one initial acceptance and alias_count - 1 idempotent replays.

Files:

- ncf_alloc5_obligation_admission.py — sequential evidence admission model
- test_ncf_alloc5_obligation_admission.py — nine regression tests
- ncf-alloc5-obligation-admission-results.json — fixed scenario results with source SHA-256 and explicit claim ceiling
- ncf-alloc5-obligation-campaign.json — machine-readable Python oracle campaign and acceptance gates
- src/obligation.rs — typed Rust sequential reference model (10 unrun test definitions)

## Production design questions not answered by the model

1. Who assigns canonical principal IDs, and how are delegates/organizations bound to them?
2. Which evidence proves a queued obligation is real and eligible? Who operates the verifier, and how can a member dispute its decision?
3. What is the global deduplication namespace and how are accepted, reversed, expired and settled obligation states retained across epochs?
4. How does admission behave under concurrent submissions from different peers? The implementation must offer serializable/linearizable uniqueness or a safe invariant-confluent partition strategy; a Python dictionary is not that proof.
5. How are privacy, recovery, key rotation, verifier compromise and migration handled without allowing duplicate claims?

## Qualification path

First port the oracle to typed Rust, reuse existing authorization/reservation primitives rather than duplicate them, and add property tests for duplicate and reordered histories. Then use a durable idempotency store and fault injection for concurrent retries, crashes and partitions. Only after the source-level state transition and authoritative verifier are established should a multi-agent/Holochain test attempt to qualify cross-agent duplicate rejection.

The expected safety gates are zero duplicate accepted obligation IDs, zero cap overshoot, no mutation on rejection, exact receipt replay for matching payloads, rejection of conflicting payloads, and zero acknowledged obligation loss. Performance should be reported for the full admission and settlement path, not just the allocation formula.


## Follow-on finding: sequential admission still needs atomic concurrency semantics

The next [NCF-ALLOC-6 campaign](NCF-ALLOC-6-admission-races.md) enumerates 70 order-preserving interleavings of two four-stage requests. It shows why a correct sequential obligation book does not establish a safe concurrent admission path: stale dedup and cap reads may race before commit. See its separate results and caveat; the model is not a database test.
