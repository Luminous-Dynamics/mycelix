# NCF-ALLOC-5 — Canonical obligation evidence and principal-scoped admission

**Status:** independent sequential Python reference model; nine unit tests pass. This is a policy-oracle model, not cryptographic verification, Rust qualification, concurrent execution, distributed/Holochain behavior, or production settlement.

## Why the paradigm must change

NCF-ALLOC-3 demonstrated that lower-median caps can reduce influence from a minority's inflated claims when shard IDs are fixed. NCF-ALLOC-4 then split the same 4,000 units of real work across 1–32 IDs and showed those arithmetic caps cease to help once identity slots multiply. More elaborate demand caps cannot solve a missing identity and duplicate-work policy.

NCF-ALLOC-5 shifts the unit of account from **reported shard demand** to **unique eligible obligations owned by a canonical principal**. It models a sequential admission oracle with:

- A unique obligation_id bound to principal, instrument and amount. Cross-shard or cross-epoch exact retries return the original receipt and do not add demand again.
- Same-ID changed amount, principal, or instrument is rejected without changing the book.
- The accepted payload also includes a canonical obligation-payload digest; reusing an ID for different underlying work is rejected even when amount/principal/instrument match.
- Principal/instrument caps aggregate distinct obligations across all shard aliases.
- Missing caps, malformed identifiers, malformed verifier receipt digests, and verifier-rejected evidence fail closed without consuming the ID.
- A rejected cap attempt does not burn the obligation ID; a corrected amount can be retried.

The model requires a canonical payload digest, but computes that digest from caller-supplied test content; digest binding is not proof of the truth of that content. The reference model's verifier_receipt_ok flag is an assumption supplied to the model. Hashes are deterministic test receipt digests, not signatures. No cryptographic principal resolution, verifier, database transaction, or distributed linearizability is implemented here.

## Local test and deterministic result

The three Python suites now contain **24 tests total**: 10 NCF-ALLOC-3, 5 NCF-ALLOC-4 and 9 NCF-ALLOC-5. The latest combined run passed all 24. The NCF-ALLOC-5 deterministic scenarios replay one 4,000-unit underlying obligation through 1, 4, 8, 16 or 32 shard aliases; every case records one unique accepted obligation, 4,000 outstanding units, one initial acceptance and alias_count - 1 idempotent replays.

Files:

- ncf_alloc5_obligation_admission.py — sequential evidence admission model
- test_ncf_alloc5_obligation_admission.py — nine regression tests
- ncf-alloc5-obligation-admission-results.json — fixed scenario results with source SHA-256 and explicit claim ceiling
- ncf-alloc5-obligation-campaign.json — machine-readable campaign and acceptance gates

## Production design questions not answered by the model

1. Who assigns canonical principal IDs, and how are delegates/organizations bound to them?
2. Which evidence proves a queued obligation is real and eligible? Who operates the verifier, and how can a member dispute its decision?
3. What is the global deduplication namespace and how are accepted, reversed, expired and settled obligation states retained across epochs?
4. How does admission behave under concurrent submissions from different peers? The implementation must offer serializable/linearizable uniqueness or a safe invariant-confluent partition strategy; a Python dictionary is not that proof.
5. How are privacy, recovery, key rotation, verifier compromise and migration handled without allowing duplicate claims?

## Qualification path

First port the oracle to typed Rust, reuse existing authorization/reservation primitives rather than duplicate them, and add property tests for duplicate and reordered histories. Then use a durable idempotency store and fault injection for concurrent retries, crashes and partitions. Only after the source-level state transition and authoritative verifier are established should a multi-agent/Holochain test attempt to qualify cross-agent duplicate rejection.

The expected safety gates are zero duplicate accepted obligation IDs, zero cap overshoot, no mutation on rejection, exact receipt replay for matching payloads, rejection of conflicting payloads, and zero acknowledged obligation loss. Performance should be reported for the full admission and settlement path, not just the allocation formula.
