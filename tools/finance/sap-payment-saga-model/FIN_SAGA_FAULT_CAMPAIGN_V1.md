# SAP Payment Saga deterministic fault campaign v1

**Status:** deterministic effects-disabled model campaign. A 10,000-trace Rust/Go presubmit and a separately dispatchable 1,000,000-trace campaign are implemented in this branch; results are pending its exact-head workflow. This is not a Holochain runtime test, production payment test or distributed-system proof.

## Why this layer exists

The Rust and Go reference implementations already have named examples for immutable intent identity, same-effect retry, typed effect evidence, a durable-history reducer and owner-scoped checkpoint subjects. Hand-authored fixtures are necessary but do not explore combinations of timing/failure points at scale.

This increment adds a small language-neutral schedule generator and runs each trace through the existing Rust and Go reducers. It does not add another payment state machine, a new ledger, a scheduler or a production mutation path.

## CounterMix64V1 schedule

All arithmetic is unsigned 64-bit arithmetic modulo 2^64.

- Root seed: 4d594843414d5031 (hex).
- Sequence multiplier: 9e3779b97f4a7c15.
- Field-domain multiplier: d1b54a32d192ed03.
- Finalizer multipliers: bf58476d1ce4e5b9 and 94d049bb133111eb.

For trace sequence n and field domain d:

    z = root XOR (n * sequenceMultiplier) XOR (d * domainMultiplier)

Apply the SplitMix64 finalizer:

    z1 = (z XOR (z >> 30)) * 0xbf58476d1ce4e5b9
    z2 = (z1 XOR (z1 >> 27)) * 0x94d049bb133111eb
    word = z2 XOR (z2 >> 31)

Every multiplication wraps modulo 2^64. There is no mutable PRNG state and no draw-order dependency. A field is solely a function of (root seed, sequence, domain).

| Domain | Field | Derivation |
|---:|---|---|
| 0 | Trace seed | word |
| 1 | Principal | 1 + word % 1,000,000 minor units |
| 2 | Fee mode | zero fee when word % 5 == 0 |
| 3 | Fee amount | otherwise 1 + word % 500 minor units |
| 4 | Local indeterminate retries | word % 4 |
| 5 | Treasury indeterminate retries | 0 for zero fee; else word % 4 |
| 6 | Lost-commit-ack mask | bit 0 = local; bit 1 = Treasury; Treasury bit cleared for zero fee |

The shared fixture tools/finance/sap-payment-saga-go-verifier/fixtures/sap-payment-saga-fault-campaign-v1.kat freezes eight sequence outputs, including zero-fee and nonzero-fee cases. Rust and Go independently regenerate the same seven-field records from this fixture.

## What each generated trace does

Each sequence constructs one immutable SAP payment intent and runs the existing models:

1. Create the sender-scoped operation once and append its creation event.
2. Authorize and prepare the local payment transaction.
3. Repeat zero to three Indeterminate -> PrepareSameEffect cycles using the exact local effect ID.
4. Commit the typed local effect evidence. This local effect models the single atomic source-chain transaction: sender debit, recipient credit, Payment, Receipt and Treasury outbox intent.
5. Optionally inject a lost acknowledgement after the local commit. The retry observes the original committed effect ID and replays the same evidence; it must not create a new effect.
6. When the fee is nonzero, prepare the separate Treasury fee effect, repeat zero to three same-ID indeterminate/retry cycles and commit the exact fee receipt.
7. Optionally inject a lost acknowledgement after Treasury commit and retry the same effect/evidence.
8. Replay the append-only event stream against its event-count/tail anchor and compare the reconstructed operation and owner-leg states with the live reducer.

The generator intentionally distinguishes the local atomic principal transaction from the separately recoverable Treasury fee effect. The campaign does not model the local debit and credit as two independent writes; that would contradict the reviewed Holochain source-chain transaction boundary. Treasury is an additional owner effect.

## Independent safety oracle

The trace runner checks integer minor-unit conservation independently from the terminal operation status:

    senderDebit = recipientPrincipalCredit + treasuryFeeCredit

For a zero-fee operation, the Treasury effect is NotRequired and no fee event is generated. For a nonzero fee, the Treasury evidence must bind the exact fee amount and stable Treasury effect identity.

The replay must preserve:
- operation identity;
- local and Treasury effect identities;
- terminal operation status;
- each owner-leg state and its typed evidence;
- event count and exact tail digest.

The input event frames are snapshotted before replay and compared afterward to check replay purity. The Rust and Go campaign tests calculate an aggregate SHA-256 result digest over every plan and its replay outcome. The exact-head workflow compares the Rust and Go digests for the same sequence range.

## Qualification tiers

- Pull-request presubmit: 10,000 deterministic traces in each language; compare aggregate Rust/Go result digests; run existing Rust formatting/tests/Clippy and Go formatting/vet/race tests.
- Full campaign: 1,000,000 deterministic traces in each language via explicit workflow dispatch. The exact-head workflow compares the aggregate result digests and retains raw logs with the checked-out source SHA and toolchain/host information.

A queued or skipped manual campaign is not a pass. Do not infer one million traces ran from the existence of the workflow or the presubmit subset.

## Industry comparison

This campaign borrows techniques from stronger verification systems rather than claiming equivalence:

- FoundationDB describes deterministic whole-cluster simulation with modeled network and machine failures, repeatability and tens of thousands of nightly simulations. The present Mycelix campaign is much narrower: it replays an effects-disabled two-leg reference model and does not simulate a cluster, network or disk. See https://apple.github.io/foundationdb/testing.
- Jepsen generates concurrent operations, records invocation/completion histories, injects nemesis faults and checks histories. This campaign has deterministic bounded schedules but no multi-node nemesis and no linearizability check over live operations. See https://github.com/jepsen-io/jepsen.
- Temporal rebuilds workflow state from durable event history. Mycelix uses a separate immutable journal event stream and owner checkpoint subject, but does not yet provide a production workflow service or durable cross-agent event store. See https://docs.temporal.io/encyclopedia/event-history.
- AWS's transactional-outbox guidance explicitly calls out duplicate delivery and idempotent consumers. The Mycelix Treasury leg therefore needs destination-side idempotency and reconciliation; sender-side journaling alone is not enough. See https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/transactional-outbox.html.
- Holochain's own source-chain model provides atomic writes within one zome invocation and returns a chain-head-moved failure for racing source-chain calls. It does not make a separate cross-agent or external Treasury effect atomic. See https://developer.holochain.org/concepts/3_source_chain/.

The differentiator should be the composition of (a) owner-specific effect identities, (b) replayable evidence histories, (c) exact effect reconciliation and (d) domain-specific authority/finality profiles—not merely a high generated-trace count.

## Current coverage and exclusions

This v1 generator covers:
- zero/nonzero fee paths;
- same-effect local and Treasury indeterminate retries;
- lost acknowledgement after either local or Treasury commit;
- exact same-evidence replay after ambiguous acknowledgement;
- anchored event replay and immutable input check;
- integer-unit principal/fee conservation;
- shared Rust/Go schedule and aggregate result parity.

It does not yet cover the full #6045 mutation programme:
- event truncation, splicing, arbitrary reordering and tampering campaigns;
- changed immutable intent under one client key;
- wrong owner, issuer, authority grant, source ActionHash or stale checkpoint;
- competing source-chain forks, DHT unavailable/false-absence behaviour;
- database crash/fsync, lease fencing, worker process kill or provider-specific retention;
- live conductor, network, bank/payment-rail or legal-finality behavior.

Those must be separate deterministic mutators with their own expected dispositions and, later, runtime tests. A million clean reference-model sequences are not a Jepsen test and do not establish Holochain or provider guarantees.

## Next qualification steps

1. Freeze the exact 10,000-trace Rust/Go aggregate result digest as a known-answer vector after both implementations match.
2. Add deterministic mutation operators and delta-debugging/minimized counterexample output.
3. Extend to owner-scoped checkpoint sequence/frontier changes and conflicting checkpoint histories.
4. Add actual multi-agent Sweettest conductor tests for chain-head movement, same-sender concurrency, lost response recovery and owner-local source-chain authority.
5. Add a matched PostgreSQL SERIALIZABLE + transactional-outbox arm using the exact same serialized commands/fault schedule.
6. Only then compare measured safety, recovery time, resource cost and operational effort with industry baselines.

## Claim ceiling

A passing campaign qualifies only the exact Rust and Go reference models on the generated synthetic schedules. It does not prove source-chain singleton uniqueness, DHT completeness, distributed linearizability, authenticated evidence, Treasury/bank settlement, legal discharge, production performance or production readiness.
