# Economic Coordination Reference v1

**Status:** research/reference implementation; not production finance, custody, or settlement software.

This standalone, dependency-free Rust crate implements one narrow vertical slice proposed by the Mycelix Economic Coordination Contract: a resource/service need moves through authorization, bounded capacity reservation, a policy-bound obligation, delivery claim, independent verification, and an internal double-entry reference-ledger posting.

Related work:
- Mycelix public PR #4959, *Economic OS evidence gates and Economic Coordination Contract v1*.
- Mycelix issue #3343, *Fiat, Web3, stablecoin, mutual-credit and external-instrument interoperability fabric*.
- Mycelix issue #3345, *Settlement rail profiles, finality algebra, and reconciliation receipts*.
- Symthaea issue #7188 / PR #7189, *Socioeconomic interoperability laboratory*.
- [ValueFlows 1.0 specification](https://www.valueflo.ws/specification/vfspec/) for economic intents, commitments, claims, resources, and observed events.

## Interoperability boundary

This crate intentionally does not serialize ValueFlows objects yet. A follow-on adapter should propose explicit mappings rather than equating similarly named records:

- `NeedRequest` is a candidate projection to a ValueFlows `Intent` (request/offer), not proof that the request is authorized or that capacity exists.
- `ObligationTerms` is a candidate commitment/contract projection with Mycelix-specific instrument, policy, and settlement metadata preserved.
- `DeliveryClaim` is a provider assertion pending verification; it must not be emitted as an observed `EconomicEvent` simply because it was recorded.
- A ValueFlows `Claim` has its own semantics: a claim for a future economic event in reciprocity for an event that already occurred. It is not a generic replacement for reputation, authority, an obligation, or an internal settlement receipt.
- A verified delivery may support an `EconomicEvent` projection, while the financial settlement record remains separately bound to a defined claim/settlement profile.

The ValueFlows specification distinguishes planned flows (`Intent` / `Commitment`) from observed flows (`EconomicEvent`) and defines `Claim` separately. Any adapter must retain those distinctions and declare which fields are native, profile extensions, or unrepresented. This README records an adapter proposal, not an implemented or qualified mapping.

## What it demonstrates

- Integer quantities and instrument-specific atomic units; no floating-point settlement arithmetic.
- Explicit separation of financial obligation, resource capacity, delivery claim, verification, and internal ledger posting.
- Verification status and evidence origin are separate axes: evidence may be `Verified` as a simulation and still have `SyntheticSimulation` origin. Authorization fails closed unless authority, rights, and ecological evidence are both `Verified` and `LiveDomain`.
- A commitment must bind the policy digest that authorized it.
- Capacity reservation is bounded in this serial reference model, and pre-commit cancellation releases its reservation.
- Only the contract's named provider may submit its delivery claim; an unrelated source cannot enter the verification path. Delivery remains a claim until an independent verifier confirms the exact resource, unit, quantity, and deadline.
- Independent verification binds to the exact provider delivery-claim ID as well as a separate verifier evidence ID.
- Unknown/stale/contested verification stays indeterminate; short or mismatched delivery becomes disputed. A `SyntheticSimulation` verification result, delivery claim, or dispute reassessment cannot promote a delivery to `Verified`; a synthetic-origin obligation cannot be committed.
- A dispute may be reassessed only by a different independent resolver bound to the exact delivery-claim ID. A resolver can contribute only one reassessment per dispute; only confirmation of full delivery resolves it to Verified. Adverse/incomplete reassessment is journaled and leaves the obligation disputed without erasing the obligation. When reassessment resolves the dispute, the primary verification accessor points to the accepted decision while the append-only verification history retains the earlier verdict and all later review decisions.
- Reassessment returns the resulting lifecycle state whether it resolves or leaves the dispute open, avoiding an error return after an already-recorded state change.
- Lifecycle event times cannot move backwards.
- Verified delivery must first consume its reserved physical capacity, producing a distinct `CapacityConsumed` lifecycle state. Consumption remains recorded even if a later financial posting fails.
- Consumed capacity is not made available to another reservation; the reference tracks reserved and consumed quantities separately.
- Settlement is attempted only after verified delivery and capacity consumption; account identity, denomination, arithmetic, and both balance bounds are checked before either balance mutates.
- Transaction IDs are single-use for reservations, including after cancellation; obligation IDs are single-use for ledger postings.
- Duplicate account/capacity declarations are non-destructive.
- Replayed lifecycle transitions cannot post the same contract twice.

The v1 reference, partial-fulfilment tranche experiment, escrow-rights ledger, isolated-replica experiment, and coordinated claim-identity registry currently contain **78 authored unit tests** (38 v1, 14 tranche-ledger, 7 escrow-ledger, 9 isolated-replica, and 10 claim-registry tests), plus **3 authored integration tests**: **81 authored test functions total**. This is a source-level count, not a claim that the tests have executed successfully.

## Experimental partial-fulfilment tranche ledger

The additive `fulfillment_tranches` Rust module explores a ValueFlows-aligned gap: one request may be fulfilled by multiple delivery tranches, each independently verified. It does not change the existing v1 single-claim lifecycle and is not wired into the v1 capacity book or settlement engine.

- Pending, indeterminate, and disputed tranches reserve their full claimed quantity. Verified accepted quantities count only when both evidence status and origin are acceptable.
- Partial acceptance counts only the accepted amount and releases the unaccepted remainder for a replacement tranche.
- Duplicate claim IDs and evidence IDs are rejected. Final tranches cannot be accepted twice. Disputed resolution requires a different verifier.
- Claims and verification decisions are retained in an append-only journal. This is a serial reference model, not a distributed concurrency protocol or financial posting authorization.
- Fourteen unit regression tests cover multiple tranches, partial acceptance, replay, overcommitment, over-fulfilment, unknown/synthetic evidence, independence, dispute resolution, deadlines, and journal ordering. They remain unexecuted until qualification records show otherwise.
- `tests/fulfillment_generated_sequences.rs` adds three dependency-free integration tests: 4,096 seeded ledgers × 32 generated claim attempts (131,072 attempts) checking quantity, journal-time, and aggregate invariants; 100 same-seed replay comparisons; and an intentional counterexample where two isolated ledgers each accept 10 units against the same 10-unit request. The counterexample demonstrates a known distributed-safety gap; it is not a desired protocol property. These tests are authored, not yet reported as executed.

## Partial-fulfilment experiment: proposed benchmarks

These are acceptance targets for the tranche prototype, not measured results:

| Benchmark | Target |
|---|---:|
| Conservation | 0 accepted units above request quantity across 100,000 generated transition sequences |
| Evidence gate | 0 synthetic/unknown-origin units counted as accepted |
| Replay resistance | 0 duplicate claim IDs or evidence IDs accepted across 10,000 replay attempts |
| Pending-claim reservation | 0 overcommitted quantities across 100,000 generated claim interleavings |
| Dispute audit | 100% of accepted decisions preserved in append-only verification history |
| Deterministic replay | 100/100 same-input runs produce identical ordered journal digests |
| Partial fulfillment | Exact agreement between per-tranche accepted quantities and aggregate accepted quantity in every generated case |

The implementation is currently a serial in-memory reference. The generated-state harness targets 131,072 claim attempts per invocation and 100 exact-seed replay comparisons; these are test workload sizes, not measured pass results. A dedicated counterexample asserts that two disconnected replicas can accept 20 units combined against a 10-unit request. A successor protocol must either use a shared reservation authority or specify bounded escrow/partition semantics and then demonstrate zero global over-allocation across generated partitions, retries, and message reordering. Concurrency/interleaving campaigns must not be presented as evidence of distributed safety until a concurrent or replicated protocol is modeled and qualified. A future integration should map tranche claims and acceptance decisions to ValueFlows semantics explicitly; ValueFlows defines an economic event as fulfilling a commitment completely or partially, but this module is not yet a ValueFlows adapter.

## Experimental replica escrow rights

The additive `fulfillment_escrow` module explores a way to bound local reservations without a globally available coordinator: partition a request quantity into disjoint, preallocated per-replica rights. A replica can reserve only rights it already holds. Pending reservations remain locked; finalization consumes only accepted units and returns the rejected remainder. Transfers move available rights and are single-use inside this serial reference ledger.

This is an accounting model, not a distributed implementation. It does not authenticate transfer certificates, prevent a partitioned replica from forking its local state, or establish Byzantine safety. A production protocol must either prevent rights transfers while disconnected or add durable authenticated single-spend transfer semantics and prove recovery/reconciliation safety.

See [FULFILLMENT_ESCROW_BENCHMARKS.md](FULFILLMENT_ESCROW_BENCHMARKS.md) for adversarial test targets. All targets remain proposals until exact-head execution evidence exists.

## Isolated-replica fixed-rights experiment

The additive `fulfillment_escrow_partition` module models the quantity-accounting contract across separate per-replica states. Each replica owns a disjoint preallocated share; reservations mutate only that replica's balance, and partial finalization consumes the accepted amount while returning the rejected remainder. It deliberately exposes no transfer or timeout-release API during a partition. The aggregate wrapper acts as an independent test oracle across replicas.

The deterministic interleaving campaign covers 1,000 seeds × 128 attempted transitions (128,000 operation steps) with 2–8 replicas and request quantities up to 97 units. These are authored generated checks, not a distributed proof and not an observed test pass.

This model's guarantee depends on assumptions that remain outside it: initial grants are issued uniquely and durably; local spent state cannot be rolled back or forked; accepted evidence is genuine; and a replica cannot use duplicated credentials to instantiate a second copy of its share. It also does not deduplicate the same external delivery identity across replica-local histories: `same_external_claim_replayed_across_replicas_is_a_known_idempotency_counterexample` demonstrates that a repeated claim ID may be accepted twice while the quantity invariant still holds. Quantity conservation therefore does not imply exactly-once delivery evidence. A successor needs a globally unique/single-use evidence or reservation identity with explicit partition behavior, plus crash/restart, snapshot rollback, transfer single-spend semantics, and reconciliation rules.

## Coordinated claim-identity experiment

The additive `fulfillment_claim_identity` module models the alternate consistency choice: all claims pass through one serialized authority that binds an external claim ID to one request, one replica, one exact SHA-256-shaped delivery digest, and one claimed quantity. Exact retries are idempotent; conflicting reuse, including the same claim ID on another request or replica, is rejected. Finalization is also idempotent only when the accepted quantity is identical; conflicting finalizations fail closed.

This demonstrates the stronger exactly-once *identity bookkeeping* available under a single authority. It is not a distributed consensus protocol, and the digest's syntax is not signature or evidence authentication. During a network partition, a replica unable to reach the registry must stay pending or decline finalization. An in-memory registry cannot prove durable single-spend behavior after crash, restore, fork or backup cloning.

The experiments therefore present two distinct operational choices: fixed local quantity rights improve partition availability within a finite allocation, while globally deduplicated claim identities require a shared serialization point or an explicitly proven distributed equivalent. Do not claim both properties from either toy model alone.

## Explicit limitations

- Evidence status and origin are caller-supplied inputs. Separating `EvidenceStatus` from `EvidenceOrigin` prevents a legitimately verified simulation result from being treated as live-domain evidence, but the enum itself does not prove provenance: a dishonest caller can mislabel origin as `LiveDomain`. This crate does not authenticate signatures, establish physical truth, or qualify who may issue evidence.
- The capacity book is in-memory and serial. It does not prove distributed reservation safety under DHT concurrency, network partitions, or Byzantine peers.
- The ledger is in-memory and internal-only. It does not move fiat, SAP, TEND, stablecoins, bank deposits, or any external asset; it does not establish legal settlement or discharge.
- The model does not decide which socioeconomic policy is legitimate, optimize resource allocation, establish fairness, or prove real-world economic outcomes.
- It is not wired into Holochain zomes or production finance authority paths.

## Reproduce

From the repository root:

```sh
cargo fmt --manifest-path mycelix-finance/tools/economic-coordination-reference/Cargo.toml -- --check
cargo test --locked --manifest-path mycelix-finance/tools/economic-coordination-reference/Cargo.toml
```

The focused GitHub Actions workflow runs these commands against the exact pull-request head. Until that run has completed successfully, the test suite should be described as authored—not passed.
