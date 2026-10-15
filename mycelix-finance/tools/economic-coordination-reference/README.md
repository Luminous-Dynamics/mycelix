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

The reference model currently contains **33 authored unit tests**. This is a source-level count, not a claim that the tests have executed successfully.

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
