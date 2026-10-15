# Economic Coordination Reference v1

**Status:** research/reference implementation; not production finance, custody, or settlement software.

This standalone, dependency-free Rust crate implements one narrow vertical slice proposed by the Mycelix Economic Coordination Contract: a resource/service need moves through authorization, bounded capacity reservation, a policy-bound obligation, delivery claim, independent verification, and an internal double-entry reference-ledger posting.

Related work:
- Mycelix public PR #4959, *Economic OS evidence gates and Economic Coordination Contract v1*.
- Mycelix issue #3343, *Fiat, Web3, stablecoin, mutual-credit and external-instrument interoperability fabric*.
- Mycelix issue #3345, *Settlement rail profiles, finality algebra, and reconciliation receipts*.
- Symthaea issue #7188 / PR #7189, *Socioeconomic interoperability laboratory*.

## What it demonstrates

- Integer quantities and instrument-specific atomic units; no floating-point settlement arithmetic.
- Explicit separation of financial obligation, resource capacity, delivery claim, verification, and internal ledger posting.
- Authorization fails closed if authority, rights, or ecological evidence is not verified, or if a required gate denies the request.
- A commitment must bind the policy digest that authorized it.
- Capacity reservation is bounded in this serial reference model, and pre-commit cancellation releases its reservation.
- Delivery is only a claim until an independent verifier confirms the exact resource, unit, quantity, and deadline.
- Unknown/stale/contested verification stays indeterminate; short or mismatched delivery becomes disputed.
- Settlement is attempted only after verified delivery; account identity, denomination, arithmetic, and both balance bounds are checked before either balance mutates.
- Replayed lifecycle transitions cannot post the same contract twice.

## Explicit limitations

- Evidence statuses and policy digests are inputs from a caller. This crate does not authenticate signatures, establish physical truth, or qualify who may issue evidence.
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
