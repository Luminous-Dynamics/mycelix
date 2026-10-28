# Nested currency reference model

This standalone, dependency-free crate is a **reference model**, not a Holochain zome, payment rail, cryptographic authorization implementation, or production ledger.

It models these narrow properties:

1. Instrument IDs are authoritative; ticker collisions do not alias instruments or balances.
2. Authorization and mutual-credit mutations resolve profiles from a canonical registry rather than accepting arbitrary caller-supplied profiles; denomination identity and scale cannot silently change for an instrument ID, and same-revision policy mutations are rejected.
3. Delegated authority binds issuer, delegate, instrument, action, profile revision, expiry, and an amount ceiling; missing or negative amounts fail closed.
4. Community mutual-credit updates are atomic at the model boundary, replay-idempotent, and preserve zero net position per instrument.
5. Federation clearing agreements resolve instrument IDs through the canonical registry and pin the exact profile revision; unknown IDs and stale profile revisions reject, and non-transferable reputation cannot become a spendable clearing asset.
6. Partition-safe reservation reference model allocates disjoint gross-exposure slices per exact community/instrument/agreement/shard/epoch. Each shard consumes only its own slice; settled capacity is reusable only inside that reservation, and old-epoch capacity is not silently reassigned. Clearing also enforces per-edge and per-instrument aggregate community gross-exposure limits; each community must explicitly configure a cap for the exact instrument before new obligations are accepted. A missing cap fails closed, a zero cap blocks new exposure, and unlike instrument units are never added as raw numbers. Opposing obligations remain separate gross legs by default; signed net balance is diagnostic only, and every settlement names its direction. Agreements bind instrument/revision and allow already-incurred obligations to settle after new obligations are paused.

It does **not** prove real-world service delivery, legal status, collateral/reserves, key security, Holochain DHT integrity, multi-agent race safety, external settlement finality, or that every production code path enforces these rules. The reservation model assumes reservation ownership is unique and stable; it does not implement signed fencing receipts, reservation transfer, or safe capacity reclamation after a partition. Those require a separate coordinated protocol.

## Static escrow allocation frontier

The dependency-free [allocation module](src/allocation.rs) builds deterministic pre-partition risk-right plans using equal split, demand-weighted mean, robust-quantile weighting, equal-floor hybrids, and a median-capped equal-floor policy. Each demand and plan is bound to one canonical `instrument_id`; mixed/missing instrument identity is rejected. Demand-dependent policies reject missing per-shard history rather than treating an unobserved community as zero demand. The allocator uses checked integer arithmetic and stable largest-remainder tie-breaking; an optional contingency reserve remains unallocated and cannot be spent by disconnected shards. The median cap limits the influence of extreme historical weights under a fixed identity set, but does not authenticate demand or resist Sybil identities.

The [NCF-ALLOC-1 specification](docs/NCF-ALLOC-1-static-allocation.md) defines the algorithm boundary and claim ceiling; [NCF-ALLOC-3](research/NCF-ALLOC-3-bounded-influence.md) defines bounded-influence demand weighting and its adversarial limitations. The [matched-trace benchmark binary](src/bin/ncf-allocation-bench.rs) compares eight policies on identical synthetic stationary and shifted-demand scenarios (16 policy/mode reports), while [the runner](run-allocation-bench.sh) binds output and binary hashes to the exact source/toolchain. A separate Python research harness under research/ ran a 20,000-case minority claim-inflation sweep; it is not Rust or distributed evidence. The follow-on [NCF-ALLOC-4 identity-splitting study](research/NCF-ALLOC-4-identity-splitting.md) keeps one principal's true workload fixed while multiplying shard IDs, demonstrating where median caps cease to help; it is also Python-only research, not an identity protocol. The typed Rust [obligation admission module](src/obligation.rs) now models canonical work-digest deduplication, principal/instrument caps, and receipts behind a crate-internal VerifiedObligation boundary. Exact request-ID retries also bind verifier version and verifier-receipt digest. Its eleven Rust unit-test definitions have not been compiled/run. The [NCF-ALLOC-5 research note](research/NCF-ALLOC-5-obligation-admission.md) also contains a Python sequential oracle, and [NCF-ALLOC-6](research/NCF-ALLOC-6-admission-races.md) enumerates concurrent check/commit interleavings. These clarify required invariants but do not qualify production verification, durable atomicity or Holochain multi-agent behavior. The dedicated [allocation workflow](../../../../.github/workflows/ncf-allocation-benchmark.yml) checks formatting, runs the crate tests and clippy, and then emits the simulation receipt.

Example:

```sh
bash mycelix-workspace/simulations/nested-currency-reference/run-allocation-bench.sh \
  16 32 1000 100000 50000 0x4d5943454c495831
```

The demand generator is synthetic. This is not yet a real-demand or distributed-concurrency benchmark. Static allocation cannot reclaim an unreachable owner's spend rights during a partition; it only experiments with placing already-authorized rights before the partition. Benchmark output must be paired with safety results, and no policy may be selected on held-out evaluation traces.

## Exact-head Rust qualification

The dedicated [Rust reference qualification workflow](../../../../.github/workflows/ncf-rust-model.yml) runs even on draft PRs, pins the source checkout to the triggering SHA, installs Rust 1.99.1 with rustfmt/clippy, and retains the logs and receipt for 30 days. Its [runner](run-ncf-rust-qualification.sh) runs format, test, and clippy independently, then emits one receipt only after recording each result. Build output and logs stay outside the source checkout.

This workflow is **defined but not yet qualified**. No pass is claimed until the exact-head receipt shows `fmt_status=test_status=clippy_status=passed`; a skipped generic run is not evidence. Clippy and tests qualify the sequential reference crate only, not multi-agent or Holochain safety.

```sh
bash mycelix-workspace/simulations/nested-currency-reference/run-ncf-rust-qualification.sh
```

## Run locally

From the repository root, with a stable Rust toolchain installed:

```sh
cargo fmt --manifest-path mycelix-workspace/simulations/nested-currency-reference/Cargo.toml -- --check
cargo test --locked --manifest-path mycelix-workspace/simulations/nested-currency-reference/Cargo.toml
```

The crate defines 32 Rust test functions across the NCF clearing model, reservation module, and sequential fencing model. Coverage includes canonical registry binding at clearing-agreement creation; unknown-instrument, stale-profile, and reputation-family rejection; amount/cap negative controls; instrument-scoped aggregate limits; failure atomicity; ticker collision and denomination stability; gross-by-direction clearing; direction-bound settlement replay; disjoint shard reservations; zero-cap behavior; cross-reservation replay rejection; and immutable counterparty binding across shards, epochs, and instruments. The existing 1,000,000-operation and 100,000-retry tests remain definitions, not evidence of passing execution. These are **targets encoded as tests**, not evidence of a passing run until the exact committed revision has been compiled and executed.

## Run a reproducible benchmark

The release benchmark refuses to emit a record unless source SHA, NCF-1 spec digest, rustc version, and toolchain identity are supplied. The wrapper calculates these from the clean checkout and stores a checksum receipt outside the repository by default. It also keeps Cargo build output outside the checkout.

```sh
bash mycelix-workspace/simulations/nested-currency-reference/run-bench.sh 1000000 1000 0x4d5943454c495831
```

The partition-safe reservation model has its own exact-head runner. Arguments are operation count, shard count, and deterministic seed; use at least two shards to exercise cross-reservation replay controls:

```sh
bash mycelix-workspace/simulations/nested-currency-reference/run-reservation-bench.sh 1000000 100 0x4d5943454c495831
```

Arguments are operation count, participant count, and deterministic seed. Defaults are 1,000,000 operations, 1,000 participants, and the same seed. The runner bounds the workload at two million operations; use larger scale campaigns only after the memory and runtime envelope has been qualified. Override `NCF_OUTPUT_FILE` and `NCF_RECEIPT_FILE` to choose output paths.

The mutual-credit JSON record measures one-process reference-model throughput and p50/p95/p99 *batch* latency, checking zero net position after each batch. The reservation JSON record measures allocated/used/unused capacity, cap utilization, per-shard outstanding, accepted obligations, settlements, over-slice rejections, retry controls, throughput, and batch latency. Both runners are single-process reference measurements, not distributed/Holochain production benchmarks. Its throughput includes registry lookup, event persistence, BTreeMap accounting and conservation scans. It is **not** a Holochain/DHT, multi-agent, external-settlement or production throughput claim. Compare builds only under matched source/toolchain/hardware/workload settings, across multiple recorded seeds, and retain the output SHA-256 receipt.

## Fencing protocol

The next design boundary is documented in [NCF-FEN-1: fencing and safe reallocation of partitioned risk rights](docs/NCF-FEN-1.md). It specifies the state machine, receipt bindings, fence proof modes, required failure injections, independent-oracle benchmarks, and conditions for ever reclaiming capacity from an old epoch. The machine-readable corpus contains 18 planned conformance scenarios: seven map to six distinct Rust reference-model test definitions (including the new 10,000-scenario deterministic lifecycle sweep); four specify independent TLA+ negative controls (timeout reallocation, stale-owner writes, duplicate shard allocation, and reservation resize during a partition); seven scenarios remain not implemented or fully qualified. Two safe TLA+ models and four expected-negative-control mutants are documented under `formal/README.md`; `.github/workflows/ncf-fen-model-check.yml` now provides a dedicated, exact-PR-head TLC job even for draft PRs. TLC has not yet been confirmed to run successfully, and the expected outcomes remain unconfirmed. All Rust test definitions remain unexecuted. The corpus has 26 invariants, four benchmark campaigns, and two qualification campaigns; the 10,000-schedule explorer and real multi-agent qualification remain future work. This is a proposal, not implementation or test evidence. Until a fencing authority is chosen and qualified, potentially live old-epoch capacity remains charged.

## Next qualification stages

- Add property-based operation sequences with reproducible seeds.
- Add multi-agent tests for concurrent submissions, replay, reordering, and partial-write recovery.
- Add a coordinated reservation-retirement/fencing protocol before reclaiming unused budget from an unreachable shard; measure stranded capacity vs safety, and inject stale-owner writes.
- Bind production operations to canonical serialized profile/policy digests and consumed authorization IDs.
- Extend profile-version tests into signed policy-digest and authorized migration receipts; the in-memory snapshot is not a cryptographic authority proof.
- Integrate only after the existing Finance author-binding, issuer-scoped capability, typed-amount, and settlement/finality requirements qualify.


The next research increment, [NCF-ALLOC-5 obligation admission](research/NCF-ALLOC-5-obligation-admission.md), changes the unit of account from self-reported shard demand to unique eligible obligations scoped to a canonical principal and instrument. The Python oracle has nine tests; the new [typed Rust reference model](src/obligation.rs) adds ten test definitions for replay, payload conflicts, principal/instrument caps, overflow, and fail-closed inputs. Its trusted-obligation marker is constructible only inside this crate, but no real verifier is implemented here. NCF-QUAL-007 defines the unrun Rust gate; NCF-BENCH-009 defines future end-to-end admission/replay/cap-contention measurements. Neither model claims cryptographic, concurrent, or Holochain qualification; see `research/ncf-alloc5-obligation-campaign.json`.
