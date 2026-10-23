# Nested currency reference model

This standalone, dependency-free crate is a **reference model**, not a Holochain zome, payment rail, cryptographic authorization implementation, or production ledger.

It models these narrow properties:

1. Instrument IDs are authoritative; ticker collisions do not alias instruments or balances.
2. Authorization and mutual-credit mutations resolve profiles from a canonical registry rather than accepting arbitrary caller-supplied profiles; denomination identity and scale cannot silently change for an instrument ID, and same-revision policy mutations are rejected.
3. Delegated authority binds issuer, delegate, instrument, action, profile revision, expiry, and an amount ceiling; missing or negative amounts fail closed.
4. Community mutual-credit updates are atomic at the model boundary, replay-idempotent, and preserve zero net position per instrument.
5. Federation clearing enforces per-edge and per-instrument aggregate community gross-exposure limits; each community must explicitly configure a cap for the exact instrument before new obligations are accepted. A missing cap fails closed, a zero cap blocks new exposure, and unlike instrument units are never added as raw numbers. Opposing obligations remain separate gross legs by default; signed net balance is diagnostic only, and every settlement names its direction. Agreements bind instrument/revision and allow already-incurred obligations to settle after new obligations are paused.

It does **not** prove real-world service delivery, legal status, collateral/reserves, key security, Holochain DHT integrity, multi-agent race safety, external settlement finality, or that every production code path enforces these rules.

## Run locally

From the repository root, with a stable Rust toolchain installed:

```sh
cargo fmt --manifest-path mycelix-workspace/simulations/nested-currency-reference/Cargo.toml -- --check
cargo test --locked --manifest-path mycelix-workspace/simulations/nested-currency-reference/Cargo.toml
```

The test corpus contains deterministic negative controls plus a million-operation conservation stress test and 100,000 retry checks. It includes profile-registry binding, missing/negative amount caps, stale revision rejection, aggregate exposure caps scoped to the exact instrument, missing-cap and zero-cap negative controls, failure atomicity, ticker-collision isolation, denomination stability, settlement-after-pause semantics, and a reverse-direction obligation regression proving gross amounts do not implicitly net. These are **targets encoded as tests**, not evidence of a passing run until the exact committed revision has been compiled and executed.

## Run a reproducible benchmark

The release benchmark refuses to emit a record unless source SHA, NCF-1 spec digest, rustc version, and toolchain identity are supplied. The wrapper calculates these from the clean checkout and stores a checksum receipt outside the repository by default. It also keeps Cargo build output outside the checkout.

```sh
bash mycelix-workspace/simulations/nested-currency-reference/run-bench.sh 1000000 1000 0x4d5943454c495831
```

Arguments are operation count, participant count, and deterministic seed. Defaults are 1,000,000 operations, 1,000 participants, and the same seed. The runner bounds the workload at two million operations; use larger scale campaigns only after the memory and runtime envelope has been qualified. Override `NCF_OUTPUT_FILE` and `NCF_RECEIPT_FILE` to choose output paths.

The JSON record measures one-process reference-model throughput and p50/p95/p99 *batch* latency, checking zero net position after each batch. Its throughput includes registry lookup, event persistence, BTreeMap accounting and conservation scans. It is **not** a Holochain/DHT, multi-agent, external-settlement or production throughput claim. Compare builds only under matched source/toolchain/hardware/workload settings, across multiple recorded seeds, and retain the output SHA-256 receipt.

## Next qualification stages

- Add property-based operation sequences with reproducible seeds.
- Add multi-agent tests for concurrent submissions, replay, reordering, and partial-write recovery.
- Bind production operations to canonical serialized profile/policy digests and consumed authorization IDs.
- Extend profile-version tests into signed policy-digest and authorized migration receipts; the in-memory snapshot is not a cryptographic authority proof.
- Integrate only after the existing Finance author-binding, issuer-scoped capability, typed-amount, and settlement/finality requirements qualify.
