# Economic OS authenticated temporal interval query

Research prototype for complete point-in-time interval-stabbing proofs under one exact scoped root.

## Scope

The reference model answers:

`valid_from <= V && (valid_to == OPEN || V < valid_to)`

for one exact verifier, policy epoch, namespace, subject, claim and source-frontier digest. The output is complete only relative to the supplied tree root and exact scope.

This is **not** a source-completeness proof, root-authentication service, trusted clock, transparency service, witness quorum, payment authorizer or production index. A valid empty result does not establish that every accepted receipt was indexed or that a claim is globally absent.

## Go reference implementation

The first reference implementation is intentionally standard-library-only:

```sh
cd tools/economic/temporal-range-index/reference/go
gofmt -w .
go test -count=1 ./...
go vet ./...
go test -run '^$' -bench '^BenchmarkAuthenticatedIntervalV1$' -benchtime=1x -benchmem
```

The benchmark harness separates tree build, linear-scan semantic baseline, proof generation and proof verification at 1k, 10k and 100k records. The linear scan is a correctness/performance baseline, not an equivalent authenticated query.

## Preliminary one-shot baseline (not a performance qualification)

A local Go 1.23.2 smoke benchmark was run on an Intel Xeon Platinum 8573C with `-benchtime=1x`. One observed run after the hash-preimage allocation reduction:

| Records | Build | Build allocations | Linear scan | Proof generation | Proof verification |
|---:|---:|---:|---:|---:|---:|
| 1k | 0.97 ms | 0.60 MB | 7.9 µs | 5.4 µs | 20.5 µs |
| 10k | 13.8 ms | 5.84 MB | 75.3 µs | 4.0 µs | 8.0 µs |
| 100k | 159.6 ms | 57.95 MB | 745.7 µs | 8.4 µs | 9.5 µs |

These are single-sample smoke measurements, not p50/p95/p99, and must not be compared directly as equivalent query semantics: the scan is unindexed, while the proof-generation/verification numbers reuse an already-built in-memory tree and do not include source completeness, checkpoint authentication, network transfer or proof-wire decoding. The 100k builder remains expensive relative to a single scan. The hash encoding change reduced one observed build from 467 ms / 147 MB to 160 ms / 58 MB and reduced verifier allocations from 167 to 3; repeated controlled measurements are required before attributing a stable improvement.

Next benchmark pass must use at least 30 independent timed samples after warm-up, report p50/p95/p99 and raw samples, pin CPU/runtime/toolchain, run all workload distributions and selectivity classes, and include proof bytes, memory retained per record, incremental update costs and the Merkle B+ / conventional interval-index baselines. The output cardinality must be reported: all-overlap queries are output-bound.

## Qualification boundary

Current status: Go reference prototype with exact endpoint and adversarial unit tests. The canonical root hash profile is documented in `docs/economic/temporal/AUTHENTICATED_INTERVAL_QUERY_V0.1.md`.

Not yet implemented: Rust implementation, independent cross-language conformance, canonical proof wire encoding/decoder, source receipt projection completeness, signed checkpoint binding, range-proof formal model, and measured repeated-run performance distributions.

Do not use this prototype to authorize journal, payment or external mutations.


## Frontier-index forest experiment

`forest.go` composes one authenticated interval root per owner into a canonical manifest bound to:

- verifier identity and policy epoch;
- accepted frontier digest;
- interval-query profile and scope;
- owner reference and stable history identity;
- owner-local sequence cut and separate append-only log tree size;
- owner receipt-log root, interval-index root and projection-audit digest;
- explicit unresolved owner gap ranges.

The caller must supply the exact owner set and forest root obtained from the separately verified frontier/checkpoint layer. Owner identifiers are sorted only for canonical encoding; sequence values from different owners are never compared or treated as a global time order.

The verifier returns only `CompleteForDeclaredIndexForest`: every result relative to the exact declared owner-index roots has been verified. `ProjectionCompletenessNotEvaluated` is always returned because this module does not verify that every accepted owner receipt was projected into the index. Gap metadata is retained in `HasOwnerGaps`; a forest result cannot silently hide it.

This is a composition prototype. It does not authenticate the frontier/checkpoint root, verify log consistency proofs, verify the projection-audit receipt, or grant a `CompleteForAcceptedFrontier` disposition.


## Receipt-to-index projection replay

`projection.go` adds an independent deterministic replay over one supplied owner-local accepted-receipt inventory. Each input receipt carries a contiguous zero-based receipt-log leaf index, a distinct owner-local sequence, a SHA-256 receipt digest and one closed projection class:

- `ReceiptProjectsIntervalAssertion`: supplies the exact assertion reference and half-open validity interval;
- `ReceiptIsNonTemporal`: supplies no interval fields and is explicitly excluded from the temporal index.

The replay checks owner/history/cut/log-root identity, full leaf-position coverage for the declared tree size, increasing owner sequences, no entries inside declared unresolved gaps, canonical record validity and duplicate assertion rejection. It rebuilds the interval index and compares the resulting root; it then computes a deterministic inventory digest and projection-audit digest for checkpoint binding.

The result is deliberately named `ProjectionMatchesSuppliedAcceptedInventory`, and always reports `ReceiptInventoryAuthenticationNotEvaluated`. This is **not** proof that the supplied inventory is the complete accepted log prefix. A separate RFC 9162 / owner-frontier verifier must authenticate the receipt bytes and prove the exact inventory coverage against the owner log root and cut. An incomplete supplied inventory must not be promoted to source completeness merely because its replay is internally consistent.

The replay audit is now a digest that may be committed into the owner-index forest, but the forest query verifier still reports `ProjectionCompletenessNotEvaluated` until an authenticated adapter consumes and verifies these audit results against the accepted frontier.


## Forest-wide projection replay and formal model

`forest_projection.go` verifies one projection replay result for every owner in the exact manifest owner set supplied by a separately verified frontier. It checks the expected forest root and each owner's committed projection-audit digest, then returns `ProjectionMatchesAllSuppliedOwnerInventories`. The result keeps `ReceiptInventoryAuthenticationNotEvaluated`; it does not establish that any inventory is the complete accepted log prefix. It also preserves unresolved gaps.

The owner cut sequence and receipt-log tree size remain separate coordinate systems. The implementation does not compare their numeric magnitudes. Log leaf indices must be a contiguous range from zero to the declared tree size; owner sequences must be strictly increasing and within the independently declared owner cut.

A first bounded TLA+ model is in `formal/TemporalIntervalProof.tla` with `TemporalIntervalProof.cfg`. It models four leaves, exact-start inclusion, exact-end exclusion, an open-ended interval, safe pruning, and complete partitioning of matching versus pruned records. It states invariants for no false pruning, no false result, complete proof partition, and exact result completeness at termination.

**Formal status:** the TLA+ module/config have been authored but TLC has not been run. Do not describe these invariants as model-checked until an exact TLC version, state count and successful run are recorded.
