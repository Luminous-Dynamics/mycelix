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
