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


## Full owner-log receipt-to-index projection replay

`projection.go` verifies the entire owner-log leaf sequence, not merely the verifier-receipt subset. This distinction is required by #6727's owner log: ordinary owner-history events may occupy log positions, while canonical `MYCELIX-KNOWLEDGE-RECEIPT-V1` entries replace particular positions when verifier-acceptance receipts are present.

The input inventory contains every log leaf in order:
- zero-based `LeafIndex`;
- owner-local `OwnerSequence`;
- exact `EntryBytes`.

The verifier recomputes RFC 9162 SHA-256 leaf/node hashing over the complete ordered leaf sequence and requires the resulting root to match the owner binding. It validates the sequence-to-leaf mapping after explicit gaps. Each canonical knowledge-receipt leaf must have exactly one typed temporal or non-temporal projection. Non-receipt owner leaves remain in the Merkle root and are counted separately; they are not silently dropped from the source history.

The positive result is `ProjectionMatchesSuppliedAcceptedInventory`, with `ReceiptInventoryAuthentication=LogRootRecomputedButTrustExternal`. This means the bytes supplied by the caller reconstruct the claimed owner-log root. It does **not** authenticate the signed checkpoint/tree head or prove that the expected root was accepted by the verifier. That requires the upstream frontier/checkpoint verification path.

The mapper from canonical assertion digest/payload into the temporal `valid_from`, `valid_to` and interval/non-temporal classification is also a separate trust boundary. This implementation reports `ProjectionMappingAuthentication=NotEvaluated`; the interval assertion must not be promoted to source-complete solely because the log root was recomputed.

## Forest-wide projection replay and formal model

`forest_projection.go` verifies one projection replay result for every owner in the exact manifest owner set supplied by a separately verified frontier. It checks the expected forest root and each owner's committed projection-audit digest, then returns `ProjectionMatchesAllSuppliedOwnerInventories`. The result keeps `ReceiptInventoryAuthenticationNotEvaluated`; it does not establish that any inventory is the complete accepted log prefix. It also preserves unresolved gaps.

The owner cut sequence and receipt-log tree size remain distinct coordinates, but the V1 owner-log contract requires one log leaf for every non-gap sequence: \`TreeSize + total_gap_cardinality = OwnerSequenceCut\`. Sequence-to-leaf mapping subtracts gaps that precede the owner sequence. The quantities are not interchangeable, and their cardinality relation must be checked rather than assuming \`TreeSize == OwnerSequenceCut\`.

A first bounded TLA+ model is in `formal/TemporalIntervalProof.tla` with `TemporalIntervalProof.cfg`. It models four leaves, exact-start inclusion, exact-end exclusion, an open-ended interval, safe pruning, and complete partitioning of matching versus pruned records. It states invariants for no false pruning, no false result, complete proof partition, and exact result completeness at termination.

**Formal status:** the TLA+ module/config have been authored but TLC has not been run. Do not describe these invariants as model-checked until an exact TLC version, state count and successful run are recorded.


## Canonical proof wire format

`wire.go` provides `MarshalIntervalPointProofV1` and `UnmarshalIntervalPointProofV1`. The wire frame begins with `MYCITPQ1`, then exact scope strings, the seconds/nanoseconds query instant, result count and a typed proof tree. Pruned nodes carry the authenticated count/summary/hash; branch nodes carry only their two ordered children; leaf nodes carry the canonical interval record. The decoder reconstructs leaf hashes/summaries and validates the proof tree before returning it.

Bounds: 64 MiB encoded proof, **1,000,000 records**, 2,000,001 proof nodes, recursion depth 64, and 1,024-byte UTF-8 strings. Nanoseconds must be below 1,000,000,000; end tags are exactly Finite or Open; trailing bytes and unknown node tags fail closed. The expected index root is intentionally **not embedded**: the caller supplies it from the independently authenticated checkpoint/frontier.

The benchmark harness now includes canonical proof wire encoding and decode-plus-verification sub-benchmarks and reports encoded proof bytes. They remain one-iteration CI smoke measurements unless explicitly run with repeated samples and a pinned environment.


## Adversarial interval workload matrix

`BenchmarkTemporalQueryWorkloadsV1` separates seven deterministic workloads at 1k / 10k / 100k intervals:

- disjoint sparse (one result);
- clustered intervals (local result groups);
- nested intervals (all overlap at the query point);
- all-overlap;
- future-only (complete empty result via future pruning);
- expired-only (complete empty result via end-boundary pruning);
- open-ended (all matching).

For each it reports build, linear-scan baseline, proof generation, proof node/pruned/expanded/returned-leaf counts, canonical proof bytes, and wire decode-plus-verification. The CI invocation uses one iteration per case only to catch harness/code regressions. It is not a stable performance measurement. Repeated pinned runs are still required for p50/p95/p99, confidence intervals and raw sample publication.
