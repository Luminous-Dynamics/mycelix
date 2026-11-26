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

## Qualification boundary

Current status: Go reference prototype with exact endpoint and adversarial unit tests. The canonical root hash profile is documented in `docs/economic/temporal/AUTHENTICATED_INTERVAL_QUERY_V0.1.md`.

Not yet implemented: Rust implementation, independent cross-language conformance, canonical proof wire encoding/decoder, source receipt projection completeness, signed checkpoint binding, range-proof formal model, and measured repeated-run performance distributions.

Do not use this prototype to authorize journal, payment or external mutations.
