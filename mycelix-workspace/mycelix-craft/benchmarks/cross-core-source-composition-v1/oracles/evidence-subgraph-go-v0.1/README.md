# Craft evidence-subgraph retrieval oracle V0.1

Independent, test-only Go scoring oracle for retrieved evidence subgraphs. It scores the support graph used to justify a decision, not only the final answer.

## Metrics

- Node precision/recall over exact node IDs and types.
- Edge precision/recall over exact relation-claim identities and endpoints.
- Source attribution precision/recall over exact source claim IDs and 32-byte digests.
- Temporal validity against a valid-time instant, source stream accepted-offset frontier and exact query scope.
- Required path consistency across the gold supporting chain.
- False-support count for stale, beyond-frontier, wrong-scope, wrong-endpoint or wrong-source facts.
- Claim-ceiling promotions and false completeness.

Every metric publishes its raw numerator and denominator. A zero denominator is undefined (JSON value null), not silently scored as zero or one.

## Boundaries

The oracle treats query frontier and source-universe status as declared inputs. It does not authenticate them. It verifies only that a candidate claim is consistent with the named frontier and exact source references supplied to the benchmark. It does not verify signatures, legal authority, water-quality truth, statistical independence or physical outcome.

The fixture suite has positive and typed-negative controls. This is a scoring oracle, not a production search engine, process miner or policy evaluator.

## Run

```sh
gofmt -w oracle/*.go
go test ./...
go test -race ./...
go vet ./...
go test -cover ./...
```

All known-answer data are synthetic and specification-level. No field effectiveness or SOTA performance result is claimed.
