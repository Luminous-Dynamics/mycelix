# Temporal historical-replay oracle (Go reference)

This is a deliberately small, independent semantic oracle for the historical-frontier noninterference work in Mycelix issue [#6788](https://github.com/Luminous-Dynamics/mycelix/issues/6788).

It does not import or call the candidate temporal interval-index implementation. Its purpose is to provide a simple executable expected-result model for adversarial tests and future Rust/Go conformance vectors.

## Contract

- A frontier is a vector of `(stable owner-history identity, owner-local sequence cut)` entries. It is never flattened into a global timestamp or sequence.
- Only events in the exact accepted owner prefix are read. Suffix events are not parsed or used by the historical query.
- `VALID_FROM` and `VALID_TO` endpoints pair only under the exact `(namespace, subject, claim, validity_interval_ref)` key.
- A finite end is exclusive. An explicit open-end disposition is required; a missing end is `Indeterminate`.
- Supersession must resolve inside the accepted prefix and stay within the same owner history, namespace, subject, claim, interval identity and endpoint dimension.
- Missing owners, prefix gaps, duplicate positions, conflicting heads, malformed identity, and invalid interval relationships fail closed.
- The result digest binds verifier, policy, namespace, subject, claim, business-time query instant, each stable owner-history identity and owner-local cut, disposition, reason and result provenance.

The positive result is named `CompleteForSuppliedOwnerPrefixes`. It does **not** prove that the required owner set was discovered correctly, that the supplied frontier/checkpoints are cryptographically authentic, or that source records are true. It is not `CompleteForAcceptedFrontier` and is not authorization to mutate financial or operational state.

## Tests

```sh
go test ./...
go test -race ./...
go vet ./...
go test -run 'TestSuffixNoninterference|TestBackdatedSuffix|TestLatestStateMutant' -count=30 ./...
```

The tests include:
- inclusive start / exclusive finite end;
- explicit OPEN vs missing VALID_TO;
- two validity episodes with a gap (rejects cross-episode endpoint pairing);
- backdated corrections accepted only beyond F0;
- two replicas with different unobserved suffixes but identical F0 replay and certificate;
- wrong owner-history identity;
- malformed post-frontier suffix isolation;
- missing owners, prefix gaps, cross-episode supersession and conflicting endpoint heads;
- 100 valid per-owner delivery-order permutations;
- 10,000 deterministic suffix-only mutations;
- a latest-state mutant that must produce a hindsight counterexample;
- a fixed SHA-256 certificate known-answer vector;
- out-of-scope interval assertions isolated from the exact query scope.

## Benchmarks

```sh
go test -run '^$' -bench 'BenchmarkReplay' -benchmem -benchtime=30x ./...
```

The benchmark varies owner count and interval count separately. It measures the independent scan-based oracle, **not** the production authenticated index, and must not be used as an SOTA performance claim. Its role is to estimate independent-oracle cost and retain a matched semantic baseline. Production index build/update, proof generation, proof bytes, independent proof verification, projection replay, and end-to-end query latency must be reported separately.

## Qualification

The Go reference was exercised with Go 1.23.2. The exact test commands, bounded benchmark observations and caveats are recorded in `BENCHMARK-OBSERVATIONS.md`. TLA+ TLC execution, Rust parity, checkpoint authenticity, owner-scope closure and semantic mapper integration remain separate qualifications.
