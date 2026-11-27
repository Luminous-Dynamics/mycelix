# Bounded scope/frontier reconciliation observations

**Classification:** local reference-oracle microbenchmark (final deterministic-order implementation); not registry latency, cryptographic verification, production index performance or SOTA.

- Date: 2026-11-27
- Go: go1.23.2 linux/amd64
- CPU: Intel(R) Xeon(R) Platinum 8573C
- Repetitions: 30 benchmark iterations per sub-benchmark (re-run after deterministic failure-order and canonical permutation controls)
- Command: `go test -run '^$' -bench BenchmarkScopeFrontierReconciliation -benchmem -benchtime=30x ./...`

```text
BenchmarkScopeFrontierReconciliation/owners=1-3       30      8679 ns/op      4232 B/op      74 allocs/op
BenchmarkScopeFrontierReconciliation/owners=10-3      30     60491 ns/op     17303 B/op     191 allocs/op
BenchmarkScopeFrontierReconciliation/owners=100-3     30    189356 ns/op    122837 B/op    1197 allocs/op
BenchmarkScopeFrontierReconciliation/owners=1000-3    30   1509846 ns/op   1321100 B/op   11097 allocs/op
BenchmarkScopeFrontierReconciliation/owners=10000-3   30  23737185 ns/op  13217261 B/op  110105 allocs/op
```

These are average operation measurements from Go's 30-iteration benchmark harness; they are not 30 independent end-to-end samples and do not provide p50/p95/p99. The measured operation allocates and sorts in-memory inputs. Registry enumeration, source authentication and network costs are excluded by design.
