# Bounded scope/frontier reconciliation observations

**Classification:** local reference-oracle microbenchmark; not registry latency, cryptographic verification, production index performance or SOTA.

- Date: 2026-11-27
- Go: go1.23.2 linux/amd64
- CPU: Intel(R) Xeon(R) Platinum 8573C
- Repetitions: 30 benchmark iterations per sub-benchmark
- Command: `go test -run '^$' -bench BenchmarkScopeFrontierReconciliation -benchmem -benchtime=30x ./...`

```text
BenchmarkScopeFrontierReconciliation/owners=1-3       30      7419 ns/op      4200 B/op      72 allocs/op
BenchmarkScopeFrontierReconciliation/owners=10-3      30     34204 ns/op     17022 B/op     189 allocs/op
BenchmarkScopeFrontierReconciliation/owners=100-3     30    510112 ns/op    119019 B/op    1194 allocs/op
BenchmarkScopeFrontierReconciliation/owners=1000-3    30   1235704 ns/op   1288322 B/op   11095 allocs/op
BenchmarkScopeFrontierReconciliation/owners=10000-3   30  20387497 ns/op  12889265 B/op  110103 allocs/op
```

These are average operation measurements from Go's 30-iteration benchmark harness; they are not 30 independent end-to-end samples and do not provide p50/p95/p99. The measured operation allocates and sorts in-memory inputs. Registry enumeration, source authentication and network costs are excluded by design.
