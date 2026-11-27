# Local oracle benchmark observations

**Classification:** bounded reference-oracle smoke measurements; not production-index measurements, not SOTA results, and not end-to-end temporal-query results.

- Date: 2026-11-27
- Go: `go1.23.2 linux/amd64`
- CPU reported by Go: `Intel(R) Xeon(R) Platinum 8573C`
- Repetitions: 30 iterations per sub-benchmark (`-benchtime=30x`)
- Command: `go test -run '^$' -bench 'BenchmarkReplay' -benchmem -benchtime=30x ./...`

## Raw output

```text
BenchmarkReplayByOwnerCount/owners=1-3           30    31272 ns/op      29730 B/op       100 allocs/op
BenchmarkReplayByOwnerCount/owners=10-3          30   301174 ns/op     287908 B/op       803 allocs/op
BenchmarkReplayByOwnerCount/owners=100-3         30  2239348 ns/op    2869446 B/op      7814 allocs/op
BenchmarkReplayByOwnerCount/owners=1000-3        30 27620980 ns/op   28706468 B/op     77912 allocs/op
BenchmarkReplayByIntervalCount/intervals=100-3   30   334638 ns/op     286216 B/op       670 allocs/op
BenchmarkReplayByIntervalCount/intervals=1000-3  30  3998956 ns/op    3772733 B/op      6211 allocs/op
BenchmarkReplayByIntervalCount/intervals=10000-3 30 43681647 ns/op   35709283 B/op     61419 allocs/op
```

## Interpretation limits

These are Go benchmark averages over 30 loop iterations, not 30 independent machine-level samples suitable for p50/p95/p99. The oracle is intentionally simple and scan-based. Owner and interval counts are varied separately; the measurements estimate oracle cost and allocation growth only. They must not be compared directly with VeriTS, VTRQ, XTDB, SQL Server, or the production authenticated index.

A fair production comparison still needs matched corpora and separate measures for index build/update, scan computation, authenticated proof construction, proof bytes, independent verification, projection replay, correction-impact discovery, memory and end-to-end latency. That campaign should publish raw per-run samples and p50/p95/p99 after warm-up.
