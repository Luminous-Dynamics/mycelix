# Verifier-relative progress antichain (reference V0.1)

This package models the algebra of progress in asynchronous temporal queries where owner logs advance independently.

## Do not conflate the two frontiers

- **Evidence frontier:** an immutable, verifier-selected vector of owner-local sequence cuts used to reconstruct a historical state.
- **Progress antichain:** mutually incomparable lower bounds on the input/proof coordinates that may still arrive.

The first is an accepted query snapshot. The second is a progress declaration about possible future arrivals. A well-formed antichain does not authenticate either the source or the declared owner universe.

## Partial-order semantics

For a fixed owner universe, a cut is a vector of owner-local sequence positions. Define the product partial order:

    a <= b iff a[owner] <= b[owner] for every declared owner.

Two cuts are incomparable when each is ahead on at least one owner coordinate. A valid progress antichain contains no duplicates and no pair of comparable points.

For query cut q, the query remains Provisional whenever some minimal possible future point p satisfies p <= q. Otherwise it is only StableRelativeToDeclaredProgress. The predicate is conservative: if a future timestamp must be above some p and p is not <= q, transitivity rules out a future timestamp <= q.

Progress advancement from old antichain A to new antichain B is monotone only if every b in B is greater than or equal to at least one a in A. An empty next antichain is a valid declared completion; an empty previous antichain cannot be reopened without a new scope/epoch.

## Trust boundary

Every evaluation and advance result sets:
- ProgressAuthentication = NotEvaluated
- SourceUniverseCompleteness = NotEvaluated

The module does not authenticate source progress, prove the progress owner controls the capabilities, verify checkpoint signatures, close the owner universe, or authorize decisions. Stable means only “no predecessor exists in this declared antichain for this query cut.” Production finalization must also require independently qualified source scope/roster closure, authenticated checkpoints, and drained proof dependencies.

## Run

    go test ./...
    go test -race ./...
    go vet ./...
    go test -run '^$' -bench . -benchmem

The tests cover scalar boundaries, incomparable cuts, dominated and duplicate points, owner-universe binding, monotone advancement, epoch/scope substitution, empty-antichain behavior, canonical digests, and deterministic failures.

## Research basis

Timely/Differential Dataflow uses partially ordered timestamps and antichain frontiers to track which timestamps may still arrive. This package adapts the progress algebra but keeps its verifier-relative evidence trust boundary separate:

- https://timelydataflow.github.io/timely-dataflow/chapter_1/chapter_1.html
- https://timelydataflow.github.io/timely-dataflow/chapter_2/chapter_2_4.html
- https://timelydataflow.github.io/timely-dataflow/chapter_5/chapter_5_2.html
- https://timelydataflow.github.io/differential-dataflow/chapter_3/chapter_3_4.html

## Benchmark contract

`progress_benchmark_test.go` covers antichain widths 0/1/2/4/8/16/64/256 and owner dimensions 1/10/100/1,000/10,000 where width does not exceed the declared owner count. It separates validation, query-cut evaluation and monotone advancement, and reports allocations plus explicit owners/op and points/op metrics.

The current benchmark is a local in-memory algebra smoke test. It excludes capabilities, networking, clock/event-time conversion, proof dependencies, source authentication, checkpoint verification and a production dataflow engine. Use repeated warm trials and publish raw samples/p50/p95/p99 before making performance claims.

Malformed inputs are committed into deterministic failure receipts using a separate domain-separated submission digest. This matters because an Indeterminate result for two different malformed antichains or query cuts must not collapse into the same certificate just because they share a failure reason.

## Bounded formal model

`formal/TemporalProgressAntichain.tla` models a two-owner product order with three finite frontiers: an initial incomparable pair, an advanced incomparable pair, and the empty declared-completion frontier. It checks antichain validity, monotone progression, and `StableSound`: a stable result may not coexist with any declared progress point that is less than or equal to the query cut.

The mutant spec uses a deliberately premature finalizer and must violate `StableSound`. This finite model does not prove arbitrary owner dimensions, arbitrary sequence values, the source capability ledger, source authentication, or completeness of the owner universe.

The PR workflow runs both the correct model and the mutant with pinned TLA+ tools, retains raw model logs, and uploads a digest-bound run receipt.

## Input-complexity bounds

The reference validator currently caps UTF-8 identifier fields at 4,096 bytes, the declared owner universe at 100,000 owners, and the progress antichain at 4,096 points. Before allocating point vectors it also caps vector materialization at 5,000,000 owner-coordinate cells, pairwise antichain comparisons at 10,000,000 coordinate checks, and cross-frontier advancement comparisons at the same 10,000,000-check budget. The point cap is important because this straightforward validator checks pairwise incomparability in quadratic time with respect to antichain width. These are explicit V0.1 resource limits, not semantic constants; changing them requires matched worst-case benchmark results and adversarial tests. Oversized submissions fail closed and their raw submitted shape is still bound into the failure receipt.
