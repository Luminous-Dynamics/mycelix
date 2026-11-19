# Mycelix Compute Placement Oracle v0.1

**Status:** Exact small-instance reference solver with authored Rust unit tests. Not yet compiled or executed in the authoring environment.

## Scope

This crate enumerates assignments for a finite batch of tasks across a small set of workers. It applies declared regional permissions and aggregate CPU, memory and GPU constraints.

It reports a Pareto frontier over four separate dimensions:

1. estimated compute cost (micro-USD from declared worker rate cards);
2. remote input bytes;
3. number of distinct failure domains used;
4. maximum normalized resource pressure.

No weighted composite score is used. The failure-domain count is only a structural diversity measure; it is not a reliability probability. Cost is an estimate derived from the caller-supplied rate card; it is not an invoice or provider quote.

## Search and correctness boundary

The search is exhaustive only within its explicit node and frontier limits. The report can be:

- `Optimal`: the full finite search was completed and the returned frontier is exact;
- `Infeasible`: the full finite search proves no feasible placement exists;
- `Incomplete`: the search-node or frontier bound was reached. No optimality or infeasibility claim is made.

The deterministic input ordering and tie-breaks make equivalent reordered inputs produce the same report.

## What this does not do

- Discover or authenticate providers or advertised hardware.
- Validate inventory freshness, benchmark trustworthiness or operator identity.
- Issue owner-authorized leases, credentials or capability tokens.
- Dispatch workloads or create external effects.
- Verify outputs or attest that computation actually occurred.
- Model network partitions, job preemption, thermal limits, measured energy or runtime concurrency.

Those are separate owners/gates in Mycelix #6322, #5900, #6189, #6193 and the parent research contract #6331.

## Exact known-answer scenario

The unit-test fixture has two workers and two tasks. The task requiring the eastern region must remain in that region. The other task may be placed locally on the western worker or moved east to use a cheaper rate card.

The expected Pareto frontier deliberately preserves both:
- higher estimated cost, zero remote bytes, two failure domains;
- lower estimated cost, 10 GiB remote movement, one failure domain.

The oracle must not silently pick one by collapsing economics, data locality and failure-domain diversity into a single unexplained weight.
