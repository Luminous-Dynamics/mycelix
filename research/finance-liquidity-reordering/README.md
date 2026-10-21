# Finance liquidity reordering research kernel

This is a **standalone research crate**, not a payment executor or production clearing rail. It works over a fixed list of gross payment instructions and attempts to reorder them so fewer units of participant liquidity must be available at the start of the sequence.

## Paradigm shift

The first question isn't “how many transfers can we settle atomically?” It is “what is the minimum initial liquidity required to execute the exact same gross obligations, in a given order?” For participant p at prefix t, the required initial liquidity is:

    L_p = max_t (0, cumulative_outflow_p(t) - cumulative_inflow_p(t))

The implementation reports the aggregate sum over participants and the largest individual requirement separately. Its declared tie-break is minimum aggregate initial liquidity, then minimum maximum-participant requirement, then lexicographically smallest sequence of payment IDs. These metrics are modeled gross-settlement prefunding requirements—not credit exposure, collateral, total-system liquidity usage, finality evidence, or a legal-netting result.

## Algorithms included

- input_order: unchanged input order / FIFO baseline.
- receiver_ready_greedy: greedy next-payment selection by incremental prefunding need, then value that can unlock the receiver's pending outgoing transfers.
- bounded_insertion_search: deterministic insertion local search with a caller-supplied candidate budget.
- simulated_annealing: dependency-free SplitMix64-seeded swap search; exact seed and iteration count reproduce the same output.
- exact_permutation_oracle: exhaustive search limited to at most eight payments. Larger input fails closed; no heuristic is presented as optimal.

Every candidate order preserves each payment exactly once. Input amount, IDs, and participant indices are validated; all calculations use integer minor units and checked arithmetic where accumulation may overflow. No algorithm mutates payment terms.

## Running

From repository root, run:

    cargo fmt --manifest-path research/finance-liquidity-reordering/Cargo.toml -- --check
    cargo test --manifest-path research/finance-liquidity-reordering/Cargo.toml --offline --locked
    cargo run --manifest-path research/finance-liquidity-reordering/Cargo.toml --offline --locked --release --example benchmark

The generated CSV compares input order, greedy, bounded insertion search, and three seeded annealing runs over reversed equal-amount chains of 4, 6, 8, 10, 30, 70, 140, and 700 transfers. Exact-oracle gaps are reported only where the full permutation oracle is feasible. This is a correctness and instrumented baseline corpus, not a representative RTGS dataset.

## Limitations / next stage

This first corpus is intentionally adversarial and simple: a reversed payment chain. It demonstrates that order can be a control variable; it does not establish production benefit or state-of-the-art performance. The next corpus should mix closed cycles, multiple disconnected and interleaved chains, heterogeneous amounts, sparse/dense participant graphs, liquidity concentration, deadlines, route capacity windows and outages. Compare on frozen common inputs; report quality and latency separately; use matched wall-clock budgets and multiple seeds; retain a Pareto frontier over aggregate prefunding, maximum participant requirement and delay. Real historical payment records must not be used without documented data rights and reproducible preprocessing.

Research tracking: issue #5385, FIN-ECO-005E3.
