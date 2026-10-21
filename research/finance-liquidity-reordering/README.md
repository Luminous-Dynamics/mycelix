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
- heuristic_portfolio: compare input order and receiver-ready greedy, then spend separate insertion-search and seeded annealing budgets starting from the best plan so far. Its aggregate evaluation count includes component effort; compare its additional runtime against quality gains rather than presenting it as a free improvement.
- exact_permutation_oracle: lexicographic exhaustive search limited to at most eight payments. Larger input fails closed; no heuristic is presented as optimal.
- exact_pareto_frontier: enumerates the same small feasible ordering space and preserves all nondominated trade-offs between aggregate initial liquidity and the largest participant's requirement. This avoids making the fairness-vs-efficiency decision by an arbitrary scalar weight. Larger input fails closed.

Every candidate order preserves each payment exactly once. Optional hard precedence edges (before-payment index, after-payment index) are validated for range, duplicate edges and cycles; FIFO uses a stable topological ordering, greedy only selects ready payments, local search and annealing reject order-breaking moves, and the exact oracle enumerates only valid linear extensions. Input amount, IDs, and participant indices are validated; all calculations use integer minor units and checked arithmetic where accumulation may overflow. No algorithm mutates payment terms.

## Running

From repository root, run:

    cargo fmt --manifest-path research/finance-liquidity-reordering/Cargo.toml -- --check
    cargo test --manifest-path research/finance-liquidity-reordering/Cargo.toml --offline --locked
    cargo run --manifest-path research/finance-liquidity-reordering/Cargo.toml --offline --locked --release --example benchmark

The generated CSV compares input order, receiver-ready greedy, bounded insertion search, three seeded annealing runs, and the combined heuristic portfolio over five deterministic synthetic topologies (reversed equal-amount chain, reversed heterogeneous chain, reversed cycle, hub round trips, and a precedence-constrained reversed chain) at 4, 6, 8, 10, 30, 70, 140, and 700 gross payments. The constrained scenario requires the order to respect a declared precedence DAG; invalid candidate orders are excluded rather than traded away for liquidity savings. It also reports three graph features—participants who both send and receive, unique receivers, and unique directed edges—to enable later study of which topology features predict liquidity-saving opportunity. The 19-column CSV reports graph features, exact Pareto-frontier size for batches up to eight payments, signed aggregate-liquidity savings and peak-participant change versus the same constraint-safe input-order baseline, plus aggregate-liquidity and maximum-participant gaps against the exact oracle. The workflow checks that all small cases have a nonempty frontier and that both exact gaps are zero for oracle rows. Exact tests also verify the Pareto frontier contains no dominated points. A 296-line output gate requires all 295 algorithm/scenario/size rows and zero aggregate and maximum-participant gaps on exact rows. These fixtures are deliberately adversarial, not representative RTGS data.

## Limitations / next stage

The current corpus is intentionally synthetic and adversarial. It demonstrates that order and graph topology can be control variables; it does not establish production benefit or state-of-the-art performance. The next corpus should add multiple disconnected and interleaved chains, sparse/dense participant graphs, realistic liquidity concentration, deadline/maximum-wait policy constraints, route capacity windows and outages, and permitted frozen real or calibrated payment traces. Precedence is only one class of hard policy constraint; it does not yet model release times, dynamic deadlines, route outages or rail-specific execution windows. Compare on frozen common inputs; report quality and latency separately; use matched wall-clock budgets and multiple seeds; retain a Pareto frontier over aggregate prefunding, maximum participant requirement and delay. Real historical payment records must not be used without documented data rights and reproducible preprocessing.

Research tracking: issue #5385, FIN-ECO-005E3. The workload-feature fields are motivated by the Bank of Canada’s 2026 TARGET2 paper, which reports that optimizability varies with sender/receiver overlap and unique receiver count. Mycelix has not trained or validated a predictor from these features yet: https://www.bankofcanada.ca/2026/09/staff-working-paper-2026-32/
