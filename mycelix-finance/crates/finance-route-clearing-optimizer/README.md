# finance-route-clearing-optimizer

Pure Rust reference optimizer for FIN-ECO-005E2. Given externally qualified net positions, one asset, one netting set, participant liquidity limits, and directed settlement routes, it computes a maximum feasible allocation and minimizes linear route cost among maximum-flow allocations.

This is deliberately a bounded optimizer, not a general payment system.

## Model and safety boundary

- One network, one asset, one policy version and one netting set per run.
- Positive net position means the participant owes; negative means the participant is owed.
- Debtor source capacity is min(net amount owed, caller-supplied available liquidity).
- Each enabled directed debtor-to-creditor route has a minor-unit capacity and a non-negative linear cost weight.
- Identical route observations with the same canonical route ID are deduplicated. Changed terms under an existing ID fail closed.
- Missing, disabled, wrong-profile, zero-capacity and role-incompatible routes cannot silently become available.
- Arithmetic uses integer amounts and checked totals. The min-cost maximum-flow solver uses successive shortest augmenting paths with reduced-cost Dijkstra and integer capacities.
- The greedy cheapest-edge-first method is included as a deliberately weaker comparison baseline.
- Residual debtor and creditor amounts are returned explicitly. A partial flow is not called full allocation.

The authority enum is an explicit caller-side gate, not a cryptographic verifier. The evidence reference is an audit link, not proof that the arrangement is legally enforceable. The caller must qualify the netting arrangement and the source records before invoking this crate. The output is not a transfer instruction, a ledger mutation, a guarantee of participant solvency, or a settlement receipt.

The cost value is a dimensionless linear optimization weight per minor unit. It is not a provider's actual fee quote. This first version intentionally does not model fixed-per-leg charges, fee floors/caps, FX spreads, deadlines, multihop routes, failed settlement, participant defaults, or nonlinear costs. A future extension should retain this linear solver as a baseline and add those costs without silently mixing objective units.

## Paradigm shift

Do not optimize a single opaque “savings” score. First maximize the amount that is actually route- and liquidity-feasible; among plans with the same flow, minimize modeled routing cost. The residual is visible, not hand-waved away. That lexicographic objective separates service completion from price optimization and avoids trading a large amount of unmet obligations for a cosmetically cheap plan.

This is exact for the stated linear flow model. It is not an assertion that it outperforms production RTGS liquidity-saving mechanisms (LSMs), nor that every real-world clearing profile can be reduced to a linear flow problem.

## Tests and benchmark harness

The crate includes unit tests for global route-cost allocation versus greedy, liquidity and route-capacity residuals, absent routes, profile mismatch, authority gating, duplicate/conflicting observations, input-order invariance, overflow and comparison with an exhaustive integer oracle on a small graph.

A reproducible, dependency-free synthetic timing harness is available as:

    cargo run --release --manifest-path mycelix-finance/Cargo.toml -p finance-route-clearing-optimizer --example route_clearing_benchmark

It prints CSV for 10, 30, 70, 140 and 700 candidate route arcs, with participant counts, flow/cost output and p50/p95 runtime for the min-cost-flow optimizer and greedy baseline. Run it on a pinned machine and record OS, CPU, Rust version, commit SHA and repetition count alongside results. Synthetic runtime is a microbenchmark, not evidence of real-world settlement savings.

Suggested next benchmark gates:
- The min-cost-flow plan must match the exhaustive oracle's maximum flow and minimum cost on all frozen small fixtures.
- Zero cross-profile allocations, zero allocation above supplied liquidity or route capacity, zero net-position conservation errors.
- Identical output for reordered inputs and stable output for a pinned toolchain.
- Report optimality gap only against an exact oracle on instances where that oracle actually completed.
- Publish p50/p95/p99, memory, maximum graph size and failure counts for frozen synthetic runs; keep route availability, liquidity, and price assumptions alongside each report.
- Compare on common fixtures with gross/FCFS, this crate's greedy baseline, small-instance exhaustive oracle, and later queueing/annealing/auction approaches. Do not compare incomparable objective functions.

## Industry context and research anchors

- The Bank of Canada, Staff Working Paper 2026-32 (September 2026), evaluates liquidity optimization for TARGET2 payment batches of 70 and 140 and compares a constrained quadratic model with simulated annealing. It reports results calibrated to its study sample; those euro amounts are not portable benchmark targets for Mycelix. https://www.bankofcanada.ca/2026/09/staff-working-paper-2026-32/
- BIS Working Paper 1318 (16 December 2025) evaluates an auction-based liquidity-saving mechanism using simulations calibrated to Peru's BCRP-RTGS, South Africa's SAMOS, SADC-RTGS and Switzerland's SIC. It reports 2.6%–12.2% liquidity savings against conventional RTGS in its trials and an average 79% auction success rate, with a trade-off between liquidity efficiency and delay. These are study-specific, not claims about Mycelix. https://www.bis.org/publications/working-paper-1318-auction-based-liquidity-saving-mechanisms
- CPMI-IOSCO Principles for Financial Market Infrastructures emphasize enforceable legal basis, comprehensive risk management, liquidity risk and clearly defined settlement finality. An optimizer cannot substitute for those controls. https://www.bis.org/committees/cpmi/pfmi/overview
- BIS Project Nexus material provides useful context for cross-border governance, risk, resilience, participant and operating-model questions. https://www.bis.org/publications/nexus-enabling-instant-cross-border-payments

## Claim ceiling

This crate demonstrates a reference algorithm and authored tests. Until the exact commit is compiled and those tests run, they are not passing evidence. Even when tests pass, they establish only behavior on the modeled inputs; they do not establish legal set-off, external liquidity availability, production-grade safety, actual settlement finality, all-in fee savings or superiority to TARGET2, SAMOS, Nexus, or any production LSM.
