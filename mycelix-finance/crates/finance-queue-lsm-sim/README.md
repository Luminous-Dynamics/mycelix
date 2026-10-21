# finance-queue-lsm-sim

Pure Rust research simulator for queue-order liquidity-saving mechanisms (LSMs). It is intentionally separate from the route-allocation / net-position clearing optimizer because these are different optimization problems.

## Model

- One explicit network and asset profile per run.
- Participants begin with a fixed, externally supplied vector of available liquidity.
- Only accepted payments matching the context are eligible. Proposed, disputed, cancelled, already-settled, unknown-state and cross-profile observations remain excluded.
- Each accepted payment must have a unique ID, positive integer-minor-unit amount, valid payer and payee, and a known opening balance for both participants.
- Identical observations under one payment ID are deduplicated; conflicting economic terms fail closed.
- The simulator repeatedly scans the submitted order. It settles every remaining payment whose payer can cover the amount, transferring liquidity from payer to payee; blocked payments remain queued and can settle in a later pass after incoming liquidity arrives. Simulation stops after a full pass with no executable payment.
- Opening and closing liquidity totals are checked. No payment creates liquidity. Every event records balances before/after and the settlement step.
- Deadline misses and priority-weighted delay are metrics in this simplified simulation. They do not enforce production deadlines or represent wall-clock time.

The model is a reference simulator, not a deployed RTGS, payment processor, settlement executor, netting agreement, or reproduction of any central-bank dataset. A simulated settlement event is not a rail receipt.

## Algorithms

- FIFO baseline, ordered by queue sequence and canonical ID.
- Unlock-potential heuristic, prioritizing payments whose recipients have larger outgoing accepted queues, with priority weight and FIFO tie-breaks.
- Seeded simulated annealing over order swaps. Record the exact seed, iteration budget and config. Its temperature uses floating-point arithmetic for search only; all monetary/ledger-like movement uses checked integer amounts.
- Exhaustive permutation oracle for at most eight eligible payments. It evaluates at most 40,320 orders and fails closed for larger batches.

The comparison objective is explicit and lexicographic: (1) maximize settled value, (2) maximize settled payment count, (3) minimize missed deadlines, (4) minimize priority-weighted settlement index. This is one reference policy, not a universal policy prescription; alternative objectives should be reported separately rather than hidden in one score.

## Run the benchmark

    cargo run --release --manifest-path mycelix-finance/Cargo.toml -p finance-queue-lsm-sim --example queue_lsm_benchmark

The benchmark builds deterministic synthetic batches of 70, 140 and 700 payments and emits CSV for FIFO, unlock-potential and seeded simulated annealing. Annealing budgets are 300, 120 and 30 swaps respectively, with five pinned seeds. Run on a pinned reference machine; record OS, CPU, Rust version and commit SHA with results.

These scenarios are synthetic and only stress the stated simulator. They do not reconstruct SAMOS, SADC-RTGS, TARGET2, PayShap or other production rails.

## Benchmarks to pursue

1. Golden traces: exact opening/closing liquidity, per-event debit/credit conservation, and stable pass semantics.
2. Tiny cases: 100% objective agreement with the exhaustive permutation oracle over the frozen fixtures; publish solution gaps for each heuristic.
3. Reproducibility: same order and report for the same inputs/seed/toolchain; stable exclusion reasons and output order.
4. Stress: compare at 70/140/700 payments on the same frozen corpus, publish p50/p95/p99 runtime, memory, settled value/count, residual queue, deadline misses and seed distributions.
5. Fairness: include starvation and priority-ablation scenarios. A queue policy must not optimize settled value while silently ignoring the declared priority/deadline objective.
6. Safety: zero double settlement, zero negative/overflowed balances, zero unknown-state execution, zero cross-profile execution and zero liquidity creation.

## Industry research anchors

- Bank of Canada Staff Working Paper 2026-32 (September 2026) studies payment-queue reordering for TARGET2 batches of 70 and 140, including comparison between a constrained quadratic solver and simulated annealing, and extends the latter to 700 payments. Its savings figures are calibrated to that study and are not targets Mycelix can claim without a like-for-like corpus and objective. https://www.bankofcanada.ca/2026/09/staff-working-paper-2026-32/
- BIS Working Paper 1318 (16 December 2025) explores an auction-based LSM calibrated to BCRP-RTGS, SAMOS, SADC-RTGS and SIC, and analyzes liquidity savings alongside settlement delays and participant incentives. https://www.bis.org/publications/working-paper-1318-auction-based-liquidity-saving-mechanisms
- SARB's Payments Ecosystem Modernisation programme separately discusses SAMOS/SADC-RTGS modernization and a pre-funded fast-payment model. Those rails have different operational and settlement assumptions and should be represented by distinct profiles. https://www.reservebank.co.za/en/home/what-we-do/payments-and-settlements/pem

## Claim ceiling

Until exact-head CI runs and reports these tests at the exact commit, authored tests are not passing evidence. Passing this simulator's tests would validate only the stated model, not production settlement finality, real liquidity savings, participant solvency, legal set-off, real-world fees or superiority to a production LSM.
