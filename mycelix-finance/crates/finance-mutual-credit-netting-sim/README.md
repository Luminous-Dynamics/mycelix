# finance-mutual-credit-netting-sim

Pure Rust reference model for FIN-ECO-005E1: calculate participant net positions and compare a hypothetical direct-settlement path with a hypothetical net-settlement path.

This crate is **not** a Holochain zome, general ledger, clearinghouse, payment executor, liquidity provider, credit authority, or legal conclusion. It creates no value and performs no settlement.

## Model

The caller supplies a versioned profile and canonical obligations. The upstream caller must already have established the actual netting-set agreement/authority and authenticated each obligation's identity, status, parties, unit and scope. This crate checks structural consistency and calculates the result; a caller-supplied status or policy string is not itself proof of authority.

Only `Accepted` obligations inside the exact same:

- network ID;
- asset/unit profile;
- netting-set ID;
- profile policy identity;

participate in the hypothetical netting calculation. Proposed, disputed, cancelled, already-settled and unknown obligations are preserved as excluded records. The report marks that at least one supplied item was excluded; the field `all_supplied_obligations_eligible` does not prove the upstream obligation census is complete.

All authoritative amounts and fee calculations use integer minor units and checked arithmetic. Variable provider/rail cost uses integer basis points with ceiling rounding:

```text
variable_fee_for_leg = min(max(ceil(leg_amount_minor * fee_bps / 10_000), minimum_leg_fee), optional_maximum_leg_fee)
batch_fee = ceil(leg_count / batch_capacity_legs) * batch_fee_per_batch
modeled_cost = leg_count * fixed_fee_per_leg + sum(variable_fee_for_leg) + batch_fee + optional_netting_overhead
modeled_savings = direct_cost_estimate - net_cost_estimate
```

The comparison assumes one direct settlement leg per eligible obligation versus one leg for each deterministic matched net debtor/creditor pair. The cost model represents per-leg fixed and variable fees, minimum and optional maximum variable fees, batch fees and a netting-only overhead. The fee model is only as realistic as its parameters. Real rails may also charge for account maintenance, liquidity, minimum amounts, failed attempts and exceptions; those are not yet modeled and must be added before interpreting a particular route.

## Worked synthetic examples

### Closed cycle

```text
Alice -> Bob   100 units
Bob   -> Carol 100 units
Carol -> Alice 100 units
```

Gross volume is 300 units. Each participant's receivable exactly offsets their payable, so theoretical net settlement volume is zero. Under the test fee model (100 minor units fixed per direct leg and 100 basis points variable, rounded up independently per leg), gross modeled cost is 303 minor units, and net modeled cost is zero when netting overhead is zero. If the netting operation itself incurs an overhead, that cost is added even if there are no final settlement legs.

This is a mathematical netting result only. It does not establish that the obligations are contractually set-off capable, that all obligations were supplied, or that an external settlement event occurred.

### Intermediate receivable

```text
Alice -> Bob   100 units
Bob   -> Carol  60 units
```

Net positions are Alice +100 payable, Bob -40 receivable, and Carol -60 receivable. The deterministic modeled legs are Alice -> Bob 40 and Alice -> Carol 60. Gross volume is 160; net settlement volume is 100. In the same synthetic fee model, with the 1% component rounded up independently for each leg, gross modeled cost is 202 and net modeled cost is also 202 minor units. This parameterization yields no modeled savings despite a 37.5% volume reduction—an important reminder that volume reduction and total fee reduction are different benchmarks.

These values are illustrative test inputs, not observed costs or a claim about any bank/PSP route.

A separate test sets the direct closed-cycle cost to 3 minor units and netting overhead to 10. The net plan then costs 10, producing modeled savings of -7. The simulator therefore does not assume that netting is always cheaper; the strategy must use measured costs and risk.

## Safety properties

- Identical duplicate physical observations under one canonical obligation ID count once.
- Different economic terms under the same canonical ID fail closed.
- Cross-network, cross-asset, cross-netting-set and unknown-member inputs fail closed.
- Non-accepted statuses are never counted in the net positions.
- Member absolute-net-exposure caps are checked.
- Net positions must sum to zero.
- Emitted net settlement legs must reconstruct every participant's exact net position and total net volume.
- Output explicitly sets `settlement_executed = false`.

## Test and benchmark plan

The crate currently includes 17 deterministic unit tests for a closed cycle, offsetting receivable, reciprocal obligations, duplicate observation, conflicting duplicate, disputed obligation, wrong asset, input-order invariance, exposure limit, fee rounding, zero/self obligations, missing scope/member identity, and a 500-obligation order-invariance/conservation stress corpus. Fee-model tests cover minimum-per-leg fees, batch charges, fee caps, invalid fee floors, and netting overhead large enough to make netting more expensive than direct settlement.

The local environment used to prepare this change did not provide `rustc`/Cargo, so these tests have been authored but **not executed here**. Do not interpret their presence as a passing test result. Exact-head CI/local Rust execution is still required.

## Industry and research grounding

- Bank of Canada Staff Working Paper 2026-32 (September 2026) studies liquidity optimization in TARGET2. It reports CQM results on 70/140-payment batches and compares them to simulated annealing; the paper reports comparable savings under the same time constraints at those sizes and extends simulated annealing to 700 payments. This is a useful benchmark target for optimizer design, not a directly comparable Mycelix performance claim: https://www.bankofcanada.ca/2026/09/staff-working-paper-2026-32/
- BIS Working Paper 1318 (16 December 2025) evaluates auction-based liquidity-saving mechanisms on data from Peru's BCRP-RTGS, South Africa's SAMOS, SADC-RTGS and Switzerland's SIC. It reports 2.6%-12.2% liquidity savings and an average 79% auction success rate in its simulations, while identifying a trade-off between liquidity efficiency and settlement delay: https://www.bis.org/publications/working-paper-1318-auction-based-liquidity-saving-mechanisms
- BIS Project Nexus describes a route to lower cross-border friction by connecting domestic instant-payment systems, while emphasizing participant governance, operating models and incentives: https://www.bis.org/publications/nexus-enabling-instant-cross-border-payments
- The CPMI/BIS Principles for Financial Market Infrastructures require a clear legal basis, defined settlement finality and appropriate risk controls. Exact net positions alone do not establish legal discharge or final settlement: https://www.bis.org/committees/cpmi/pfmi/overview
- CPMI analysis of multilateral platforms cautions that governance, costs, jurisdictional differences and risk require a proper cost-benefit analysis; a new platform is not automatically more efficient: https://www.bis.org/publications/can-multilateral-platforms-improve-cross-border-payments
- Firm-level research on mutual-credit systems found nuanced results: benefits to small firms became more positive during economic turbulence, while membership could impose costs during stable periods. That supports measuring actual workloads and conditions rather than claiming universal superiority: https://doi.org/10.1016/j.jebo.2024.106861

Follow-on benchmark work should freeze the input corpus, Rust version, hardware, compiler profile and cost model. Exercise 10, 100, 1,000, 10,000 and 100,000 obligation profiles within configured bounds, plus input-order permutations and adversarial duplicates/conflicts. Report throughput and memory as measurements, plus gross/net amount, count of legs, fixed/variable/total modeled cost, unresolved-obligation coverage and all failures. Compare against direct incumbent settlement with matched parameter assumptions. Do not claim a production cost reduction without matched real quote and settlement data.

## Related work

- FIN-ECO-005E: https://github.com/Luminous-Dynamics/mycelix/issues/1143
- FIN-ZERO-COST-001: https://github.com/Luminous-Dynamics/mycelix/issues/5322
- HOLO-MONEY-001: https://github.com/Luminous-Dynamics/mycelix/issues/5326

