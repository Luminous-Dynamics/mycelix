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
variable_fee = ceil(volume_minor * fee_bps / 10_000)
modeled_cost = leg_count * fixed_fee_per_leg + variable_fee
modeled_savings = direct_cost_estimate - net_cost_estimate
```

The comparison assumes one direct settlement leg per eligible obligation versus one leg for each deterministic matched net debtor/creditor pair. The fee model is only as realistic as its parameters. Real rails may charge for batches, account maintenance, liquidity, minimum amounts, failed attempts and exceptions; add those terms before interpreting a particular route.

## Worked synthetic examples

### Closed cycle

```text
Alice -> Bob   100 units
Bob   -> Carol 100 units
Carol -> Alice 100 units
```

Gross volume is 300 units. Each participant's receivable exactly offsets their payable, so theoretical net settlement volume is zero. Under the test fee model (100 minor units fixed per direct leg and 100 bps variable), gross modeled cost is 303 minor units, and net modeled cost is zero.

This is a mathematical netting result only. It does not establish that the obligations are contractually set-off capable, that all obligations were supplied, or that an external settlement event occurred.

### Intermediate receivable

```text
Alice -> Bob   100 units
Bob   -> Carol  60 units
```

Net positions are Alice +100 payable, Bob -40 receivable, and Carol -60 receivable. The deterministic modeled legs are Alice -> Bob 40 and Alice -> Carol 60. Gross volume is 160; net settlement volume is 100. In the same synthetic fee model, gross modeled cost is 202 and net modeled cost is 201 minor units. This particular parameterization yields only one minor unit of modeled savings despite a 37.5% volume reduction—an important reminder that volume reduction and total fee reduction are different benchmarks.

These values are illustrative test inputs, not observed costs or a claim about any bank/PSP route.

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

The crate currently includes 13 deterministic unit tests for a closed cycle, offsetting receivable, reciprocal obligations, duplicate observation, conflicting duplicate, disputed obligation, wrong asset, input-order invariance, exposure limit, fee rounding, zero/self obligations, missing scope/member identity, and a 500-obligation order-invariance/conservation stress corpus.

The local environment used to prepare this change did not provide `rustc`/Cargo, so these tests have been authored but **not executed here**. Do not interpret their presence as a passing test result. Exact-head CI/local Rust execution is still required.

Follow-on benchmark work should freeze the input corpus, Rust version, hardware, compiler profile and cost model. Exercise 10, 100, 1,000, 10,000 and 100,000 obligation profiles within configured bounds, plus input-order permutations and adversarial duplicates/conflicts. Report throughput and memory as measurements, plus gross/net amount, count of legs, fixed/variable/total modeled cost, unresolved-obligation coverage and all failures. Compare against direct incumbent settlement with matched parameter assumptions. Do not claim a production cost reduction without matched real quote and settlement data.

## Related work

- FIN-ECO-005E: https://github.com/Luminous-Dynamics/mycelix/issues/1143
- FIN-ZERO-COST-001: https://github.com/Luminous-Dynamics/mycelix/issues/5322
- HOLO-MONEY-001: https://github.com/Luminous-Dynamics/mycelix/issues/5326

