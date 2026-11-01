# GCPN Lifecycle Cost and Accessible-Reach Benchmark v0.1

**Status:** deterministic synthetic lifecycle-cost benchmark; non-operational.  
**Parent:** issue #5625.  
**Package:** `benchmarks/gcpn-lifecycle-cost`.  
**Architectures:** centralized hub-and-spoke, regional federation, local-first federation.  
**Cost units:** invented synthetic units; not a currency, market price, procurement estimate or national budget.

## 1. Why cost needs its own benchmark

Resilience and accessible reach are necessary but not sufficient for an implementable early-warning system. A regional or national roadmap must also identify capital, connectivity, staffing, training, maintenance, replacement and recovery costs over a defined operating horizon.

WMO's October 2026 review of 35 national EW4All roadmaps emphasizes costed and time-bound implementation. SADC's current MHEWS capacity/interoperability procurement likewise concerns sustainable institutional and technical capability, not only a software product.

This benchmark introduces comparable lifecycle accounting for the three abstract architectures already used by the topology benchmark. Cost, staffing, reach and accessibility stay separate. The tool does not decide which architecture should receive public funds.

References:
- WMO, 35 EW4All national roadmaps (29 Oct 2026): https://wmo.int/media/update/from-global-ambition-national-action-take-aways-from-35-ew4all-roadmaps
- SADC MHEWS capacity/interoperability consultancy: https://sadc.int/procurement-opportunities/consultancy-undertake-capacity-building-arising-identified-mhews-gaps-and
- UNDRR Common Alerting Protocol: https://www.undrr.org/early-warnings-for-all/common-alerting-protocol

## 2. Frozen synthetic assumptions

The cost fixture contains 24 rows across three architecture profiles. Each has:
- initial capital and integration;
- channel setup;
- annual operations;
- annual staffing and staff hours;
- annual maintenance;
- a scheduled replacement in year three;
- scenario-specific restoration/contingency rows for selected failure scenarios.

The common planning assumptions are:
- service horizon: 5 years;
- real discount rate: 5%;
- yearly recurring costs occur at the end of each year;
- one-time capital/setup costs occur at year zero;
- scheduled replacement and scenario restoration occur in their stated year;
- low/base/high input ranges are explicit and must be monotonic.

The outcome fixture contains 21 synthetic records: three architectures across seven service conditions. Every architecture sees the same 2,000 intended-receiver denominator for each scenario. Outcomes separately record unique receivers reached, accessible-format receivers reached and duplicate channel receipts.

These inputs are invented solely to validate arithmetic and trade-offs. They are not estimated from any actual operator, government, national budget, vendor quote, community survey or deployed early-warning system.

## 3. Cost calculation

For a discount rate (r), cost (C_t) in year (t), and service horizon (T), lifecycle present value is:

[
PV = sum_{t=0}^{T} rac{C_t}{(1+r)^t}
]

The evaluator applies each fixture component according to its cost pattern:
- one-time: discounted once at the declared year;
- annual: repeated once per service year;
- scheduled: included once in its scheduled year if inside the horizon;
- scenario restoration: included only for the scenario to which it is bound.

Equivalent annualized cost is calculated with a capital-recovery factor:

[
EAC = PV cdot rac{r}{1-(1+r)^{-T}}
]

For (r=0), EAC is (PV/T).

The model reports low/base/high present value, annualized base cost, component breakdown and annual staff hours. It does not infer hidden costs or automatically add a replacement reserve.

### Staffing units

The evaluator now reports both:
- annual staff hours: the recurring staffing assumption for one representative year;
- staff hours over horizon: annual hours multiplied by the service horizon.

The baseline profiles therefore imply 1,600 / 1,950 / 2,400 staff hours per year for centralized / regional / local-first architectures, respectively, and 8,000 / 9,750 / 12,000 staff hours over the five-year horizon. No salary or wage rate is inferred from these hours.

## 4. Service and unit-cost denominators

Report separately:
- intended receivers;
- unique receivers reached;
- unique receivers receiving an accessible format;
- duplicate channel receipts;
- cost per intended receiver;
- cost per unique receiver reached;
- cost per unique accessible receiver reached.

Unit-cost denominators use unique people, not message sends or the sum of channel receipts. An outcome with zero unique or zero accessible recipients returns an undefined unit cost. It must not return zero, infinity or an artificially cheap result.

## 5. Baseline synthetic results

For the 5-year, 5% base-cost profile in the all-available scenario:

| Architecture | Lifecycle present value | Annualized cost | Accessible receivers | Cost per accessible receiver |
|---|---:|---:|---:|---:|
| Centralized | 28,026.38 | 6,473.39 | 1,800 | 15.57 |
| Regional federation | 31,443.19 | 7,262.58 | 1,850 | 17.00 |
| Local-first federation | 38,847.47 | 8,972.79 | 1,900 | 20.45 |

All values are synthetic cost units. The fixture intentionally does not identify a winner:
- centralized has the lowest baseline lifecycle cost in this constructed profile;
- local-first reaches more people with an accessible format in the same synthetic baseline;
- failure scenarios change service reach and scenario-specific restoration costs;
- no cost per accessible receiver is defined when accessible reach is zero.

The baseline comparison alone is insufficient for any investment choice. Resilience performance, legal mandate, geography, channel availability, community preference, implementation capability and local cost data would all be required.

## 6. Fault scenarios

The service fixture includes:
- all services available;
- central hub outage;
- interregional partition;
- origin regional service outage;
- local-first path outage;
- competent authority unavailable;
- remote last-mile outage.

Scenario-specific restoration costs are applied only to their named failure scenarios. They are not added to all scenarios or to architectures without the specified restoration row.

The authority-unavailable case intentionally has zero reach for all architectures. Lifecycle cost remains positive, while cost per accessible receiver is undefined. This prevents the benchmark from making an unavailable service look economically efficient because its denominator is zero.

## 7. Sensitivity analysis

The public API supports re-evaluation of the same fixture at a different horizon and discount rate. The test suite compares:
- five-year / 5% reference;
- ten-year / 5%;
- five-year / 0%.

This is deterministic sensitivity arithmetic, not a probabilistic cost forecast. Real analysis should add inflation, tax, procurement risk, foreign exchange, equipment lead times, power reliability, grant/concessional finance, replacement uncertainty and country-specific staffing cost data with evidence and ranges.

## 8. Industry comparison protocol

A fair lifecycle comparison must use the same service scope and denominator. At minimum, match:
- hazard and geographic coverage;
- intended and unique accessible receiver definitions;
- channel and regional interoperability requirements;
- operating horizon and discount-rate convention;
- staffing, training, maintenance and replacement obligations;
- security, privacy, governance and independent audit;
- restoration expectations and service levels.

Compare costs per intended, unique and accessible recipient separately, and publish the full component breakdown. Also report reach, accessibility, latency and resilience. Do not reduce these to a weighted architecture score.

This v0.1 synthetic fixture is not a comparison against a named national service, telecom operator, vendor or public procurement. Named-system comparisons require public or permissioned cost data, equivalent service definitions and independent review.

## 9. Reproduction

From repository root:

```bash
cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-lifecycle-cost/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-lifecycle-cost/Cargo.toml
```

Fixture FNV-1a fingerprints:
- lifecycle cost profiles: `0x52072da4d6fb1ba6`;
- service outcomes: `0x376c727b26d887aa`.

These hashes are regression identifiers, not cryptographic authenticity proofs.

## 10. Non-claims and governance

This benchmark does not estimate real system costs, create a procurement recommendation, allocate funding or determine national priorities. All prices are synthetic units; all receiver counts are fictional. Real budgeting needs local ownership, open assumptions, affected-community participation, procurement transparency, sensitivity analysis and independently reviewed evidence.
