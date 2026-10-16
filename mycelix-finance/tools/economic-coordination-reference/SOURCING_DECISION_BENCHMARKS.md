# Capability-first sourcing: adversarial benchmark contract

**Status:** proposed benchmark and acceptance contract. The implementation's local source tests are authored, not yet observed passing on a hosted runner. No live procurement or measured latency is claimed.  
**Scope:** the standalone Rust sourcing reference module in Economic Coordination Reference.  
**Authority ceiling:** evidence-gated candidate comparison only. It is not a web crawler, product-equivalence oracle, supplier-authentication system, purchase-order authority, or payment executor.

## Paradigm shift

Traditional shopping-style procurement starts from a vendor SKU and optimizes a price list. Capability-first procurement starts from an operational need and asks which independently qualified offers can satisfy a versioned capability profile under delivery, budget, trust, and risk constraints.

That requires two different steps:

1. **Feasibility:** enforce hard constraints before optimization. An unverified, stale, incompatible, unavailable, over-budget, or late offer must not win because a weighted score makes its price attractive.
2. **Trade-offs:** compare only feasible offers and return the Pareto frontier over landed cost, arrival time, and a risk value bound to the same versioned risk model. Do not hide value choices inside one universal scalar score. If the frontier contains multiple offers, policy or a human chooses among them explicitly.

A capability profile digest is an equality boundary, not semantic proof. Alternative products may enter the same comparison only through a separately reviewed, deterministic and versioned equivalence mapping; an LLM or scraper cannot silently declare equivalence.

## Current evaluator contract

The evaluator:
- uses an explicit evaluation timestamp to make replay deterministic;
- requires a non-empty source locator, source-payload digest reference and extraction-profile identity; these are preserved in the report but not authenticated or recomputed by this module;
- requires exact capability ID/profile digest, canonical unit, currency, and risk-model digest matches;
- requires quote and supplier-credential evidence to be marked verified by an upstream process;
- rejects future/stale/expired quotes, expired credentials, insufficient quantity, late delivery, malformed risk values, and integer-overflowed or over-budget totals;
- computes landed cost using checked integer arithmetic in minor currency units;
- sorts its report by offer ID so input ordering cannot decide an outcome;
- distinguishes Pareto-frontier offers, dominated offers, and rejected offers; it never emits a purchase decision.

The current evidence-status fields are inputs to this reference model. The model does not authenticate their origin or prove that supplier stock physically exists.

## Hard safety gates

Every gate is a zero-tolerance pass/fail condition; failures must not be averaged against quality or speed.

| Gate | Campaign | Proposed target |
|---|---|---:|
| Hard-constraint precedence | Adversarial offer corpus with mandatory-profile, credential, quote, inventory, currency, unit, risk-model, deadline and budget violations | 100% of violating offers excluded |
| Cost arithmetic | Boundary values, maximum integers, fees, taxes, and quantity multiplication | Zero overflow, wrapping, or understated totals |
| Currency/unit/profile isolation | Mixed and deliberately mislabeled offers | Zero cross-currency, cross-unit or unapproved-profile comparisons |
| Evidence freshness | Quotes at, inside and beyond the age boundary; future and expired records | Zero stale/expired offers accepted |
| Source attribution | Accepted and rejected offers with absent source locator, payload-digest reference or extractor identity | Zero offers missing required source metadata may become eligible; output retains supplied references |
| Pareto correctness | Hand-reviewed fixtures plus a separately implemented oracle | Exact frontier agreement |
| Abstention | Empty catalogues, only-ineligible offers, conflicting IDs, unresolved evidence | Zero forced winner or purchase |
| Deterministic replay | 100 equivalent input permutations and repeated identical executions | Byte-identical canonical report |
| Duplicate identity | Repeated offer IDs, including payload conflicts | Fail closed; never select between ambiguous identities |
| Non-domination | Offers that trade cost against time/risk and exactly equal offers | Preserve all non-dominated options; no hidden weighted-score tie-break |
| Purchase authority | Static source audit and integration contract tests | Zero route from this evaluator directly to money movement or order dispatch |

## Proposed next benchmark campaign

These numbers are targets for future execution, not current results.

### 1. Frozen adversarial corpus

Create a versioned, license-reviewed fixture set covering at least 30 hand-authored cases and 100,000 deterministic generated requests. Vary:
- 0–1,000 offers per request;
- quantity and price skew; fixed and variable shipping/tax/fee combinations;
- quote age, quote expiry, credential expiry, and delivery deadlines;
- competing quality/cost/lead-time/risk trade-offs;
- duplicate IDs, missing claims, stale observations, incompatible units/currencies, and arithmetic boundaries;
- supplier concentration and alternative-source coverage once that evidence model exists.

Each case needs a stable ID, input digest, expected rejection reasons, expected eligible set, expected Pareto frontier, oracle version, and an explanation of the governing invariant. Synthetic data must remain explicitly synthetic.

### 2. Independent oracle

Implement the oracle separately from production ranking logic. For each frozen fixture:
- calculate hard eligibility independently;
- calculate pairwise dominance independently;
- compare all output IDs and rejection reasons exactly;
- preserve minimal counterexamples for every mismatch.

Do not let a source-owned output decide its own expected result. The first module's unit tests are not a substitute for the independent oracle.

### 3. Baselines

Compare the capability-first evaluator against:
- cheapest listed offer (baseline demonstrating why price alone is unsafe);
- a fixed weighted-score selector (baseline demonstrating sensitivity to arbitrary weights);
- eligible cheapest total-landed-cost selector (economic baseline, not the universal answer);
- a policy-explicit choice among the same Pareto frontier (decision-process baseline).

All strategies must receive the same frozen input. Report feasibility violations separately from outcome quality.

### 4. Operational metrics after safety qualification

Report, without mixing the dimensions:
- cost and time regret relative to a stated oracle and declared policy;
- proportion of requests with at least one feasible offer;
- eligible-offer recall and false eligibility rate;
- frontier size and proportion of decisions requiring human/policy resolution;
- cost breakdown completeness and quote staleness at dispatch recheck;
- supplier concentration / single-source exposure where measured data support it;
- p50/p95/max latency at 10, 100, 1,000 and 10,000 offers;
- memory use, bytes retained per offer, and canonical report size;
- results by scenario family, including censored/failed cases.

Initial performance hypothesis after compilation is measured: p95 below 50 ms for 1,000 offers and below 1 s for 10,000 offers on a named, pinned host. The current pairwise frontier algorithm is quadratic; these targets may fail and must be revised only after preserving the measurements and examining algorithmic bottlenecks. No performance result is claimed.

## Cross-project contract

- **Symthaea** may search and extract candidate offers, but must preserve source URLs, fetch time, raw-source digests where lawful, extraction version, uncertainty, and field-level evidence. Untrusted page instructions never grant authority.
- **Sol Atlas** supplies regional context such as facility location, production-capacity evidence, transport alternatives, and dependency concentration. A modeled or estimated quantity must not be mislabeled as live inventory.
- **Mycelix** owns request coordination, supplier identity/credentials, RFQs, accepted commitments, delivery claims, dispute histories and explicit authorization.
- **Execution gateway** is a separate future boundary. It rechecks quote freshness and authorization, confirms supplier/item/quantity/total/destination, binds an idempotency key, and reconciles ambiguous outcomes. This evaluator cannot call it.

Future adapters should use standards as mappings, not as truth or authority: [ValueFlows](https://www.valueflo.ws/specification/vfspec/) for economic intent/commitment/claim/event distinctions; [GS1 EPCIS/CBV](https://www.gs1.org/standards/epcis) for visibility events; and [Open Contracting Data Standard](https://www.open-contracting.org/data-standard/) for public procurement lifecycle publication. The Open Contracting Partnership's [red-flags guide](https://www.open-contracting.org/resources/red-flags-in-public-procurement-a-guide-to-using-data-to-detect-and-mitigate-risks/) is a useful reference for public-sector risk indicators, not a substitute for jurisdiction-specific policy or findings of wrongdoing.

## Advancement gates

1. Exact-head Rust formatting, unit tests and dependency-free qualification workflow pass.
2. Independent oracle and frozen adversarial corpus are added and run.
3. Field-level provenance and versioned capability equivalence are specified and tested.
4. Offline web-capture fixtures and at least one sandbox supplier adapter pass freshness, hostile-input and replay tests.
5. A human-approved draft RFQ/purchase order is compared end-to-end with Mycelix need-to-fulfilment and the Logistics Commons S0 model.
6. Only after these gates: a limited execution gateway with a hard budget cap, explicit principal authorization, idempotency and reconciliation tests.

Passing the reference evaluator alone proves none of the higher gates.
