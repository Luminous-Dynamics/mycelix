# Robust portfolio procurement: contract and benchmark plan v1

**Status:** proposed engineering contract; no portfolio optimizer, live integration, or benchmark result is claimed by this document.  
**Parent work:** [PROC-PORTFOLIO-001 #5088](https://github.com/Luminous-Dynamics/mycelix/issues/5088).  
**Prerequisite:** capability-first single-offer evaluator in [PR #5072](https://github.com/Luminous-Dynamics/mycelix/pull/5072), stacked on economic coordination reference PR #5037.  
**Composition targets:** [LOG-COMMONS #4900](https://github.com/Luminous-Dynamics/mycelix/issues/4900), [offline twin #4901 / PR #4902](https://github.com/Luminous-Dynamics/mycelix/pull/4902), production/dependency network #4814, power/dependency model #4806.

## 1. Paradigm shift

Traditional sourcing often asks, "Which eligible offer looks best?" The next question is, "Which feasible allocation portfolio meets this need while remaining affordable, supportable, contestable, and resilient when part of the supply network fails?"

A low-cost individual offer does not establish that a low-cost, low-risk portfolio exists. Two suppliers may share a parent company, factory, critical sub-tier, port, route, energy source, or correlated disruption exposure. Conversely, sourcing from multiple names does not prove meaningful diversification.

The portfolio layer therefore consumes independently qualified candidate offers and **does not replace** product identity, capability equivalence, evidence verification, supplier credentials, inventory authority, logistics events, financial settlement, or public-contracting semantics. It evaluates allocation proposals only.

Core invariants:

- Recommendation != authorization.
- Supplier name diversity != demonstrated failure-domain independence.
- Forecast capacity != verified available capacity.
- A synthetic scenario != an empirical probability distribution.
- Unknown lifecycle impact != zero impact.
- A Pareto frontier != a unique winner.
- Simulation success != real-world qualification.

## 2. Responsibilities and boundaries

| Component | Owns | Must not own |
|---|---|---|
| Mycelix | Authorized request/policy records, supplier identity links, evidence references, award authority, dispute/appeal records, receipts | Treating a score as authorization or a signature as proof of physical truth |
| Capability-first sourcing evaluator (#5072) | Eligibility checks and individual-offer comparison | Semantic equivalence of products, live source authentication, stock mutations, RFQs, awards, spend or payments |
| Portfolio evaluator (this proposal) | Feasible allocation sets, portfolio trade-offs, scenario exposure, concentration and explanations | Canonical supplier/product masters; legal determinations; award or execution authority |
| Symthaea | Advisory demand forecasts, candidate-research plans, uncertainty estimates, outcome calibration | Supplier admission, silent equivalence decisions, award, inventory/route/order/payment execution |
| Sol Atlas | Spatial/dependency projection with source and freshness labels | Turning estimated capacity or a modelled route into verified inventory or live dispatch authority |
| Symtropy | Bounded physical/operational counterfactuals where its model is appropriate | Making the simulated world authoritative for real operations |

Adapters must preserve upstream identities and provenance rather than creating parallel registries.

## 3. Input contract

Every evaluation binds to a versioned request, policy, input manifest and scenario set.

### Request

Required fields:

- stable request ID and capability-profile ID/digest;
- required quantity in an explicit canonical unit;
- budget and currency, including the currency minor-unit convention;
- required-by window and permitted partial-fill/substitution rules;
- allocation policy version (split allowed, min/max lot, max supplier share, minimum qualified failure domains where required);
- required evidence origin, evaluation timestamp, freshness limits and exact evaluator profile;
- declared hard constraints and declared objectives, each versioned.

The capability profile digest is an equality boundary, not proof that two different products are equivalent. Cross-product substitutions require a separate reviewed, versioned mapping with evidence and scope; language-model inference alone cannot authorize equivalence.

### Offer and allocation projection

Each candidate needs a stable offer identity and references to the #5072 eligibility result, supplier and credential identities, capability mapping, unit/currency, quote and capacity observations, minimum lot, order increments, maximum allocatable quantity, delivery window, fee/tax treatment, relevant logistics path, and evidence/provenance status.

Capacity evidence must include who observed it, when, the validity bound, source and extraction profiles, and its evidence status/origin. A supplier's catalog capacity, its committed capacity, observed on-hand inventory, and inventory reserved by another order are distinct concepts. The portfolio evaluator may allocate only the capacity semantics explicitly admitted by policy.

Failure-domain links may represent ownership, production site, critical sub-tier, facility, route/port, utilities, region, or other declared common-cause exposures. Each link has its own source, timestamp, scope and evidence qualification. Missing links remain unknown: do not infer independence from absence.

### Scenario set

Each scenario has a stable ID, version, input digest and explicit perturbation semantics, such as a site outage, route closure, capacity loss, lead-time shock, or supplier withdrawal. Record whether scenarios are empirical, calibrated, expert-authored, or synthetic. Do not interpret an unweighted synthetic test suite as event probabilities or report an invented expected loss.

## 4. Portfolio semantics

For offer i, define a non-negative integer allocation quantity x_i in the request's canonical unit. More complex multiple-line requests generalize this to x_(line, offer), only when the offer is admitted for that line by a reviewed capability/substitution mapping.

### Required integration seam with #5072

Do **not** feed the existing single-offer report directly into a split-award optimizer and assume it represents every usable source. The current single-offer contract rejects an offer whose available quantity is below the entire request quantity. That is appropriate for single-source fulfillment, but it can exclude a valid partial-allocation supplier before portfolio optimization begins.

The implementation must therefore separate reusable offer evidence/qualification checks from allocation-dependent checks. The reusable layer should validate identity/provenance presence, admitted capability/profile, unit/currency, evidence status/origin, source/quote/credential freshness, and risk-model compatibility. The allocation layer must then check the proposed quantity against observed capacity, lot rules, request budget and deadline for that allocation. If fees are nonlinear or apply per order, the portfolio cost function must encode the fee basis explicitly rather than multiplying the whole-request landed total by a quantity ratio.

This is a contract/refactor requirement, not a claim that #5072 currently supports partial-offer portfolios. Keep its current single-source behavior intact until a tested shared eligibility API is available. Differential tests must show that (a) a source with only part of total demand can participate when split awards are allowed, (b) the same source is correctly rejected for a single-source request, and (c) no portfolio can use a source whose evidence/credential gates fail.

At minimum, a feasible portfolio must enforce:

1. **Demand conservation:** allocated/accepted quantities satisfy the requested quantity, or the policy explicitly permits partial fulfillment and the shortfall is reported.
2. **Capacity:** allocation never exceeds qualified and policy-eligible capacity; allocations respect observation freshness, validity, and concurrent reservation semantics.
3. **Lot constraints:** nonzero allocations obey minimum lot, order increments, and maximum lot/capacity. Infeasible indivisible lots are rejected, not rounded silently.
4. **Budget:** all included price, shipping, taxes, and other declared cost components use checked arithmetic and consistent currency. Missing mandatory cost components make the portfolio unqualified or invoke an explicit policy bound; they must not default to zero.
5. **Time:** each allocated quantity's arrival/acceptance window is assessed against the request. A late fraction cannot hide behind an on-time average.
6. **Admission:** every contributing offer passes the upstream hard gates. A portfolio cannot wash an ineligible offer into eligibility by pairing it with a good one.
7. **Policy:** split/partial fill, supplier-share limits, approved exceptions and any minimum independence requirement are explicit and versioned.
8. **Evidence:** every derived value traces to source input IDs, policy/version, transformations, and the evaluator profile. Contradictory capacity or dependency claims remain unresolved until the relevant authority or evidence process resolves them.

The first exact solver should support a deliberately bounded problem size and enumerate all feasible allocations for a small-instance oracle. A scalable heuristic comes later, only after differential agreement on tractable cases. A heuristic must state its method, stopping condition and any known optimality bound; it must not call its answer optimal without proof.

## 5. Objectives without a hidden universal score

Hard safety, legal, compatibility, qualification and spend-authority constraints are feasibility gates, not weighted objectives. After those pass, report the vector of portfolio properties separately.

Recommended dimensions:

- total landed cost and explicitly itemized unknown costs;
- quantity delivered by deadline and quantity late/unfulfilled;
- worst-case shortage over the declared scenario set;
- recovery time or time to a feasible substitute after an explicitly modelled disruption;
- source/site/critical-sub-tier/route concentration and known dependency overlap;
- lifecycle cost and repair/reuse/refurbishment option where evidence supports a comparable boundary;
- environmental impact per comparable usable unit where methodology and source data are qualified;
- contestability/accessibility indicators: qualified bids, small-supplier access, award splits and appeal outcomes, subject to privacy and applicable law.

Do not collapse these dimensions into an opaque universal score. There are two acceptable decision modes:

1. Preserve all non-dominated feasible portfolios for an authorized human/policy decision.
2. Apply a declared, versioned policy (for example, a hard concentration cap followed by a lexicographic ordering of service constraints and landed cost). Publish the decision rule and the reason it narrows the choice set.

If a portfolio is preferred because it passes a target supplier-share cap or improves a scenario outcome, report the exact input and rule. A supplier count alone is not a resilience certificate.

## 6. Independent verification

The verification path must be logically separate from the production allocation algorithm.

### Small-instance exhaustive oracle

For bounded cases (initially 2–8 qualified offers and small integer demand), enumerate every allowed allocation, apply constraints independently, and derive the feasible/non-dominated portfolio set without reusing the production optimizer's selection routine. Compare canonical portfolio IDs, allocations, objective vectors and failure/rejection reasons exactly.

Persist smallest counterexamples for every mismatch. If the state-space bound is reached, report INCOMPLETE rather than quietly sampling and implying exhaustive coverage.

### Required properties

- sum of allocations never exceeds request or accepted demand policy;
- no allocation exceeds the qualified capacity, lot increment, or source-specific cap;
- a disallowed split never occurs and a required split/independence condition is never faked;
- no ineligible offer becomes eligible through portfolio combination;
- currency, unit, capability, evidence origin, policy version and risk/scenario profile are never mixed silently;
- input order permutations yield byte-identical canonical reports;
- equal objective portfolios remain co-equal unless a declared rule legitimately distinguishes them;
- conflicting source/capacity/failure-domain claims are preserved;
- replay of an allocation/effect event cannot duplicate demand or capacity consumption;
- removing/revoking an organization blocks new authority without rewriting historical receipts;
- privacy views expose no confidential price, customer identity, worker movement or sensitive facility data outside the receiving purpose's access policy;
- recommendations and simulated allocations are structurally distinct from committed orders and payments.

## 7. Frozen adversarial corpus

Start with at least 40 hand-authored, reviewable cases. Add deterministic generated small instances for exhaustive differential testing, then broader invariants at scale.

Mandatory case families:

1. Cheapest single offer fails under one declared disruption.
2. Naive equal split violates minimum lot or order increment.
3. Same parent company submitted under several supplier names.
4. Different suppliers share one critical plant, sub-tier, or route.
5. Two sources appear independent but one dependency link is unknown.
6. Correlated supplier/site/route outages defeat assumed independence.
7. Capacity snapshot expires before allocation confirmation.
8. Competing portfolios oversubscribe the last unit or transport slot.
9. A substitution has a matching text description but no admitted equivalence mapping.
10. A low-cost offer has an incompatible unit, currency, fee basis or tax scope.
11. Missing lifecycle data would incorrectly appear as zero impact.
12. Partial fulfillment is disallowed but the optimizer proposes it.
13. Cost objective wins only because late/unfulfilled quantity is averaged away.
14. An infeasible portfolio is made to look feasible by rounding.
15. Supplier concentration constraint is bypassed through related entities.
16. An optimizer stops early yet claims optimality.
17. Equal portfolios change winner under candidate reordering.
18. A synthetic failure scenario is mistakenly reported with calibrated event probability.
19. A public projection reveals confidential bid or facility information.
20. Revoked qualification is used by an old cached offer.

Each case contains a stable ID, versioned input manifest, expected feasibility or rejection, expected allocation/objective set where tractable, invariant ID, source/licence information and an explanation. Keep synthetic evidence distinctly marked.

## 8. Baselines and benchmark plan

Compare identical frozen inputs against:

- cheapest eligible single-source allocation;
- cheapest eligible portfolio with no resilience constraint;
- deterministic policy-constrained greedy allocation;
- naive equal split when splitting is allowed;
- exact exhaustive optimum for tractable small cases;
- declared multi-objective/lexicographic policy operating on the same feasible set.

Report safety failures separately; an unsafe low-cost baseline cannot make a failed constraint look like a quality trade-off.

### Proposed metrics

| Metric | Measurement |
|---|---|
| Feasibility violations | Count by invariant; target zero |
| Allocation correctness | Exact agreement with exhaustive oracle on every tractable case |
| Demand/capacity accounting | Zero conservation or oversubscription violations |
| Cost regret | Gap versus stated objective optimum on tractable cases; show denominator/degenerate-case handling |
| Service regret | Deadline shortfall/late quantity versus named baseline per scenario |
| Disruption robustness | Unfilled quantity and recovery options per named scenario; no invented probabilities |
| Concentration | Largest supplier share, parent/site/sub-tier/route exposure, and unclassified share |
| Contestability | Qualified offer count, small-supplier access, share of requests with one eligible source; report by category |
| Circularity | Repair/reuse/refurbishment share only for comparable, evidenced options |
| Performance | p50/p95/max wall time, peak memory, input/output sizes, solver method and stopping reason |
| Stability | Byte-identical replay for identical manifest/toolchain inputs |
| Abstention | Requests without a feasible portfolio correctly report infeasible/unknown rather than inventing a winner |

### Scale stages

- **S1 — contract:** 2–8 offers, very small integer demand, exact enumeration and independently written oracle.
- **S2 — generated correctness:** at least 10,000 deterministic small portfolio cases, comparing all cases within the declared exhaustive bound; broader generated cases verify invariants.
- **S3 — heuristic evaluation:** 16, 32, 64 and 128 offers with declared allocation granularity. Compare heuristic against exact results on small overlapping cases, and disclose an unknown optimality gap on large cases.
- **S4 — shadow workload:** replay privacy-reviewed historical or explicitly synthetic workloads, with source SHA, data license/authority, manifest digest and raw measurements.

Initial runtime hypotheses (not measured results): for one line, at most 32 eligible offers, integer demand up to 100 units, and at most 16 deterministic disruption scenarios, a reference heuristic should target p95 below 250 ms and bounded memory below 256 MiB on a named pinned host. Reassess after baseline implementation. Do not sacrifice correctness, provenance or conflict visibility to hit latency.

No result from S1/S2 demonstrates economic superiority, legal conformity, production readiness, or operational resilience. Those require appropriate data and independently qualified pilots.

## 9. Cross-project interoperability

Prefer mappings rather than replacements:

- **ValueFlows:** preserve distinctions between intents, commitments, claims and events; a recommendation is not an economic commitment.
- **GS1 EPCIS/CBV:** map qualified product visibility, custody, shipping/receiving and relevant sensor/certification events without treating a received message as physical truth by itself.
- **OCDS:** use public contracting lifecycle identifiers and publishable planning/tender/award/contract/implementation projections for public procurement, with privacy and jurisdiction rules.
- **ISO 20400:** consider sustainability in the procurement process without inventing environmental evidence or conflating supplier identity with performance.

Standards references:
- https://www.valueflo.ws/specification/vfspec/
- https://www.gs1.org/standards/epcis
- https://www.open-contracting.org/data-standard/
- https://www.iso.org/standard/63026.html

The data standard is an interoperability contract, not an authority source and not a supplier-truth oracle.

## 10. Exit gate and non-claims

This contract may be accepted only after the exact implementation revision, frozen corpus, independent oracle and failure cases are reviewed. An implementation must not be described as passing until an exact-head run completed successfully and an independent verifier agrees.

This proposal does **not** authorize live supplier scraping, product equivalence inference by an LLM, sending RFQs, real purchase orders, stock mutation, payments, dispatch, or changes to public procurement systems. Begin with civilian maintenance supplies, repairable IT equipment, food essentials, and community infrastructure.
