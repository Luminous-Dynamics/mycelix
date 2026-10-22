# NRVCA-004: decision value under explicit constraints

Status: synthetic benchmark prototype, not a field evaluation. All interventions and numbers are fictional. This does not measure Johannesburg's water system or any actual country's productivity.

## Research shift

National accounting should not stop at “what was produced/spent?” or even “what services exist?” A practical test is whether a decision-maker, using evidence-aware multidimensional information, can choose a better feasible allocation under explicit budget, service, distribution, reliability and physical-resource constraints.

This prototype does not equate “better” with a single unqualified scalar. It compares:
1. a strong simple baseline that maximizes nominal service output under a fixed budget;
2. an NRVCA-style declared policy profile that maximizes the same service objective while enforcing independently specified non-compensable floors.

This makes the normative choice visible. In a real deployment, the equity floor and environmental/reliability limits would require legitimate public or operator governance and sensitivity analysis; the model must never invent these weights or silently relax them when infeasible.

## Toy instance

The fixture defines four fictional water-service interventions. Each has costs, service quantity intervals, underserved-service intervals, physical emissions intervals and reliability intervals. For robust feasibility, the prototype uses conservative lower bounds for service and underserved coverage, an upper bound for emissions, and a service-unit-weighted reliability lower bound. That ratio is bounded over every lower/upper service-volume endpoint combination; merely weighting each intervention by its lower service bound would not in general yield a valid worst-case ratio. This is a deterministic interval stress test, not a probability distribution or a forecast. The optimization objective maximizes nominal service units among feasible choices.

Because there are four binary interventions, an independent exhaustive oracle checks every one of the 16 allocations. The baseline selects central-main repair plus pressure management: 1,450 nominal service units at cost 10, but the lower bound for underserved service is only 410 and its emissions upper bound is 13.5, exceeding the declared cap of 12. Under the declared policy profile, the chosen set is peripheral leak repair plus pressure management: 1,250 nominal service units at cost 9, an underserved-service lower bound of 900, emissions upper bound of 9.5, and a conservative service-weighted reliability lower bound of about 0.9504.

The policy-constrained choice is not claimed to be universally better: it is feasible under a stated normative profile the output-only baseline ignores. The benchmark asks whether all constraints and trade-offs are reported transparently. A stricter underserved-service floor produces an explicit infeasible result; constraints must not be weakened silently to manufacture a recommendation.

## Sensitivity to normative choices and deep uncertainty

Four additional policy profiles are enumerated against the same 16 allocations. With a loose equity floor and an emissions cap of 14, the service-maximizing central-main repair + pressure-management package is feasible. With an underserved-service floor of 800, peripheral leak repair + pressure management is selected instead. With an equity floor of 800 plus a reliability floor of 0.96, or an equity floor of 901 under the tighter emissions cap, the problem is infeasible. Each result is part of the fixture, not inferred from an unlabeled score.

This structure follows established decision-making-under-deep-uncertainty methods rather than claiming to invent them. The World Bank has described Robust Decision Making as useful for multidimensional trade-offs when scenario likelihoods are difficult to assign, and OECD work recommends assessing wider social and distributional impacts alongside economic impacts ([World Bank: Investment Decision Making Under Deep Uncertainty](https://ppp.worldbank.org/library/investment-decision-making-under-deep-uncertainty); [OECD Regulatory Policy Outlook 2025](https://www.oecd.org/en/publications/oecd-regulatory-policy-outlook-2025_56b60e39-en/full-report/regulating-for-people_6d2fc8d4.html)).

The next robustness tier should replace independent interval endpoints with explicitly correlated, physically admissible scenarios where infrastructure effects, demand, staffing, rainfall/drought and repair lead times interact. Report worst-case performance, scenario regret and the conditions causing failure; do not invent scenario probabilities when none are defensible.

## Tests and acceptance gates

- Enumerate all (2^4=16) allocations and match the frozen baseline and policy choices.
- 100% agreement with the independent exhaustive oracle on this tiny problem.
- Zero allocations accepted if they violate declared budget, equity, emissions or reliability constraints.
- A stricter policy floor that cannot be met must yield infeasibility, not a silently relaxed floor.
- Increasing budget cannot shrink the set of feasible allocations.
- Changing a descriptive/marketing label cannot change numeric outcomes or the optimum.
- Evidence status and uncertainty bounds remain explicit; missing values must not become zero.
- Keep service, distribution, reliability and physical burden in their own units in reports. The scalar objective here is only a declared selection rule inside one toy constrained optimization problem.

These are prototype gates, not established performance against a production allocator.

## Benchmark ladder

1. **Synthetic exhaustive oracle** — this slice, with known expected answers.
2. **Historical replay** — reconstruct previous proposals and allocations without changing operations; compare the incumbent, service-only and constrained candidates.
3. **Shadow mode** — generate recommendations from frozen source vintages but do not dispatch resources; measure feasibility, operator review burden, stability to revisions and how often recommendations would change.
4. **Prospective trial** — preregister an outcome such as cost per quality-adjusted service unit, access gaps, uptime, and external-resource burden. Use an appropriate comparison design and independent review.
5. **Replication** — repeat with another operator, region, period and independent dataset.

## Metrics that can justify a real superiority claim

- Feasibility rate and count of constraint violations.
- Objective value against exhaustive oracle on small cases; regret where useful.
- Delivered units and quality, with uncertainty and coverage denominator.
- Underserved / worst-group floors and gap changes.
- Physical burden, recovery and resilience.
- Decision stability under plausible intervals and policy thresholds.
- Data/provenance coverage, total human review time and fully loaded deployment cost.
- Prospective outcome difference with uncertainty, not merely a model score.

## Limitations

The current toy model assumes interventions combine additively and are binary; it does not include network interactions, shared crews, lead times, geographic distance, legal duties, intervention dependencies or causal effects. These are explicit next extensions. No real-world benefit, impact, optimal policy, SNA/SEEA conformity or economic value estimate is claimed.
