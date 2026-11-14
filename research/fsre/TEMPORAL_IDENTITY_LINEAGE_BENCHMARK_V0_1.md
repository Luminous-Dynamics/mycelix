# FSRE temporal identity lineage benchmark v0.1

## Research objective

Evaluate whether a typed event-sourced identity history can preserve correct entity identity and denominators across time without silently converting uncertain links into continuity claims.

## Current reference implementation

The companion crate `tools/fsre-temporal-identity-lineage` validates event shape and replays a history using separate recorded-time and effective-time cutoffs. It currently supports typed event cardinality, evidence/adjudication metadata presence, duplicate event-ID rejection, cycle rejection and deterministic output.

It is a structural reference implementation, not a probabilistic matcher, source authenticator, population estimator or service entitlement allocator.

## Adversarial benchmark families

1. Rename with stable physical identity.
2. Identifier reassignment to a different physical asset.
3. One works split into multiple operational units.
4. Multiple works merged under one operator.
5. Multiple treatment trains at one physical site.
6. Service-boundary change without asset relocation.
7. Relocation with reused source-native identifier.
8. Ownership/operator transfer with delayed registry update.
9. Temporary shutdown, retirement and recommissioning.
10. Sparse bridge record that would falsely connect two clusters.
11. Late correction after a historical decision cutoff.
12. Overlapping or contradictory effective-time transitions.
13. Crosswalk evidence whose source snapshot is superseded.
14. Split/merge with no defensible service-entitlement allocation.

## Metrics and proposed targets

These are research targets, not achieved results.

- False-positive identity bridge rate: 0 on adversarial gold fixtures.
- Event-type precision: >=99.5% on held-out adjudicated events.
- Event-type recall: >=95%, reported with confidence intervals and abstention rate.
- Historical as-of replay: 100% exact agreement with frozen oracle views.
- Row-order invariance: 100% byte-identical reports.
- Denominator-transition correctness: 100% on fixtures with explicit allocation rules; otherwise abstain.
- Late-arrival leakage across recorded-time cutoffs: 0.
- Unresolved/conflicting transitions silently promoted to identity: 0.
- Performance: report p50/p95 latency and peak memory at 10^3, 10^4, 10^5, 10^6 and 10^7 events on declared hardware.

## Comparators

Compare against (a) static one-to-one crosswalks, (b) current-state register joins, (c) probabilistic record linkage with human adjudication, (d) event-sourced domain models without bitemporal replay, and (e) FSRE lineage with each invariant ablated.

All methods receive the same frozen event evidence, identity vocabulary, adjudication labels, time cutoffs and resource budgets. Report precision, recall, abstention, false bridges, denominator error and replay cost. Do not compare unmatched datasets or imply superiority from synthetic fixtures alone.

## Next mathematical connection

A future partial-identification layer should consume unresolved lineage as a set of admissible identity/denominator worlds. It may bound downstream service outcomes only where the set of worlds is explicit and the evidence is eligible. Rights, authority, safety and unresolved source authentication remain hard gates; they are not numeric uncertainty parameters.
