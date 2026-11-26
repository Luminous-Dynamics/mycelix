# Craft Impact Qualification V1 — Benchmark Protocol

- Work item: CRAFT-IMPACT-QUALIFICATION-001 (#6763)
- Status: proposed qualification protocol; no results measured
- Version: 1.0-draft
- Date: 2026-11-26

## Matched baselines

Compare equivalent tasks using identical source/evidence packets, context, authority metadata, time budget and owner-native leases:
1. Blanket transitive invalidation.
2. Static traceability plus manual change review.
3. TTL-only expiry.
4. CISA/NTIA VEX-style scoped impact status, where the task is actually a product/component-vulnerability question.
5. Human domain assessment with independent review.
6. Claim Support dispositions without alternative support-set rules.
7. Craft Impact Qualification V1.

Do not compare VEX to general evidence sufficiency as though they solve the same problem. Do not claim Jama Connect or IBM DOORS Next lack impact analysis; compare configured real workflows and report non-equivalent dimensions.

## Synthetic adversarial families

- exact event, artifact, edge or context digest mismatch;
- changed jurisdiction or population scope;
- source is linked but absent from the exact use path;
- not-affected justification lacks evidence;
- assessor and reviewer are the same principal;
- high-consequence packet lacks a SafetyAuthority review;
- reviewer lease expires or review digest changes;
- UnderInvestigation attempted as NotAffected;
- one alternative set is stale while another remains directly supported;
- two documents share a source, instrument, operator or model lineage;
- k-of-n quorum is reached only by duplicated/correlated sources;
- stale or partial evidence is counted as direct support;
- contradictory evidence is hidden by a favorable alternative;
- a determination is replayed against a newer downstream revision.

## Proposed gates

- 0 false NotAffected dispositions in frozen high-consequence adversarial fixtures.
- 100% rejection of scope, revision, digest, review, lease and authority mismatch.
- 0 high-consequence reliance while UnderInvestigation.
- 0 false invalidations when a valid predeclared alternative support set remains sufficient.
- 0 k-of-n quorum inflation from duplicated IDs or shared independence groups.
- 100% correctness over exhaustive four-candidate support masks and all k values against an independent oracle.
- 100% deterministic replay across independent implementations.

These are frozen-corpus goals, not universal risk guarantees or achieved results.

## Human and operational measures

Report false holds, false exemptions, source common-cause misses, reviewer minutes, requalification latency, assessor disagreement, and the fraction of decisions for which the context or independence graph is incomplete. Report each separately; do not create a universal confidence score.