# AC-007 — Qualified Equivalence Views and Reversible Metric Projection

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-007 is the analytical boundary between AC-006 identity reconciliation and AC-004 capture metrics.

It answers a dangerous question conservatively:

> When several source records have been independently qualified as referring to the same public/legal entity, how can analysis group them without rewriting history?

The answer is an **ephemeral equivalence view**, never a destructive graph merge.

## Core invariant

`source graph + qualified identity links -> reversible analytical view`

never:

`source graph + identity links -> rewritten canonical graph`

AC-003 nodes and edges remain unchanged.

## Inputs

AC-007 accepts AC-006 `EntityIdentityLink` records only when every supplied link:

- passes the complete AC-006 structural validator;
- is in `Corroborated` state;
- is `aggregation_eligible`;
- has no unresolved challenge;
- remains reversible.

Invalid or merely proposed links fail closed rather than being ignored.

## Deterministic components

Qualified links form provisional undirected components.

Component membership and component IDs are deterministic over sorted source node IDs. Each component retains the exact sorted AC-006 link references that produced it.

The view also retains the node-to-component mapping used by analytical projection.

## Cross-link contradiction detection

A link can be valid in isolation while becoming contradictory when combined transitively with other links.

AC-007 therefore validates provisional components before exposing them for use.

The initial contradiction theorem is deliberately conservative:

- collect public entity identifier evidence across the component;
- for each identifier scheme, inspect all distinct identifier values represented in that component;
- if one scheme contains multiple identifier values, fail closed unless authoritative crosswalk evidence explicitly reconciles every conflicting pair under that scheme.

Example:

`A -- GB-COH:11111111 -- B`

and

`B -- GB-COH:22222222 -- C`

must **not** silently imply one Companies House entity unless authoritative evidence reconciles `11111111` and `22222222`.

This protects transitive closure from amplifying a local identity mistake into a large false component.

## Node-kind consistency

If the same source node ID appears across supplied links with incompatible AC-003 node kinds, AC-007 rejects the view.

This prevents a single textual identifier from becoming both an organization and a different graph class through cross-link composition.

## Duplicate link identity

Duplicate AC-006 link IDs are rejected rather than deduplicated silently. A receipt must have an unambiguous link lineage.

## Procurement concentration projection

The first analytical consumer is AC-004 procurement supplier concentration.

AC-007 computes two observations:

1. the baseline AC-004 supplier HHI on the original source edges;
2. the projected AC-004 supplier HHI under the qualified equivalence view.

Only equivalence components containing at least two supplier nodes that actually occur in the current award population are applied to the projected calculation.

This means the receipt records only identity links that can change the current grouping, not unrelated qualified links that happen to exist elsewhere.

## No source mutation

For the projected calculation, AC-007 clones the relevant edge population and substitutes an ephemeral `Aggregate` source node ID for members of an applied equivalence component.

Original AC-003 edge IDs, provenance, targets and source graph records remain untouched.

The projected observation still preserves AC-004's exact input edge references.

## Projection receipt

`IdentityProjectedObservation` preserves:

- a stable projection ID;
- baseline AC-004 observation;
- projected AC-004 observation;
- exact applied equivalence components;
- exact AC-006 link references responsible for regrouping;
- explicit projection method reference.

The projected observation also carries an uncertainty limitation stating that supplier identity grouping came only from AC-006 aggregation-eligible links and did not mutate source graph identity.

## Interpretability

A consumer can therefore report:

> Raw records imply N supplier identities and HHI X/Y. Applying qualified identity links L1..Ln changes the grouping to M effective entities and HHI A/B. Removing or challenging any qualifying link recomputes the view without changing source records.

This is substantially stronger than storing one unexplained post-deduplication concentration number.

## Reversal

If an AC-006 link becomes challenged, rejected or superseded, it becomes ineligible before AC-007 view construction.

A later calculation simply omits that link and rebuilds the component structure.

No rollback of AC-003 is required because AC-003 was never rewritten.

## Failure semantics

AC-007 fails closed for:

- duplicate identity-link IDs;
- invalid AC-006 links;
- non-aggregation-eligible links;
- unresolved challenges;
- cross-link node-kind conflicts;
- conflicting same-scheme identifiers without authoritative reconciliation;
- baseline AC-004 calculation failures;
- projected AC-004 calculation failures.

It does not silently drop a problematic identity link and continue with a partial view.

## Non-goals

AC-007 does not:

- create or review identity links;
- reconcile natural persons;
- mutate AC-003 nodes or edges;
- declare one source node canonical;
- resolve corporate succession or parent/subsidiary relationships;
- infer corruption from increased concentration;
- change AC-004's underlying HHI arithmetic or award-edge weighting assumptions;
- authorize sanctions or rights-affecting action.

## Qualification gate

Before AC-007 is qualified:

1. exact-subject Civic workspace tests pass;
2. rustfmt and warnings-denied Clippy pass;
3. property tests confirm source edges are byte-for-byte unchanged after projection;
4. mutation tests prove challenged/non-corroborated AC-006 links cannot enter a view;
5. adversarial transitive-closure tests cover conflicting same-scheme identifiers, duplicate link IDs and node-kind conflicts;
6. crosswalk fixtures prove an explicit authoritative reconciliation can resolve a same-scheme identifier conflict;
7. independent arithmetic reproduces both baseline and projected HHI exact ratios;
8. lineage tests prove unrelated identity components are excluded from `applied_identity_link_refs`;
9. reversal tests prove withdrawing one link deterministically reconstructs the expected components and metric;
10. review confirms AC-007 remains an analytical view and does not introduce a write path into AC-003 source identity.

## Next tranche

AC-008 should add **capture-sensitive identity projection diagnostics**: quantify how much entity reconciliation changes concentration, expose whether one identity component dominates the change, and flag results that are highly sensitive to a small number of reconciliation assumptions. This should remain an uncertainty/robustness analysis, not an accusation engine.
