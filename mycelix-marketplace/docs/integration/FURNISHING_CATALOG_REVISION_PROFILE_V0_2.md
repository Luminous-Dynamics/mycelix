# Furnishing catalog profile v0.2: fork-aware offers

**Status:** additive draft contract stacked on furnishing bridge v0.1. The v0.1 schema, fixtures and semantics remain unchanged.

## Why v0.2 exists

Marketplace updates are append-only actions. When one listing's locally observed update graph has more than one leaf, selecting a price or inventory record by timestamp, action-hash ordering or preference silently invents authority. A furnishing profile must preserve the stable listing root, every locally observed leaf, the resolution policy and the limited observation scope.

## Contract invariants

- Single observed head: exactly one action hash is present; the exact selected action equals that head; a revision label is present.
- Concurrent heads: at least two distinct heads; no selected action and no revision label; observed state is `conflict`; price, quantity/availability, delivery and terms remain unknown/null.
- Receipt policy is versioned and scoped to `local_dht_update_graph`.
- Coverage must be `local_observed_links_only`: one conductor's visible links and update graph are not proof of global DHT convergence or a complete catalogue.
- The schema and Rust cross-field auditor separately validate structure and semantic consistency. The auditor checks count consistency, head uniqueness, selected-head equality, and prohibited conflicting commercial claims.
- The contract creates no live listing, stock, quote, reservation, order or transaction. v0.1 remains valid for its original, less expressive shape.

## Test plan

The focused furnishing-profile audit validates all v0.2 fixture profiles against the Draft 2020-12 schema, passes the semantic fixture audit, and rejects:
- a selected action/revision while two heads are in conflict;
- seller-asserted price or confirmed quantity while conflict is unresolved;
- a single-head receipt whose selected action disagrees with its sole observed head;
- any claim of global catalogue completeness.

All profile values in the v0.2 fixture are synthetic only.
