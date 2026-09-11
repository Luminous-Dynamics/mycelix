# mycelix-business-field-qualification

Field-qualification evidence contracts for read-only Business Fabric pilots.

Shadow accuracy is not enough to claim a useful field integration. A real pilot also needs to establish what adapter/mapping versions were used, which operational scope was observed, whether the data was sufficiently complete/fresh/conflict-free, whether important slices had enough coverage, and which limitations remain unresolved.

This crate makes those requirements preregistered and evidence-bearing. It does not execute business actions, grant authority, or infer causal savings from unexecuted recommendations.

## Guardrails

- connector and mapping identities are frozen by digest for the evaluation window;
- data-quality thresholds are registered before evaluation rather than chosen after seeing results;
- missing, conflicting, and stale records count against the gate instead of being silently excluded;
- important evaluation slices have explicit minimum sample sizes, preventing aggregate accuracy from hiding weak dayparts/locations/classes;
- field evidence binds the exact shadow-protocol digest and operational scope;
- a passing result supports only the capability/profile/scope represented by the plan;
- known limitations remain explicit and are never erased by a passing gate;
- a field-shadow pass remains non-authorizing A0-A2 evidence. It is not permission for autonomous writes and is not a causal estimate of financial impact.
