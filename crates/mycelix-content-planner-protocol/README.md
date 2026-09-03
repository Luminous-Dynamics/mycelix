# mycelix-content-planner-protocol

Versioned external-planner interoperability for Mycelix Content Fabric.

This crate lets an external recommendation engine such as Symthaea receive a narrow, auditable planner request without importing Holochain, Iroh, CAS, marketplace, lease, or executor APIs.

## Security boundary

- requests are derived from a CF-06A policy-qualified pool and CF-06B deterministic baseline;
- request IDs commit every decision-relevant request field using explicit domain-separated framing;
- JSON is transport encoding only and is never used as stable identity;
- recommendations must echo the exact request ID and planner-input ID;
- rankings must be complete duplicate-free permutations of the qualified candidate universe;
- selected actions must be unique, known, ranking-order-preserving, and pass CF-06A `validate_selection()`;
- accepted external recommendations remain `PlannerAuthorityV1::RecommendationOnly`;
- acceptance IDs commit engine identity/version, request/input IDs, ranking, target, and selected subset.

The request/acceptance hashes are audit commitments, not signatures or proof that upstream qualification was legitimately performed. Execution boundaries must independently establish authorization and revalidate hard policy.
