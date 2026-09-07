# SSF Pre-Invocation Actuator Qualification v0.1

Freshly qualifies one exact durable claimed-effect lineage against the exact live execution/provider/actuator environment immediately before any external invocation protocol.

- No actuator call exists in this crate.
- Live execution generation and source-owned provider generation must equal the generations already carried by the claimed capability lineage.
- The actuator declares an explicit recovery mode: transactional claim-key deduplication, idempotent replay by claim key, or non-idempotent/no-automatic-retry.
- Fresh handle-resolution and deduplication evidence are rebound to the exact durable claim record.
- Fresh uncertainty-aware trusted time must fit inside every inherited/current validity ceiling.
- No mode is described as exactly-once unless a future actuator-specific proof establishes stronger semantics.
