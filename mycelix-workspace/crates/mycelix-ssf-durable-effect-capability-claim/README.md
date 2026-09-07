# SSF Durable Effect Capability Claim v0.1

This crate durably claims one exact transient effect capability **before** any provider-handle resolution or actuator invocation. A claim is not an effect.

The store must enforce both attempt-ID uniqueness and durable-capability-record uniqueness. One `durable_capability_record` may produce at most one claim record across all claim attempt IDs.

Claim preparation consumes the owned transient capability together with fresh uncertainty-aware trusted time. The exact journaled subject binds the complete transient capability audit binding plus claim-time evidence. Stale capability authority fails before the external write boundary.

After `claim_effect_capability` begins, store errors, descriptor changes, malformed receipts, conflicting attempt IDs, pre-existing claims, and invalid durable frontiers become `OutcomeUnknown`; they never become blind retry permission.

A successful `DurablyClaimedEffectCapabilityV1` still performs no provider resolution or actuator invocation. It preserves the owned capability for the live path, requires another final pre-invocation generation/time check, and uses the durable claim record as the execution-attempt lineage.
