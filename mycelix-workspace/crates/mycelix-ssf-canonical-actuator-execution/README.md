# SSF Canonical Actuator Execution v0.1

This crate is the canonical bridge from `FinalActuatorInvocationReadyV1` to the provider-neutral actuator-effect protocol.

Reservation remains non-effecting. The wrapper consumes the final readiness typestate, retains a copyable audit binding to the exact durable invocation journal lineage, reserves the exact stable effect identity, and refuses to invoke unless fresh trusted time proves that the readiness, attempt, actuator generation, and reservation are all still live.

Immediately before and immediately after the external actuator call, the same independently expected trusted-time profile is queried. Post-invocation time failure, rollback, authority expiry during the call, actuator/reservation expiry, or a stale effect receipt forces the canonical qualified disposition to `OutcomeUnknown`.

The actuator's raw reported receipt is retained separately from the canonical disposition, so a reported `Confirmed` result is not erased merely because the authority/time layer cannot certify it terminally.

The wrapper never accepts caller-supplied payload bytes or a replacement operation handle, does not claim exactly-once external effect, and does not create replay authority.