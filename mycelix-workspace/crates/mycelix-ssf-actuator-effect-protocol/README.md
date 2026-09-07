# SSF Actuator Effect Protocol v0.1

Provider-neutral contract for the first external-effect state machine after pre-invocation qualification.

The protocol separates:

1. exact effect subject;
2. actuator-side durable claim-key reservation;
3. one exact invocation attempt;
4. `Confirmed | ProvenNotApplied | OutcomeUnknown` result;
5. mode-specific recovery policy.

The durable SSF claim record is the actuator idempotency/deduplication key. Invocation attempt IDs may vary across later retries, but the effect subject behind a claim key may never change.

This crate defines interfaces and pure validation rules only. It does not implement an actuator, resolve provider bytes, or perform a physical/external effect.
