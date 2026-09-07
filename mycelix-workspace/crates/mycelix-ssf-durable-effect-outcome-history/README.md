# SSF Durable Effect Outcome History v0.1

Append-only durable outcome history for one exact journaled actuator invocation.

This crate keeps **effect reality** separate from **outcome-recording durability**. `Confirmed`, `ProvenNotApplied`, and `OutcomeUnknown` are actuator facts; failure to persist one of those facts does not rewrite it.

Outcome histories are monotonic. `OutcomeUnknown` may later refine to `Confirmed` or `ProvenNotApplied`; terminal outcomes cannot be reversed or overwritten. The exact journal record, invocation manifest, actuator receipt/evidence, predecessor head, observation time, and store generation remain bound.

This crate does not authorize replay, invoke an actuator, claim exactly-once execution, or infer non-application from a missing acknowledgment. Replay remains a separate authority-bearing transition.