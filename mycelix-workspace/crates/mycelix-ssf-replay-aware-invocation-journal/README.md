# SSF Replay-Aware Invocation Journal v0.1

Durably journals one explicitly authorized **second** actuator invocation attempt.

The initial-attempt journal remains strict: one durable claim record produces at most one initial attempt. This crate adds a separate replay lane that can exist only by consuming `AuthorizedReplayV1` plus a newly prepared, freshly qualified actuator attempt.

The replay attempt must preserve the exact stable effect identity while carrying fresh attempt-specific qualification evidence and a new attempt ID. The store must enforce one replay authorization and one prior attempt -> at most one replay journal record.

v0.1 permits only one second attempt. It does not authorize a third attempt, invoke an actuator, infer idempotency, or treat replay policy as effect authority.