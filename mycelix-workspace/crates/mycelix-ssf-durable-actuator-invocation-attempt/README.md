# SSF Durable Actuator Invocation Attempt v0.1

Durably journals one exact actuator invocation attempt **before** any external actuator call.

v0.1 deliberately enforces at most one journaled invocation attempt per durable SSF claim record, regardless of the actuator's eventual replay mode. This avoids crash ambiguity while the system does not yet possess durable outcome/replay authorization.

The journal store must enforce:

- first-seen invocation-attempt ID permanently binds the exact manifest;
- one durable claim record may produce at most one journaled invocation attempt in v0.1;
- `ProvenNotJournaled` is final for that attempt ID.

A journal write is not an external effect. If journaling is ambiguous, actuator invocation is blocked. Same-attempt reconciliation is allowed; creating a fresh attempt is not.
