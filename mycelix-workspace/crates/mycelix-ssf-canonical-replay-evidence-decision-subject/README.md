# SSF Canonical Replay Evidence Decision Subject v0.1

Purpose-scoped, non-authoritative semantic subject for future replay-evidence qualification.

This layer consumes the exact fresh qualification request and rechecks its nested evidence subject, qualification-time receipt, latest plausible qualification time, explicit Unix-millisecond time basis, natural-expiry ceiling, and zero-authority boundary.

Before freezing the decision purpose it independently re-proves both the latest archived observation's historical self-consistency and the replay basis. It requires exact initial invocation provenance, durable head-to-latest-record binding, latest evidence/provenance identity, predecessor continuity and non-terminality, manifest validity at its original recording time, store-lifetime bounds, recording chronology after pre/post-invocation evidence and before the later history read, exact +1 history generation, prior attempt/effect/stable-identity equality, and canonical outcome/recovery-mode consistency.

`ProvenNotApplied` may only map to the corresponding replay-evidence basis. `IdempotentOutcomeUnknown` requires the exact idempotent actuator recovery mode and matching canonical recovery policy. Confirmed, transactional-unknown, and non-idempotent-unknown outcomes are structural rejection states.

It then freezes one closed decision purpose: `EvidenceFitnessForAtMostOneReplay`.

This crate deliberately does **not** compute a cryptographic digest or define a byte serialization. A future crypto/verifier adapter must canonically encode and cryptographically bind this exact semantic decision subject, including its purpose domain. That prevents the pure contract layer from inventing ad-hoc hashing while giving future signatures/receipts one unambiguous object to bind.

The decision subject contains no replay authority, no effect authority, and no third-attempt permission.
