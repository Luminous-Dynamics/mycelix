# SSF Use-Time Lease Revalidation v0.1

This crate proves that one exact durably issued semantic lease is still eligible for a later execution-surface admission decision **at use time**.

It does not reuse the pre-issuance time-basis attestation as if issuance had not happened. The exact `IssuedAuthorityLeaseBindingV1` must participate in a fresh Unix-millisecond time-basis receipt together with a freshly qualified current local context.

All six local prerequisites—membership, identity key, revocation snapshot, active policy, machine health, and scientific qualification—must again be bounded by known freshness. `Unknown` and `NotApplicable` fail closed.

The fresh local state must still equal the exact policy-approved source state. Any changed local state requires a new qualification lineage in v0.1.

The final use-time validity ceiling is the minimum of the durable lease lifetime, fresh local-context lifetime, six prerequisite lifetimes, fresh time-basis attestation, and trusted uncertainty-aware current-time receipt.

The resulting token remains non-executable. It is only eligible for a later exact execution-surface admission boundary and contains no direct state-install or effect capability.
