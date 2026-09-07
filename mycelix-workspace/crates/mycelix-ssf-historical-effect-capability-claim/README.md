# SSF Historical Effect Capability Claim v0.1

This crate reconciles one old exact durable effect-capability claim under a currently trusted reconciler for the **same stable claim-store identity** after policy/key/verifier generations rotate.

Historical reconciliation is read-only. It may establish that the old attempt was `Claimed` or finally `ProvenNotClaimed`; it never creates a new claim, reconstructs the transient capability, resolves the provider handle, or invokes an actuator.

The original journaled claim manifest remains immutable. A newer reconciler may use a different policy/generation only when its stable claim-store identity matches the original exactly. A different identity is a trust discontinuity.

`reconciliation_valid_until` is freshness of the new historical fact. `claim_eligibility_valid_until` remains capped by the original transient capability and original claim-time evidence, so historical reconciliation cannot resurrect stale effect authority.
