# SSF Historical Effect Capability Issuance v0.1

This crate reconciles one old exact durable effect-capability issuance attempt under a currently trusted reconciler for the **same stable issuance-store identity** after store policy/key/verifier generations rotate.

Historical reconciliation is read-only. It may refresh knowledge that the old attempt was `Issued` or `ProvenNotIssued`, but it cannot issue a new record, recreate the typed source-owned operation material, mint a transient capability, or invoke an actuator.

The original write-time manifest remains immutable. A newer reconciler may use a different policy and generation only when its stable store identity matches the original store identity exactly. A different store identity is a trust discontinuity.

`reconciliation_valid_until` is freshness of the new historical fact. `capability_eligibility_valid_until` remains capped by the original material lifetime, so fresh historical evidence can never resurrect stale effect power.
