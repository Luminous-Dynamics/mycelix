# SSF Effect Capability Claim Rebind v0.1

Converges direct durable capability claims and historical restart reconciliation onto one exact durable claimed-effect lineage.

- Direct path consumes the live claimed capability; it does not preserve a reusable transient capability object.
- Historical path requires exact equality between the archived capability audit binding and the durable claim manifest.
- Matching claim IDs, capability IDs, or endpoints alone are insufficient.
- The result is still not an actuator capability and cannot invoke the provider or actuator.
- Fresh pre-invocation execution/provider/time revalidation remains mandatory.
