# SSF Effect Capability Issuance Rebind v0.1

This crate converges direct durable effect-capability issuance and historical restart reconciliation onto the exact typed source-owned operation material lineage that produced the durable record.

Historical `Issued` evidence alone cannot recreate the typed operation material. Independently rehydrated material alone cannot claim durable capability issuance. Only exact equality with the original journaled issuance subject produces `ReboundIssuedEffectCapabilityLineageV1`.

The rebound token retains whether evidence was direct or historical, the exact capability record/frontier, verification descriptor, receipt commitment, evidence lifetime, and inherited capability-eligibility ceiling.

Rebind is still not a transient capability. Both direct and historical paths require a fresh final execution/provider/time proof before any move-only effect capability may be constructed.
