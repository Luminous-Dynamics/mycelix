# SSF Historical Authority Lease Reconciliation v0.1

This crate permits read-only reconciliation of one exact old durable authority-lease issuance attempt after lease-store verifier generation or policy rotation, provided the stable store identity remains unchanged.

It does not issue a lease, migrate store identity, recreate a typed bounded candidate, or grant semantic/effect authority. A fresh historical attestation may refresh knowledge of what happened, but `authority_eligibility_valid_until` remains capped by the original requested lease lifetime.

A later exact issued-lineage rebind must converge an `Issued` historical fact with the independently rehydrated bounded lease candidate before any semantic lease lineage can be recovered after restart.
