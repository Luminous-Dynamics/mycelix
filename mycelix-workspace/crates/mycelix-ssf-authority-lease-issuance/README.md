# SSF Durable Authority Lease Issuance v0.1

This crate adds the crash-safe durable boundary that converts one exact bounded authority-lease candidate into at most one durable **semantic lease authority** lineage.

It does not create an actuator capability. `DurablyIssuedAuthorityLeaseV1` still requires later use-time revalidation and effect admission, carries no direct state-install/effect capability, forbids delegation, and cannot be recovered after ambiguity by starting a different issuance attempt.

Store implementations must permanently bind first-seen attempt IDs to exact manifests **and** enforce that one exact single-use issuance reservation can produce at most one durable lease record. `OutcomeUnknown`, attempt-ID conflict, or an already-issued reservation never become retry permission.

The journaled manifest is plain crash-recoverable data. Restart reconciliation is read-only and can recover historical issuance facts without recreating the typed candidate lineage or semantic authority.
