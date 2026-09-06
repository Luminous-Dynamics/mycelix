# SSF Adoption Authority Registration v0.1

This crate defines durable registration and restart-safe reconciliation for non-executable SSF adoption-authority candidates.

The core rule is that an external registration attempt has exactly one durable identity: its `RegistrationAttemptId` plus the full `RegistrationAttemptManifestV1`. An ambiguous result may only be reconciled by querying that exact attempt. It may never be retried as a fresh write.

`ProvenNotPersisted` is a final state, not a transient lookup miss: by store contract it permanently closes that exact attempt ID against later commit and carries durable absence evidence. `OutcomeUnknown` and attempt-ID conflict remain frozen.

Even `Persisted` registration contains no state-install or effect authority. Authority issuance, use-time freshness, revocation/health revalidation, execution, and effect-outcome reconciliation remain separate later boundaries.
