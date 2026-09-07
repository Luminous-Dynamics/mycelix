# SSF Canonical Replay Invocation Journal v0.1

Consumes one `AuthorizedCanonicalReplayV1` plus a newly prepared, freshly qualified actuator attempt and durably records at most one canonical replay lineage.

The journal reuses the established replay-journal store identity/frontier/record vocabulary, but canonical replay uses its own manifest and receipt commitment domains. Canonical and legacy replay authorizations are not interchangeable.

Three durable uniqueness keys are required: first-seen new attempt ID permanently binds its exact manifest, one canonical replay authorization may produce at most one replay record, and one prior attempt may produce at most one replay record in v0.1.

The new attempt must use a different attempt ID, preserve the exact stable effect identity, use a non-regressing fresh attempt time relative to the replay-policy decision, and remain within the canonical authorization, actuator-generation, prepared-attempt, and journal-store lifetimes.

`ProvenNotJournaled` is final for the old attempt ID. It returns the replay authorization but deliberately discards that prepared attempt, requiring a new attempt-preparation step before another journal write.

Restart recovery is evidence-only and cannot reconstruct replay authority or a prepared actuator attempt from the receipt alone.

A successfully journaled canonical replay still performs no actuator invocation and structurally permits no third attempt.