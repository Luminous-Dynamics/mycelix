# SSF Canonical Completed Outcome History Audit v0.1

Defensive, non-authoritative self-audit for qualified canonical completed-outcome history heads.

A current qualified store read proves what the store returned. This crate additionally proves that the exact paired latest entry was internally valid historical evidence when recorded: exact record/head pairing, schema and store generation, invocation/provenance continuity, predecessor/generation continuity, manifest lifetime at recording, store lifetime bounds, and recording chronology relative to pre/post invocation evidence.

Historical evidence may be expired today. The audit does not refresh it or create replay authority; it only rejects records that were malformed or temporally invalid when created.

The resulting opaque audit token contains no replay or effect authority. Replay provenance policy remains a separate downstream boundary.
