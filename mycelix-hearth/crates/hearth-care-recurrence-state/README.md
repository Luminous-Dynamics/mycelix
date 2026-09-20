# hearth-care-recurrence-state

Pure append-only recurrence revision semantics for Hearth Care.

A durable recurrence revision stores only the household recurrence **definition** plus the stable Care schedule root and an optional parent recurrence-state reference. It does not embed its own state reference.

When the DHT layer stores a revision, the revision EntryHash becomes the exact recurrence-state reference. Readers bind that EntryHash plus the CareSchedule root back into an A6.1 `CareRecurrenceSpec` for expansion.

This avoids circular content addressing while preserving exact provenance.

The canonicalizer:

- collapses identical duplicate records with the same content-state reference;
- rejects semantic collisions for one state reference;
- requires one authoritative root;
- follows exactly one child from each revision;
- fails closed on competing children/forks;
- ignores disconnected orphan evidence while reporting it;
- returns the effective head as an exact A6.1 bound spec.

No DHT APIs, timezone engine, or recurrence expansion logic live in this crate.
