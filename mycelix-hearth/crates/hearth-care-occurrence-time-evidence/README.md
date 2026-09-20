# Hearth Care Occurrence Time Evidence

Pure append-only theorem for binding one already-admitted Care occurrence to its exact recurrence/time authority.

The contract deliberately keeps assignment authority separate. Existing occurrence-to-assignment-state evidence remains authoritative for responsibility; this crate preserves the independent recurrence/time chain:

- occurrence content reference;
- schedule root;
- exact recurrence-state reference;
- recurrence instance key;
- timezone + engine/tzdb identity;
- original/requested/effective local starts;
- exact UTC window;
- DST resolution and override disposition.

Identical evidence duplicates collapse deterministically. Different recurrence/time evidence for the same occurrence fails closed even if the resulting UTC window is identical. Same value is not same authority.
