# SSF Canonical Replay Invocation Activation v0.1

Adds a fresh post-journal activation boundary for the canonical replay lane.

The crate accepts only the exact live `DurablyJournaledCanonicalReplayAttemptV1` or exact same-generation read-only recovered journal evidence. Its sealed input trait cannot be implemented by external crates.

Activation rechecks the exact durable `Journaled` receipt/frontier, stable-effect identity, prior canonical invocation record, canonical replay authorization, journal store time basis, and all relevant validity ceilings.

Fresh trusted time must not regress behind either the newly prepared replay attempt time or the canonical replay-policy decision time. The replay authorization ceiling and journal-evidence ceiling remain separately auditable, while the activated lineage lifetime is their minimum with trusted time, attempt lifetime, and actuator-generation lifetime.

This crate performs no actuator reservation or external invocation and never permits a third attempt. Historical same-identity journal-store key rotation remains a separate future reconciliation boundary rather than an implicit relaxation of same-generation recovery.
