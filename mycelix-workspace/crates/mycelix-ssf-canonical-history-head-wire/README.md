# SSF Canonical History Head Wire v0.1

Complete canonical byte encoding of `CanonicalCompletedOutcomeHistoryHeadV1`.

The v0.1 profile writes, in fixed order:

- composite encoding version;
- completed invocation provenance + exact durable invocation-record commitment;
- history generation as u64 big-endian;
- explicit `None` / `Some` tag for the durable head record, followed by the exact 32-byte record commitment when present;
- explicit `None` / `Some` tag for terminal state, followed by the explicit terminal-state tag when present.

This makes genesis, active non-terminal, confirmed-terminal, and proven-not-applied-terminal heads distinct without relying on Rust `Option` or enum layout.

## Claim boundary

This crate completely covers the history-head composite only. It does **not** encode the latest canonical observation manifest or its embedded completed-effect evidence, and therefore does not complete `CanonicalReplayEvidenceSubjectV1` or the #359 decision subject.

No hashing, signing, verification, evidence qualification, replay authorization, or effect authorization is implemented here.
