# SSF Canonical Replay Evidence Subject v0.1

Opaque, copyable, non-authoritative qualification subject produced only by consuming the structurally eligible first-attempt replay evidence candidate.

The subject freezes the exact initial invocation record, audited history head, head-read receipt, paired latest entry, history-read trusted-time receipt/time, prior attempt, exact effect subject, stable effect identity, structural replay basis, and inherited audit validity ceiling.

Its flattened freshness values explicitly carry the closed `UnixMillisecondsUtc` time basis inherited from the audited history, so later absolute-time comparisons never depend on an implicit unit convention.

Construction rechecks invocation, attempt, subject, and stable-effect bindings before flattening the evidence. There is no public field constructor, so downstream qualification cannot substitute a different history entry or effect after provenance gating.

The subject contains no replay authority, no effect authority, and no third-attempt permission. A future independent evidence qualifier may evaluate this immutable subject but cannot treat its copyability as authority.
