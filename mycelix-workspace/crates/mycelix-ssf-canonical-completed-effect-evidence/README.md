# SSF Canonical Completed Effect Evidence v0.1

Unified non-authoritative evidence envelope for both the existing canonical actuator execution path and the canonical replay execution path.

The envelope is created only by consuming one of the exact opaque post-effect outcome types through a sealed source trait. It preserves the raw actuator receipt, low-level interpretation, canonical qualified disposition, trusted pre/post time, and exact durable invocation provenance without reconstructing execution or replay authority.

Invocation provenance distinguishes the existing initial/legacy-replay record domain from the new canonical-replay record domain, so a future unified history can retain which execution constitution produced each effect observation.

The verifier additionally checks post-time self-freshness and rejects spurious clock-rollback ambiguity. Raw evidence and canonical belief remain separate and immutable.

The resulting evidence is copyable for audit, creates no replay authority, and cannot authorize a third attempt.
