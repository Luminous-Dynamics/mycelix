# Verifier transparency log migration

Protocol `mycelix.proof.verifier-transparency-migration.*.v1` contains a compromised transparency log without discarding its witnessed history.

A migration is acceptable only when all of these facts are bound together:

- an authenticated compromise notice identifies the final trusted source checkpoint;
- a hash-linked source-history commitment is complete and monotonic;
- an independent witness quorum confirms the source endpoint;
- a separate witness quorum confirms the replacement log and its history anchor;
- a distinct recovery-authority quorum signs the exact migration plan;
- an externally pinned migration checkpoint makes rollback or silent log substitution observable.

The destination log does not inherit trust merely because it is reachable. Its first trusted checkpoint commits to the complete source-history hash. Truncated histories, duplicate recovery signers, same-log “migrations,” expired plans, and checkpoint rollback fail closed.

This is a source contract. It does not claim deployed log transport, production witness keys, production recovery authorities, or an installed migration checkpoint.
