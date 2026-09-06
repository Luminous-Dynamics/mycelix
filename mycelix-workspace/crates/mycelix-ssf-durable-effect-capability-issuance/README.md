# SSF Durable Effect Capability Issuance v0.1

This crate records a crash-safe durable capability-issuance lineage for one exact source-owned operation material token. It does **not** mint the transient move-only capability and never invokes an actuator.

The store must enforce two independent uniqueness rules: first-seen issuance attempt ID permanently binds its exact manifest, and one exact durable effect-admission record may produce at most one capability-issuance record across all attempt IDs and rematerializations.

The journaled subject binds the exact rebound possible-effect lineage, capability-time revalidation receipt, source-owned provider material receipt, and material validity ceiling. Caller-selected effect bytes never enter this boundary.

After the external write begins, store errors, conflicts, pre-existing issuance, descriptor changes, malformed receipts, and invalid frontiers become `OutcomeUnknown`; none creates blind retry permission. Restart recovery queries the same exact journaled manifest read-only.

A successful durable issuance record is still historical authority evidence only. It contains no transient single-use effect capability and requires exact rebind plus a fresh final generation/time check before a later capability may be materialized.
