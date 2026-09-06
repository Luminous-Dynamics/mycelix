# SSF Historical Effect Admission Reconciliation v0.1

This crate reconciles one old exact durable effect-admission attempt under a currently trusted **same stable admission-store identity** after store policy/key/verifier generations rotate.

It is read-only. It cannot create a new durable effect admission, recreate an execution-surface admission candidate, mint an execution capability, or call an actuator.

A fresh historical reconciliation may refresh confidence about whether the old admission occurred, but it cannot refresh the original execution-admission authority lifetime. Stable store identity is the continuity anchor; policy and verifier generations may rotate, while a different store identity is a trust discontinuity.

`Admitted`, `ProvenNotAdmitted`, and `OutcomeUnknown` remain distinct. Attempt conflicts, pre-existing admission ambiguity, malformed receipts, identity substitution, descriptor changes during reconciliation, and invalid durable frontiers all remain `OutcomeUnknown`.
