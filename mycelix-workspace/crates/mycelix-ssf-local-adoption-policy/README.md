# SSF Local Adoption Policy v0.1

This crate is the local policy boundary after SSF remote-path authorization coverage and local adoption readiness.

The evaluator receives an immutable subject that retains the exact common ancestor and qualifier generation, exact current local state and qualifier generation, prerequisite evidence/freshness, exact remote target, and every remote authorized history step with its verifier descriptor and receipt commitment.

Policy evaluation has three closed-world outcomes: `Approve`, `Reject`, and `Defer`. Reject/Defer are valid policy outcomes, not verifier errors, and are structurally restricted to an observation-only follow-on consequence ceiling.

An approval carries an exact adoption-scope commitment plus a maximum follow-on consequence. Any later authority-lease layer must attenuate both; it may never derive broader scope or greater consequence from the approval.

Even an `ApprovedLocalAdoptionV1` contains neither state-install authority nor effect authority. It is evidence that the exact candidate is locally policy-qualified; later authority must still be separately bounded, durably registered, freshness-checked, and reconciled with outcome evidence.
