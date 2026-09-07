# SSF Canonical Replay Policy v0.1

Consumes only freshly replay-qualified canonical outcome evidence and asks an independently expected replay-policy generation whether one exact same-effect replay may be authorized.

This crate reuses the existing replay-policy identity/policy/generation/time-basis descriptor so canonical replay does not create a competing policy root. The policy subject binds the exact evidence-qualification subject and receipt, qualification commitment, evidence lifetime, stable effect identity, replay basis, and a newer trusted policy-decision time.

Policy time may not regress behind evidence qualification. Policy receipts may not outlive the evidence qualification, decision-time evidence, or policy generation.

`Reject` and `Defer` are first-class outcomes. Only `AuthorizeExactSameEffect` creates `AuthorizedCanonicalReplayV1`, and that token authorizes at most one new attempt lineage while still containing no external-effect authority.

Fresh pre-invocation qualification and a durable replay journal remain mandatory downstream.