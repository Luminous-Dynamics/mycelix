# SSF Historical Canonical Replay Activation v0.1

Converts one historically reconciled canonical replay journal into a fresh, still non-effecting activation token.

Historical reconciliation proves that the old exact journal event happened under a newer same-identity reconciler. Activation separately proves that the old replay attempt remains eligible now.

Fresh activation time may not regress behind either the original replay-attempt time or the historical reconciliation time. The final lifetime is bounded by the historical eligibility ceiling, original replay authorization, replay attempt, actuator generation, reconciliation receipt, and fresh trusted-time receipt.

The result performs no actuator reservation or external invocation and cannot authorize a third attempt. Historical truth may be refreshed; effect authority may only shrink.
