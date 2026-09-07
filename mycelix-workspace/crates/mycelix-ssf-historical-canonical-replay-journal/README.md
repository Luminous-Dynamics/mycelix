# SSF Historical Canonical Replay Journal v0.1

Adds read-only long-duration reconciliation for an already journaled canonical replay attempt.

The existing historical invocation-journal reconciler identity/policy/generation vocabulary is reused. A newer reconciler generation may attest an old canonical replay journal only when the stable replay-journal store identity is unchanged.

The old canonical journal receipt may be expired today, but it must have been valid at the original replay-attempt time. The old replay authorization must likewise have covered that attempt. A newer reconciler cannot launder evidence that was already stale when the journal event occurred.

Fresh historical knowledge never refreshes old effect authority. The resulting eligibility ceiling remains bounded by the original canonical replay authorization, replay-attempt lifetime, actuator generation, fresh reconciliation receipt, and trusted current-time receipt.

This crate performs no journal write, reconstructs no replay authorization or prepared attempt, performs no actuator invocation, and creates no third-attempt authority.
