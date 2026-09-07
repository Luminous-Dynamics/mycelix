# SSF Historical Actuator Invocation Journal v0.1

Provides read-only historical reconciliation for already-journaled initial and replay actuator attempts after the journal-store policy/key generation rotates.

The continuity anchor is the original journal store's stable identity. A newer reconciler generation may freshly attest the exact historical `Journaled` receipt only for that same stable identity; cross-identity substitution is rejected.

Fresh historical evidence does not refresh the original effect authority. Invocation eligibility remains bounded by the original attempt lifetime, exact actuator generation, and replay-authorization lifetime where applicable, intersected with the fresh reconciliation/time evidence.

This crate performs no journal write, no replay authorization, no actuator reservation, and no external effect.