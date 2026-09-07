# SSF Final Canonical Replay Readiness v0.1

Converges ordinary/same-generation canonical replay activation and historical same-identity activation into one final non-effecting pre-actuator typestate.

The final boundary rechecks the exact replay-attempt schema, anti-clock-rollback relationship, activation lifetime, attempt lifetime, and actuator-generation lifetime. It also rejects any upstream activation that claims permission for a third attempt.

A copyable audit binding records whether the final replay path came from current/same-generation recovery or historical same-identity recovery, while the non-copyable activation authority remains consumed inside the final token.

`FinalCanonicalReplayInvocationReadyV1` performs no actuator reservation or external invocation and structurally permits no third attempt. It is intended to be the only canonical replay input accepted by a later replay execution adapter.
