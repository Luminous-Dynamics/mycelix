# HTH-AUTO-003 design note

`hearth-edge` is intentionally a local execution library in this tranche, not a daemon and not a device controller.

The qualification target is the execution theorem:

- validate the semantic plan before dispatch;
- refresh authority before every step and retry;
- fail closed on unknown preconditions;
- choose adapters locally without embedding adapter syntax in durable intent;
- reuse one idempotency key for all retries of a logical step;
- keep command acceptance distinct from outcome verification;
- never return `Completed` without a verified final outcome;
- allow cancellation/manual override between steps and retries;
- make compensation explicit and separately traceable.

The synthetic adapter exists to prove these properties before Matter, Home Assistant, OCPP, OpenADR, MQTT, or vendor adapters are admitted.