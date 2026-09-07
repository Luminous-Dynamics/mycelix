# SSF Actuator Invocation Activation v0.1

Adds a fresh activation boundary after durable invocation journaling and immediately before any actuator reservation or invocation work.

Durable journaling proves that an invocation lineage exists; it does not prove that the lineage is still live when effect work begins. This crate consumes either the exact initial journal typestate or the exact replay-journal typestate, rechecks its durable binding, and requires fresh uncertainty-aware trusted time.

Activation fails closed if the attempt, actuator generation, journal evidence, replay authorization ceiling, journal time basis, or current-time evidence is stale or inconsistent.

`ActivatedActuatorInvocationV1` performs no actuator reservation or external effect. It only proves that one exact durable invocation lineage is live now and may proceed to the later actuator protocol.