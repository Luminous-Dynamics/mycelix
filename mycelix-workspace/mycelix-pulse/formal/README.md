# Pulse delivery model-checking starter

This directory is a deliberately small, bounded TLA+ model and qualification plan for one logical message and one recipient. It is a design aid—not a checker result and not a proof of the Rust implementation.

## Run

With the TLA+ tools installed, from this directory:

~~~sh
java -cp tla2tools.jar tlc2.TLC -config PulseDelivery.cfg PulseDelivery.tla
~~~

The command assumes tla2tools.jar is available locally; it is not vendored here.

## Model represented

- Finite worker epochs and explicit lease reacquisition after restart.
- A crash toggles worker availability and records that an interrupted attempt requires recovery.
- An interrupted attempt is classified as unknown during recovery rather than inferred to be failed or delivered.
- unknown can be resolved to delivered only with abstract receipt evidence, receiver inbox commitment, and a current worker lease.
- A verified receipt must be associated with a committed inbox in this abstraction.
- New send authorization checks the worker/lease epoch, revocation, and expiry.

## Machine-readable qualification artifacts

- invariants.json defines 10 stable safety/liveness property IDs, current starter-model coverage, known gaps, and proposed negative controls. Its coverage labels are plans, not proof results.
- benchmark-plan.md separates hard correctness gates from provisional performance goals and specifies fault-boundary, scale, and provenance requirements.

## Important limitations

No TLC syntax/configuration validation or model execution is claimed here. No Rust compile/test or CI pass is claimed. The model intentionally abstracts away cryptographic receipt contents, exact message/recipient/commitment binding, durable storage and torn writes, multiple messages/recipients, concurrent duplicate handlers, transport failure classes beyond interrupted attempts, retry scheduling, event-journal replay, and real transaction semantics. The receiver inbox and receipt evidence are Booleans, so the model does not implement collection-backed deduplication or cryptographic verification.

A successful bounded TLC run would show only that the encoded properties hold in the specified finite model. It would not establish an unbounded theorem, implementation refinement, storage durability, production liveness, or real-world delivery reliability.

Next steps are tracked in issue #5091: validate syntax; review assumptions and reachable states; add explicit identity/commitment maps, receiver concurrency, durable write/recovery boundaries, policy-generation races, and bounded history; then require a deliberately broken mutant for each critical invariant and a deterministic Rust regression for every counterexample. Bind any eventual report to the exact commit SHA, model/config/tool digests, bounds, state count, and raw counterexample traces. Keep queued, skipped, and missing evidence distinct from passed.
