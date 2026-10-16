# Pulse delivery model-checking starter

This directory contains a deliberately small TLA+ state model for one logical message and one recipient.

## Run

With the TLA+ tools installed, from this directory:

\`\`\`sh
java -cp tla2tools.jar tlc2.TLC -config PulseDelivery.cfg PulseDelivery.tla
\`\`\`

The command assumes \`tla2tools.jar\` is available locally; it is not vendored here.

## What this model currently checks

- State typing over a finite worker-epoch bound.
- A delivered state requires receipt evidence and a committed receiver inbox.
- Receipt evidence cannot exist before inbox commitment.
- Send authorization checks revocation, expiry, and current worker/lease epoch.

## Important limitations

This is a bounded design sketch, not a model-checking result and not a proof of the Rust implementation. No TLC execution result is claimed in this change. The model intentionally abstracts away real cryptography, durable storage semantics, retries, multiple messages/recipients, receipt contents, and actual concurrent storage transactions. The inbox is represented by a Boolean, so its at-most-once property is structural rather than a stress test of a collection-backed deduplication algorithm.

Next steps are to expand the model alongside issue #5091: add nondeterministic crash points around durable boundaries, concurrent duplicate handlers, persistent idempotency keys, stale worker resumption, policy-generation races, and mutant checks. Each model counterexample should become a deterministic Rust regression test. Keep assumptions and boundedness explicit; a successful bounded check would not establish an unbounded theorem or real-world delivery guarantee.
