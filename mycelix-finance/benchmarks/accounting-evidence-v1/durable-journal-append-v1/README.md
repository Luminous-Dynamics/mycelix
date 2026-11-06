# FIN-ECO-006 durable append conformance scenarios v0.1.0

This is a public synthetic **contract fixture set**, not a durable storage implementation, database benchmark or backend qualification. It freezes 14 scenarios covering exact append/replay, source conflicts, journal identity conflicts, pre-commit failure, post-commit response loss, ambiguous commits, partial state, concurrency, reversal caps, restart reconstruction, historical retry and stored-byte tampering.

Run any future adapter against these scenarios only after binding its exact storage profile, code/build, canonical journal input bytes and independent verifier. The concurrent cases are outcome-multiset tests: the winning process may vary, but the final durable state and allowed outcome multiset may not.

The request fields use symbolic synthetic digests/IDs; they are not real journal bytes. Atomic amounts are decimal strings to avoid loss through JSON floating-point representations.

The public scenarios and this schema are not blind candidate evaluation. Evaluator-only model answers/labels must be kept separately for any future benchmark. A v0.1.0 pass can establish only conformance to these synthetic expected behaviors, not production durability or full accounting correctness.
