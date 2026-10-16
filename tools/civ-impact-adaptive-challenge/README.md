# CIV-IMPACT Adaptive Challenge Verifier

**Status:** first semantic-verifier prototype; qualification requires exact-head checks.  
**Tracking:** [CIV-IMPACT-003 #5065](https://github.com/Luminous-Dynamics/mycelix/issues/5065)  
**Protocol:** [CIV_IMPACT_ADAPTIVE_CHALLENGE_V0_1.md](../../docs/research/CIV_IMPACT_ADAPTIVE_CHALLENGE_V0_1.md)

## Commands

```sh
cargo test --manifest-path tools/civ-impact-adaptive-challenge/Cargo.toml
cargo run --manifest-path tools/civ-impact-adaptive-challenge/Cargo.toml -- validate-prereg path/to/preregistration.json
cargo run --manifest-path tools/civ-impact-adaptive-challenge/Cargo.toml -- verify-ledger path/to/preregistration.json path/to/exposure-ledger.json
```

The CLI loads both frozen Draft 2020-12 schemas, hashes the exact preregistration file bytes, verifies that the ledger points to those bytes and the same sealed-bank commitment, then checks candidate/comparator/evaluator/scorer identities, the cumulative evaluation budget, regime coverage quotas, random-audit minimum, event order, prior-event digest chaining, selection/execution/freeze/disclosure/rotation consistency, and mutation invalidation.

## What is implemented

- Strict schema validation for preregistration receipts and evaluator-only exposure ledgers.
- Event hash verification under the documented v0.1 serialization convention: parsed JSON, compact serde_json serialization with lexicographic object keys, arrays retained in order, and the event_sha256 property excluded.
- Append-only sequence / previous-event checks.
- Selection audit: selected items must be eligible; listed selected commitments must exactly match selected selector decisions; selected items require selection probability; duplicate selections are rejected.
- Lifecycle: execution only after selection; result freeze only after execution; disclosure only after a prior freeze; rotation only after disclosure.
- Any material post-freeze identity change requires a matching mutation/invalidation event and blocks a confirmatory claim.
- Confirmatory completion requires every selected challenge to have been executed, frozen, disclosed, and rotated, with a preregistered design-based estimator, a non-descriptive uncertainty method, and positive inclusion probabilities for every eligible challenge. Descriptive completion is separately supported but cannot carry a confirmatory claim.
- The ledger is checked against the actual preregistration digest, artifact identities, sealed-bank commitment, cost budget, minimum regime quotas, and random-audit fraction.
- Twenty-four authored tests cover schema contracts, preregistration digest binding, budget and quota checks, random-audit floor, tampered hashes, broken chain links, pre-freeze disclosure, unselected execution, duplicate selector rows, post-freeze mutation, public-item confirmatory credit and peeking without sequential control.

## Deliberate limitations

- This is a verifier, not a sealed-bank service, key manager, random/adaptive selector, challenge runner, or independent process sandbox.
- The schemas bind a commitment but do not prove that the underlying secret bank was honestly generated or remained secret. That requires controlled storage/access, an audited commitment procedure, and independent execution.
- The verifier checks the *declared* random-audit count and selection probabilities but does not prove that the selection RNG was independent of candidate outputs or statistically sound. A sealed RNG/seed-commitment execution receipt and independent seed custody are still needed.
- The v0.1 hash convention is local to this Rust/serde_json profile. Cross-language verifiers must implement the exact same serialization or a future protocol version must choose a language-neutral canonical JSON standard.
- Hash chaining alone is not authentication: a writer that can rewrite the entire ledger can recompute the chain. Confirmatory use requires an independently retained signed/transparency-log checkpoint; signature/anchor generation and verification are not implemented here.
- This prototype does not establish model quality, broad generalization, causal impact or economic benefit.
- The public regression corpus must receive zero confirmatory credit. Sealed cases must rotate after disclosure under a new bank/protocol version.
- Direct dependency versions are pinned, but a committed Cargo.lock and reproducible build/binary identity remain subsequent work.

## Evidence ladder

The desired progression is contract validation → public regression → frozen sealed challenge + random audit → held-out scenario families → allowed source replay → no-side-effect shadow evaluation. A successful schema or unit test advances only the contract layer; it is not a benchmark win.
