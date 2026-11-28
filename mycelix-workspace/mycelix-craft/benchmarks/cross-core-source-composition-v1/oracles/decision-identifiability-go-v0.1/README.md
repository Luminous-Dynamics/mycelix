# Craft Decision Identifiability Envelope V0.1

Dependency-free Go reference oracle for one narrow question: **does the same action remain safe and optimal across every world in the declared evidence-compatible set?**

## Run

```sh
go test -count=1 ./...
go test -race -count=1 ./...
go vet ./...
```

## Semantics

- No probability is assigned to a possible world.
- An action is `robust_safe` only when it has no hard-safety violation in any listed world.
- The action optimal in every listed world is the intersection of the per-world safe minimum-loss action sets. The output field is deliberately named `optimal_in_every_listed_world` to avoid implying unqualified global coverage.
- For robust-safe actions, the oracle reports minimum/maximum loss and exact worst-case loss/regret.
- `minimax_regret_actions` and `minimax_loss_actions` are decision-analysis reference sets, not unique authorizations.
- A mandatory fallback must be safe in every listed world.
- Input order does not affect output ordering.
- Duplicate JSON keys, unknown fields, malformed loss maps, missing worlds and trailing bytes are rejected.

## Critical proof boundary

The evaluator only reasons **inside the world set it receives**. `fixture_exhaustive` is accepted only for the three allowlisted frozen fixtures whose exact canonical Go-JSON SHA-256 digests are pinned in the oracle. Changing any decision-relevant fixture field invalidates the digest. This is a bounded test-fixture attestation, not a production proof that the real world set is complete. `declared_set` yields a conditional result. `not_evaluated` always produces `WORLD_SET_COVERAGE_NOT_EVALUATED`, even when one action is optimal in every listed world. A producer cannot promote an unqualified world set into a global decision claim merely by supplying a finite list.

The evaluator does not authenticate sources, prove source-universe completeness, validate the loss function, establish legal authority, verify real-world independence or observe physical effects. It is an advisory test oracle only.

## Frozen cases

- `CDIE-001`: repair is optimal in every exhaustive fixture world.
- `CDIE-002`: repair and sample are optimal in different worlds; sample minimizes worst-case regret (3) without becoming an invariant optimum.
- `CDIE-003`: release has lower nominal loss but is vetoed by a hard-safety violation in one compatible world; hold remains the only robust-safe action.
- `CDIE-004`: repair is optimal inside the listed worlds, but global coverage is NotEvaluated, so no global decision-identification claim is emitted.

The result includes a deterministic decision-flip witness when two listed worlds have disjoint optimal-action sets, including their competing optima and differing declared dimensions. It is a witness, not proof that a particular observation is admissible or sufficient. The test suite additionally compares the implementation against a separately written exhaustive reference over 324 tiny safety/loss combinations and checks input permutation and world-set expansion invariants.
