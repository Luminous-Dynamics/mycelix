# Mycelix constitutional formal verification

Status: experimental, non-activating formal artifacts.

These models provide complementary bounded views of the constitutional authority program. They do not themselves activate runtime enforcement and they are not evidence-bearing until the exact files have been checked by pinned tools under an exact recorded lineage.

## Evidence classes

Keep these claims distinct:

1. **Formal artifact present** — a `.tla`, `.cfg`, or `.als` file exists and is reviewable.
2. **Parser-valid** — the pinned formal tool parsed the exact file successfully.
3. **Bounded model-check PASS** — the pinned checker searched the recorded finite state/scope and found the declared invariant/expectation outcomes.
4. **Executable reference conformance** — Rust tests demonstrate selected concrete semantics corresponding to the formal abstraction.
5. **Runtime refinement evidence** — production Holochain/runtime behavior has an explicit, qualified mapping to the checked formal model.

Never collapse a lower evidence class into a higher one.

## Division of responsibility

### TLA+

Use TLA+ for temporal/concurrent questions:

- competing consumption claims;
- finality before side effect;
- revocation ordering;
- late contradictory evidence;
- integrity-fault halting;
- dependency indeterminacy;
- idempotent redelivery;
- explicit safety vs liveness assumptions.

`ConstitutionalConsumptionV2.tla` uses a monotonic logical event clock. A later observation may carry an authenticated earlier revocation effective sequence, which models delayed evidence without allowing ordinary transitions to choose arbitrary earlier event times.

The two checked-in configurations are separate constitutional profiles:

- `ConstitutionalConsumptionV2.finality.cfg` — finality is the commit point;
- `ConstitutionalConsumptionV2.effect.cfg` — authority must remain valid until the effect.

Both profiles include `RevocationCutoffConsistentWhenFaultFree`, an independent theorem rather than merely relying on the transition guard that implements the cutoff policy. When no integrity fault is active, every accepted commit point must remain consistent with all revocation evidence currently known. Late contradictory evidence is permitted only by moving the model into the explicit fault state.

### Alloy

Use Alloy for bounded structural questions:

- unique sovereign-power ownership;
- constituent sovereignty distinct from branches;
- automated-agent exclusion;
- static and same-matter separation of duty;
- concurrence-holder diversity;
- delegation attenuation;
- nondelegable powers;
- canonical root usage budgets;
- subset/disjoint sub-allocation;
- duplicate-finalization exclusion.

Alloy commands declare regression intent with `expect 1` for expected-SAT witnesses/negative controls and `expect 0` for expected-no-counterexample checks. Qualification must independently classify the actual SAT/UNSAT result; do not assume CLI exit status alone enforces `expect`.

Alloy results are bounded. “No counterexample” means no counterexample was found in the exact recorded finite scope, not an unbounded proof.

### Rust

Rust remains the executable reference semantics for authority, envelopes, consumption, and later runtime adapters. Formal-model evidence does not automatically prove Rust behavior.

### Runtime

Production Holochain or external-finality behavior requires explicit refinement evidence tying runtime types/transitions to the formal abstractions. A green TLA+/Alloy run is not permission to claim the distributed runtime has been proven equivalent.

## Negative controls

Qualification must demonstrate that the tools find intentionally weakened models/structures. Examples include:

- TLA+ double-finalization when the competing-finalization guard is removed;
- TLA+ effect-before-finality when the finality precondition is removed;
- TLA+ budget overrun when the budget guard is removed under a one-use test config;
- TLA+ post-fault state change when the fault guard is removed;
- TLA+ effect-cutoff violation when known revocation is ignored;
- Alloy fake concurrence from multiple keys owned by one holder;
- Alloy duplicate local allowance from ledgers sharing only a budget identifier;
- generated weakened Alloy fixtures that remove sibling-disjointness or scope attenuation.

Negative controls must use isolated temporary/generated fixtures, never mutate the canonical file under qualification.

## Tool pins

Initial target tools:

- TLA+/TLC 1.7.4 (`tla2tools.jar`), pinned by SHA-256 before evidence-bearing use;
- Alloy 6.2.0 (`org.alloytools.alloy.dist.jar`), Java 17+, pinned by SHA-256;
- Sat4j as the initial explicit Alloy solver baseline.

Every receipt should record tool artifact hash/version, Java runtime, command line, worker/solver settings, exact model/config hashes, exact scope/config, output/log digest, Git head, and postflight immutability.

## Refinement/drift

See MYC-CONST-003CR. Rust, TLA+, Alloy and later Holochain enforcement are multiple verification views of one constitutional meaning, not independent constitutions. New Rust authority variants or formal symbols require an explicit mapping or an explicit `out_of_model` decision before formal evidence is used to justify production behavior.
