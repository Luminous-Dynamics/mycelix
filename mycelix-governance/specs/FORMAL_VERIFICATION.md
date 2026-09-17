# Mycelix constitutional formal verification

Status: experimental, non-activating formal artifacts.

These models provide complementary bounded views of the constitutional authority program. They do not themselves activate runtime enforcement and they are not evidence-bearing until the exact files have been checked by pinned tools under an exact recorded lineage.

## Evidence classes

Keep these claims distinct:

1. **Formal artifact present** — a `.tla`, `.cfg`, or `.als` file exists and is reviewable.
2. **Parser-valid** — the pinned formal tool parsed the exact file successfully.
3. **Bounded safety PASS** — the pinned checker searched the recorded finite state/scope and found no counterexample to the declared safety invariants.
4. **Bounded reachability demonstrated** — the pinned checker found the specifically named expected witness state in the recorded finite bound.
5. **Negative-control sensitivity demonstrated** — intentionally weakened temporary fixtures fail in the expected named way.
6. **Executable reference conformance** — Rust tests demonstrate selected concrete semantics corresponding to the formal abstraction.
7. **Runtime refinement evidence** — production Holochain/runtime behavior has an explicit, qualified mapping to the checked formal model.

Never collapse a lower evidence class into a higher one. In particular, safety and reachability are separate claims: an invariant can pass vacuously when the important history is unreachable.

## Division of responsibility

### TLA+

Use TLA+ for temporal/concurrent questions:

- competing consumption claims;
- finality before side effect;
- revocation effective order vs observation order;
- late contradictory evidence;
- integrity-fault halting;
- dependency indeterminacy;
- idempotent redelivery;
- explicit safety vs reachability vs liveness assumptions.

`ConstitutionalConsumptionV2.tla` uses a monotonic logical event clock. A revocation records both its authenticated effective sequence and the later model event at which that evidence was observed. This models delayed evidence without allowing ordinary transitions to move backward in logical time.

The two canonical safety configurations are separate constitutional profiles:

- `ConstitutionalConsumptionV2.finality.cfg` — finality is the constitutional commit point;
- `ConstitutionalConsumptionV2.effect.cfg` — authority must remain valid until the effect.

Both canonical profiles include `RevocationCutoffConsistentWhenFaultFree` and `RevocationObservationConsistent`. The former is an independent cutoff theorem rather than merely relying on the transition guard. The latter requires every observed revocation to have one recorded observation sequence at/after its authenticated effective sequence and no future observation timestamp.

### Temporal non-vacuity

Four dedicated TLA+ configurations establish bounded reachability by asserting the negation of a named witness and expecting TLC to violate that exact invariant:

- `ConstitutionalConsumptionV2.reach-effect-cancel.cfg` — Effect mode can finalize, later learn an earlier-effective revocation without fault, and remain unapplied;
- `ConstitutionalConsumptionV2.reach-finality-fault.cfg` — Finality mode can finalize then fault on later-observed earlier-effective revocation;
- `ConstitutionalConsumptionV2.reach-finality-postcommit-effect.cfg` — Finality mode can observe a revocation effective after finality and still apply the already-committed effect;
- `ConstitutionalConsumptionV2.reach-effect-posteffect-fault.cfg` — Effect mode can apply an effect then fault if later evidence proves a revocation effective at/before that effect.

A reachability case passes only when the exact named `Never...` invariant is violated while the ordinary safety invariants in the same config remain intact up to that witness. An unrelated TLC failure is not reachability evidence.

Reachability within `MaxSeq = 6` is still bounded evidence. It does not establish liveness, fairness, or eventual progress in production.

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

The revocation cutoff mapping is especially explicit:

- Rust `RevocationCutoff::Finality` ↔ TLA+ `Cutoff = "Finality"`;
- Rust `RevocationCutoff::Effect` ↔ TLA+ `Cutoff = "Effect"`.

The Rust reference model now measures contradiction against the selected commit point: accepted finality in Finality mode and an already-applied effect in Effect mode. The TLA+ witness profiles are designed to exercise both sides of that distinction.

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

Every receipt should record tool artifact hash/version, Java runtime, command line, worker/solver settings, exact model/config hashes, exact scope/config, output/log digest, raw Git head, and postflight immutability.

## Refinement/drift

See MYC-CONST-003CR. Rust, TLA+, Alloy and later Holochain enforcement are multiple verification views of one constitutional meaning, not independent constitutions. New Rust authority variants, temporal states, cutoff semantics, or formal symbols require an explicit mapping or an explicit `out_of_model` decision before formal evidence is used to justify production behavior.
