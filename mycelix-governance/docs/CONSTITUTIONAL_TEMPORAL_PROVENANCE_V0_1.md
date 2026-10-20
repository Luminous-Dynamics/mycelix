# Constitutional Temporal Provenance v0.1

Status: experimental, non-activating, unqualified until exact-head Rust/TLC evidence runs.

This tranche separates **when constitutional evidence became effective** from **when a verifier learned it**. It also defines when a finality domain may claim that an earlier evidence interval is complete.

## Why this exists

The current constitutional-consumption reference model already permits a finality proof whose authenticated `finalized_at_seq` predates a known revocation, even when the proof is observed later. That means packet/arrival order is not constitutional order.

A second distinction is equally important: observing no earlier proof is not the same as proving no earlier proof exists. A claim affected by revocation therefore cannot always become terminal merely because the revocation is known. The relevant finality domain must either provide a legitimate closure/watermark or explicitly use a reviewed policy that disallows late historical proof.

## Core model

`TemporalOrder` binds:

- finality/evidence domain;
- effective sequence;
- observation sequence.

The basic invariant is:

`effective_seq <= observed_seq`

Observation order is monotonic within one `TemporalEvidenceState` across finality and revocation evidence.

`FinalityEvidence` binds one claim/proof to this dual order and to the selected `FinalityProfile`.

`RevocationEvidence` gives revocation the same dual-order treatment. An already-known revocation effective at sequence `R` rejects newly observed finality effective at `R` or later. A later-observed finality proof effective before `R` remains admissible until an appropriate closure says that earlier interval is complete.

`EvidenceClosure` asserts that every **admissible** finality proof in one domain with effective sequence <= `closed_through_effective_seq` is complete. It binds:

- finality domain;
- closure effective watermark;
- observation sequence;
- finality profile;
- policy version;
- previous closure ID for monotonic chaining;
- an authenticated proof/reference supplied by the runtime.

Invalid finality that is already blocked by a known revocation is rejected before closure contradiction checks. Such evidence therefore does not falsely fault a completeness watermark that speaks only about admissible finality.

## Closure capability by finality profile

### LocalIdempotent

May use `LocalCheckpoint` only when completeness is genuinely confined to that local/single-writer authority domain. Publication of that checkpoint to a DHT does not transform it into global DHT completeness.

### DetectionOnly

Cannot produce a constitutional closure in v0.1.

The absence of currently visible competing evidence is not a proof that no admissible earlier evidence exists. A DHT query miss, unavailable dependency, temporary fork absence, or timeout is therefore insufficient.

### WitnessedSingleSpend

Uses `WitnessQuorum`. The policy fixes minimum witness count and minimum independent constitutional/finality domains. Duplicate identities or multiple witnesses from one domain do not satisfy diversity by themselves.

The runtime still has to prove why this particular witness protocol is authorized to attest **completeness**, not merely individual claim validity.

### StrongConsensus

Uses `StrongConsensus`, binding both a finalized checkpoint reference and a commitment to the relevant finality namespace. A consensus proof for one transaction is not automatically namespace completeness.

## Post-closure contradictory evidence

If new **admissible** finality is observed after an accepted closure and its effective sequence lies within the already-closed interval:

1. it is retained as quarantined evidence;
2. it is not silently accepted into canonical finality history;
3. `TemporalIntegrityFault::FinalityAfterClosedWatermark` is raised;
4. later finality evidence remains retained for audit but is quarantined;
5. later revocation evidence may still be recorded;
6. new closure advancement is blocked while the fault remains unresolved.

This preserves contradictory evidence without rewriting history or pretending the finality domain remains trustworthy.

## Revocation terminalization query

Under the current consumption rule, a revocation effective at sequence `R` blocks finality effective at `R` or later, while earlier finality (`< R`) may still be admissible.

Therefore:

- for `R = 1`, there is no positive pre-revocation finality interval;
- for `R > 1`, closure through at least `R - 1` is required before a lifecycle layer can say that no admissible earlier finality proof remains outstanding.

The crate exposes `pre_revocation_interval_is_closed(R)` for this purpose. It does **not** itself change claim lifecycle; #1427 consumes this fact later.

## TLA+ companion model

`ConstitutionalEvidenceClosure.tla` independently models:

- monotonic observation clock;
- delayed finality and revocation evidence;
- known-revocation rejection of finality effective at/after revocation;
- late-observed pre-revocation finality acceptance;
- closure prohibition for `DetectionOnly`;
- closure monotonicity;
- quarantine + integrity fault for post-closure earlier-effective admissible finality;
- continued evidence intake after fault while canonical finality/closure remains frozen;
- reachability of late-observed pre-revocation finality;
- reachability of closure and post-closure contradiction histories.

`AdvanceObservation` only advances the abstract observation clock to represent unrelated authenticated events. It is **not** a liveness or constitutional-progress claim and must not be reused as such in #1426.

The TLA+ model abstracts away the concrete cryptographic form of `LocalCheckpoint`, `WitnessQuorum`, and `StrongConsensus`; those profile-specific proof-shape checks are owned by the Rust reference crate in this tranche. The refinement crosswalk must record that abstraction explicitly.

## Evidence classes

The checked-in model/config files are specifications only until exact qualification runs. Future evidence must keep separate:

- Rust compile/test evidence;
- TLA+ bounded safety;
- TLA+ named-history reachability;
- negative-control sensitivity;
- Rust ↔ TLA+ bounded refinement;
- runtime/Holochain refinement.

A green result at one finite bound is never an unbounded theorem.

## Non-goals

This tranche does not:

- activate Holochain validation or storage;
- authenticate real witness signatures;
- choose a consensus technology;
- resolve claim lifecycle states;
- define recovery from an integrity fault;
- claim that Holochain DHT visibility provides completeness;
- replace the already-qualified ConstitutionalConsumptionV2 evidence.

The intended dependency is:

`003B3 temporal provenance + closure -> 003B2 claim lifecycle -> 003C3 quiescence/liveness -> refinement/runtime enforcement`.
