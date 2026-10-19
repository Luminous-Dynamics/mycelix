# Mycelix Constitutional Consumption / Finality v0.1

Status: **experimental semantic/reference model**

Tracking: #1261 (MYC-CONST-003B)

This tranche defines when an already-authorized constitutional action is considered consumed/final enough for a real-world or public side effect. It does **not** implement Holochain persistence, signatures, witness discovery, or consensus.

## 1. Authorization is not consumption

`constitutional-authority` answers which constitutional domain may hold a power or entitlement.

`constitutional-envelope` binds that authority to an exact holder, actor, action, resource, matter, provenance, time window, nonce/use policy, concurrence profile, and review route.

This tranche answers a separate question:

> Has this exact authorized use reached enough finality that applying the side effect is safe under its consequence profile?

A valid envelope is therefore necessary but not sufficient for an irreversible action.

## 2. Safety target

The model allows conflicting concurrent claims to exist. Distributed systems cannot safely assume those conflicts will never arise.

The core safety properties are instead:

1. at most one claim may finalize for each use index;
2. no side effect may be applied before finality;
3. duplicate delivery of the same finalized use is idempotent;
4. finalization after effective revocation is forbidden;
5. revocation semantics explicitly declare whether finality itself is the irrevocable commit point or authority must remain live until effect;
6. contradictory later evidence that places revocation before an already accepted finality raises an integrity fault and halts further effects;
7. finalized use count never exceeds the authorization budget;
8. delegated descendants consume from the same budget identity rather than minting a fresh allowance;
9. indeterminate/missing distributed evidence never becomes proof of non-consumption;
10. finality evidence must satisfy the minimum consequence profile;
11. witness thresholds count independent authenticated witnesses/domains, not repeated strings or keys.

## 3. Finality profiles

### `LocalIdempotent`

For local/reversible/idempotent actions where one authenticated writer is authoritative and later duplicate detection is acceptable.

This is **not** global public finality.

### `DetectionOnly`

A deterministic claim can be published and conflicts can later be detected.

Useful for low-consequence public coordination, but insufficient for irreversible single-spend actions.

### `WitnessedSingleSpend`

Finality requires an authenticated witness set satisfying configured identity/domain diversity. Witnesses must bind the exact claim/envelope/use index/target/payload/matter under runtime cryptographic policy.

A witness set is a constitutional notary mechanism, not a declaration that all witnesses share sovereign power.

### `StrongConsensus`

For domains whose consequences require a stronger external finality service. The semantic model requires a non-empty consensus reference and does not prescribe the consensus technology.

## 4. Revocation cutoff is explicit

`FinalityRequirement` includes `RevocationCutoff`:

### `Finality`

Finality is the irrevocable commit point. A revocation ordered after finality blocks future uses but does not cancel the already-finalized effect.

This fits domains where finality itself is equivalent to settlement/commitment.

### `Effect`

Finality reserves the use, but the authority must still be unrevoked at the authenticated logical sequence when the real-world/public effect is applied.

This fits controls where a delayed action should remain cancellable after finality but before actuation, such as some life-critical, administrative, or strategic controls.

This distinction prevents runtimes from silently disagreeing about whether a revocation between finality and execution should stop the action.

## 5. Use budget

Each authorization lineage has a stable `UsageBudget`:

- `budget_id`;
- `max_uses`.

Each claim has a deterministic `use_index` in `[0, max_uses)`.

A delegated child must retain the same budget identity unless a new independent constitutional authorization explicitly creates a new budget. Delegation does not reset spent allowance.

This means a parent with two total uses cannot delegate to two children and accidentally create four total uses.

## 6. Consumption claim

A `ConsumptionClaim` binds:

- claim ID;
- exact authorization-envelope digest;
- nonce;
- use index;
- jurisdiction/runtime finality domain;
- verified `MatterId`;
- exact target digest;
- exact payload digest;
- shared budget ID.

Runtime must authenticate/derive these fields from the envelope and execution request. The semantic type is not proof that the derivation was correct.

## 7. Competing claims are evidence, not automatic catastrophe

The reference state deliberately permits multiple pending claims for the same use index.

For example, during a partition two actors might attempt:

```text
claim-A -> use 0
claim-B -> use 0
```

The safety boundary is:

```text
at most one of claim-A / claim-B may reach Finalized(use 0)
```

A runtime may preserve the losing/conflicting claim as integrity evidence, but it must not apply its irreversible side effect.

## 8. Revocation ordering and contradiction handling

The model uses authenticated logical/event sequence ordering rather than packet-arrival time or ordinary wall-clock arrival.

If finality is sequence 10 and revocation is sequence 20, finality happened first even if the verifier receives revocation first. Whether the later effect is still allowed depends on the declared `RevocationCutoff`.

If revocation is sequence 10 and a finality proof claims sequence 10 or later, finalization is denied.

A harder case is when a verifier has already accepted finality at sequence 10 and later receives authenticated evidence of an effective revocation at sequence 5. The model does **not** silently ignore the revocation and does not rewrite an already-applied external effect. It raises `IntegrityFault::LateEarlierRevocation`, halts new finalization/effects for that lineage, and preserves the contradiction for investigation/remedy.

This fault means the freshness/order assumptions used by the earlier finality decision were unsound. A production finality profile should make this state unreachable under its stated fault assumptions.

Runtime must define what supplies trustworthy event ordering for each finality profile. A local timestamp alone is not sufficient for adversarial ordering.

## 9. Indeterminacy is not authorization

The model has explicit evidence availability:

- `Complete`;
- `Indeterminate`.

`Indeterminate` can represent unavailable witnesses, partitioned dependency state, unresolved revocation data, missing consensus proof, or similar uncertainty.

It never authorizes finality.

Runtime should return pending/indeterminate/temporarily unavailable rather than translating missing evidence into `unused` or `authorized`.

## 10. Idempotent side effects

`apply_effect` is defined only for finalized claims and records the authenticated logical sequence at which the effect was applied.

A second delivery of the same finalized claim returns the original recorded output rather than executing the effect again. Historical duplicate reads remain available even if a later integrity fault blocks new effects.

This should be carried into every practical integration that can support idempotence, including:

- appropriations/expenditure execution;
- credential issuance where uniqueness matters;
- mandate transitions;
- infrastructure configuration changes;
- emergency actuator commands.

## 11. Holochain boundary

Holochain source-chain and DHT validation provide valuable tamper/fork detection, but high-consequence constitutional semantics must not equate eventual detection with pre-effect single-spend finality.

The runtime implementation may use Holochain records, validation receipts, witness services, countersigning where appropriate, or external finality systems. The required safety invariant remains independent of the transport.

## 12. Formal/executable models

This tranche contains two complementary artifacts:

1. `constitutional-consumption` — an executable Rust reference state machine and adversarial tests;
2. `specs/ConstitutionalConsumption.tla` — a small TLA+ safety model for competing claims, finalization, revocation, and effects, with a bounded TLC configuration.

The TLA+ artifact is included as a reviewable formal specification only until a TLC/Apalache qualification lane is added. Its presence must not be reported as model-check PASS without executing a checker against the exact file bytes.

The Rust model is also a reference model, not a proof of the distributed runtime. Runtime implementations must show refinement/conformance to these semantics.

## 13. Runtime work deliberately deferred

Not implemented here:

- cryptographic witness signatures;
- Holochain entries/links/validation callbacks;
- trusted MatterId derivation;
- authenticated event-sequence/freshness service;
- revocation fetch/storage;
- witness-set snapshot governance;
- common-control analysis;
- countersigning integration;
- external consensus adapter;
- real-world side-effect adapters;
- automated recovery from an integrity fault.

## 14. Core principle

> **Conflicts may be observable; irreversible constitutional effects must still be single-finalized. Missing knowledge never proves unused authority, revocation semantics must be explicit, and contradictory ordering evidence must halt rather than be papered over.**
