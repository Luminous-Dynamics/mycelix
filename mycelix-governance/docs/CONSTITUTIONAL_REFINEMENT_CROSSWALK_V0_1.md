# Constitutional Refinement Crosswalk v0.1

Status: **draft schema/validator scaffold** under #1532 / MYC-CONST-003CR1.

This tranche does not claim runtime refinement. It establishes the machine-readable vocabulary and the first qualified mapping slice: the already-qualified MYC-CONST-003B2 lifecycle status projection.

## Why this exists

Formal models and Rust reference semantics can both be locally correct while their correspondence silently drifts. A refinement claim is therefore valid only when the mapping itself is explicit, versioned, census-checked, and bound to exact artifacts.

The v1 crosswalk makes the following properties first-class rather than prose:

- relationship kind;
- action-refinement cardinality;
- concrete persistence;
- crash observability;
- concrete transaction/commit boundary;
- formal role;
- whether a formal state corresponds to a concrete commit point;
- qualification status;
- exact concrete and formal Git blob identities.

## Closed relationship vocabulary

`kind` is one of:

- `exact_enumeration`;
- `projection`;
- `action_refinement`;
- `ghost_environment_state`;
- `stuttering_refinement`;
- `out_of_model`;
- `future_unimplemented`.

`action_cardinality` is one of:

- `one_to_one`;
- `concrete_atomic_to_formal_microsteps`;
- `formal_atomic_to_concrete_microsteps`;
- `environment_plus_concrete_action`;
- `projection_only_no_action_correspondence`.

No free-text relationship type may substitute for these enums.

## Initial qualified slice

The first manifest maps the qualified Rust `ClaimLifecycleStatus` reference semantics to the qualified `ConstitutionalClaimLifecycle.tla` status atoms.

Rust variants project as:

- `PendingExecutable` -> `Pending`;
- `BlockedAwaitingEvidenceClosure` -> `Blocked`;
- `Finalized` -> `Finalized`;
- `RejectedConflict` -> `Rejected`;
- `RevokedClosed` -> `RevokedClosed`;
- `IntegrityHalted` -> `Halted`.

`Absent` is different: there is intentionally no Rust `ClaimLifecycleStatus::Absent`. The formal `Absent` atom projects from the absence of a `ClaimRecord` in `ClaimLifecycleState::claims`.

These are **projections**, not byte-level equivalences. Rust retains provenance and evidence fields that the TLA status atom intentionally abstracts.

## Persistence boundary

The concrete side in this initial manifest is the pure Rust reference state, not a production persistence layer.

Therefore the initial lifecycle rows explicitly record:

- `persisted = false`;
- `crash_observable = false`;
- no runtime commit correspondence.

Future Holochain/runtime rows must not inherit those properties. #1535 must define the real durable commit/crash/outbox protocol, and #1532 must then map that protocol separately.

## Pending extensions are not evidence

The initial manifest remains `status: draft` and records later work only under `pending_extensions`:

- #1487 / 003B4 ClaimBinding exact identity;
- #1426 / 003C3 quiescence/readiness/ghost-state refinement;
- #1535 runtime crash consistency.

A candidate head appearing in `pending_extensions` is provenance for future work only. It is not a qualified relationship and may not be cited as refinement evidence.

A future `status: qualified` manifest must contain no `pending_extensions`.

## Validator gates

`tools/formal/validate_refinement_crosswalk.py` uses only the Python standard library plus local Git.

It checks:

1. top-level manifest identity and closed enums;
2. exact source/model Git blob identities;
3. unique relationship IDs;
4. qualification-status consistency;
5. exact Rust `ClaimLifecycleStatus` census parsed independently from source;
6. exact TLA `Statuses` census parsed independently from the model;
7. one mapping per Rust lifecycle variant;
8. one mapping per TLA status atom;
9. `Absent` maps exactly once to `derived_absence`;
10. a qualified manifest cannot retain pending extensions.

The validator emits a machine-readable receipt containing manifest/schema SHA-256 values, source/model Git blob hashes, both independent censuses, and mismatch diagnostics.

## Drift behavior

The current scaffold must fail if, for example:

- Rust adds a lifecycle status without a mapping row;
- TLA adds/removes/renames a status atom without a mapping decision;
- a mapped source/model file no longer matches the bound Git blob identity;
- a row uses an unknown relationship/action-cardinality class;
- `Absent` is incorrectly invented as a Rust enum variant;
- a draft extension is relabeled as qualified without being moved into an exact relationship;
- the manifest is marked qualified while pending extensions remain.

## Next qualification step

This scaffold itself still needs a dedicated exact-head qualifier. The qualifier should include mutation controls that:

- add an unmapped Rust lifecycle variant;
- add/remove a TLA status atom;
- duplicate a mapping;
- bind a row to the wrong Git blob SHA;
- convert `Absent` from `derived_absence` to an enum variant;
- mark the draft manifest qualified while pending extensions remain.

Only after that validator qualification should the schema scaffold be treated as a stable refinement waist.

## Non-goals

No whole-program verification, no generated political policy, no runtime persistence claim, no ClaimBinding qualification claim, no 003C3 qualification claim, and no waiver for unmapped future symbols.
