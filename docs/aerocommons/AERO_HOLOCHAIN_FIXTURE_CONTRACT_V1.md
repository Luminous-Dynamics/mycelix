# AeroCommons Holochain Fixture Contract v1

This contract binds the 14-case authority-ceiling corpus to Holochain-shaped validation fixtures.

It is deliberately one layer below a production AeroCommons integrity zome. The fixtures describe the operation shape and dependency semantics that a real HDI validator must implement; they do not emulate the Holochain conductor.

## Fixture fields

Each case declares:

- the authority-ceiling case ID;
- operation kind and variant;
- the engineering subject role;
- dependency retrieval mode;
- whether validation would depend on mutable current state;
- the expected Holochain validation outcome;
- the engineering-layer status;
- the forbidden authority escalation.

## Dependency modes

`none` means the operation can be checked from its own deterministic input.

`addressable_valid_record` means the intended validator dependency is an addressable record retrievable through a deterministic `must_get_valid_record`-style path.

`current_link_collection` is intentionally represented only for the negative determinism fixture. Mutable link state is not a permitted validation dependency.

## Important distinction

The fixture contract does not say that a `Valid` operation is an engineering-valid claim.

For example, AC-AUTH-006 deliberately expects `Valid` for a referenced CreateRecord while separately forbidding the inference that this proves later Update/DeleteLink operations are valid. Holochain's current documentation explicitly notes that `must_get_valid_record` checks the CreateRecord operation and does not necessarily capture later Update/DeleteLink validation failures.

Likewise, AC-AUTH-011 preserves the distinction between `Invalid` and `Unresolved`: if an addressable dependency cannot currently be retrieved, validation is indeterminate and can be retried rather than converted into a negative engineering conclusion.

## Next binding layer

The next implementation should introduce a **test-only HDI fixture zome** that consumes these IDs and constructs real `Op` values.

That zome should prove:

1. deterministic `validate(Op)` behavior;
2. `must_get_valid_record` dependency behavior;
3. `UnresolvedDependencies` preservation;
4. rejection of mutable link/current-state dependencies;
5. separation of Holochain hashes from AeroCommons engineering identities;
6. separation of protocol validity from physical/engineering authority.

Only after that qualification should a production AeroCommons integrity zome be allowed to depend on the graph schema.

This is a qualification artifact, not a certification mechanism.
