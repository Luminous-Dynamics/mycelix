# IG-007C0 — Observed governance-config authority profile

## Scope

IG-007C0 freezes the exact source-visible authority used to mutate Mycelix's runtime governance consciousness configuration on production subject:

`fca2c107a1ea5108823ce617ba4111b6f7f77230`

It is evidence for P0 #943. It does not repair the runtime updater.

Authority is strictly:

`ObservedSourceBound`

## Source binding

The profile binds four exact files:

```text
bridge/coordinator/src/consciousness_config.rs
26da234e588bf26d0e25c10dbec34502e00c191a

bridge/integrity/src/lib.rs
61b20610216e2fd69c701ecaea5322c448eda119

proposals/coordinator/src/lib.rs
eb8358353ee259ef9c3b46617a61d3439f1c714c

proposals/integrity/src/lib.rs
986bc0526aec8d37436efbe5ba798bc41705e3cf
```

## Profile identity

```text
id        mycelix-governance-config-observed-fca2c107-v1
revision  1
SHA-256   de4435a69356557b1812f8beb46d654c66b9c957be9d18c64bd0431f92546d5a
authority ObservedSourceBound
```

SHA-256 is content identity only.

## Runtime authority

The observed bridge reads `GovernanceConsciousnessConfig` at runtime through:

```text
get_dynamic_consciousness_gate(...)
get_dynamic_min_voter_consciousness(...)
```

When no config exists it falls back to hardcoded defaults.

This makes the config an operational mechanism input, not merely documentation or telemetry.

## Declared design vs observed predicates

The source comments on `update_consciousness_config` declare a design requiring:

```text
ProposalExists
ProposalApproved
ProposalTypeConstitutional
```

C0 records this only as `DeclaredDesign`.

The implementation source-visibly establishes:

```text
proposal_id non-empty
proposals::get_proposal invoked
response decodes as Option<Record>
Some(record) exists
config shape/range/monotonicity valid
hardcoded upper/ceiling controls pass
```

It does not source-visibly inspect or bind:

```text
proposal status
proposal type
exact authorized action bytes
caller governance role
execution/signature authorization receipt
```

The proposals read endpoint returns proposal records and does not itself turn an arbitrary returned record into approval/type evidence.

## Integrity boundary

The bridge integrity zome validates config create/update through `check_consciousness_config`.

C0 records no source-visible integrity reconstruction of:

```text
changed_by_proposal -> proposal existence/status/type
entry author -> authorized governance executor
config delta -> approved action commitment
```

Shape validity and authorization are intentionally kept separate.

## Policy-effect observation

The frozen integrity tests contain a structurally valid custom config with:

```text
consciousness_gate_basic = 0.1
```

Therefore a missing authorization predicate can affect runtime participation thresholds; it is not only metadata provenance.

This observation is not a normative statement that the particular threshold is good or bad.

## Hardcoded controls

The coordinator also contains hardcoded anti-tyranny bounds. C0 records them as policy bounds, not authorization.

A numerical value satisfying a bound is not evidence that the actor changing it has authority to do so.

## Relationship to #944

#944 is a different theorem: the constitution's Phi-sync helper targets an absent `update_phi_config` bridge entrypoint.

Do not repair #944 by simply pointing it at `update_consciousness_config` while #943 remains unresolved. A more reachable incompletely authorized updater would be a regression in authority architecture.

## Successor requirement

A corrected system should create a new content-bound profile in which config mutation is derived from an explicit authorization receipt binding at least:

```text
authoritative proposal revision
allowed lifecycle state
allowed proposal/decision profile
exact config delta
action commitment
execution/signature evidence where required
policy revision
```

Historical C0 remains unchanged.

## Non-claims

C0 does not claim a live unauthorized mutation occurred, does not qualify deployment currentness, and does not declare the governance mechanism safe or unsafe.