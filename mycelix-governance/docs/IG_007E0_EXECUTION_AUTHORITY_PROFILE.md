# IG-007E0 — Observed execution authority profile

Status: **ObservedSourceBound / execution-authority observation**

Tracks: #905. Motivated by P0 #904.

## Purpose

Freeze the exact source-visible authority chain from timelock construction through executable governance action dispatch before production hardening.

This lineage is intentionally separate from the IG-007A voting-profile stack.

## Frozen source subject

Production subject:

`fca2c107a1ea5108823ce617ba4111b6f7f77230`

Bound source blobs:

```text
execution coordinator
3dbb8a8f69b377e494ccf24164c94bd80f54e0ef

execution integrity
657edaee9a314f100a0c4b1609a4596cf243e61d
```

## Profile

Profile id:

`mycelix-execution-observed-fca2c107-v1`

Canonical payload SHA-256:

`c977bdcef9e5faac83351050999451432b618d5cc523bece804eba5dd1ae81f6`

SHA-256 is deterministic content identity only, not a governance signature.

Authority class:

`ObservedSourceBound`

## Observed timelock construction

`create_timelock` receives caller-supplied:

```text
proposal_id
actions
duration_hours
```

with duration constrained to `1..=8760` hours.

The observed coordinator function has no proposal lookup that proves the proposal exists or has the required approval/signature lifecycle state, and does not derive the executable actions or duration from an authoritative proposal/policy object.

The integrity layer for creation checks only:

```text
expires > started
actions are valid JSON
initial status == Pending
```

Those are structural checks, not an execution-authorization chain.

## Observed Ready transition

`mark_timelock_ready` checks:

```text
timelock exists
caller == timelock creator
status == Pending
```

and changes status to `Ready`.

No threshold-signature verification is observed in that transition.

## Observed execution signature policy

`execute_timelock` requires expiry and binds `executor_did` to the caller.

The signature behavior differs by state:

```text
Ready:
  trust Ready state
  no threshold-signature lookup in this branch

Pending:
  attempt threshold-signature lookup
  if threshold-signing authority is unavailable:
      emit warning
      continue execution
```

This profile records the behavior; it does not claim it is the desired design.

## Executable payload relevance

The timelock's stored `actions` are passed to `execute_actions`.

Observed typed actions include:

```text
TransferCredits
  -> governance_bridge::transfer_credits

UpdateParameter
  -> constitution::update_parameter

EmitEvent
  -> signal emission
```

Downstream authorization is explicitly outside this profile. The fact recorded here is only that the execution zome dispatches these action classes from the timelock payload.

## Fail-closed validator

`validate_ig007e0_execution_profile.py` rejects attempts to silently rewrite the old observation by:

- adding authoritative proposal lookup;
- making actions proposal-derived;
- adding threshold-signature verification to Ready;
- changing signing-authority unavailability to fail-closed;
- changing the frozen source blobs;
- deleting the #904 gap;
- adding generic safety/security verdict fields.

A production repair must create successor evidence instead.

## Next tranche

IG-007E1 should freeze source-control-flow counterexamples:

- unbound timelock construction;
- creator-only Pending→Ready without signature predicate;
- Ready branch skipping threshold-signature lookup;
- executable financial/constitutional payload classes;
- Pending fail-open when threshold-signing is unavailable.

No live financial or constitutional mutation should be executed as part of the evidence campaign.

## Non-claims

This profile does not establish a live exploit, downstream authorization failure, deployment currentness, or governance safety. It freezes the execution zome's own source-visible authority assumptions so the successor can be qualified against them.
