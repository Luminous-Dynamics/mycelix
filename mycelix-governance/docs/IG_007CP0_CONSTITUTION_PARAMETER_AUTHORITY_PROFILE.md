# IG-007CP0 — Constitution parameter authority profile

Issue: #1003

Parent security finding: #1002

## Purpose

Freeze the exact source-visible authority predicates around legacy `GovernanceParameter` mutation before any repair.

This evidence is intentionally separate from upstream execution P0 #904 and governance-bridge config P0 #943.

## Frozen subject

```text
semantic production subject
fca2c107a1ea5108823ce617ba4111b6f7f77230

tree-equivalent authoring main
31ede2365b81365bb119cd9351b2739119974130
```

Bound source blobs:

```text
execution coordinator       3dbb8a8f69b377e494ccf24164c94bd80f54e0ef
constitution coordinator    923a1ce789c8319c79df7f33a9241af50804ec55
constitution integrity      f83a457a8ff40b0003c07dba9da598a478c5e6f6
```

## Profile

```text
id        mycelix-constitution-parameter-observed-fca2c107-v1
revision  1
authority ObservedSourceBound
SHA-256   770552d12489df1d2cdf8b0af676b01ea9a3da21940f70ed8a71910deaa35009
```

## Execution dispatch

The observed `GovernanceAction::UpdateParameter` carries only:

```text
parameter
value
```

when calling `constitution::update_parameter`.

It does not carry `proposal_id` or a qualified authorization reference.

## Constitution coordinator

`UpdateParameterInput` accepts:

```text
parameter
value
proposal_id: Option<String>
```

with serde default for `proposal_id`.

The coordinator forwards that optional string into `set_parameter`. No source-visible proposal existence/status/type lookup, exact parameter/value authorization binding, execution authorization binding, or caller-authority theorem is reconstructed there.

### Existing-parameter containment

The observed `set_parameter` gate is not empty:

```text
existing parameter + proposal_id=None -> reject
```

This is a meaningful positive control and is preserved in the profile.

However:

```text
existing parameter + proposal_id=Some(...) -> presence satisfies this coordinator gate
```

without observed reconstruction of the claimed proposal authority.

### New-parameter bootstrap path

For a previously absent parameter:

```text
proposal_id=None
```

is not rejected by the observed coordinator gate.

The resulting entry records `changed_by_proposal=None`.

This is modeled as an observed contract boundary, not a live mutation claim.

## Storage/currentness projection

Each write uses `create_entry` and creates another `ParameterIndex` link.

`get_parameter(name)` chooses the link with maximum timestamp and returns its target.

The observed source does not define a separate fail-closed authority rule for competing same-name parameter publications, so timestamp-selected projection is not promoted into authoritative currentness.

## Integrity boundary

The parameter integrity path establishes structural validity only:

- create: non-empty name + JSON-parsable value;
- update: JSON-parsable value.

The create/update action author arguments are not used to establish parameter mutation authority. The integrity validator does not reconstruct `changed_by_proposal`, proposal status/type/action authority, or a canonical current parameter lineage.

## Core non-equivalences

```text
proposal_id present != proposal authority verified

parameter shape valid != mutation authorized

timestamp-selected parameter != authoritative current parameter

execution dispatch != downstream authority continuity
```

## Successor direction

A corrected path should consume a content-bound authorization theorem binding exact prior state/absence, parameter name/value, proposal/action, current governance/constitutional epoch, executor principal, and authorization identity.

Bootstrap should become an explicit bounded genesis profile rather than ambient `proposal_id=None`.

## Non-claims

This profile establishes no live unauthorized mutation, exploit success, legal/constitutional invalidity, deployment currentness, or governance-safety verdict.
