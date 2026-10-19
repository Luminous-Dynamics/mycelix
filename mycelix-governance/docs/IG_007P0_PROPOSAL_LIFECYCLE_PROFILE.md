# IG-007P0 — Legacy proposal lifecycle observed profile

Issue: #992

Underlying authority finding: #66

## Purpose

Freeze the exact legacy Proposal creation/read/update semantics on the current production tree without conflating them with the stronger successor governance-authority stack in #44/#59/#63+.

Authority is `ObservedSourceBound` only.

## Source binding

```text
semantic production subject fca2c107a1ea5108823ce617ba4111b6f7f77230
current same-tree main        31ede2365b81365bb119cd9351b2739119974130
proposals coordinator         eb8358353ee259ef9c3b46617a61d3439f1c714c
proposals integrity           986bc0526aec8d37436efbe5ba798bc41705e3cf
```

## Profile identity

```text
id        mycelix-proposal-lifecycle-observed-fca2c107-v1
revision  1
SHA-256   7f42e2a8df25df94112d23f261d1f3ffe299d46d37cb3a5a6fe02aca0aa6c108
authority ObservedSourceBound
```

## Observed creation boundary

Creation has meaningful safeguards that must not be lost in the gap analysis:

- proposal author is caller-supplied at the coordinator but integrity-bound to the committing agent;
- new proposals start Draft at version 1;
- voting end must follow voting start;
- actions must be valid JSON;
- ProposalById is linked to the creation action.

The profile records those positive controls separately from lifecycle-currentness gaps.

## Observed read/currentness boundary

`get_proposal` first follows `ProposalById`, selects the maximum link timestamp, fetches that linked action, and returns the linked Proposal directly.

The observed update path does not refresh ProposalById. Therefore the normal linked lookup is not an update-aware lifecycle projection.

The local-chain scan that takes the last matching Proposal is a fallback only when linked lookup does not return a record.

No explicit fail-closed competing-update fork projection is observed.

## Observed update boundary

Proposal update integrity freezes `id` and `author` and validates allowed status-transition shape and exact version increment.

However:

- the update action author is not inspected by `validate_update_proposal`;
- semantic content is frozen only if the original status is already non-Draft;
- Draft->Active therefore does not structurally reject simultaneous title/description/actions/type changes;
- voting start/end and created timestamp are not source-visibly immutable at update validation;
- updated timestamp is not source-visibly bound to the Holochain update action timestamp;
- update validation does not re-run the create-time voting-end-after-start condition.

These are source-contract observations, not live exploit evidence.

## Coordinator interaction

`update_proposal_status` obtains the current record from `get_proposal` and updates that record's action address. No source-visible ProposalById refresh follows the update.

This makes proposal read semantics and update-branch semantics part of the same lifecycle-currentness problem identified by #66.

## Migration boundary

Draft successor PRs such as #44, #59 and #63 contain stronger authority models, including immutable proposal authority context and append-only/fenced execution semantics. They are not represented by this production profile and must not be used to retroactively strengthen it.

A future successor/hybrid manifest may bind those exact lineages after qualification/deployment evidence exists.

## Validator

`validate_ig007p0_proposal_lifecycle_profile.py` fails closed on attempts to:

- make linked reads update-aware without a source change;
- invent update-author binding;
- invent Draft->Active content freezing;
- invent temporal-field authority;
- invent deterministic fork resolution;
- erase #66;
- promote authority or currentness/safety claims.

## Non-claims

No live unauthorized update, production exploit, deployment currentness, or governance-safety claim is made.
