# hearth-assignment-transition

Pure Rust contract for append-only household responsibility transitions.

This crate exists because accepting a planner proposal must not silently rewrite a mutable CareSchedule. The schedule remains a durable template; responsibility changes are represented as explicit transitions whose authority can be traced back to a consented proposal.

## Core theorem

A responsibility change is an append-only transition from one exact assignment state to another. Reads derive the effective assignee by walking the transition chain from a trusted base state.

## Concurrency

Identical duplicate transition evidence collapses deterministically. Competing semantic transitions from the same previous state are a fork and fail closed; Hearth never resolves concurrent responsibility changes with last-writer-wins ordering.

## State references

The contract treats state references as opaque stable strings. A DHT adapter may bind a base state to a CareSchedule ActionHash and transition states to content identity such as an EntryHash. The pure contract does not depend on Holochain.

## Scope

This tranche defines chain semantics only. It does not evaluate proposal consent, mutate Care, authorize devices, or perform Holochain calls. The DHT follow-up must prove each transition is backed by accepted, current proposal evidence before admitting it into the chain.
