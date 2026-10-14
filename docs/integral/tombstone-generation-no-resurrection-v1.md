# Tombstone, generation, and no-resurrection integrity v1

Status: **ReferenceModelOnly**

This tranche closes the ABA/resurrection seam created by long-lived federation.

## Core rule

A semantic value can legitimately recur while still belonging to a different semantic generation. Conversely, the same generation can be replayed after revocation, deletion, or supersession.

Therefore:

- visible value is not generation identity;
- identifier reuse is not authorization;
- absence is not deletion;
- a tombstone is a semantic fact;
- reactivation is a new successor transition, not resurrection of the retired generation.

## Tombstones

`SemanticTombstone` records the retired generation, its creation event, causal frontier, reason, and provenance. The tombstone remains historical evidence even when projections, indexes, or cached materializations are rebuilt.

## Reactivation

A successor must provide:

- a new generation identity;
- the exact predecessor tombstone;
- the exact tombstone causal frontier;
- a distinct successor event;
- the same semantic environment;
- an explicit successor state.

A delayed write that names the retired generation is blocked. A delayed write that reuses the retired creation event is also blocked.

A new generation may intentionally return to the same visible value. That is not resurrection when the successor relation is explicit and the generation identity is new.

## Resource conservation

Authority, consent, and consequence-capacity claims belong to generations. A successor cannot inherit retired claims merely because the identifiers look the same. Explicit reallocation is required.

## Compaction and caches

Compaction manifests must preserve the required tombstone frontier. A cache entry for a retired generation is never normative merely because the cache is newer than the tombstone.

Indexes and projections are disposable; anti-resurrection evidence is not.

## Federation

Branch reconciliation preserves tombstones. If one branch retains a retirement and another presents the retired generation as live without an explicit successor, the histories are a resurrection conflict rather than a last-arrival merge.

## Symthaea boundary

Symthaea may detect likely resurrection, ABA patterns, or stale replay, but it cannot clear tombstones, mint successor authority, or turn an inferred reactivation into a normative transition.

## Qualification boundary

The executable tests qualify only deterministic no-resurrection reference semantics. They do not establish storage durability, legal deletion, privacy compliance, or production distributed-system safety.