# Stable frontier, safe history reclamation, and cold-start integrity v1

Status: **ReferenceModelOnly**

This tranche closes the next long-lived federation seam after D6I: not merely preserving tombstones, but defining when historical material may safely be reclaimed.

The governing distinction is:

```
old enough
!=
causally stable
```

A stable frontier is therefore a qualified semantic boundary, not a wall-clock age or a local compaction preference.

## Stability scope

`SemanticStabilityScopeV1` binds:

- the exact semantic environment;
- an explicit membership epoch;
- each known participant's role and lifecycle state;
- membership evidence;
- explicit unknown authority-bearing participants.

Authority-bearing participants are treated differently from mirrors and observers.

An authority-bearing participant that is merely offline is still part of the safety scope. It cannot be silently omitted from a stability calculation.

A participant can leave the active scope only through an explicit `Fenced` or `Retired` state with corresponding evidence. This makes membership change part of the theorem rather than an operational side note.

## Stable frontier certificate

`StableFrontierCertificateV1` binds:

- exact scope identity;
- semantic environment;
- membership epoch;
- candidate frontier root;
- the set of frontier roots explicitly covered by the certificate;
- per-participant coverage evidence;
- a certificate commitment;
- a claim ceiling.

For every active authority-bearing participant, coverage must:

1. use the exact membership epoch;
2. use the exact semantic environment;
3. explicitly identify the participant;
4. carry evidence;
5. report the exact candidate frontier root.

A stale frontier from one peer therefore cannot be combined with current frontiers from other peers to manufacture stability.

A fenced or retired participant does not count as an active acknowledgement; its exclusion must itself be explicit and evidenced.

## Unknown and disconnected branches

An unknown authority-bearing participant blocks stability.

This is intentionally conservative. A simple minimum over the replicas currently visible to one node is not sufficient when the membership view itself may be stale.

The reference model therefore makes the scope closed before it permits a pruning theorem.

This matches the broader distributed-systems requirement that safe causal garbage collection depends on knowing which replicas must be covered; when that knowledge is absent, reclamation may safely block rather than silently assume the missing replica is irrelevant.

## Retention boundary

`RetentionBoundaryV1` separates:

- known history roots;
- reclaimable history roots;
- retained history roots;
- known tombstones;
- reclaimable tombstones;
- retained tombstones.

Both inventories must be explicitly closed and every known item must be classified.

A tombstone is reclaimable only when its causal frontier is included in the stable frontier's explicit coverage set.

Thus compaction can legitimately remove old representation after qualified stability, but it cannot simply omit a tombstone because it looks old, inconvenient, or absent from a local cache.

## Pruning receipts

`PruningReceiptV1` binds the actual reclamation operation to:

- scope;
- semantic environment;
- membership epoch;
- source snapshot;
- exact retention-boundary commitment;
- exact reclaimed history roots;
- exact reclaimed tombstones;
- resulting snapshot;
- claim ceiling.

Two different retention boundaries therefore cannot be silently interchanged.

A pruning receipt produced for one boundary cannot be replayed against another boundary, even when both refer to the same snapshot lineage.

## Cold-start reconstruction

`ColdStartManifestV1` defines the minimum semantic material required to reconstruct a normative node:

- node identity and incarnation;
- semantic environment;
- membership epoch;
- committed snapshot root;
- snapshot frontier;
- required history roots;
- retained tombstone anchors;
- reconstruction profile;
- expected normative state root.

A node with a self-consistent local snapshot but missing required history or tombstones is not automatically normative.

`ReconstructionReceiptV1` must reproduce the exact normative state root and retained lifecycle anchors described by the manifest.

This preserves the distinction between:

```
locally parseable state
!=
qualified semantic state
```

## Rejoin fencing

A rejoining node must present:

- a known participant identity;
- a new or explicit incarnation;
- the current semantic environment;
- the current membership epoch;
- the current normative frontier.

Fenced or retired participants cannot regain normative status through replay.

A stale membership epoch is rejected even if the node's payload otherwise looks valid.

A stale frontier is likewise rejected.

This is the same class of zombie-prevention invariant used in other distributed systems: stale replica/member epochs are treated as fencing information rather than as harmless metadata.

## Conservation under forgetting

History reclamation cannot create:

- authority claims;
- capacity claims;
- consent claims.

The reference predicate requires the post-reclamation claim sets to be subsets of the pre-reclamation claim sets.

Forgetting representation may reduce available historical evidence, but it cannot mint new semantic rights or conserved resources.

## Relationship to D6I

D6I established:

```
tombstone + generation + explicit successor
=> no resurrection
```

D6J adds:

```
stable frontier + retention boundary + reconstruction gate
=> safe history reclamation
```

The two tranches are intentionally coupled.

D6J does not weaken D6I. Instead, it establishes the conditions under which some old D6I representation may eventually be discarded without allowing an offline generation to reappear.

## Symthaea boundary

Symthaea may:

- detect likely reclamation opportunities;
- analyze frontier coverage;
- identify stale participants;
- recommend compaction;
- propose reconstruction plans;
- compare alternative retention boundaries.

Symthaea may not:

- declare a frontier stable;
- omit an unknown authority-bearing participant;
- clear a tombstone;
- authorize pruning;
- convert a self-consistent cold-start snapshot into normative authority;
- un-fence a participant;
- mint authority, capacity, or consent through history loss.

The semantic root remains Mycelix.

## Qualification boundary

The executable model establishes only deterministic reference semantics.

It does **not** establish:

- durable storage guarantees;
- production distributed-system safety;
- Byzantine consensus;
- legal deletion;
- privacy compliance;
- real-world availability;
- physical durability;
- cryptographic authenticity of the supplied roots or evidence.

Runner/check evidence remains required before raising the claim ceiling above `ReferenceModelOnly`.
