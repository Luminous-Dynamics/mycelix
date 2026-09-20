# FIN-SYNC-001 input and ingress boundary

FIN-SYNC-001 constructs and commits a bounded semantic settlement graph from an
already-decoded `SettlementGraphInputV1`.

Its collection constants and `BoundedText` rules are **semantic construction
bounds**. They are not, by themselves, a proof that an arbitrary raw payload can
be parsed with bounded memory or bounded allocation amplification.

## What FIN-SYNC-001 establishes

Once a caller has a representable `SettlementGraphInputV1`, the graph builder
fails closed when the semantic input exceeds the frozen V1 leg, dependency,
group, group-member, text, role, connectivity or dependency-DAG rules.

Authority-significant FIN-SYNC Serde inputs also fail closed on unknown fields:

- `SettlementGraphInputV1`;
- `SettlementLegSpecV1`;
- the nested leg amount object admitted by FIN-SYNC's schema-closed amount deserializer;
- `DependencySpecV1`;
- `CoordinationGroupSpecV1`;
- nested semantic profile references.

FIN-SYNC deliberately reconstructs the qualified FIN-ECO-001 `AssetAmount` from
only exact `atomic_units` + `asset` fields at this boundary instead of relying on
that imported type's more permissive generic derived deserializer. This does not
change FIN-ECO-001 bytes or arithmetic semantics.

This prevents a caller from attaching fields such as `authorized`, `settled`,
provider balances, or opaque provider receipts and having those fields silently
discarded while a graph is constructed from the remaining visible semantics.

## What FIN-SYNC-001 does not establish

A generic decoder may allocate strings or vectors while it is deserializing,
before `build_settlement_graph_v1` can evaluate collection counts.

Therefore:

```text
MAX_LEGS / MAX_DEPENDENCIES / MAX_GROUPS
!= pre-deserialization allocation bound

BoundedText validation
!= raw-request byte admission

SettlementGraphInputV1 deserializes
!= parser resource budget qualified
```

FIN-SYNC-001 does not claim:

- a maximum raw request size;
- streaming refusal of oversized sequences;
- maximum parser nesting/resource use;
- maximum aggregate decoded text allocation;
- HTTP, Holochain or provider transport admission;
- protection against every parser-level allocation-amplification attack.

## Follow-on theorem

FIN-SYNC-001A / #2498 owns the stronger ingress theorem:

```text
raw bytes
   ↓
BoundedSettlementGraphIngressV1
   ↓
SettlementGraphInputV1 candidate
   ↓
build_settlement_graph_v1
   ↓
SettlementGraphV1
```

That tranche must enforce the selected raw-byte and decode-resource envelope
before or during parsing rather than inferring it from post-deserialization
`Vec::len()` checks.

## Authority boundary

Neither successful parsing nor successful graph construction conveys authority:

```text
payload admitted
!= graph well formed
!= graph authorized
!= rail capability available
!= capacity reserved
!= operation dispatched
!= settlement synchronized
```
