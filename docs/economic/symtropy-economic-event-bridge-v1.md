# Symtropy ↔ Mycelix Economic Fabric bridge

Status: cross-repository engineering reference.

## Boundary

Symtropy owns deterministic simulation state. Mycelix owns portable economic semantics and recognition/settlement boundaries.

`Symtropy EconomicEventEnvelopeV1 -> EvidenceEnvelope -> EconomicEventV1 -> adapter policy -> authorization -> settlement/recognition -> outcome`

## Current Symtropy refinement

The first executable envelope is implemented at:

- repository: `Luminous-Dynamics/symtropy`
- branch: `mycelix-economic-fabric-interoperability`
- source commit: `b4b0de0b90bea9d7dc578f3bb9872bad0de2dd9f`
- implementation: `crates/bridges/symtropy-mycelix-bridge/src/economic_events.rs`
- API: `EconomicEventEnvelopeV1`
- schema marker: `EconomicEventEnvelopeV1`

The implementation provides deterministic `stable_identity()`, a replay fingerprint covering event kind, actors, resources, quantities/units and causal parents, wall-clock exclusion from identity, correction-lineage validation, and an explicit simulation claim ceiling.

## Required preservation

`world_id`, `episode_id`, `simulation_build_id`, `replay_id`, `tick`, `source_event_id`, causal parents, resource identity, quantity/unit, origin, evidence references, validity and correction lineage.

## Non-collapse invariants

- simulation event != physical observation
- simulation ownership != legal title
- TEND != generic currency
- replay equality != independent verification
- recognition != origin rewrite
- adapter projection != source-of-truth mutation
- market acceptance != settlement
- settlement != physical outcome

## First vertical slice

1. Emit deterministic Symtropy envelope.
2. Bind it as simulation-scoped evidence in Mycelix.
3. Map to a Valueflows EconomicEvent without widening semantics.
4. Map the same source to Integral COS evidence without minting ITC.
5. Map to a TEND instrument transaction candidate preserving TEND policy.
6. Produce an accounting projection that remains derived.
7. Exercise duplicate, stale, mutated-replay and origin-rewrite adversarial cases.

## Cross-repo traceability rule

Every adapter implementation must preserve the original Symtropy `source_event_id`, `origin`, replay identity and correction lineage. A target identifier is a recognition/projection identifier, not a replacement for simulation identity.

## Claim ceiling

This bridge can establish semantic interoperability and simulation-level conformance only. It does not establish real-world financial, legal, physical, or economic performance claims.
