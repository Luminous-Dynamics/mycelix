# Symtropy ↔ Mycelix Economic Fabric bridge

Status: cross-repository engineering reference.

## Boundary

Symtropy owns deterministic simulation state. Mycelix owns portable economic semantics and recognition/settlement boundaries.

`Symtropy EconomicEventEnvelopeV1 -> EvidenceEnvelope -> EconomicEventV1 -> adapter policy -> authorization -> settlement/recognition -> outcome`

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

## Claim ceiling

This bridge can establish semantic interoperability and simulation-level conformance only. It does not establish real-world financial, legal, physical, or economic performance claims.
