# Mobility identity and lineage

## MOBILITY-COMMONS-024: configuration applicability transitions

Configuration supersession and physical applicability are independent lineage facts.

If configuration revision `config-r2` supersedes `config-r1`, that fact does **not** implicitly transfer `config-r1 AppliesTo artifact-a` to `config-r2`.

A transition may contain an explicit successor-to-predecessor `Supersedes` edge, a predecessor `AppliesTo` edge, and an optional independently validated successor `AppliesTo` edge.

Three states remain distinct:

1. predecessor applies to an artifact; successor has no applicability assertion;
2. predecessor and successor explicitly apply to the same artifact;
3. predecessor and successor explicitly apply to different artifacts.

The predecessor applicability remains historical in all three cases. A successor never rewrites the predecessor relation.

This is a provenance/lifecycle rule only. It does not establish physical conformance, equivalence, safety, certification, regulatory validity, or measurement truth.

The Rust `ApplicabilityTransition` tests reject implicit inheritance and silently retargeted predecessor applicability while accepting explicit same-artifact and explicit artifact-transition cases.

## MOBILITY-COMMONS-025: physical-artifact lifecycle transitions

Physical-artifact lifecycle state is distinct from physical-artifact identity and from configuration applicability.

The contract now represents four explicit transition classes:

1. **Repair** — a MaintenanceEvent RepairedAs PhysicalArtifact edge preserves the same artifact identity and cannot create a successor artifact.
2. **Replacement** — a PhysicalArtifact ReplacedBy PhysicalArtifact edge requires distinct predecessor/successor identities. The successor never inherits the predecessor's AppliesTo edge implicitly.
3. **Retirement** — an ArtifactLifecycleEvent Retires PhysicalArtifact edge records retirement without deleting or replacing the artifact identity or its historical evidence.
4. **Reactivation** — an ArtifactLifecycleEvent Reactivates PhysicalArtifact edge requires an explicit prior retirement event for the same artifact. Reactivation does not restore configuration applicability automatically.

A replacement may carry an explicit successor AppliesTo edge, but absence of that edge means applicability is unasserted, not inherited.

A repair, retirement, or reactivation does not create a new physical-artifact identity. Historical evidence remains addressable as history rather than being rewritten into the current state.

This follows the digital-thread requirement to preserve associations across product lifecycle stages, including maintenance/repair, demobilization, and retirement. NIST describes digital threads as persistent lifecycle links and explicitly identifies full-lifecycle demonstrations that include inspection, maintenance/repair, demobilization, and retirement. Holochain's source chain and DHT are append-only/immutable, while validation is deterministic and may distinguish definitive invalidity from unresolved dependencies; those properties support the provenance boundary but do not themselves establish engineering truth.

This contract remains semantic/structural qualification only. It does not establish physical equivalence, conformance, safety, certification, regulatory validity, or measurement truth.
