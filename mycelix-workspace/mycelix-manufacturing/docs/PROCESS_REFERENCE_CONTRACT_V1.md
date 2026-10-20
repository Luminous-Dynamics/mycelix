# Mycelix manufacturing process reference contract v1

This is the additive MFG-PROC-001A reference slice for [issue #3085](https://github.com/Luminous-Dynamics/mycelix/issues/3085). It introduces versioned typed references without changing the current Holochain operation or routing entry layouts, so existing DHT records are not silently reinterpreted.

## Ownership boundary

- Symthaea's manufacturing-process owner defines process identity and the semantics of process definitions, recipes, capability requirements, and process-model profiles.
- Mycelix references those artifacts for distributed registry, routing, work-order, and supplier coordination. It does not copy their physics or infer resolution from an identifier-shaped string.
- Existing machine-type strings, operation names, display labels, and legacy aliases remain compatibility or discovery metadata. They are not sufficient evidence that a resource is capable or qualified.
- ContentDigestV1 records an algorithm and validates digest syntax. An adapter must recompute the digest over exact source bytes whenever those bytes are available; this DTO does not fetch sources, authenticate signers, or prove truth.
- RegistryLifecycleV1 records declared registry lifecycle only. It is not a process-qualification or equipment-release status.
- validate_typed_routing_steps ensures a sidecar batch targets one source routing revision/digest and uses each step sequence once. It does not itself resolve or authorize that source routing.

## Reference semantics

Each CanonicalIndustrialArtifactRefV1 binds:
- artifact kind;
- namespace and identifier scheme;
- opaque source identifier;
- semantic version;
- exact-content digest with its declared algorithm.

Labels and aliases are deliberately outside the canonical reference. Unknown or future namespaces remain representable, but typed presence does not mean the target exists, capability matches, a recipe is admitted, or a job may run.

## Migration and acceptance

1. Existing routes remain readable and unchanged.
2. A migration adapter emits sidecar bindings only when it has an exact routing revision and digest.
3. Legacy operations without typed bindings remain representable; they cannot satisfy downstream typed-reference requirements.
4. An integration layer resolves referenced artifacts and applies the source owner's validator. This tranche introduces no new zome write API, machine command, or execution authority.
5. Run the manufacturing_common tests and Clippy on the exact head before treating this contract as qualified.

## Standards mapping

Use domain standards as source profiles rather than as one global identity:
- ISA-95 / IEC 62264 for enterprise-to-manufacturing-operations concepts;
- OPC UA companion models and MTConnect for equipment representation and observations;
- ISO 23247 concepts for manufacturing digital-twin context;
- QIF / ISO 23952 for inspection/metrology references;
- ISO/ASTM 52900 taxonomy references for additive-manufacturing process families.

This package does not claim conformance with any of those standards. It establishes a typed cross-reference boundary on which specific adapters can later be tested.
