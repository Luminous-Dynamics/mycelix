# Hardware Manufacturing Projection

One-way adapter from canonical Mycelix hardware design semantics into the existing `mycelix-manufacturing` operational BOM model.

The projector produces a **BOM candidate**, never a manufacturing authorization. It has no reverse path capable of mutating a `DesignRevision`.

Key boundaries:

- `DesignComposition != BillOfMaterials`;
- production lot quantity does not rewrite per-unit design quantities;
- procurement part IDs are explicit mappings, never inferred from semantic component IDs;
- sub-designs require explicit sub-assembly BOM mappings;
- proposed substitutions are rejected from the exact projection path until an external substitution-qualification boundary exists;
- mapping provenance may be referenced, but presence of a digest does not mean that provenance has been verified;
- work-order completion and physical conformity remain outside this crate.
