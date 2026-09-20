# mycelix-hardware-core

Transport-independent semantic primitives for Mycelix open hardware.

This crate intentionally contains no Holochain, CAD/EDA, solver, signing, manufacturing-execution, or filesystem dependencies. It defines stable semantic identities and relationships that adapters can project into other systems.

Key boundaries:

- `DesignComposition` is design intent, not a manufacturing BOM.
- `EvidenceReference` records scoped evidence and interpretation; evidence presence does not imply satisfaction.
- `ConstraintEvaluation::Unknown` is distinct from `Unsatisfied`.
- `DesignRevision` validates evidence subjects/claim references against the containing revision.
- Digests are references to evidence/artifacts verified by the subsystem that owns those bytes.

See `docs/hardware/HARDWARE_AUTHORITY_CONSTITUTION.md` for the normative authority model.
