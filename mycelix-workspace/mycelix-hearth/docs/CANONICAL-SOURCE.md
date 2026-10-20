# Hearth canonical source boundary

Hearth's authoritative implementation tree is:

`mycelix-workspace/mycelix-hearth/`

The repository-root `mycelix-hearth/` tree is legacy and is not a valid source for Holochain 0.7 qualification, packaging, or new development.

## Rule

New repository consumers must reference `mycelix-workspace/mycelix-hearth/`. References to the legacy root tree must be removed or explicitly handled as part of the legacy migration/archive boundary.

This boundary exists because dependency/toolchain qualification is only meaningful when the source tree being compiled and packaged is unambiguous.

## Qualification relationship

Holochain 0.7 qualification applies to the canonical workspace tree only. The canonical tree pins the 0.7-compatible HDK/HDI stack and is the subject of the 0.7 build gate.

The legacy tree must not silently become a second source of truth.
