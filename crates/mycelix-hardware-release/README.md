# mycelix-hardware-release

Evidence-scoped hardware release manifests and admission profiles for Mycelix open hardware.

This crate does not implement artifact storage, signing, provenance, physical testing, or regulatory certification. It references receipts produced by the systems that own those concerns and requires subject binding before an external result can influence release admission.

Key boundaries:

- manifest validity is not artifact verification;
- artifact inventory verification is not provenance verification;
- digital provenance is not physical conformity;
- a valid receipt for the wrong subject is rejected;
- missing external verification remains `Unknown`;
- the crate exposes admission under an explicit release profile, never a generic `safe` verdict.
