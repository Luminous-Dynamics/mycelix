# mycelix-welfare-evidence

Pure `#![no_std]` AMSAP-005 evidence-provenance substrate for artificial moral-status research.

Every record binds a claim to an exact operational subject (`ModelId`, `LineageId`, `InstanceId`) plus configuration, environment, policy, and optional scaffold/memory commitments. This prevents evidence gathered from one scaffolded runtime from silently becoming evidence about every model sharing its weights.

Evidence direction is explicit and symmetric:

- `Supports`
- `Challenges`
- `Null`
- `Mixed`

Negative and null findings are therefore valid records rather than absence of evidence.

Verifier profiles preserve six fault-domain dimensions: organization, evaluator, method, theory family, tooling, and evidence source. `assess_verifier_independence` refuses to call replications materially independent when the organization or evaluator is shared, even if superficial verifier IDs differ elsewhere; materially independent classification requires distinct organization/evaluator plus at least four distinct dimensions overall.

Developer-organization overlap must be declared consistently with the subject scope. Other conflicts remain explicit data rather than hidden metadata.

A `QualifiedEvidenceRecord` means only that provenance/lineage structure is coherent. It does not establish scientific truth, welfare protection, legal standing, governance authority, currentness, or external-effect authority.
