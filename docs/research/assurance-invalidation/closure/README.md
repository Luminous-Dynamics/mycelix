# LISE Invalidation Dependency-Closure Seed

**Corpus:** `LISE-INV-CLOSURE-12-v1`  **Owner:** [#6219](https://github.com/Luminous-Dynamics/mycelix/issues/6219)
**Status:** public synthetic benchmark-only graph projection; not a production dependency graph.

## Why this exists
The 50-case invalidation corpus names domain changes and claim outcomes, but executable invalidation closure also needs explicit claim-to-dependency relations. This seed tests the graph mechanics separately from owner-specific currentness/applicability decisions.

## Ownership
- Reuse [#1243 LEX-NET-030](https://github.com/Luminous-Dynamics/mycelix/issues/1243) dependency/provenance relations.
- Reuse [#5762](https://github.com/Luminous-Dynamics/mycelix/issues/5762) multi-valued/transitive closure and unknown inventory treatment.
- Owner-specific final states remain with #1087, #2631, #1207 and #181.

## Files
- `scenarios.v1.json`: 12 cases in six A/B pairs.
- `oracle.v1.json`: separate expected graph-impact sets.
- `scenario.schema.json` / `oracle.schema.json`: strict Draft 2020-12 schemas.

## Benchmark-only graph rule
Dependency edges are directed from an upstream dependency node to the claim-dimension node it supports or conditions. Qualified change events seed a graph traversal; reachable claim dimensions are candidate-affected. Unknown/unavailable change coverage only makes explicitly scoped claim dimensions unknown. Remaining claim dimensions are unaffected by the declared change set—not automatically current or valid.

This calculation does **not** decide historical validity, EvidenceApplicability, CurrentnessAssessment, authority, or effect permission. A downstream owner theorem must evaluate those states. The graph file is a bounded test projection, not a replacement dependency service or canonical graph schema.

## Cases
1. Key rotation affects current signing authority but not historical signature integrity.
2. Transitive policy import change affects current effect authorization but not historical decision bytes.
3. Tokenizer update affects model behavior claims but not an independent policy-engine claim.
4. New alternate route affects complete mediation but not PEP identity.
5. Incomplete dynamic route coverage makes mediation unknown while preserving historical static route tests.
6. Shared upstream observer dependency affects independence but not authenticity of recorded observation bytes.

All files and answers are public calibration material, not blind holdouts or independent adjudication.
