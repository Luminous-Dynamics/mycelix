# ITOPS qualification-impact reference v0.1

Public evaluator-side known-answer fixture for Mycelix #6885.

This is an ITOPS-specific profile of the cross-domain invalidation owner #6219, composed with ITOPS-HARNESS-OPS-001 #6877 and Symthaea harness evolution #11030. It does not recreate the generic invalidation graph or event store. Each claim consumes a compact, exact dependency-ref projection resolved by the owner-native lineage layer; #6219 remains the shared corpus/oracle owner.

## Four independent questions

1. Capability qualification/applicability: does a changed tool, skill or specialist invalidate a specific capability claim?
2. Execution authority: may the principal invoke the operation now? Revocation blocks effects without rewriting historic competence evidence.
3. Outcome-evidence currentness: does the observer and coverage frontier still support a claimed service recovery?
4. Benchmark comparability: can scores be pooled under the same evaluator/task protocol?

Do not collapse these into one global valid/invalid bit.

## Known-answer cases

Eight deterministic cases cover tool-contract change, compatible tool addition, permission revocation, observer-adapter drift, specialist identity change, evaluator-protocol change, incomplete change coverage and a no-op control.

The reference tests direct dependencies only; each dependency list is an exact, bounded projection from the owner-native lineage layer, not a second graph engine. A known direct dependency change still requires requalification even if another part of the feed is incomplete; claims exposed only to unknown changes remain Under Investigation. Observer change events must point to the exact observer whose evidence they affect.

## Validation

- Draft 2020-12 schema.
- Pinned jsonschema 4.26.0 schema and instance validation.
- jq semantic checks for four separate output axes.
- Twelve adversarial mutations.
- Exact-revision workflow.

The fixture is synthetic and evaluator-side. It does not grant effect authority or prove real change-feed completeness.

## Connection to the larger programme

This profile is intended to become a separate output axis in the world×harness factorial #6877. A skill may remain qualified while its permission is revoked; a capability may remain qualified while its outcome evidence becomes indeterminate; and a historical score may become incomparable while the capability itself is unchanged.
