# Integral D6X — Qualified Semantic Dependency Closure

Status: **ReferenceModelOnly**

## Source-bound certificate verification

The existing self-check, `DependencyClosureCertificateV1::valid()`, answers an internal consistency question: the certificate's own fields, identities, dependency-resolution keys, status, and commitment agree with one another. That is necessary but not sufficient for source-bound consumption.

D6X now exposes `DependencyClosureCertificateV1::verifies_against_sources(...)` as the stronger semantic gate. It deterministically re-executes the declared closure policy against the supplied projection, semantic environment, derivation profile, and closure profile, then compares the complete semantic closure result. Audit-only resolution evidence remains outside that semantic comparison while still remaining commitment-bound.

This mirrors a broader validation principle: provenance validity is expressed through explicit consistency constraints, and Holochain's validation model requires deterministic results for a given operation while treating unavailable dependencies separately from definitive invalidity.

The resulting boundary is:

`self-consistency -> source correspondence -> downstream consumption`

A certificate can therefore be internally coherent after a malicious root, cycle, or status substitution and still fail the source-correspondence gate. D6W uses this stronger check before constructing a downstream input commitment.

## Purpose

D6X makes semantic dependency selection executable without equating graph reachability with semantic dependency. A closure is computed from:

```
qualified projection + semantic environment + derivation profile + D6X closure profile
    -> exact dependency closure certificate
    -> D6W input commitment
```

The DKG remains the persistent semantic substrate. D6X is a bounded, task-specific projection over already-qualified material.

## Closure contract

The closure profile fixes:

- root node IDs;
- explicitly required node IDs;
- explicitly required D6P receipt commitments;
- typed edge traversal rules;
- currentness requirements;
- excluded-boundary policy;
- node/edge resource limits;
- D6X algorithm version (currently `D6X-CLOSURE-4`);
- claim ceiling.

The certificate binds:

- closure-profile commitment;
- source DKG snapshot commitment;
- projection commitment;
- semantic-environment commitment;
- derivation-profile commitment;
- exact included node commitments;
- exact selected D6P receipt commitments;
- exact included edge commitments;
- explicit missing dependency IDs plus typed semantic dependency references;
- closure status;
- cycle detection state.

The certificate distinguishes two identities:

- `commitment` is an audit/provenance certificate commitment and remains bound to the candidate projection;
- `closure_identity_commitment` is the candidate-independent semantic identity consumed by D6W.

The semantic identity binds the exact selected node-id → node-commitment mapping and selected edge-id → endpoint/kind/commitment mapping. Selected and missing dependencies are represented through a typed reference algebra (`Node`, `Edge`, `D6PReceipt`). Edge references additionally bind their exact endpoint node IDs and edge kind; this prevents an edge's semantic identity from collapsing to edge ID/commitment alone. D6P receipt references require their identifier and commitment to be identical. Each dependency also has an explicit resolution state (`Present`, `Missing`, `Stale`); not-selected material is represented by absence rather than an `Excluded` dependency. The selected typed set is the canonical semantic dependency set; the parallel node/edge/D6P collections remain explicit compatibility/audit views. Certificate validation reconstructs the canonical selected set from those views and rejects omission, injection, or commitment/type drift. Distinct selected node IDs and edge IDs must also have distinct commitment values at the certificate boundary; duplicate commitments would collapse the parallel commitment sets and therefore fail closed rather than becoming ambiguous membership. The legacy flat missing-ID view is likewise required to equal the identifier projection of the typed missing-dependency set. It deliberately excludes irrelevant candidate material, so adding unused material must not perturb the closure identity or downstream D6W input identity.

## Canonical certificate encoding

D6X certificates contain maps keyed by typed `SemanticDependencyReferenceV1` values. Those maps are represented as ordered key/value records for commitment hashing rather than as JSON object keys. This is part of the executable D6X-CANON-1 boundary: structured dependency keys remain structured semantic data, and the commitment path never relies on a serializer's treatment of non-string JSON map keys.

The cross-layer Integral fixture also binds its source snapshot identifier to a canonical SHA-256 commitment before D6W consumption. A symbolic snapshot label is not sufficient at the stricter D6W gate.

## Monotonic resolution semantics

Resolution state is monotonic for the purposes of qualification: once a selected dependency has been classified `Stale` by a `CurrentOnly` rule, later traversal bookkeeping MUST NOT downgrade it to `Present`. This is particularly important when the target node is discovered through one edge and dequeued later; traversal order must not erase a stronger blocking condition.

The executable model therefore preserves `Stale` across node dequeue/inclusion, and the final closure status remains `BlockedCurrentness` whenever any selected dependency is stale.

For a `CurrentOnly` edge, currentness is established explicitly from the selected target node and the semantic environment: the node must not be marked `historical_only`, and both the node and environment must carry a frontier root with exact equality. A missing frontier on either side is not evidence of currentness. This remains a D6X policy check rather than a mutation of the upstream D6S node commitment format, preserving upstream compatibility while preventing a mutable frontier annotation from being treated as cryptographically bound merely because the node commitment matches.

Resource truncation is also endpoint-closed: D6X never records a selected edge whose target node cannot enter the selected closure because the node budget is already exhausted. This keeps the certificate's selected-edge view internally closed even for blocked resource results.

## Status semantics

- **Complete** — all required dependencies found and no blocking currentness/resource condition.
- **BlockedMissingDependency** — at least one required dependency is absent.
- **BlockedCurrentness** — a selected dependency is historical where the profile requires current material.
- **BlockedResourceLimit** — deterministic bounds prevent completion.

Status precedence is deterministic: BlockedMissingDependency dominates BlockedCurrentness, which dominates BlockedResourceLimit. A self-consistent certificate cannot relabel a closure with a stronger blocker as a weaker status; source-bound verification additionally re-executes the policy to confirm the complete result.

Cycles are permitted at the graph level. The traversal uses a deterministic visited set, while cycle detection is recorded separately. A graph cycle is not treated as a recursive semantic derivation; recursive fixpoint evaluation remains separately qualified by D6U.

## Downstream reference normalization

D6W derived-layer objects are themselves qualified-consumption artifacts. Their inter-layer references are required to use the canonical 64-hex commitment representation before an object is considered valid. This prevents a self-recommitted opaque identifier from appearing structurally valid at a downstream boundary after the stronger D6X gate has already established canonical source bindings.

## Important boundary

D6X does not re-qualify truth, causality, authority, current-finality, authorization, or actuation. It consumes the qualified projection and existing Mycelix qualifications. Symthaea can propose candidate closures, but serialization of a proposal does not promote it to qualified authority.

Provenance and custody edges are excluded unless the closure profile explicitly selects them. This prevents incidental reachability from becoming semantic dependency. Currentness is likewise evaluated only on edges actually selected by the closure traversal; an unrelated incoming edge cannot make a selected node stale. If multiple rules match the same edge, `CurrentOnly` dominates `Any` deterministically rather than depending on rule iteration order. A dangling edge whose kind/rule does not qualify for traversal is also ignored rather than promoted into a missing semantic dependency.

## D6P provenance boundary

The strict D6X D6P entrypoint is a **qualified-composition** boundary, not an independent reconstruction of D6N/D6O authority.

It verifies that every required D6P receipt:

- is explicitly named by the projection and closure profile;
- matches the supplied receipt commitment exactly;
- matches its supplied D6P composition exactly;
- satisfies the composition's semantic validation;
- and, when requested, is bound to the caller-supplied current frontier, which must equal the semantic environment's declared current frontier.

A strict frontier argument therefore cannot introduce a second notion of "current": the environment and the D6P receipt must agree on the same frontier root before the receipt enters the D6X closure.

This prevents an opaque or historical receipt from silently crossing into D6X, but it does not prove how the supplied composition was originally produced. Authoritative D6N/D6O reconstruction therefore remains an upstream qualification step. D6X must not be described as creating that authority merely because the D6P objects are internally consistent.

### End-to-end corpus qualification

The golden conformance corpus now carries a typed authoritative_d6n_d6o source fixture containing the effect, route, D6N qualification profile, observation set, D6N assessment, D6M evidence, D6O lifecycle profile/ledger, and current-frontier inputs required by the authoritative reconstruction function.

The executable corpus test first calls:

compute_dependency_closure_from_authoritative_d6n_d6o(...)

which internally reconstructs D6P from authoritative D6N/D6O inputs, verifies the supplied D6P receipt as an exact projection of that reconstruction, and only then delegates to the strict D6X D6P admission path. The underlying D6O step requires the admitted eligibility receipt to be the exact receipt registered in the lifecycle ledger; a self-consistent but unregistered receipt is rejected.

The authoritative D6P reconstruction also binds the caller-supplied independent-observer threshold to the `FinalityQualificationProfileV1.required_independent_observations` value. A lower or otherwise substituted threshold is rejected instead of becoming an implicit policy override.

The corpus also mutates D6M, D6N, and D6O inputs into internally self-consistent alternatives and requires the authoritative reconstruction boundary to reject each one. This establishes the intended chain for the ReferenceModelOnly test corpus:

D6M evidence -> D6N assessment -> D6O eligibility -> D6P composition -> D6P receipt -> D6X closure

This mirrors the useful part of SLSA's verification model: expected inputs and parameters should be explicitly checked, and recognized dependencies can be recursively verified rather than accepted merely because a higher-level object is internally consistent. It is an architectural analogy, not a claim of SLSA conformance.

## D6W binding

D6W now requires the exact D6X `closure_identity_commitment` in its input layer:

```
C_input = H(
  source snapshot,
  projection,
  environment,
  dependency closure,
  exact node set,
  exact edge set,
  D6P receipt set,
  claim ceiling
)
```

Therefore:

- changing the closure changes `C_input`;
- changing only irrelevant DKG material or irrelevant D6P receipts leaves the semantic closure identity and `C_input` unchanged;
- a D6P receipt becomes semantically relevant only when the closure profile explicitly requires it;
- a blocked closure remains explicit rather than being silently replaced by a smaller closure;
- downstream derivation/result commitments remain layered above the changed input.

## Adversarial corpus

The reference model currently includes fixtures for:

1. irrelevant DKG material does not change the semantic closure identity (while the audit certificate remains candidate-bound);
2. missing required dependency blocks closure;
3. provenance-only material does not enter a semantic closure;
4. deterministic traversal produces the same commitment repeatedly;
5. selected node commitment changes the closure identity;
6. selected edge commitment changes the closure identity;
7. edge-free complete closures remain consumable by D6W;
8. blocked closures fail closed at the D6W input boundary;
9. irrelevant D6P receipts do not perturb the closure identity;
10. required D6P receipts are explicit closure dependencies.
11. typed dependency references distinguish node, edge, and D6P-receipt domains.
12. the canonical selected dependency set contains exact selected node/edge identities.
13. set insertion order does not alter semantic closure identity.
14. mutating the canonical selected dependency set invalidates the certificate.
15. adding an unselected typed dependency invalidates the certificate.
16. mutating the legacy missing-ID compatibility view invalidates the certificate.
17. selected, missing, and stale dependency resolution states are explicitly bound and validated.
18. changing a selected edge endpoint or edge kind changes semantic closure identity even when the edge ID/commitment is unchanged.
19. dependency-domain structural validation rejects malformed Edge/D6PReceipt references.
20. currentness state cannot create a stale-resolution entry for an unselected/resource-truncated node.
21. an unrelated current-only incoming edge cannot make a selected node stale unless that edge is itself selected.
22. overlapping matching rules resolve deterministically with `CurrentOnly` dominating `Any`.
23. an unmatched dangling edge does not block an otherwise complete closure.
24. resolution evidence may change the audit certificate commitment without changing semantic closure identity.
25. resolution evidence for an unselected dependency invalidates the certificate;
26. selected-dependency observed commitments in resolution evidence must match the dependency commitment when supplied;
27. missing-dependency resolution evidence cannot claim an observed semantic commitment;
28. resolution evidence may omit an observed commitment while retaining independent retrieval/qualification context evidence;
29. the runtime-neutral resolution adapter distinguishes Action, Entry, and External address domains without importing those runtime identifiers into semantic identity;
30. Retrieved attempts require an observed commitment; Historical attempts also require an observed commitment because they identify a concrete stale object; Unavailable attempts may preserve an address but MUST NOT claim an observed semantic commitment.
31. the runtime adapter maps Retrieved/Unavailable/Historical monotonically to Present/Missing/Stale without performing commitment equality checks itself.
32. a lossless runtime-resolution evidence envelope round-trips address domain, outcome, address, and observations while leaving closure identity unchanged.
33. the typed runtime-resolution envelope rejects drift between its typed attempt and legacy opaque evidence representation.
34. the D6X closure identity explicitly binds the canonicalization contract version rather than relying only on an implicit implementation dependency.
35. the candidate-bound closure certificate commitment is mutation-tested across every semantic certificate field; only the stored commitment itself is excluded from its own preimage.
36. duplicate selected node commitments fail closed at the D6X constructor boundary.
37. duplicate selected edge commitments fail closed at the D6X constructor boundary.
38. self-consistent status substitutions that violate missing/currentness precedence fail D6X certificate validation.
39. a selected CurrentOnly dependency with a mismatched node frontier is Stale and blocks closure.
40. a selected CurrentOnly dependency with omitted frontier metadata is Stale and blocks closure.
41. a selected Any dependency remains Present when its node frontier differs from the environment frontier.
42. node-budget truncation cannot leave a selected edge pointing at an unselected target node.
43. the machine-readable D6X golden corpus freezes closure and certificate commitments for baseline, edge-free, missing, cyclic, currentness, and resource cases.

The machine-readable corpus is at `testdata/d6x_qualified_closure_golden_vectors.json`. Its commitment values were independently reconstructed from the exact D6X-CANON-1 reference encoding and cross-checked against the existing D6S golden corpus. The corpus is self-contained: it carries baseline typed inputs plus declarative mutation recipes so another implementation can reproduce the fixtures without depending on Rust test-helper names. Rust reference-model execution has not yet been run. Before interoperability or production claims, execute the Rust/WASM/Holochain conformance corpus and reconcile its emitted values against these vectors.

Claim ceiling: **ReferenceModelOnly**.

## Identity/evidence boundary

D6X semantic dependency identity is intentionally kept independent of runtime retrieval evidence. The certificate may carry optional `SemanticDependencyResolutionEvidenceV1` keyed only by selected/missing dependencies; this is audit/provenance material and participates in the certificate commitment, but is excluded from `closure_identity_commitment`. A dependency reference says **what semantic object is required**; its resolution state says whether the closure selected it as present, missing, or stale under the named profile. Future Holochain addresses, retrieval receipts, validator observations, and retry metadata should be represented as resolution evidence rather than silently incorporated into the semantic dependency identity. This lets retrieval evidence vary while preserving the semantic dependency identity. Evidence is nevertheless not unconstrained: it must reference a selected or missing dependency, a supplied observed commitment must agree with the semantic dependency commitment for `Present`/`Stale` resolutions, and `Missing` resolutions cannot claim observed semantic content. Retrieval references and qualification-context commitments remain independent audit fields. Strict evidence-backed verification now additionally computes a canonical `resolution_evidence_commitment` (versioned as `D6X-RE-1`) over the exact dependency/evidence map, so the summary identifies the precise audit set it summarizes without making that evidence part of semantic `closure_identity_commitment`. Consumers can use the combined context-bound predicate rather than manually composing evidence-completeness and context checks.

This separation matches the architectural direction suggested by Holochain's validation model: dependencies used for deterministic validation need addressable retrieval, and unavailable dependencies are represented as unresolved so validation can be retried. D6X remains a reference-model analogue, not a claim of runtime equivalence. 


## Verification-summary result semantics

D6XVerificationSummaryV1::valid() answers whether a verification-summary record is structurally coherent and commitment-consistent. A structurally valid summary may carry either `Passed` or `Failed` as its verification result.

Downstream qualification is a stronger question. `is_passed()` explicitly exposes the acceptance-relevant result check, while `verifies_against_expectations(...)` combines that requirement with the expected verifier identity, policy commitment, source bindings, closure scope, and source-bound closure correspondence. The strict evidence-backed verifier adds complete resolution evidence, a single qualification context, and the exact resolution-evidence commitment.

This separation mirrors the current SLSA v1.2 VSA model, where `verificationResult` may be `PASSED` or `FAILED`, while consumer verification separately requires `PASSED` when the artifact is to be accepted. D6X adopts the semantic distinction only; it remains ReferenceModelOnly and unsigned.

## Runtime resolution adapter boundary

The reference model now exposes a separate d6x_resolution_adapter layer. It defines audit-only address and attempt domains:

- Action — suitable for a runtime mapping such as Holochain ActionHash;
- Entry — suitable for a runtime mapping such as Holochain EntryHash;
- External — suitable for a runtime mapping such as Holochain ExternalHash.

These are deliberately adapter concepts, not additional semantic dependency kinds. The adapter can record whether a retrieval was Retrieved, Unavailable, or Historical, and can carry observed and qualification-context commitments. It converts into the existing opaque D6X resolution-evidence payload without changing closure_identity_commitment. Because that legacy evidence payload intentionally omits address-domain and outcome tags, the adapter now also exposes a lossless audit envelope carrying the original attempt alongside the derived D6X evidence. This prevents an adapter from silently collapsing Action/Entry/External or Retrieved/Unavailable/Historical distinctions. The envelope is audit-only and does not enter semantic closure identity. The envelope also validates that its typed attempt and derived opaque evidence agree, preventing contradictory audit representations from being serialized as if they were one retrieval event.

This boundary matters because Holochain distinguishes action, entry, and external identifiers, and its validation model treats unavailable deterministic dependencies as unresolved rather than as ordinary validation failure. D6X therefore keeps the runtime address and retrieval outcome in audit evidence while leaving the semantic closure to describe only the dependency itself and its profile-derived resolution state. Holochain-specific runtime behavior remains outside the ReferenceModelOnly contract.
