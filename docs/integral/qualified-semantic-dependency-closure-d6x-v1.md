# Integral D6X — Qualified Semantic Dependency Closure

Status: **ReferenceModelOnly**

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
- D6X algorithm version;
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

The semantic identity binds the exact selected node-id → node-commitment mapping and selected edge-id → endpoint/kind/commitment mapping. Selected and missing dependencies are represented through a typed reference algebra (`Node`, `Edge`, `D6PReceipt`). The selected typed set is the canonical semantic dependency set; the parallel node/edge/D6P collections remain explicit compatibility/audit views. Certificate validation reconstructs the canonical selected set from those views and rejects omission, injection, or commitment/type drift. The legacy flat missing-ID view is likewise required to equal the identifier projection of the typed missing-dependency set. It deliberately excludes irrelevant candidate material, so adding unused material must not perturb the closure identity or downstream D6W input identity.

## Status semantics

- **Complete** — all required dependencies found and no blocking currentness/resource condition.
- **BlockedMissingDependency** — at least one required dependency is absent.
- **BlockedCurrentness** — a selected dependency is historical where the profile requires current material.
- **BlockedResourceLimit** — deterministic bounds prevent completion.

Cycles are permitted at the graph level. The traversal uses a deterministic visited set, while cycle detection is recorded separately. A graph cycle is not treated as a recursive semantic derivation; recursive fixpoint evaluation remains separately qualified by D6U.

## Important boundary

D6X does not re-qualify truth, causality, authority, current-finality, authorization, or actuation. It consumes the qualified projection and existing Mycelix qualifications. Symthaea can propose candidate closures, but serialization of a proposal does not promote it to qualified authority.

Provenance and custody edges are excluded unless the closure profile explicitly selects them. This prevents incidental reachability from becoming semantic dependency.

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

Before interoperability or production claims, add cross-language golden vectors, currentness/D6P fixtures, contradiction-preservation fixtures, cycle fixtures, resource-limit fixtures, and execute the Rust/WASM/Holochain conformance corpus.

Claim ceiling: **ReferenceModelOnly**.
