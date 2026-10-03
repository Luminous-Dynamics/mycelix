# Mobility evidence disposition transition semantics v1

This document defines the append-only transition layer for epistemic disposition.

## Separate dimensions

A mobility evidence record carries three independent dimensions:

1. **Event interval** — when the evidence-generating event occurred.
2. **Effectivity interval** — when the evidence was asserted applicable to the exact configuration/artifact binding.
3. **Disposition** — the currently documented epistemic state of that evidence.

A disposition transition never edits either temporal interval.

## Transition rules

- The initial disposition transition starts from `Active`.
- Every later transition names exactly one explicit predecessor.
- Every transition names an addressable typed basis witness.
- `Active` may transition to `Disputed`, `Superseded`, `Retracted`, or `Unresolved`.
- `Disputed` may resolve to `Active`, or transition to `Superseded`, `Retracted`, or `Unresolved`.
- `Unresolved` may transition to `Active`, `Disputed`, `Superseded`, or `Retracted`.
- `Superseded` and `Retracted` are terminal.
- Same-state transitions are rejected.
- Self-referential predecessors and untyped witnesses are rejected.

A missing predecessor or other external dependency is a dependency-resolution problem at the protocol layer; it must not be silently interpreted as a negative epistemic finding.

## Concurrency

Two independently authored transitions may describe competing branches from the same predecessor. This model does not silently collapse those branches. Reconciliation must itself be explicit and addressable.

## Scope boundaries

These transitions do not establish truth, safety, certification, regulatory approval, causal correctness, or engineering authority. They encode provenance and documented epistemic state only.

The deterministic dependency model is intentionally compatible with Holochain validation: addressable dependencies can be validated deterministically, while unavailable dependencies remain unresolved. Holochain's source-chain model is itself append-only and explicitly links records through prior history.

For lifecycle traceability, this is consistent with NIST work describing digital threads as explicit associations across lifecycle stages and temporal alignment of execution data.

## Graph validation

Each immutable transition has its own typed `transition_id`. Graph validation resolves predecessor references by that exact identity, checks evidence identity continuity and requires each child's `from` state to equal its predecessor's `to` state. Duplicate transition identities and multiple genesis assertions for one evidence event are invalid. A closed predecessor cycle is also invalid: every complete finite branch must bottom out at a single genesis assertion rather than looping through otherwise valid transition records.

A missing predecessor is returned as an unresolved dependency, not a negative evidence judgment. When multiple children reference one predecessor, the graph assessment reports the branch point and retains every branch; it does not select a winning state. A separate, explicit reconciliation record is required before a consumer may treat the competing branches as reconciled. This graph check validates only the supplied dependency set; callers must retrieve the complete referenced records by address before treating the assessment as complete.

The canonical assessment also exposes structural failures through the typed `EvidenceDispositionCoverageValidationError::Structural` error boundary. Dependency absence is deliberately not encoded as this error: the assessment returns `Unresolved` with explicit missing identities. This keeps definitive structural contradiction separate from dependency availability and lets an eventual Holochain callback map the two outcomes directly to `Invalid` and `UnresolvedDependencies`.

## Append operation boundary

The transition graph can also be qualified as an explicit append operation: `validate_append` first validates the existing graph, then validates the candidate transition, rejects any candidate whose exact `transition_id` is already present, and revalidates the combined graph. This makes record identity the immutability boundary for an append: a new transition may extend an existing branch or create a new branch, but it cannot replace an already-addressed transition.

A candidate that names an unavailable predecessor remains `Unresolved` rather than being accepted as complete. If a later append supplies that previously missing predecessor under its exact referenced identity, the combined graph may become `Complete` without rewriting the already-addressed child; this is still additive record publication. A candidate with a present predecessor must satisfy the same evidence and state-continuity rules as ordinary graph validation. This API does not claim that a storage layer can physically prevent mutation of bytes already persisted; it provides the semantic append/replacement boundary that a storage or Holochain layer can enforce against addressable records.

## Bounded transition dependency walk

The canonical composed coverage assessment resolves each named branch head with one deterministic predecessor walk that stops at the declared reconciliation branch point. The walk returns the exact visible ancestry, any first unavailable predecessor, and the validated transition records reached within that boundary.

The same walk result is used for both dependency collection and ancestor/descendant contradiction detection. A referenced predecessor identity is retained in the visible ancestry even when that predecessor record is unavailable, so a present descendant can still establish a nested named-head contradiction. The overall assessment remains unresolved for the missing record, but the known structural contradiction retains precedence. This keeps unresolved-dependency handling and competing-head semantics aligned: an unavailable record remains unresolved, while a relationship that is already visible inside the bounded cone remains structurally rejectable. No traversal is performed beyond the declared branch point.


## Explicit branch reconciliation

A competing-branch condition is not resolved by selecting the first, latest, or locally observed branch. `EvidenceDispositionReconciliation` is an explicit addressable witness that names the exact evidence event, branch point, at least two competing branch heads, an addressable authority witness, and any addressable basis witnesses. Graph validation verifies that each named branch head descends from the declared branch point and concerns the same evidence event.

The reconciliation witness deliberately contains no implicit winner field. It records that a set of branches was explicitly reconciled and who/what supplied the authority and basis; any resulting disposition assertion remains a separate transition record. Missing branch ancestry remains unresolved rather than being interpreted as a negative finding.


### Competing means incomparable

A reconciliation branch head must be a proper descendant of the declared branch point. Two named heads must also be incomparable in the predecessor graph: neither may be an ancestor of the other. Otherwise the pair describes one branch continuing forward rather than competing branches. The reconciliation validator rejects a branch point used as a head and rejects nested head pairs.


### Bounded reconciliation coverage

A reconciliation may carry a separate coverage witness that names the exact set of branch heads examined under an explicit addressable boundary. The coverage witness must include every branch head named by the reconciliation and must validate those heads against the same supplied predecessor graph. Branch-head collections are sets, so their serialization order is not semantically significant.

The covered heads must themselves be pairwise incomparable descendants of the declared branch point. Coverage matching uses set equality for branch heads and basis witnesses; ordering in serialized vectors does not change the represented set. A nested ancestor/descendant pair is rejected because it describes one continuing branch rather than two covered heads. The canonical bounded assessor resolves every explicitly covered head named by the coverage boundary, including additional heads beyond the reconciliation's minimum head set, so the declared coverage boundary cannot silently widen or narrow the transition dependency cone.

This is intentionally a **bounded coverage claim**, not a global DHT enumeration claim. The boundary and basis witnesses are addressable provenance objects. The validator does not inspect mutable link collections or infer that an unmentioned branch does not exist. Consequently, coverage can establish consistency with the declared dependency set and declared boundary, while global completeness remains outside this structural validator.


### Coverage authority and reconciliation binding

The coverage boundary is explicitly scoped to one reconciliation witness and carries the same authority witness as that reconciliation. Coverage therefore cannot be detached from the reconciliation it claims to cover, nor can a different authority or authority-scope witness be silently substituted at the boundary layer. This is a structural provenance binding, not an assertion that the authority is objectively correct, certified, or globally entitled.

The boundary remains addressable and finite: it names the exact evidence event, branch point, branch heads, authority, and basis used for the bounded claim. The validator still makes no claim that unmentioned mutable links or globally undiscovered records do not exist.


The reconciliation basis is the minimum provenance basis for the bounded coverage boundary: every reconciliation basis witness MUST be carried into the boundary basis. The boundary MAY add coverage-specific basis witnesses. This is deliberately an inclusion relationship rather than exact equality: reconciliation establishes why the competing branch set was reconciled, while additional boundary witnesses may justify why the explicitly examined dependency boundary is sufficient for the bounded coverage assertion. Basis omission is invalid; additional addressable basis is permitted.

### Explicit authority scope

An `EvidenceDispositionAuthorityScope` is now a separate addressable witness that binds the exact reconciliation subject to the exact authority witness. The reconciliation references that scope identity, and graph validation requires the supplied scope witness to match both the reconciliation identity and authority. The scope basis must also contain every reconciliation basis witness, while permitting additional scope-specific provenance.

This prevents an authority witness from being structurally reused for an unrelated reconciliation merely because both identities are individually well-typed. It still does **not** establish real-world institutional authority, delegation legitimacy, certification, or entitlement; those remain external claims that must be represented by appropriate evidence.

The authority-scope witness is also carried through coverage validation: the boundary must name the exact same scope identity as the reconciliation, and the supplied scope witness must bind that identity to the same authority and reconciliation subject. This closes the remaining structural gap between naming an authority and naming the scope in which that authority witness is being relied upon.

Consumers that stop at the scope layer can use the chain-aware scope validator to validate the exact named delegation chain in the same bounded operation. The same scope-layer validator preserves an absent named delegation target as unresolved. A present target remains subject to exact scope binding checks even when its predecessor chain is incomplete; structural contradictions therefore retain precedence over unresolved ancestry. A missing target is a dependency-resolution state, not a structural contradiction merely because the secondary exact-binding witness cannot yet be retrieved. This prevents a correctly bound scope from being mistaken for evidence that its delegation ancestry is complete; missing predecessors remain unresolved and reachable structural contradictions remain invalid. The scope-layer validator therefore exposes the same Complete/Unresolved distinction as the delegation graph validator rather than collapsing dependency absence into success.

The same isolation rule applies at the scope consumer boundary: unrelated invalid delegation records are outside the named chain and do not affect the result. At the composed coverage layer, however, an unresolved predecessor on the exact named delegation chain propagates as unresolved rather than being converted into a successful coverage qualification.


### Authority delegation chains

Both composed coverage validators treat the explicitly supplied delegation witness as the direct target record. The additional delegation slice supplies predecessor dependencies rather than requiring the target to be duplicated there. If that slice also contains the target identity, the record must be structurally identical to the direct witness; a conflicting same-identity record is rejected. This avoids treating the current operation's own record as an unavailable DHT dependency while preserving exact-identity conflict detection.

An `EvidenceDispositionAuthorityDelegation` may reference an exact predecessor delegation. When a predecessor is supplied, the delegated subject MUST remain identical and the predecessor grantee MUST equal the current grantor. This makes the delegation edge explicit rather than allowing an authority to appear from an unrelated witness.

Delegation graph validation is finite and address-based:

- duplicate delegation identities are invalid;
- a missing predecessor is unresolved rather than treated as a negative authority finding;
- predecessor subject changes are invalid;
- predecessor grantee/current grantor discontinuity is invalid;
- closed predecessor cycles are invalid because they have no historical root;
- multiple independent root delegations may coexist and are retained rather than collapsed.

The scope witness must reference the exact delegation identity it relies upon. Reconciliation and coverage witnesses must carry that same delegation identity through their authority bindings. This closes the structural gap where a correctly typed delegation could otherwise be silently substituted for the delegation actually named by a scope.

These rules describe provenance continuity only. They do not prove that a grantor possesses real-world legal or institutional authority, that a delegation is legally effective, or that a subject is otherwise entitled to act. They also do not claim that the supplied delegation graph is globally complete. This bounded treatment follows Holochain's validation model: addressable dependencies can be deterministically checked, while unavailable dependencies remain unresolved rather than becoming inferred negative facts.

Delegation provenance is also monotonic across predecessor edges: every basis witness carried by a predecessor MUST remain present in the child delegation basis. A delegated authority may add provenance, but it may not silently discard the provenance on which the inherited delegation depends. This is a provenance-continuity rule, not a claim that any basis witness is objectively authoritative.

The inclusion rule is asymmetric by design: an inherited basis set is a minimum carried-forward set, not an exact child schema. A child delegation may add new addressable witnesses while retaining every predecessor witness. This permits provenance accumulation without permitting silent provenance loss.

A consumer that needs one named delegation chain can use target-scoped validation instead of treating unrelated delegation records as dependencies of that chain. The target-scoped check follows only the named delegation and its explicit predecessors; unrelated delegation records are not validated as dependencies of that chain. Therefore unrelated missing or invalid records do not contaminate the target result. A missing predecessor on the named chain remains unresolved, while cycles remain invalid.

The composed coverage validator reuses this target-scoped authority-chain check after validating the bounded reconciliation, scope, boundary, and transition graph. This gives a single bounded qualification entry point for consumers that need both coverage integrity and the exact delegation provenance used by that coverage. It is isolated to the named delegation chain, so unrelated malformed delegation records do not contaminate an otherwise valid bounded coverage qualification.

The same composed path permits provenance accumulation without loss: delegation basis may extend, scope basis may carry inherited scope provenance, reconciliation basis may identify its required minimum, and the coverage boundary may add boundary-specific witnesses. Each inclusion is checked at its corresponding layer. The boundary must also carry every witness in the authority-scope basis, so scope-specific provenance cannot disappear between authority qualification and bounded coverage.


### Dependency absence at the target boundary

Target-scoped delegation validation treats the named delegation record itself as an addressable dependency. When that target record is absent from the supplied dependency set, validation returns `Unresolved` rather than `Err`. This keeps a missing target semantically distinct from a present-but-malformed delegation and preserves the protocol-layer distinction between unavailable evidence and invalid evidence.

### Composed bounded qualification assessment

The canonical composed coverage entry point, `validate_against_graph_and_authority_chain_assessment`, exposes the same distinction across the entire bounded dependency path. It first validates structural identities and cross-layer bindings, then resolves the exact named transition ancestry. A missing branch point, branch head, or predecessor is returned as `Unresolved`; present-but-contradictory transitions remain invalid. This ordering is deliberate: dependency absence cannot mask a structural contradiction already present in the supplied records.

When transition dependencies are complete, the same assessment validates the exact named delegation chain and propagates delegation dependency absence as `Unresolved`. If both transition and authority dependencies are unavailable, their missing addresses are accumulated and deduplicated deterministically. A structurally invalid reachable authority chain remains `Err` even when transition dependencies are also missing. Unrelated delegation records remain outside the named authority-chain dependency boundary.

The canonical bounded assessment is target-scoped on the transition side as well: it validates the declared reconciliation branch point, declared branch heads, and only their reachable predecessor records. Unrelated malformed transition records therefore do not contaminate an otherwise valid bounded assessment. The legacy whole-supplied-graph validator remains available for callers that explicitly require whole-set validation. The canonical bounded result is also invariant to the serialization/order of the supplied transition records. The traversal stops at the declared reconciliation branch point; ancestors before that boundary are not dependencies of the bounded claim. A supplied branch head that cannot reach the declared boundary is structurally invalid when the failure is fully observable, while a missing record needed to establish the path remains unresolved.

### Authority validity is a separate dimension

A complete delegation chain establishes structural provenance for the named authority relationship; it does not establish that the delegation is currently effective. This model deliberately does not infer current validity from local timestamps, record recency, or the mere existence of a delegation chain. Holochain's validation guidance notes that action timestamps come from an author's local clock and can be forged, and identifies signed validity/revocation evidence from a trusted authority as an appropriate pattern when a privilege may be revoked. Therefore any future validity, expiry, suspension, or revocation semantics should be represented by separate addressable witnesses with their own deterministic validation rules, rather than by interpreting this chain as a mutable "current authority" lookup.

### Shared qualification-status algebra

The disposition validators share a common protocol-status algebra in the standalone qualification crate:

- `QualificationStatus::Complete(T)` means the supplied named dependency set was sufficient for the bounded qualification.
- `QualificationStatus::Unresolved { missing, partial }` means qualification cannot yet finish because one or more addressable dependencies are unavailable; the deterministic partial result remains available without interpreting absence as a negative finding.
- `QualificationStatus::require_missing(...)` composes additional unavailable dependency paths into the same status, preserving the partial result and canonicalizing the union of missing identities.
- `QualificationOutcome::require_missing(...)` provides the same composition at the `Result` boundary while preserving structural errors unchanged; this keeps future protocol adapters from accidentally converting semantic invalidity into unresolved state.
- `QualificationStatus::zip(...)` composes independent qualification paths as a deterministic product: both partial payloads are retained, while missing dependencies are unioned and canonicalized.
- Dependency accumulation is partition-invariant: adding missing identities incrementally or as one batch yields the same canonical unresolved set, and `zip(...)` preserves the same missing-set law across different groupings.
- `QualificationDecision<T>` is the runtime-neutral adapter boundary: `Valid(T)`, `Invalid { reason }`, and `Unresolved { missing, partial }` are mutually exclusive and contain no Holochain-specific types. This keeps the pure qualification crate portable while giving a future integrity-zome adapter a direct three-outcome mapping.
- The intended Holochain adapter mapping is direct: `Valid(T)` → `ValidateCallbackResult::Valid`, `Invalid { reason }` → `ValidateCallbackResult::Invalid(reason)`, and `Unresolved { missing, partial }` → `ValidateCallbackResult::UnresolvedDependencies(...)`; host/runtime failures remain outside this semantic result algebra.
- The unresolved identities exposed by the pure layer are logical engineering identifiers, not Holochain hashes. The runtime adapter must perform an explicit identity-to-address binding before constructing `UnresolvedDependencies`; no implicit string/hash interpretation is permitted.
- `QualificationValidationError::Structural { reason }` means the supplied records contain a definitive structural contradiction.

Transition-graph, authority-delegation, and composed-coverage validators expose adapters into this same algebra. Their existing `Result<_, String>` entry points remain available, and the existing coverage typed-error name is retained as a compatibility alias to the shared structural-error type.

This is intentionally an internal protocol algebra, not a Holochain type dependency. An eventual integrity-zome adapter can map `Complete` to `Valid`, `Structural` to `Invalid`, and `Unresolved` to `UnresolvedDependencies` at the Holochain boundary, while keeping the qualification crate dependency-free. Holochain's current validation documentation defines those three semantic outcomes and reserves `Err` for true host/runtime failures rather than semantic invalidity.
