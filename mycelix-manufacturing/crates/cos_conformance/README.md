# COS conformance harness

This crate is the executable companion to Mycelix issues #3332/#3333/#3334 and
`docs/integral/cos-reference-node-v1.md`.

It tests semantic evidence boundaries only. It deliberately does not execute
manufacturing, Holochain, Integral governance, ITC policy, or FRS operations.

## Run

`cargo test -p cos_conformance`

## Report

The library exposes `conformance_report_json()` for machine-readable export.
The report contains the corpus identity, formal-obligation mappings, and claim
ceiling. It is intentionally not a scalar verification score.

## Claim ceiling

A passing suite establishes only that this reference model rejects the specified
semantic collapses and accepts their explicitly bound counterparts. It does not
establish physical productivity, safety, qualification, economic/ecological
outcomes, or Integral validation.

## Heterogeneous federation

`federation.rs` provides the deterministic reference oracle for Integral/Mycelix heterogeneous federation. It preserves local-vs-foreign authority, logical delivery identity, schema/authorization generations, causal dependencies, reconnect idempotence, conflicting observations, and privacy-minimized projections. See `docs/integral/heterogeneous-federation-reference-v1.md`.

## Branch reconciliation

`federation_reconciliation.rs` extends the federation oracle with explicit branch identity, frontier closure, compatibility classification, conflict-preserving reconciliation, capacity double-spend detection, authority-validity fencing, and branch-aware cockpit projections. See `docs/integral/heterogeneous-federation-reconciliation-v1.md`.

## Identity and alias integrity

`federation_identity.rs` keeps identifiers, credentials, principals, accounts, devices, resources, locators, and entities distinct. It provides scoped equivalence classes, typed substitution profiles, append-only lifecycle events, resource-capacity alias checks, privacy projections, and deterministic identity-resolution witnesses. See `docs/integral/semantic-identity-alias-integrity-v1.md`.

## Causal time

`federation_causal_time.rs` separates causal ancestry from wall-clock observations, distinguishes concurrent from incomparable histories, models bounded clock uncertainty, evaluates profile-bound freshness, and requires explicit revalidation for long-offline branches. See `docs/integral/causal-time-long-lived-federation-v1.md`.

## No-resurrection integrity

`no_resurrection.rs` models first-class tombstones, semantic generations, explicit successor/reactivation, resource/authority conservation across generations, cache invalidation, compaction preservation, and branch lifecycle reconciliation. See `docs/integral/tombstone-generation-no-resurrection-v1.md`.

## Stable frontier and safe reclamation

`stable_frontier.rs` models closed authority-bearing membership, explicit frontier coverage, stable-frontier certificates, retention boundaries, pruning receipts, cold-start reconstruction, rejoin fencing, and conservation of authority/capacity/consent claims across reclamation. See `docs/integral/stable-frontier-safe-history-reclamation-v1.md`.

## Semantic archive continuity

`archive_continuity.rs` models historical-evidence profiles, archive manifests, explicit archive/frontier continuity, profile and membership transitions, historical-claim ceilings, contested archive sets, and reconstruction gates. Archives can support historical analysis or cold-start reconstruction but cannot become current authority, actuation, or policy authority by themselves. See `docs/integral/semantic-archive-continuity-v1.md`.

## Semantic recovery and provider substitution

`substitution_continuity.rs` binds provider routes to one stable Mycelix semantic effect, exact request/resource/tenant/amount/unit/authority/consent semantics, provider-profile allow-lists, lifecycle generation, and explicit route succession. Unknown or pending outcomes block independent failover unless an exact outcome-resolution witness or contract-wide idempotency witness qualifies continuation. Provider operation IDs remain provider-scoped and cannot replace the semantic effect ID. Archive recovery is exposed as reconstruction input only; it cannot authorize current provider execution. See `docs/integral/semantic-recovery-provider-substitution-v1.md`.


## External-effect finality and compensation lineage

`effect_finality.rs` separates provider-reported outcomes from independently qualified external-state evidence. Finality receipts bind the exact effect, route, provider operation, provider profile, outcome, observation frontier, and semantic environment. Provider-only assertions, unresolved external state, stale current-finality observations, lifecycle tombstones, and mismatched evidence fail closed.

Reversal, refund, remediation, and correction are modeled as new semantic effects with explicit causal links. The predecessor effect remains immutable. Compensation coverage is explicit and cannot overlap conserved capacity already compensated in the reference ledger; new capacity creation/release remains a separate semantic transition.

Archive recovery remains historical reconstruction input only. An archive cannot establish current external finality or current actuation authorization, even when its source frontier equals the current frontier. See `docs/integral/external-effect-finality-compensation-v1.md`.


## Contestable multi-observer external finality

`contestable_finality.rs` models observer independence, dependent/correlated evidence, explicit observation-set qualification, contested external state, and qualified finality resolution. Distinct observer IDs never imply independent evidence: shared evidence/custody/upstream roots are conservatively treated as dependent. Provider self-report, archive mirrors, stale observations, lifecycle tombstones, and unresolved contradictions cannot manufacture current finality.

Finality resolution is a new semantic evidence transition. It preserves contradictory observations, rejects arrival-order/majority heuristics, and cannot authorize actuation or mutate the predecessor effect. See `docs/integral/contestable-multi-observer-finality-v1.md`.

## Observer and evidence lifecycle continuity

`observer_lifecycle.rs` extends D6N with time-scoped observer generations, append-only lifecycle status transitions, immutable dependency snapshots, explicit rotation continuity, and generation-bound evidence eligibility.

The model preserves:

- observer independence as temporal rather than permanent;
- logical frontier sequence/root rather than wall-clock authority;
- historical evidence after later suspension, revocation, or rotation;
- explicit predecessor/successor generation continuity;
- dependency-root changes as immutable snapshots rather than generation mutation;
- current-finality revalidation against the current continuous observer generation and dependency snapshot;
- D6N `ObservationClassificationV1` as the conflict taxonomy;
- archive evidence as historical-only;
- lifecycle proposals/receipts as non-authoritative evidence;
- explicit non-authority for actuation, authority, capacity, and consent.

Out-of-order lifecycle transitions and dependency snapshots may be recorded as temporarily incomplete; status/currentness queries fail closed until the missing predecessor arrives. Same-frontier divergent transitions are conflicts, and a later-arriving record never wins by delivery order. See `docs/integral/observer-evidence-lifecycle-continuity-v1.md`.

Claim ceiling: **ReferenceModelOnly**. This does not establish real-world revocation, observer trust, key authenticity, physical observation correctness, distributed consensus, durable storage, production finality, or actuation safety.


## Evidence bundles and claim-graph closure

`evidence_claim_graph.rs` gives the evidence chain an explicit typed graph: Source -> Evidence -> Statement -> Registration Receipt -> Validation -> Assessment -> Conclusion -> Human Disposition.

The model enforces endpoint/type compatibility, rejects dangling edges and semantic cycles, and distinguishes structural closure from evidentiary sufficiency. Graph reachability is never treated as truth. Provenance and custody edges cannot become causal support, registration cannot become endorsement, and human disposition cannot become evidence.

Historical-only nodes block current reuse. Symthaea graph proposals remain non-authoritative; conclusion assessment cannot authorize actuation. See `docs/integral/evidence-bundle-claim-graph-v1.md`.

Claim ceiling: **ReferenceModelOnly**. This does not establish source truth, causal validity, cryptographic authenticity, legal authority, production finality, or actuation safety.


## Evidence bundles and claim-graph closure

D6Q adds `evidence_claim_graph.rs`, a reference-model envelope for the typed chain:

`Source -> Evidence -> Statement -> Registration Receipt -> Validation -> Assessment -> Conclusion -> Human Disposition`.

The graph enforces endpoint compatibility, rejects dangling edges and semantic cycles, and keeps structural closure separate from evidentiary sufficiency. Reachability is not truth; provenance/custody are not causal support; registration is not endorsement; human disposition is not evidence; and conclusion is not authorization.

D6Q remains `ReferenceModelOnly` and does not establish source truth, causality, cryptographic authenticity, legal authority, production finality, or actuation safety. See `docs/integral/evidence-bundle-claim-graph-v1.md`.
