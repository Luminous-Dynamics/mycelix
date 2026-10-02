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


## Explicit branch reconciliation

A competing-branch condition is not resolved by selecting the first, latest, or locally observed branch. `EvidenceDispositionReconciliation` is an explicit addressable witness that names the exact evidence event, branch point, at least two competing branch heads, an addressable authority witness, and any addressable basis witnesses. Graph validation verifies that each named branch head descends from the declared branch point and concerns the same evidence event.

The reconciliation witness deliberately contains no implicit winner field. It records that a set of branches was explicitly reconciled and who/what supplied the authority and basis; any resulting disposition assertion remains a separate transition record. Missing branch ancestry remains unresolved rather than being interpreted as a negative finding.


### Competing means incomparable

A reconciliation branch head must be a proper descendant of the declared branch point. Two named heads must also be incomparable in the predecessor graph: neither may be an ancestor of the other. Otherwise the pair describes one branch continuing forward rather than competing branches. The reconciliation validator rejects a branch point used as a head and rejects nested head pairs.


### Bounded reconciliation coverage

A reconciliation may carry a separate coverage witness that names the exact branch heads examined under an explicit addressable boundary. The coverage witness must include every branch head named by the reconciliation and must validate those heads against the same supplied predecessor graph.

The covered heads must themselves be pairwise incomparable descendants of the declared branch point. A nested ancestor/descendant pair is rejected because it describes one continuing branch rather than two covered heads.

This is intentionally a **bounded coverage claim**, not a global DHT enumeration claim. The boundary and basis witnesses are addressable provenance objects. The validator does not inspect mutable link collections or infer that an unmentioned branch does not exist. Consequently, coverage can establish consistency with the declared dependency set and declared boundary, while global completeness remains outside this structural validator.
