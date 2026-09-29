# Evidence bundle and claim-graph closure v1

Status: **ReferenceModelOnly**

D6P makes the evidence chain explicit without allowing graph structure to manufacture semantic truth.

The reference chain is:

    Source -> Evidence -> Statement -> Registration Receipt -> Validation -> Assessment -> Conclusion -> Human Disposition

Each edge has a declared semantic meaning and endpoint type. Graph reachability is weaker than evidentiary support.

## Semantic boundaries

D6P preserves:

    source != evidence
    evidence != statement
    statement != truth
    registration receipt != endorsement
    validation != conclusion
    conclusion != authorization
    human disposition != evidence
    provenance != causality
    graph reachability != semantic validity
    complete bundle != true proposition

A complete graph is therefore a representation of claim lineage, not a truth oracle.

## Typed nodes

`ClaimGraphNodeV1` represents one immutable semantic artifact. Node kinds are Source, Evidence, Statement, RegistrationReceipt, Validation, Assessment, Conclusion, and HumanDisposition.

Each node carries an opaque content commitment, optional provenance/custody roots, an explicit historical-only flag, optional frontier information, and the reference-model claim ceiling.

## Typed edges

`ClaimGraphEdgeV1` supports `DerivedFrom`, `Supports`, `Contradicts`, `Registers`, `Validates`, `Assesses`, `Concludes`, `Disposes`, `Provenance`, and `Custody`. Endpoint compatibility is explicit.

A source cannot directly support a conclusion merely because both are nodes in the same bundle. Registration only connects a statement to a registration receipt; disposition only connects a conclusion to human disposition.

## Structural closure

A bundle is structurally closed only when node identities, commitments, edge identities, endpoints, endpoint types, and claim ceilings are valid and the graph has no cycle.

The cycle rule is conservative: a cycle could permit semantic self-support, so it is rejected rather than interpreted.

Structural closure is not evidentiary sufficiency.

## Evidence binding

A conclusion becomes `SupportedByBoundEvidence` only when an explicit `Supports` edge binds evidence/assessment to that conclusion and the supporting node carries a current frontier binding.

A path such as source -> evidence -> statement -> receipt -> validation -> assessment -> conclusion does not by itself constitute support.

Likewise provenance and custody edges cannot become causal support.

## Conflict preservation

A `Contradicts` edge into a conclusion produces `DisputedByBoundEvidence` rather than selecting a winner. Conflict is represented instead of erased by insertion order, traversal order, node count, or human disposition.

D6P does not introduce a second conflict-resolution taxonomy; D6N/D6O remain responsible for qualified observer/finality transitions.

## Historical boundary

A bundle containing historical-only material remains historical. A matching frontier does not turn historical material into current evidence.

## Human disposition boundary

Human disposition is a governance fact represented as its own node. It is not evidence, corroboration, validation, a cryptographic proof, or automatic authorization.

## Symthaea boundary

Symthaea may analyze graphs and propose missing-node reports, cycle reports, provenance inconsistencies, conflicts, and revalidation targets. It cannot convert reachability into truth, manufacture evidence, promote validation into conclusion, or authorize actuation.

Mycelix remains the semantic root. Xenia remains cryptographic mechanism/verification.

## Deterministic adversarial corpus

The module tests structural closure, dangling edges, provenance/support separation, reachability without support, contradictory evidence, historical preservation, endpoint mismatch, cycle rejection, current-frontier requirements, missing nodes, disposition/evidence separation, conclusion/actuation separation, Symthaea non-authority, registration/endorsement separation, legacy negative/positive cases, formal-obligation coverage, and claim-bounded reporting.

These are authored source-level witnesses until executed by functioning CI or an independent local run.

## Claim ceiling

**ReferenceModelOnly.** The model establishes deterministic semantics for the exact graph structures supplied to it.

It does not establish source truth, commitment authenticity, real-world causality, observer independence, legal authority, statistical confidence, production finality, durable storage, Byzantine consensus, or actuation safety.

A structurally closed bundle is a well-formed representation, not an assertion that its conclusion is objectively true.