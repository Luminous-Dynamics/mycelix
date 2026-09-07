# Mycelix Business Qualification

Reusable proof-carrying closure qualification above `mycelix-business-core`.

## Layering

```text
mycelix-business-core
    zero dependencies
    transport-neutral institutional invariants
        ↓
mycelix-business-qualification
    exactly one path dependency: mycelix-business-core
    reusable closure profile/basis algebra
        ↓
Golden Path policy fixtures
    service sale / procurement / month-end / future paths
```

The qualification crate is deliberately **not** another authority-owning domain. It does not decide whether Commerce accepted an agreement, Finance reconciled settlement, Supply Chain accepted receiving, Accounting recognized an economic event, or Governance activated a closure policy. Those meanings remain with the owning domain/profile.

## Observed abstraction

Three materially different Golden Paths independently produced the same structural requirement:

```text
exact policy/profile source
        +
(organization, qualification profile, closure policy, closure class)
        +
exact instance basis
        ↓
sealed WorkflowClosureReceipt
```

The reusable profile fixes:

- organization context;
- qualification semantic profile;
- closure policy;
- closure semantic profile;
- closure class;
- exact policy/profile source result;
- closure derivation root.

The exact basis carries:

- the immutable `QualificationCut`;
- one explicit closure dependency DAG;
- exact factual boundary results bound to reachable DAG nodes;
- exact obligation requirements and disposition provenance;
- exact exception provenance;
- exact compensation provenance.

## Policy provenance is distinct from factual dependency provenance

A closure policy/profile is not allowed to float free of evidence merely because code can name a `ClosurePolicyRef` or `SemanticProfileId`.

`ClosureQualificationProfile` therefore retains one exact `QualifiedInputRef` supplied by the owning policy adapter/domain as the source for the selected policy/profile/class. Qualification requires that exact source to be present and current in the same cut.

The generic layer does **not** decide that the source substantively corresponds to the named policy/profile/class. That interpretation remains owning-domain authority, exactly as with authorization, disposition, compensation, and exception bindings.

The policy source is intentionally not treated as a factual DAG boundary:

```text
policy/profile source
    = semantic authority for how the proof is interpreted

qualified boundary result
    = factual/result prerequisite consumed by the proof
```

This distinction lets a month-end Business proof contain, for example, one Governance/policy source plus one Accounting-qualified close boundary without pretending that policy activation is itself an Accounting fact.

## Graph grounding invariant

Acyclicity alone is insufficient. An acyclic graph can still be unrelated decorative metadata.

Qualification therefore requires:

```text
all graph nodes are reachable from the closure root

and

every reachable leaf has exact qualified boundary provenance

and

every node carrying an exact qualified boundary result is reachable from the closure root
```

A qualified boundary may be either a leaf or a reachable internal node. This matters because an accepted result may itself depend on another accepted result without ceasing to be a valid cross-domain boundary. For example, an accepted Commerce invoice may depend on an accepted agreement, and an accepted Accounting event may depend on Finance/work results.

The currently supported boundary kinds are only those independently required by GP-002, GP-003, and GP-006:

- `QualifiedInputRef`;
- `DomainReconciliationRef`.

Do **not** add another `QualifiedBoundaryRef` variant merely because a domain exposes another record type. Add one only when a materially different Golden Path proves that the cut-level reference kind must participate directly in cross-domain closure qualification and cannot be represented by an exact qualified input owned by that domain.

## Proof compression

A boundary result may itself represent a qualified result over a deeper private proof graph, whether it appears at a leaf or a reachable internal node.

Example:

```text
Accounting internal event/policy/adjustment/reconciliation graph
        ↓
exact Accounting-qualified period-close result
        ↓
Business month-end orchestration DAG boundary
```

Business does not need to flatten or dereference the Accounting proof graph to compose the result. This preserves authority boundaries, reduces disclosure, and prevents Business from becoming a universal evidence warehouse.

Policy provenance follows the same compression principle. The Business qualifier consumes an exact policy-source result; it does not need to recursively ingest every Governance fact that caused the policy adapter to produce that result.

## Closure-class specificity

Closure class belongs to the reusable profile, not the instance basis. A caller therefore cannot take a basis intended for one terminal meaning and select a stronger class as a free argument.

Domain/policy code should define separate profiles when different closure classes have different proof shapes, for example:

```text
procurement / Satisfied
    -> settlement + reconciliation + receiving + accounting

procurement / Terminated
    -> payment cancellation + terminal dispositions + accounting
```

The core receipt still performs the final non-strengthening checks over obligation dispositions, exceptions, and compensation.

## Non-goals

This crate does not provide:

- a workflow engine;
- a generic predicate language;
- a Turing-complete policy DSL;
- domain semantic interpretation;
- policy-source semantic interpretation;
- authority evaluation;
- provider verification;
- accounting recognition;
- settlement truth;
- inventory truth;
- identity truth;
- automatic dereferencing of upstream evidence graphs.

The intended shape is a deterministic structural qualification layer over exact, already-authoritative boundary and policy results.
