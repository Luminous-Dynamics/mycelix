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

The qualification crate is deliberately **not** another authority-owning domain. It does not decide whether Commerce accepted an agreement, Finance reconciled settlement, Supply Chain accepted receiving, Accounting recognized an economic event, or Governance granted power. Those meanings remain with the owning domain/profile.

## Observed abstraction

Three materially different Golden Paths independently produced the same structural requirement:

```text
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
- closure derivation root.

The exact basis carries:

- the immutable `QualificationCut`;
- one explicit closure dependency DAG;
- exact boundary results bound to DAG leaves;
- exact obligation requirements and disposition provenance;
- exact exception provenance;
- exact compensation provenance.

## Graph grounding invariant

Acyclicity alone is insufficient. An acyclic graph can still be unrelated decorative metadata.

Qualification therefore requires:

```text
all graph nodes are reachable from the closure root

and

reachable graph leaves
    ==
exact nodes carrying qualified boundary results
```

A leaf can currently be grounded by only the boundary kinds independently required by GP-002, GP-003, and GP-006:

- `QualifiedInputRef`;
- `DomainReconciliationRef`.

Do **not** add another `QualifiedBoundaryRef` variant merely because a domain exposes another record type. Add one only when a materially different Golden Path proves that the cut-level reference kind must participate directly in cross-domain closure qualification and cannot be represented by an exact qualified input owned by that domain.

## Proof compression

A boundary leaf may itself represent a qualified result over a deeper private proof graph.

Example:

```text
Accounting internal event/policy/adjustment/reconciliation graph
        ↓
exact Accounting-qualified period-close result
        ↓
Business month-end orchestration DAG leaf
```

Business does not need to flatten or dereference the Accounting proof graph to compose the result. This preserves authority boundaries, reduces disclosure, and prevents Business from becoming a universal evidence warehouse.

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
- authority evaluation;
- provider verification;
- accounting recognition;
- settlement truth;
- inventory truth;
- identity truth;
- automatic dereferencing of upstream evidence graphs.

The intended shape is a deterministic structural qualification layer over exact, already-authoritative boundary results.
