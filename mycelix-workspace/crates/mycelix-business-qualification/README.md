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
    reusable closure proof-schema algebra
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
reusable closure proof schema
        +
exact instance basis
        ↓
sealed WorkflowClosureReceipt
```

The reusable `ClosureQualificationProfile` fixes:

- organization context;
- qualification semantic profile;
- closure policy;
- closure semantic profile;
- closure class;
- exact policy/profile source result;
- closure derivation root;
- dependency-DAG topology;
- exact set of boundary roles;
- structural requirement for each boundary role;
- exact set of policy-required obligation roles and their owning domains.

The exact `ClosureQualificationBasis` carries:

- the immutable `QualificationCut`;
- exact transaction-specific boundary results for the profile's declared roles;
- exact transaction-specific obligation identities for the profile's declared obligation roles;
- exact domain-owned disposition provenance for those obligations;
- exact exception provenance;
- exact compensation provenance.

The basis does **not** own the dependency graph, boundary-role set, or required-obligation set. A transaction therefore cannot weaken a reusable proof by deleting an edge, omitting a policy-required internal boundary, or deciding that an inconvenient policy-required duty does not count.

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

## Reusable proof schema

Acyclicity, reachability, boundary roles, and required obligation roles are policy semantics, not transaction inputs.

A profile therefore owns one `ClosureDependencyGraph`, one exact boundary-role schema, and one exact required-obligation-role schema. Qualification requires:

```text
profile graph is acyclic

all profile graph nodes are reachable from the closure root

every reachable leaf is a declared boundary role

every declared boundary role is reachable from the closure root

basis boundary-role set
    ==
profile boundary-role set

basis obligation-role set
    ==
profile obligation-role set
```

A qualified boundary may be either a leaf or a reachable internal node. This matters because an accepted result may itself depend on another accepted result without ceasing to be a valid cross-domain boundary. For example, an accepted Commerce invoice may depend on an accepted agreement, and an accepted Accounting event may depend on Finance/work results.

The profile additionally declares a `QualifiedBoundaryRequirement` for each boundary role. v0.1 supports:

- exact input from a named authoritative domain under an exact semantic profile;
- reconciliation identity from a named authoritative domain.

This does not interpret the underlying record. It prevents structurally invalid substitutions such as:

```text
Accounting role <- Commerce input

or

Accounting role @ semantic profile v1 <- Accounting input @ semantic profile v2
```

Exact record IDs and versions remain instance-specific in the basis.

## Required-obligation completeness

Policy-required duties are part of the reusable proof schema, not caller-selected closure metadata.

Each required obligation role:

- names the proof/boundary role that carries its disposition result;
- fixes the owning authoritative domain;
- must also be an exact input boundary role;
- receives an exact transaction-specific `DomainObligationRef` from the basis;
- must have exactly one disposition binding for that exact obligation;
- requires that binding to use the same exact `QualifiedInputRef` as the declared boundary result for the role.

The qualifier then derives `ClosureRequirements` itself before calling the sealed core receipt constructor.

Therefore a transaction cannot close by:

```text
omitting a required obligation

substituting a Supply Chain obligation into a Finance role

or

using Finance disposition result B
while the declared Finance boundary role is exact result A
```

The generic qualifier still does **not** decide whether an owning-domain disposition result substantively means `Satisfied`, `Terminated`, or another disposition. That interpretation remains with the owning domain/profile; the qualification layer enforces exact structural provenance and completeness.

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

Closure class and proof schema belong to the reusable profile, not the instance basis. Policies with different terminal meanings therefore carry different reusable proof shapes.

For example:

```text
procurement / Satisfied
    -> settlement + reconciliation + receiving + accounting

procurement / Terminated
    -> payment cancellation + terminal dispositions + accounting
```

The compact cross-Golden-Path corpus proves that these are represented by different profile DAGs, boundary schemas, and obligation-role schemas, while the core receipt still performs the final non-strengthening checks over obligation dispositions, exceptions, and compensation.

## Non-goals

This crate does not provide:

- a workflow engine;
- a generic predicate language;
- a Turing-complete policy DSL;
- domain semantic interpretation;
- policy-source semantic interpretation;
- obligation-disposition semantic interpretation;
- authority evaluation;
- provider verification;
- accounting recognition;
- settlement truth;
- inventory truth;
- identity truth;
- automatic dereferencing of upstream evidence graphs.

The intended shape is a deterministic structural qualification layer over exact, already-authoritative boundary and policy results, with reusable proof topology and duty completeness owned by the profile rather than rewritten per transaction.
