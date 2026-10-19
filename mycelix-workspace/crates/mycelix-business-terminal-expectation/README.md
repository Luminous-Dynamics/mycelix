# mycelix-business-terminal-expectation

Instance-specific terminal expectation provenance above `mycelix-business-terminal-qualification`.

## Why this layer exists

A terminal effect can occupy the correct reusable source role and still be the wrong transaction-specific effect.

For example, a policy may require:

```text
terminal source role = Finance accepted refund effect
operation profile    = finance.refund@1
```

Those constraints distinguish a refund from an unrelated Finance result or a generic credit, but they do not by themselves distinguish a required `$50` refund from a `$60` refund under the same operation profile.

Likewise, a retained-exception source role does not by itself identify which exact `DomainExceptionRef` an authorized resolution required.

## Model

```text
independent authoritative expectation boundary
        ↓ owning adapter interprets
instance expectation binding
        ↓ exact equality
actual terminal binding
        ↓
terminal source-role qualification
        ↓
closure proof qualification
        ↓
sealed core receipt
```

The reusable expectation profile maps each terminal role to a separate exact input boundary role that supplies its instance expectation.

The transaction basis supplies:

- a `CompensationExpectationBinding` containing the exact expected `CommittedIntent` plus its exact authoritative source; and/or
- an `ExceptionExpectationBinding` containing the exact expected `DomainExceptionRef` plus its exact authoritative source.

Qualification requires the actual terminal effect identity to equal the expected identity exactly.

## Independence rules

An expectation must not bootstrap itself from the effect it is supposed to constrain.

Therefore v0.1 requires:

- an expectation role is not itself any terminal role;
- the expectation boundary is an exact input role in the lower proof schema;
- the expectation node does not depend on any terminal-result node in the reusable dependency DAG;
- the expectation binding source exactly equals the input occupying the declared expectation role.

The terminal-result node may depend on the expectation node. The reverse is forbidden.

## Claim boundary

The expectation bindings are structural adapter assertions, not generic semantic truth.

This crate does **not** establish that:

- a Governance resolution really authorizes the expected refund amount;
- a Finance or dispute policy really requires the expected exception identity;
- a refund amount is adequate;
- an exception is legitimate;
- an external effect actually occurred beyond the lower domain-owned evidence.

The owning adapter/profile must substantively establish the mapping:

```text
exact authoritative source -> expected terminal identity
```

This layer preserves that mapping and prevents the actual terminal effect from drifting away from it.

## Layering

```text
mycelix-business-core
        ↓
mycelix-business-qualification
        ↓
mycelix-business-terminal-qualification
        ↓
mycelix-business-terminal-expectation
```

The crate depends only on those three lower Business crates and adds no clock, network, filesystem, environment, process, database, provider, Holochain, UI, or AI authority surface.
