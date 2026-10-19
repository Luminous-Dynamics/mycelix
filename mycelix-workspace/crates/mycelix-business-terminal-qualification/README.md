# mycelix-business-terminal-qualification

Policy-declared terminal-role correspondence above `mycelix-business-qualification`.

## Purpose

Exact provenance is necessary but not sufficient for terminal closure semantics. A valid Finance result cannot become refund evidence merely because it is current, exact, and present in the same qualification cut. Likewise, a valid Finance dispute result cannot automatically occupy the retained-exception role selected by a closure policy.

This crate binds terminal effects to already-declared exact input boundary roles.

```text
ClosureQualificationProfile
    owns the DAG + exact boundary schema

TerminalClosureQualificationProfile
    additionally declares:
      compensation boundary role -> required operation semantic profile
      retained-exception boundary role

TerminalClosureQualificationBasis
    supplies exact role-keyed CompensationBinding / ExceptionBinding instances
```

The terminal qualifier proves:

- the terminal role already exists as an exact input boundary in the reusable closure profile;
- the transaction supplies exactly the policy-declared compensation/exception role set;
- each terminal binding uses the exact source occupying its declared boundary role;
- compensation uses the exact operation semantic profile declared by the terminal policy;
- distinct required compensation roles do not silently alias one `CommittedIntent`;
- distinct required exception roles do not silently alias one `DomainExceptionRef`;
- final closure still passes the reusable qualification algebra and sealed core receipt checks.

## Deliberate non-authority

This crate does **not** decide:

- whether a refund amount is adequate;
- whether a refund/credit operation is substantively correct;
- whether a dispute or exception is justified;
- whether an owning-domain result truthfully represents the event it claims;
- whether one domain policy should require compensation or an exception in the first place.

Those meanings remain owned by the relevant domain and policy adapters. This layer proves only reusable structural correspondence.

## Evidence reuse vs terminal identity

The default v0.1 rule is:

```text
terminal_role_a != terminal_role_b
    ->
terminal_effect_identity_a != terminal_effect_identity_b
```

That does **not** imply one evidence record per effect. Distinct compensation intents or exception identities may share an exact batch/corroborative source where owning-domain policy permits it. Evidence reuse and institutional-effect identity remain separate semantics.

## Layering

```text
mycelix-business-core
    zero-dependency institutional invariants
        ↓
mycelix-business-qualification
    reusable closure proof schema
        ↓
mycelix-business-terminal-qualification
    policy-declared terminal-role correspondence
```

The crate intentionally has no dependencies other than the two lower Business crates and introduces no clock, network, filesystem, environment, process, database, Holochain, provider, UI, or AI authority surface.
