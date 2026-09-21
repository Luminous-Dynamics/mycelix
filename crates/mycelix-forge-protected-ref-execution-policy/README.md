# FORGE-009G — Protected Ref Execution Policy

This tranche makes the durable consumption-marker namespace from FORGE-009E an explicit typed execution profile **without changing the existing meaning of `RepositoryPolicyState.policy_digest`**.

That distinction matters because the current gittuf adapter already interprets `policy_digest` as the commitment to the active external gittuf policy subject.

## Problem

FORGE-009E proves an atomic transaction can create:

```text
target ref: expected -> proposed
consumption marker: absent -> proposed
```

But:

```text
atomic marker creation
!= external repository policy protects marker from later delete/rewrite
```

If an actor can delete or rewrite `refs/mycelix/forge/consumed/**`, the one-shot replay theorem is no longer durable.

## Typed execution profile

`ProtectedRefExecutionPolicyV1` fixes the v1 desired enforcement semantics:

```text
reserved namespace = refs/mycelix/forge/consumed/
normal target inside reserved namespace = forbidden
marker creation = create-only
marker deletion = forbidden by policy
marker rewrite = forbidden by policy
target CAS + marker create = one atomic transaction
Git ref realization = --no-deref
```

These are protocol constants, not caller-selectable booleans.

## External policy subject remains authoritative for repository policy

FORGE-009G deliberately does **not** require:

```text
RepositoryPolicyState.policy_digest == execution_policy.digest
```

That would conflict with the current gittuf adapter, where the repository-policy digest commits the active gittuf policy ref tip.

Instead the theorem is:

```text
RepositoryPolicyState(external policy subject)
+ ProtectedRefExecutionPolicyV1
+ RepositoryExecutionPolicyObservationV1
+ independent RepositoryExecutionPolicyVerifierV1
        ↓
QualifiedProtectedRefExecutionPolicyV1
```

The qualifier proves exact cross-links between:

- project;
- exact monotonic repository-policy state;
- exact external policy subject already committed by that state;
- exact typed protected-ref execution profile;
- provider-native enforcement evidence;
- exact verifier identity/evidence.

The positive type has no `Deserialize` implementation.

## Git plan qualification

`PolicyQualifiedGitRefTransactionPlanV1` additionally requires the positive FORGE-009F plan to satisfy the reserved namespace surface:

- protected target is outside `refs/mycelix/forge/consumed/`;
- marker is inside that reserved namespace;
- marker does not alias the target;
- argv is exactly the fixed `update-ref --no-deref --stdin -z` profile.

## Claim boundary

A positive FORGE-009G result proves that a **named repository-policy verifier** accepted evidence that the exact external repository-policy subject enforces the typed protected-ref execution profile.

It does not establish verifier correctness independently of that evidence, M0 qualification, merge authorization, trusted time, actual execution, or hostile-host resistance.

For gittuf specifically, the next concrete tranche should inspect/verify the exact active policy subject and prove the reserved namespace cannot be deleted/rewritten through ordinary authorized repository operations while marker creation is admitted only through the intended transaction path.

## Why this preserves existing repository semantics

Current repository verification already binds:

```text
RepositoryPolicyState.policy_digest
        == external gittuf policy subject commitment
```

FORGE-009G keeps that unchanged and adds a second theorem:

```text
that exact external policy subject
        enforces
ProtectedRefExecutionPolicyV1
```

So no old repository-policy commitment is silently reinterpreted.

## Tests

Focused tests cover:

- external policy subject and typed execution-policy digest remaining distinct;
- exact provider-verified enforcement qualification;
- unrelated execution-policy substitution rejection;
- protected target inside reserved namespace rejection;
- marker outside reserved namespace rejection;
- deviation from exact Git transaction argv rejection;
- public plan qualification requiring a positive `GitRefTransactionPlanV1`.

## Claim chain

```text
FORGE-009E AtomicProtectedRefConsumptionIntentV1
        ↓
FORGE-009F GitRefTransactionPlanV1
        ↓
RepositoryPolicyState(external policy subject)
+ ProtectedRefExecutionPolicyV1
+ provider/verifier evidence
        ↓
FORGE-009G QualifiedProtectedRefExecutionPolicyV1
        ↓
PolicyQualifiedGitRefTransactionPlanV1
```

Still:

```text
PolicyQualifiedGitRefTransactionPlanV1
!= M0 qualified
!= merge authorized
!= transaction executed
```
