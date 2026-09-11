# Mycelix Integration Execution Binding

This crate is the first executable slice of **INT-04**. It does not call a provider and it does not mint institutional authority.

Its job is narrower:

```text
exact INT-03 ExecutionClaim
+ exact typed IntegrationCommand
+ current signed provider execution profile
    -> QualifiedExecutionBinding
```

The result proves that one live attempt is bound to the same command identity/commitment, connector, system, operation kind, target class, side-effect classification, idempotency metadata, reconciliation contract, materializer release, profile generation, and provider-profile trust root.

## What it deliberately does not prove

```text
QualifiedExecutionBinding != CurrentExecutionAuthority
ProviderProfileTrustRoot != institutional authority
SignedProviderProfile != permission to dispatch
IdempotencyKeyPresent != provider idempotency guarantee
ProviderProfile != payload materialization
Binding != DispatchStarted
```

`QualifiedProviderExecutionProfile` and `QualifiedExecutionBinding` are non-deserializable positive objects. Both report `grants_execution_authority() == false`.

## Provider currentness

Provider profiles are Ed25519 signed and accepted only when:

- the signer key ID matches the supplied trust root;
- the signature verifies over the canonical v1 profile preimage;
- the profile generation exactly equals the root's current generation;
- the qualification/use time falls inside the profile validity window;
- the profile is internally coherent.

The trust root is an explicit input from the enclosing configuration/authority system. This crate does **not** claim that a caller-created trust root is globally trusted.

## Retry / ambiguity rule

A side-effecting command must have provider reconciliation support. `IdempotencyKey` reconciliation additionally requires a signed provider contract stating a non-zero request-key retention interval and lookup support. Merely carrying an idempotency key on a command is not enough.

## Command bytes remain closed

INT-03 intentionally does not expose command bytes on `ExecutionClaim`. This crate does not reopen that path. The future materialization layer should deterministically encode the exact typed command only after this binding and fresh live authority are both present, then bind the payload digest to the declared materializer release before `DispatchStarted` is persisted.

## Next theorem

The remaining INT-04 composition is:

```text
QualifiedExecutionBinding
+ non-forgeable fresh CurrentExecutionAuthority
+ exact provider materializer/release qualification
    -> QualifiedProviderPayload
    -> persist DispatchStarted
    -> provider call
```

That final authority must consume the existing Mycelix authority/effect-safety/durable-attempt line. This crate must not grow a parallel authorization model.