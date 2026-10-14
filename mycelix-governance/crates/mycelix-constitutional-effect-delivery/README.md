# Mycelix Constitutional Effect Delivery

This crate is the sink-neutral V1 delivery core for MYC-CONST-003D1E.

It sits after release eligibility and before any real network, device, payment, email, or other provider adapter:

    qualified release
        -> prepared intent
        -> committed intent
        -> Pending
        -> AttemptStarted
        -> OutcomeUnknown / KnownSuccess / KnownNoEffect / IntegrityHalted
        -> durable local completion

## Core guarantees

- Prepared intents have no dispatch permit.
- Only a definite durable commit classification can produce a committed delivery intent.
- The exact upstream semantic binding is retained; this crate does not derive or replace EffectInstanceRoot, EffectContractRoot, payload commitment, SinkRoot, or SinkEpoch.
- Binding drift on retry or observation is rejected and fail-closes to IntegrityHalted.
- Transport acceptance and provider acknowledgement are evidence only.
- Ambiguous dispatch becomes OutcomeUnknown, never KnownNoEffect.
- NoAutomaticRetry blocks retry while unknown.
- IdempotentByEffectInstance permits retry only with the exact same semantic binding.
- Reconciliation references the exact prior observation and attempt.
- Contradictory success/no-effect evidence enters IntegrityHalted without erasing prior history or committed completion.
- Duplicate identical observations are idempotent.
- Caller acknowledgement is volatile and omitted from the durable snapshot.
- CommitOutcomeUnknown grants no delivery authority.

## Claim ceiling

This is a deterministic local contract, not a provider implementation. It establishes neither physical exactly-once execution nor any external provider's idempotency, finality, or reconciliation capability.

No real external sink is invoked by this crate.
