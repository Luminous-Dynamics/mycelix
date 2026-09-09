# Response Execution Intent Invariants

This crate binds one selected response decision to the exact action bytes already committed by the current executor authority domain. It is a byte-continuity theorem, not an effect-admission theorem.

## Positive path

```text
PreparedResponseAuthorityResolution
        +
VerifiedCurrentExecutorAuthorityReceipt
        +
untrusted adapter payload candidate
        ↓
#513 response/executor semantic coverage
        ↓
canonical response execution action envelope
        ↓
shared exact-byte execution digest
        ↓
exact equality with executor actions_digest/profile
        ↓
QualifiedResponseExecutionIntent
```

## Canonical action envelope

The governance action bytes are a deterministic one-element JSON array whose object fields are emitted in this exact struct order:

1. `protocol`
2. `response_proposal_id`
3. `response_proposal_digest`
4. `response_decision_id`
5. `response_option_id`
6. `authority_domain`
7. `action_class`
8. `jurisdiction`
9. `adapter_profile`
10. `payload_utf8`

`payload_utf8` is serialized as a JSON string. It is **not parsed or normalized** by this theorem. Therefore changing whitespace, JSON key order, escaping or any other payload byte changes the governance action digest.

## Required exact equality

Positive qualification requires the provider projection to use exactly:

`mycelix-governance-execution-authority-v1-blake3-exact-json`

and requires the locally recomputed digest of the canonical action bytes to equal `receipt.projection.actions_digest` exactly.

A caller may suggest payload bytes, but cannot make them authoritative: altered payload bytes qualify only if the current executor authority domain already committed those exact resulting action bytes.

## Authority separation

A `QualifiedResponseExecutionIntent` proves exact action-byte continuity only.

It explicitly does **not** prove:

- that the deserializable provider receipt originated from the designated local provider;
- that the authority/currentness verifier deployment or coordinator code is the approved live deployment;
- that pending authority subjects such as effect-safety policy are satisfied;
- that the named adapter profile is safe or enforceable;
- that an execution attempt has a unique claim/idempotency/fencing identity;
- that an external effect may occur.

The positive type is serializable for audit but not deserializable as a positive object.

## Size and interpretation

Adapter payload candidates are bounded to 2048 UTF-8 bytes. The final canonical action envelope is independently bounded by the shared 4096-byte governance action digest contract.

Adapter semantics belong to a later effect-safety theorem. This crate must not grow adapter-specific interpretation, Holochain calls, DHT queries, persistence, locks, attempts or external effects.
