# Governance Resolution Types v0.1

`governance-resolution-types` is the transport-neutral semantic vocabulary for exact terminal outcomes authorized by Mycelix Governance.

It exists so downstream domains do not have to interpret opaque Governance JSON and so Governance does not have to depend on Business orchestration types.

## Ownership boundary

Governance owns the meaning of an authorized resolution outcome.

The v0.1 vocabulary contains only two terminal outcomes currently justified by the Business GP-005 refund/dispute work:

- an exact compensating refund expectation;
- an exact retained-exception identity.

The crate deliberately contains no Holochain/HDK, Business, Finance runtime, database, network, clock, filesystem, process, provider, UI, or AI API.

A Holochain Governance zome may later persist and validate this vocabulary. A bridge may later translate a verified exact Governance outcome into Business terminal-expectation bindings.

Neither step changes ownership of the underlying semantics.

## Refund outcome

`RefundResolutionV1` binds:

- an opaque expected logical effect ID;
- the original order reference;
- beneficiary reference;
- exact unit/currency identifier;
- integer minor-unit amount.

Floating-point money is intentionally excluded.

The type exposes `canonical_material_v1()`, a deterministic collision-resistant textual preimage using UTF-8 byte-length prefixes. A downstream bridge may commit this exact material into an execution-domain intent without inventing or normalizing resolution terms.

Governance deliberately does **not** name a Finance operation profile here. Mapping the Governance-owned refund outcome into a Finance-owned execution profile is a separate versioned bridge contract and must be reviewed independently.

## Retained exception outcome

`RetainedExceptionResolutionV1` binds the authoritative domain namespace and exact local exception identity.

It does not decide whether the exception remains current at a later closure time; downstream qualification still owns current-cut/freshness checks.

## Semantic profiles

The v0.1 Governance outcome profiles are:

- `governance.authorized-refund-resolution@1`
- `governance.authorized-exception-resolution@1`

Changing the material meaning of either profile requires a new version. Existing historical values retain their v1 meaning.

## Validation

Constructors and custom deserialization fail closed for empty required identifiers and zero-value refund effects. This prevents invalid wire values from bypassing constructor validation through Serde.

An exact persisted Governance record/action is still required to establish institutional authority. This crate defines vocabulary; it does not by itself prove that a resolution was authorized.

## Composition target

The intended future chain is:

```text
Governance proposal / dispute process
        ↓
Governance authorization / execution rules
        ↓
exact AuthorizedResolutionOutcomeV1 record
        ↓ Governance-owned verifier
verified Governance resolution outcome
        ↓ versioned cross-domain bridge
provenance-bound Business terminal expectation
        ↓
actual Finance / exception result
        ↓
Business equality + closure qualification
```

Business therefore never interprets an opaque Governance JSON payload and never manufactures the resolution terms used to prove its own closure.
