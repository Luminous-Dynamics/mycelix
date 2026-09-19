# MYC-CONST-003D1D-F1B2 — Constitutional Payments Provider Admission v0.1

## Status

F1B2 is a **pre-registration threat model and admission oracle**. It adds no live Holochain entry/link registration, validate callback, coordinator extern, payments-zome edit, DNA edit, query endpoint, replay-capability mint, automatic replay, Treasury route or Governance route.

Parent semantic subject: F1B1 `c3ab0cad027ad3a3d8da0cb05390f3fb47ca3c08`.

## Purpose

F1B0 froze append-only provider-journal truth. F1B1 proved that the exact representation can cross the HDI serialization boundary without creating a DHT surface. F1B2 defines the hostile-write admission theorem that must be satisfied **before** those types are ever registered.

The reference oracle is constructed from:

```text
validated F1B0 journal
+ expected provider author
+ reference-model authority state
```

A candidate DHT write does not supply the expected predecessor, provider operation key, request commitment, receipt binding or authority target. Those are derived from the canonical reference journal.

`QualifiedForModelOnly` authority is intentionally non-serializable and exists only to exercise the future-authorized branch of the threat model. It is not a credential, capability or qualification claim.

## Record creates

For a candidate journal record create:

1. provider authority must be available in the reference model;
2. candidate author must equal the expected provider author;
3. F1B1 envelope shape must be valid;
4. candidate sequence must exist in the canonical F1B0 journal;
5. every non-genesis record requires the immediately preceding admitted record;
6. predecessor commitment must exactly match the admitted predecessor;
7. candidate must exactly refine the canonical F1B0 record through F1B1 projection.

Exact duplicate create is idempotent. After a canonical sequence slot has been admitted, different content for that same sequence is an integrity conflict and makes the halt sticky.

## Index creates

The future execution index has exactly this meaning:

```text
F0 execution_id
    -> F1A provider_operation_key
    -> F1B0 genesis record commitment
```

The genesis journal record must be admitted before the index. An exact duplicate index is idempotent. A conflicting index after canonical admission halts; it never replaces the canonical mapping.

Historical `Payment.id` is never operation/index authority.

## Mutation policy

Provider journal facts and the operation index are immutable constitutional evidence.

```text
record update   -> reject
record delete   -> reject
index delete    -> reject
```

F1B2 models no repair transition after an integrity halt.

## Transplant defenses

Because admission requires an exact F1B1 projection of the canonical F1B0 record, the following attacks are rejected:

- receipt evidence transplanted from another operation;
- request commitment transplanted from another effect;
- attempt horizon changed to cover a different dispatch history;
- provider operation key or execution identity substitution;
- forged predecessor commitment;
- orphan/skipped journal records.

This is stronger than validating each field independently because the candidate must equal the canonical forward projection of the source journal fact.

## Concurrency semantics

Two concurrent exact proposals for the same semantic slot converge to:

```text
Accepted -> ExistingSame
```

Two proposals for the same sequence/index identity with different content converge to:

```text
first canonical fact preserved
+ IntegrityHalted
```

There is no last-write-wins rule.

## F1B3 boundary

A future active integrity tranche must reconstruct this oracle using actual DHT facts and independently qualified provider-author authority. It must not accept caller-supplied booleans, serialized reference-model authority, mutable status fields or historical payment IDs as authorization.

Registration of `EntryTypes`, `LinkTypes` or a live validate callback is outside F1B2 and should remain blocked until the prerequisite exact-head qualification evidence exists.

## Non-claims

F1B2 does not establish Holochain persistence, provider-author authority, provider truth, payments-zome correctness, replay qualification, capability minting, public-fund authority, physical exactly-once settlement, external finality, live Governance routing, deployment currentness or qualification.
