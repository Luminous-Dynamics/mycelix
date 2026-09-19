# MYC-CONST-003D1D-F1B1 — Inactive Payments Provider HDI Refinement v0.1

## Status

F1B1 is an **inactive HDI serialization/refinement contract** stacked on F1B0 exact semantic head `1d0dc3f07ac29dccba9f33b45aef13380ecdb870`.

It deliberately does not register a Holochain entry type, link type, integrity callback or coordinator extern. It therefore opens no new DHT write surface.

## Purpose

F1B0 froze the authoritative semantic journal:

```text
OperationIntent
DispatchAttempt
Observation
```

with hash-linked append-only records and replay-derived state.

F1B1 answers a narrower question:

> Can those exact frozen records be represented at an HDI serialization boundary without inventing a second journal, second hash algorithm, second state reducer or new authority?

The answer is represented by `ProviderJournalEntryEnvelope`, an unregistered `#[hdk_entry_helper]` type.

## Exact projection

The only supported construction path is:

```text
validated F1B0 ProviderJournalRecord
            ↓
ProviderJournalEntryEnvelope::from_journal_record(...)
```

The envelope preserves:

- F1B0 journal schema version;
- provider operation key;
- record sequence;
- predecessor record commitment;
- journal record commitment;
- exact semantic record body;
- audit timestamp as metadata.

`validate_refines_record()` reconstructs the canonical projection from the supplied F1B0 record and requires exact equality. F1B1 does not recompute F1B0's journal hash and does not reimplement its reducer.

That prevents two subtly different persistence semantics from emerging.

## Identity namespaces remain separate

F1B1 preserves the distinction:

```text
F0 execution_id
    != F1A provider_operation_key
    != historical Payment.id
    != dispatch attempt_id
    != provider receipt_id
```

A successful observation may carry a historical `Payment.id`, but that value remains outcome evidence. It is never promoted to the provider operation key or operation index.

## Future operation index

`ProviderOperationIndexProjection` describes the exact future index relationship:

```text
F0 execution_id
        ↓
F1A provider_operation_key
        ↓
F1B0 genesis journal-record commitment
```

This is only a pure projection in F1B1. No `#[hdk_link_types]` enum and no `create_link()` call exist.

## Authority remains fail-closed

Successful serialization does not establish write authority.

F1B1 explicitly freezes:

```text
provider_author_binding = PendingQualifiedProviderAuthority
provider_authority_is_qualified = false
entry_write_authority_established = false
observation_write_authority_established = false
```

The Rust helper `provider_authority_is_qualified()` therefore returns `false` in this tranche.

A future active integrity layer must prove which author may create operation intents, dispatch attempts and provider observations. That theorem cannot be inferred from possession of a correctly shaped entry.

## What an active integrity successor must prove

Before these types can be registered in a live DNA, the next integrity tranche must independently enforce at least:

1. exact frozen F1B1 entry representation;
2. exact predecessor-record commitment for every non-genesis record;
3. exact operation index binding to execution ID, provider operation key and genesis commitment;
4. qualified provider-author authority;
5. immutable append-only facts—no update/delete semantics;
6. F1B0 attempt-horizon monotonicity and sticky integrity halt;
7. KnownSuccess receipt evidence bound to the exact provider operation and F0 request;
8. historical `Payment.id` remains separate from provider operation identity;
9. no live payment dispatch until persistence/integrity qualification is independently green.

## Non-claims

F1B1 does **not** establish DHT registration, DHT persistence, provider-author authority, payments-zome wiring, provider truth, replay qualification, capability minting, public-fund authority, physical exactly-once settlement, live Governance routing or qualification.
