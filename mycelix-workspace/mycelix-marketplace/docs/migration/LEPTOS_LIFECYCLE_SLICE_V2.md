# Leptos transaction lifecycle slice v2

## Scope

This slice closes the latest-revision gate from v1 without silently choosing between concurrent Holochain update branches.

Supported UI actions:

- seller: `Pending -> Confirmed`
- seller: `Confirmed -> Shipped`, with non-empty tracking information
- buyer: `Shipped -> Delivered`
- buyer or seller: `Pending|Confirmed -> Cancelled`

The transaction's original Create action is the stable route identity. Every read reduces the locally visible update tree. One leaf is current; multiple leaves are an explicit conflict. A conflicted transaction cannot be mutated through the coordinator or Leptos UI.

## Intentionally disabled

Finance settlement is visible but compiled behind the opt-in `finance-settlement` feature. The default build keeps it disabled until the patched Finance role is included in the deployed hApp and the live retry/recovery scenario passes. Marketplace remains `Delivered`; Finance records carry economic finality.

`dispute_transaction` remains disabled until dispute creation, arbitration-case persistence, assignment, and reputation effects are bound to one reviewable workflow.

## Static validation

Run:

```sh
scripts/validate-leptos-migration.sh
```

This checks the canonical zome manifest, typed client methods, lifecycle evidence contract, MessagePack wire fixtures, browser bridge bundle, Rust formatting/tests, and both live and fixture WASM builds.

## Two-agent conductor evidence

The optional Node scenario uses two installed Marketplace agents and each conductor's admin interface to authorize ephemeral zome-call signing credentials. It creates a listing and proves these paths against live conductors:

1. buyer creates a pending transaction;
2. seller observes and confirms it;
3. buyer observes the confirmed revision;
4. seller marks it shipped with tracking;
5. buyer confirms delivery;
6. both agents reduce the stable root to one delivered leaf with four revisions;
7. a second transaction is cancelled and resolves to one cancelled leaf.

Required environment variables:

- `MARKETPLACE_SELLER_APP_URL`
- `MARKETPLACE_SELLER_ADMIN_URL`
- `MARKETPLACE_SELLER_TOKEN_BASE64`
- `MARKETPLACE_BUYER_APP_URL`
- `MARKETPLACE_BUYER_ADMIN_URL`
- `MARKETPLACE_BUYER_TOKEN_BASE64`

Run:

```sh
npm --prefix frontend-leptos/bridge run test:conductor-lifecycle
```

The two agents must share the Marketplace DNA network. Finance settlement and disputes are explicitly outside this two-agent lifecycle scenario; they have separate evidence contracts.
