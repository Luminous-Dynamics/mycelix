# Marketplace Leptos Vertical Slice V1

## Purpose

This gate proves the smallest honest Marketplace journey before broader Svelte parity work:

1. Connect through the authenticated official-client bridge.
2. Read listings from `listings.get_all_listings`.
3. Decode an `ActionHash` from a Leptos route and read `listings.get_listing`.
4. Create a pending transaction through `transactions.create_transaction`.
5. Open the returned transaction action directly through `transactions.get_transaction`.
6. Reload the direct transaction route and recover the pending transaction from the conductor.
7. Confirm the transaction appears in `transactions.get_my_transactions`.

The default build never substitutes fixtures after a connection failure. The fixture build must be selected explicitly with `--no-default-features --features dev-fixtures` and is not conductor evidence.

## Security properties added with this slice

The coordinator resolves the requested listing before transaction creation. Client-provided seller and total fields remain on the compatibility wire, but the backend verifies them and constructs the transaction from the DHT-backed listing terms.

The transaction integrity zome now enforces:

- Buyer authorship at creation.
- Pending initial state with no tracking data.
- Immutable parties, listing, quantity, price, and creation timestamp.
- Buyer/seller authorization for each lifecycle transition.
- No skipped or terminal-state transitions.
- Tracking mutation only during `Confirmed -> Shipped`.
- Persistent materiality only on completion.

## Live verification

Build and install the current DNA/hApp, expose its authenticated app interface through Launcher or Tauri, and supply the runtime configuration expected by `frontend-leptos/bridge/src/index.ts`.

Run the migration validator before the browser test:

```sh
scripts/validate-leptos-migration.sh
```

Then use two agents so the buyer is not the listing owner:

1. Agent A creates an active listing with inventory of at least two.
2. Agent B opens the Leptos application and confirms the listing appears.
3. Agent B opens the listing route and creates a quantity-one purchase.
4. Record the returned transaction route.
5. Reload the browser on that exact route.
6. Confirm the recovered transaction remains `pending`, with Agent B as buyer, Agent A as seller, and total equal to listing price times quantity.
7. Open `/transactions` and confirm the same action appears.
8. Attempt a modified raw zome call with a spoofed seller or total and confirm rejection.

Capture conductor version, DNA hash, hApp hash, both agent keys, listing action hash, transaction action hash, and the test timestamp in the release evidence directory.

## Deliberate gate

Seller confirmation, shipment, delivery, completion, cancellation, and dispute controls remain disabled in the Leptos UI. Link-based transaction queries currently point to creation actions, while later lifecycle states are update actions. Those controls should activate only after the coordinator has one tested reducer that follows update relationships and returns the newest valid, non-deleted transaction revision consistently across direct and link-based reads.
