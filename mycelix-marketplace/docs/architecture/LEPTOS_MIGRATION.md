# Marketplace Leptos migration

## Decision

The canonical Marketplace frontend will migrate from SvelteKit to Leptos CSR. The existing Svelte application remains available as a visual and behavior reference until the Leptos application passes the mandatory live journeys.

This is a staged replacement, not a line-by-line rewrite and not an immediate deletion of `frontend/`.

## Current truth

- `frontend/` is the existing SvelteKit implementation.
- `frontend-leptos/` is the new canonical migration target.
- The backend zome contract is recorded in `contracts/zome-api.json` and checked directly against `#[hdk_extern]` functions.
- Browser commerce must never silently fall back to fabricated data after a failed live connection.
- A live browser transport must use authenticated, signed Holochain calls. Placeholder signatures are prohibited.

## Migration gates

### Gate 0 — contract stability

- Every frontend call comes from one typed client module.
- Zome and function names match `contracts/zome-api.json`.
- Holochain hashes and agent keys use explicit wrapper types.
- No route constructs raw zome-call strings.

### Gate 1 — read-only vertical slice

- Connect to the installed hApp.
- Load real listings through `listings.get_all_listings`.
- Open a listing through `listings.get_listing`.
- Distinguish disconnected, unavailable, empty, loading, and error states.
- Development fixtures are enabled only by an explicit feature.

### Gate 2 — two-agent commerce journey

- Buyer creates a transaction.
- Seller confirms and marks it shipped.
- Buyer confirms delivery.
- Settlement succeeds before completion.
- Restarting either UI recovers the same transaction state.

### Gate 3 — trust and dispute journey

- Reviews derive from completed transactions.
- Reputation changes derive from validated marketplace events.
- A dispute is filed, assigned, voted, finalized, and rendered from live data.

### Gate 4 — Svelte retirement

The Svelte frontend may be archived only when:

- all supported routes have an explicit parity decision;
- Gate 2 and Gate 3 pass in mandatory CI;
- browser tests cover the canonical workflows;
- no production documentation points to the Svelte application;
- mock mode cannot activate in a production build.

## Work policy during migration

New product features belong in Leptos. The Svelte application receives only critical security fixes, documentation corrections, and small changes needed to preserve migration evidence.
