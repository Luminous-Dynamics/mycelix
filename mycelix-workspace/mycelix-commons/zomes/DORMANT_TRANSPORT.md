# Transport zomes — DORMANT

**Status: dormant, unpacked, never deployed. Do not treat as a shipped capability.**

Covers `transport-routes/`, `transport-sharing/`, `transport-impact/`.

Written 2026-07-30 after a coverage audit, so the next reader does not have to
re-derive any of this. Every claim below was verified against source at that
date; re-check before relying on it, this tree moves fast.

---

## Status

- **Last substantive commit: `b9e6ca377d`, 2026-03-28** ("consciousness coding
  session updates — care circles, water, transport, housing, food, mutual aid").
  Everything after it is mechanical: the `gate_consciousness` → `gate_civic`
  migrations (2026-04-10/11), a pure `cargo fmt` reflow (`e0b322713e`,
  2026-05-15 — it is transport-only with 73 insertions and looks substantive
  under `--stat`; it is not), and the `mycelix-workspace/` path move
  (`7596b27473`, 2026-07-02).
- **Not packed.** No `.dna`, `.happ` or `.wasm` artifacts exist anywhere under
  `mycelix-commons`. The zomes *are* correctly declared in both commons DNA
  manifests and wired into the commons-bridge routing table — that is where
  reality stops.
- **Never built for wasm32.** CI's `test-commons` runs `cargo test --workspace`
  against the **host** target. Nobody has confirmed these zomes compile to
  `wasm32-unknown-unknown`. Any effort estimate that assumes they do is void
  until someone runs it.
- **No frontend.** `mycelix-commons/apps/leptos/src/pages/transport.rs` is ~50
  lines of hardcoded literals ("67 Shared Vehicles", "Electric Van #3") with
  zero zome calls and no `dist/`. Port 8104 is unserved.
- Sweettests are `#[ignore]`d, single-agent, and load a DNA that does not exist;
  `mycelix-commons/Cargo.toml` excludes the tests crate from the workspace.

## Size, so nobody re-counts

6,858 LOC across 6 crates · 37 product externs (+3 `validate` callbacks) ·
406 `#[test]` attributes. Test counts are attribute greps — upper bounds. Most
are serde round-trips: routes 26/28 and sharing 27/32 are serde-named.
`transport-impact` is the exception with ~25 genuine logic tests — and those are
what enshrine the carpool double-discount below.

There is also a well-tested TypeScript client at
`mycelix-workspace/sdk-ts/src/integrations/transport/index.ts` — 643 LOC, 35
methods, 1,605-line test file. It is the single most reusable artifact here: the
product API shape already exists. Note it exposes 35 of the 37 externs
(`getNearbyRides` / `getNearbyRoutes` are absent).

---

## The four structural defects

These are not TODOs. Each one makes a headline feature non-functional.

### 1. Ride offers have no location, so no ride is discoverable

`RideOffer` (`transport-sharing/integrity/src/lib.rs`) is
`{vehicle_hash, route_hash, driver, departure_time, seats_available,
price_per_seat, status}` — there is no origin or destination coordinate on it,
and `post_ride_offer` writes no geo-index link.

`find_nearby_rides` (`transport-sharing/coordinator/src/lib.rs:356-385`)
consequently decodes and distance-filters **only `CargoOffer`**. The code says
so itself:

> `// Check if it's a RideOffer — we can't filter by location on RideOffer`
> `// since it only has route_hash, but we can return all offers within`
> `// the system for now and let the client filter by route details.`

`CargoOffer` is the only transport entity carrying coordinates. Both
ride-discovery paths are dead. `get_nearby_routes` has a sibling problem: it
returns **Stops**, not Routes, because only `add_stop` writes a geo link.

### 2. Matching verifies nothing

`match_ride` (`transport-sharing/coordinator/src/lib.rs:107-136`) binds both
sides as `let _offer` / `let _request` — it confirms the two records *resolve*,
then creates the match. No seat availability check, no status check, no
double-booking prevention, no time-overlap check.

`seats_available` is **never read or decremented anywhere in production code** —
every occurrence in the coordinator sits after line 457, inside
`#[cfg(test)] mod tests`. The same is true of every `OfferStatus::` and
`RequestStatus::` variant: the offer/request state machines exist only as enum
variants exercised by serde round-trip tests. Nothing transitions them.

### 3. The two-party booking handshake is structurally impossible

`confirm_match` takes `UpdateMatchStatusInput { match_hash, new_status }` and
then hardcodes `ride_match.status = MatchStatus::Confirmed`, ignoring
`input.new_status` entirely.

Worse, it calls `update_entry(record.action_address(), ...)`. Holochain only
permits an agent to update actions it authored, so **only the agent who created
the match can ever confirm it** — the counterparty cannot. A rider-initiated
match can never be driver-confirmed, and vice versa. This is a design problem,
not a missing guard.

### 4. Driver ratings compute the inverse quantity, and are forgeable

`RideReview` has `reviewer` but **no reviewee/subject field**.

`review_ride` links the review at `create_link(review.reviewer, ...,
LinkTypes::AgentToReviews, ())` — base is the *reviewer*. `get_driver_rating(agent)`
then reads links based at that same `agent`. So it returns the average rating an
agent **gave to others**, never the rating they received.

Not fixable without a schema change. Separately, `review.reviewer` is
client-supplied and never bound to `agent_info()`, so reviews are forgeable at
the create path.

---

## Other verified gaps

- **No money.** `fare_` returns zero hits anywhere in `mycelix-workspace`. No
  fares, no ticketing, no settlement. `mycelix-finance/zomes/payments` has real
  `debit_sap`/`credit_sap`, with **zero linkage from transport**.
- **No routing or ETA.** Waypoints are stored, never traversed. Nothing computes
  a path or a time. `Route.estimated_minutes` is unvalidated — a test asserts
  `0` is acceptable.
- **No live position.** `send_remote_signal` has 11 call sites in the repo, all
  in `mycelix-pulse`. Commons has none.
- **Carbon credits mint from unverified self-report.** No distance cap, no
  dedupe, `earned_at` taken from the client's `logged_at`, and an `== 0.0`
  bypass (submit `0.0001`). Redemption is non-atomic, with a test asserting a
  `-5.0` balance round-trips.
- **Emission factors are uncited literals** (11 of them, `transport-impact/
  coordinator/src/lib.rs:30`). `Flying: 0.255 // Helicopter/eVTOL average` is
  actually a short-haul *fixed-wing* figure. Carpool is double-discounted:
  `calculate_trip_emissions_carpool_splits` divides again on top of the already
  reduced `TripMode::Carpool` factor, and a test enshrines it.
- **Global anchors, no deletes.** 53 `Anchor("all_*")` link bases across commons
  (37 distinct names), with zero deletes in the transport zomes. Monotonically
  growing link sets, one `GetStrategy::Network` round-trip per offer inside a
  single WASM call. This violates the repo's own
  `memory/feedback_dht_scalability_traps.md`.
- **Link-tag tests are self-referential.** 40 of them assert against a
  re-implementation defined inside `#[cfg(test)]`
  (`transport-sharing/integrity:1111-1141`); they would still pass if the
  production validator were deleted. No test anywhere invokes `validate(op)`.

## What is actually correct here — do not "fix" these

- **Update/delete author binding is right.** All three zomes route through
  `check_author_match` (`mycelix-bridge-entry-types:497-510`), including the
  generic `RegisterUpdate`/`RegisterDelete` dispatcher. This is *not* an
  instance of the P0-#1 unbound-validator pattern. (Defect 3 above is partly a
  *consequence* of getting this right.)
- **`gate_civic` fails closed.** Verified 2026-07-30. `require_civic` →
  `gate_civic` is applied to every mutating extern, 20+ call sites. If
  `commons_bridge` is unreachable, the sovereign path's
  `if let Ok(ZomeCallResponse::Ok(..))` falls through to `gate_consciousness`,
  which uses `call(...)?` and returns `Err` on transport failure, on a non-`Ok`
  response, on decode failure, and on ineligibility. No path grants access on
  error. One wart worth knowing: that fall-through is silent, so a broken
  sovereign-credential path degrades invisibly to the legacy evaluator, which
  has different thresholds.
- **`transport-impact::log_trip` already binds correctly** — it derives `holder`
  and both link bases from `agent_info()`. Leave it out of any author-binding
  sweep.

## If this is ever revived

Create-path author binding is needed on exactly four entities — `Vehicle.owner`,
`RideOffer.driver`, `RideRequest.requester`, `RideReview.reviewer` — plus
`EntryTypes::RideMatch(_) => Valid`, whose create arm validates no fields at
all. That work belongs on the P0-#1 backlog, not on a transport product plan.

Before anything else, run
`cargo build --workspace --target wasm32-unknown-unknown --release` and
`hc dna pack` / `hc app pack`. Check for a `.cargo/config.toml` with
`getrandom_backend="custom"` first — `mycelix-craft` needed one.

---

## Related

- Root `MASTER_ROADMAP.md` → Mycelix section, transport row.
- A shared-code bug found during this audit is **already fixed**:
  `commons_types::geo::geohash_neighbors` returned the ring at grid distance 2,
  breaking proximity queries in 8 call sites across commons *and* civic
  (`8007929573`). If you are reading old proximity-query behaviour, that is why.
