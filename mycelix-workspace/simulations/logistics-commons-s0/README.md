# Logistics Commons S0 — deterministic offline twin

This is an **effects-disabled synthetic model**, not a production logistics service. It uses only the Rust standard library and is intentionally isolated from the Holochain workspace. It opens no network connections, has no credentials, mutates no real inventory, and does not move money or vehicles.

## Run

From the repository root:

```sh
cargo +1.99.0 test --locked --manifest-path mycelix-workspace/simulations/logistics-commons-s0/Cargo.toml
cargo +1.99.0 run --quiet --locked --manifest-path mycelix-workspace/simulations/logistics-commons-s0/Cargo.toml
# Emit the full manifest + input corpus + outcomes + summary artifact
cargo +1.99.0 run --quiet --locked --manifest-path mycelix-workspace/simulations/logistics-commons-s0/Cargo.toml -- --canonical
```

The executable generates a fixed synthetic scenario from the checked-in manifest. The correct last-unit hot-spot result is an explicit unresolved conflict, not two confirmations and not an arbitrary winner. Canonical artifacts are stable for the same source, manifest, and seed.

## First-slice guarantees

- Inventory snapshots carry an observed-at/valid-until interval and exact revision. Stale, future, malformed, or mismatched observations fail closed.
- Reservations carry an idempotency key and lease. Replays coalesce; a key reused with a different effect payload is rejected.
- An over-subscribed concurrent batch is marked unresolved instead of being resolved by arrival order.
- Shipment transitions separate pick/pack, dispatch, carrier delivery report, and recipient acceptance. Recipient acceptance requires separate evidence; invalid transitions become unresolved.
- The invariant checker is separately written against outputs and original inputs. It checks duplicate IDs, inventory bounds, revision/freshness, idempotency, conflict sets, and shipment evidence ordering; it does not invoke the planner or shipment reducer.
- A public shipment projection contains neither the customer address, supplier price, nor worker route.
- Forecast observations carry version/input/uncertainty metadata but are not accepted by the reservation planner. A forecast cannot confirm stock.

## Current limits

The frozen manifest explicitly versions the Xorshift64* generator and SKU mapping, records the seed and quantity ranges, and fixes the hot-spot hub/SKU indices. The corpus covers 24 organizations, 8 hubs, all 2,048 catalog SKUs, and 12,000 orders, including a controlled 64-order hot spot for the last unit. Its canonical artifact includes the checked-in scenario manifest, every generated input, canonical reconciliation output, and a machine-readable summary. The partition-reconciliation module treats split-brain claims as unresolved and other capacity-fitting claims as pending until a fresh authoritative inventory revision is checked. This still does not implement real network partitions, physical stock adapters, transport routing/capacity, Holochain DHT validation, or Symthaea inference, and it does not establish Amazon/DLA-scale throughput or real-world reliability.

The S0 generator uses a frozen seed, a checked-in manifest, fixed synthetic observation times, and an explicitly non-cryptographic PRNG used only for reproducible fixture generation. CI uses Rust 1.99.0, runs the Rust unit suite, reproduces the complete artifact twice, compares it byte-for-byte, and reports its SHA-256 digest. Only after those gates pass does it emit a machine-readable qualification receipt tied to the exact source commit and workflow-run/attempt IDs, with hashes for both the manifest and full corpus; the receipt is an unsigned CI record, not an external attestation. The independent checker verifies the reconciled outcomes against the original inputs. Partition reconciliation never emits a committed/confirmed state: it emits either an explicit conflict, rejection/duplicate alias, or awaiting-authoritative-recheck.

## Acceptance posture

Only an exact-head successful automated run constitutes a CI pass. A queued, pending, missing, or stale run is not a pass. Simulation output is evidence about this model only, not proof that an external warehouse event occurred.
