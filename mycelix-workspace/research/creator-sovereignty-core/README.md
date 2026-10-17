# Creator Sovereignty Core (experimental)

A standard-library-only Rust prototype for deterministic creator career accounting. This is a **research crate**, not a production service and not yet an integrated Mycelix hApp.

## Current scope

- Exact minor-unit financial arithmetic without cross-currency aggregation.
- Distinct receipts, refunds, fees, and operating costs.
- Creator-recorded work sessions with checked time arithmetic and overlap rejection, so one person's overlapping sessions cannot inflate the work-hour denominator.
- Idempotent identical replays; fail-closed rejection when the same record ID is reused for different data.
- Exact rational net-contribution-per-work-hour representation; missing work duration remains unknown rather than becoming zero.
- Independent summary checker implemented with a separate aggregation path.
- Hostile/golden unit tests; no network, secrets, telemetry, database, or third-party dependencies.

## Run

From the repository root:

```sh
cargo test --manifest-path mycelix-workspace/research/creator-sovereignty-core/Cargo.toml
```

## Explicit limitations

- No CSV/JSON parser or export adapter yet.
- Currency validation checks three uppercase ASCII letters, not whether a code is currently assigned by ISO 4217.
- No conversion between currencies, tax treatment, accrual accounting, invoices, or legal/tax advice.
- Refunds are classified as negative contribution but are not yet required to reference and reconcile to an original receipt.
- A record digest/manifest and standalone serialized-input verifier remain future tasks; this checker verifies an in-memory summary against its source ledger.
- Work time is creator-recorded and is not independently measured.
- A high revenue-per-hour figure is not a measure of artistic merit, creator health, or causation.

These limits are intentional: establish precise semantics before layering import adapters, distributed storage, or AI recommendations on top.
