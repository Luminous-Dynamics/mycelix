# Creator Sovereignty Core (experimental)

A standard-library-only Rust prototype for deterministic creator career accounting. This is a **research crate**, not a production service and not yet an integrated Mycelix hApp.

## Current scope

- Exact minor-unit financial arithmetic without cross-currency aggregation.
- Distinct receipts, refunds, fees, and operating costs.
- Creator-recorded work sessions with checked time arithmetic and overlap rejection, so one person's overlapping sessions cannot inflate the work-hour denominator.
- Idempotent identical replays; fail-closed rejection when the same record ID is reused for different data.
- Exact rational net-contribution-per-work-hour representation only when the ledger has one currency and recorded work time; missing time or multiple currencies returns an explicit unavailable reason.
- Independent summary checker implemented with a separate aggregation path.
- A deterministic revenue stress-scenario module for explicit user-entered source declines; it is labeled as a what-if, never a forecast, holds deductions constant visibly, marks data coverage, keeps currencies separate, and uses conservative minor-unit rounding.
- Hostile/golden tests for arithmetic, tampering, higher gross receipts with worse economics, mixed-currency abstention, source shocks, incomplete evidence, and input rejection. Source contains 10 ledger unit tests, 9 ledger integration tests, and 12 scenario unit tests; these are not yet executed because the current environment has no Rust toolchain.
- No network, secrets, telemetry, database, serialization library, or third-party dependencies.

## Run

From the repository root:

```sh
cargo test --manifest-path mycelix-workspace/research/creator-sovereignty-core/Cargo.toml
```

## Explicit limitations

- No CSV/JSON parser or export adapter yet.
- Scenario v1 holds refunds, fees, and operating costs constant when a source declines. This is an explicit stress assumption, not a prediction of how variable costs change.
- Scenario inputs must share a declared window and currency-specific deductions; partial coverage stays visibly partial.
- Currency validation checks three uppercase ASCII letters, not whether a code is currently assigned by ISO 4217.
- No conversion between currencies, tax treatment, accrual accounting, invoices, or legal/tax advice.
- Refunds are classified as negative contribution but are not yet required to reference and reconcile to an original receipt.
- A record digest/manifest and standalone serialized-input verifier remain future tasks; this checker verifies an in-memory summary against its source ledger.
- Work time is creator-recorded and is not independently measured.
- A high revenue-per-hour figure is not a measure of artistic merit, creator health, or causation.

These limits are intentional: establish precise semantics before layering import adapters, distributed storage, or AI recommendations on top.
