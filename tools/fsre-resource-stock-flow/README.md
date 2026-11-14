# FSRE resource stock-flow reference v0.1

**Status:** Dependency-free deterministic arithmetic reference over one frozen resource and contiguous time horizon. Synthetic fixture only. Not an inventory ledger, replenishment order system, scheduler, or operational control.

## Purpose

Verify owner-supplied opening stock, replenishment received, consumption, losses and closing stock over successive intervals:

`closing = opening + replenishment - consumption - loss`

The reference checks non-negative arithmetic, inter-interval continuity, contiguous time boundaries and exact units. It abstains across unknown intervals and rejects later known intervals unless a new independent series boundary is supplied as a separate evaluation.

## Input contract

Metadata declares scenario, resource ID, unit and time basis. Rows have a stable interval index, interval ID, start/end seconds, quantities, source owner/reference and KNOWN/UNKNOWN state. KNOWN intervals require all five quantities. UNKNOWN intervals cannot assert a closing balance; unknown stock continuity propagates through the remainder of the series.

Quantities are non-negative integers. No implicit unit conversion, interpolation, stock substitution or replenishment credit is performed. Source evidence is referenced but not authenticated.

## Verification

`cargo test --locked --offline --manifest-path tools/fsre-resource-stock-flow/Cargo.toml`

`cargo run --locked --offline --manifest-path tools/fsre-resource-stock-flow/Cargo.toml -- tools/fsre-resource-stock-flow/fixtures/synthetic-balanced.psv`

The tool does not own inventory records, place orders, schedule deliveries, assign operators or actuate infrastructure. It computes whether a frozen source-backed resource series is arithmetically and temporally consistent.
