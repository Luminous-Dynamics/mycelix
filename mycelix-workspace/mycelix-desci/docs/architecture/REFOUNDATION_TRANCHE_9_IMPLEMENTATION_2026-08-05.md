# Refoundation Tranche 9 — Unified SQL Authority and Signed Delivery

This tranche moves scientific credentials and threshold governance into the
same PostgreSQL authority boundary introduced for scientific events.

## Implemented

- Version-2 PostgreSQL authority schema and external migration.
- Transactional credential and governance journals with indexed/JSON replay
  cross-checks.
- Atomic proposal execution across optional credential mutation, governance
  execution, and publication outbox.
- Dedicated signed authority-delivery envelopes.
- SQL-backed startup and per-request projection synchronization.
- Fail-closed rejection of mixed PostgreSQL/file authority configurations.
- Atomic offline import from validated file histories into an empty SQL schema.
- Read-only relational recovery verification and a PITR/failover runbook.
- Repair of v0.9 textual merge defects in PostgreSQL startup code.

## Deliberate limits

The file backend remains a deterministic reference implementation. PostgreSQL
integration, failover, PITR, and lock-contention tests still require a real
server. `Cargo.lock` must be regenerated after SQLx resolution with the pinned
Rust toolchain before merge.
