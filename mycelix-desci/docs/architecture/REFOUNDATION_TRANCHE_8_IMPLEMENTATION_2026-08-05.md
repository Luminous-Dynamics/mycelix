# Refoundation Tranche 8 — Transactional Authority and External Mirrors

Date: 2026-08-05

## Objective

Move canonical scientific acceptance from cooperating file transactions to a
multi-process PostgreSQL boundary, preserve reliable external publication through
an atomic outbox, and make transparency evidence resilient to later witness
compromise discoveries.

## Implemented

- `PostgresAuthorityStore` implementing canonical event reads, authority-audit
  reads, and atomic governed commits.
- Serializable per-stream event + receipt + outbox transactions.
- Named schema ownership with conflict detection before domain DDL and an
  operator-controlled runtime migration switch.
- Exact signed-event retry semantics that return the original append receipt
  without creating a duplicate mutation.
- Domain projection rebuilding inside the authoritative SQL commit path.
- SQL startup reconciliation of scientific projections, indexed-column bindings,
  and immediately adjacent receipt hash chains.
- Leased at-least-once outbox publisher for multiple API replicas.
- Signed checkpoint-mirror observations persisted with their outbox record.
- Governance-controlled witness-compromise intervals.
- Readiness that revalidates mirror organizations against current compromise
  knowledge rather than counting stored rows.
- Docker and environment examples selecting PostgreSQL for canonical event and
  receipt state.

## Deliberately incomplete

Credential and threshold-governance runtime state remain on their existing
transactional file journals. SQL tables are present only as explicit future
migration targets. A later tranche must extract shared repository interfaces and
commit credential proposal execution, registry mutation, governance acceptance,
and publication outbox in one serializable SQL transaction.

No claim is made that the webhook receiver, checkpoint mirror, or database backup
has been independently operated or externally audited.

## Required pre-merge validation

1. Regenerate `Cargo.lock` using Rust 1.96.
2. Run formatting, Clippy, all workspace tests, and SQLx compile checks.
3. Run PostgreSQL integration tests for serialization conflicts, idempotency,
   receipt-chain forks, rollback, outbox leasing, and replica races.
4. Exercise backup/restore and point-in-time recovery.
5. Verify mirror documents from a separate network and organization.
