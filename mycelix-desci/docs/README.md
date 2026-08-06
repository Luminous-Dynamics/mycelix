# Mycelix-DeSci Documentation

This index separates the current signed-event authority model from historical
compatibility material. The refoundation remains experimental and requires the
validation listed in the repository README before release.

## Current authority model

- [Canonical Scientific Event API](CANONICAL_EVENT_API.md)
- [Scientific Credential Registry](SCIENTIFIC_CREDENTIAL_REGISTRY.md)
- [Threshold Credential Governance](THRESHOLD_CREDENTIAL_GOVERNANCE.md)
- [Risk-Tiered Credential Governance](RISK_TIERED_CREDENTIAL_GOVERNANCE.md)
- [Authority Receipts and Replay Audit](AUTHORITY_RECEIPTS.md)
- [External Transparency Witnesses](EXTERNAL_TRANSPARENCY_WITNESSES.md)
- [Checkpoint Mirrors and Witness Compromises](CHECKPOINT_MIRRORS_AND_COMPROMISES.md)
- [PostgreSQL Canonical Authority Backend](POSTGRES_AUTHORITY_BACKEND.md)
- [Transactional SQL Credential Governance](TRANSACTIONAL_SQL_CREDENTIAL_GOVERNANCE.md)
- [Signed Authority Delivery](SIGNED_AUTHORITY_DELIVERY.md)
- [PostgreSQL Recovery Runbook](POSTGRES_RECOVERY_RUNBOOK.md)
- [Governed Authority Database Epochs](AUTHORITY_DATABASE_EPOCHS.md)
- [Epoch Promotion and Recovery Runbook](POSTGRES_EPOCH_RECOVERY_RUNBOOK.md)
- [Signed Authority Delivery Acknowledgements](SIGNED_AUTHORITY_DELIVERY_ACKNOWLEDGEMENTS.md)
- [Hardware-Backed Authority Signing](HARDWARE_BACKED_AUTHORITY_SIGNING.md)
- [Externally Signed Authority-Write Fencing](AUTHORITY_WRITE_FENCING.md)
- [Failover and PITR Validation Scenarios](POSTGRES_FAILOVER_PITR_VALIDATION.md)
- [Transactional Reference File Storage](TRANSACTIONAL_FILE_STORAGE.md)
- [Legacy Claim Migration](LEGACY_MIGRATION.md)

## Refoundation implementation records

- [Tranche 1 — Event Kernel](architecture/REFOUNDATION_TRANCHE_1_IMPLEMENTATION_2026-08-04.md)
- [Tranche 2 — Identity and Authorization](architecture/REFOUNDATION_TRANCHE_2_IMPLEMENTATION_2026-08-04.md)
- [Tranche 4 — Authority Receipts](architecture/REFOUNDATION_TRANCHE_4_IMPLEMENTATION_2026-08-04.md)
- [Tranche 5 — Credential Registry](architecture/REFOUNDATION_TRANCHE_5_IMPLEMENTATION_2026-08-04.md)
- [Tranche 6 — Threshold Governance](architecture/REFOUNDATION_TRANCHE_6_IMPLEMENTATION_2026-08-04.md)
- [Tranche 7 — Risk Tiers and Witnesses](architecture/REFOUNDATION_TRANCHE_7_IMPLEMENTATION_2026-08-05.md)
- [Tranche 8 — PostgreSQL Authority and Mirrors](architecture/REFOUNDATION_TRANCHE_8_IMPLEMENTATION_2026-08-05.md)
- [Tranche 9 — Unified SQL Authority and Signed Delivery](architecture/REFOUNDATION_TRANCHE_9_IMPLEMENTATION_2026-08-05.md)
- [Tranche 10 — Governed Database Epochs](architecture/REFOUNDATION_TRANCHE_10_IMPLEMENTATION_2026-08-05.md)
- [Tranche 11 — External Write Fencing and Remote Signing](architecture/REFOUNDATION_TRANCHE_11_IMPLEMENTATION_2026-08-05.md)

## Operator entry points

- [Repository README](../README.md)
- [Security Policy](../SECURITY.md)
- [Contributing Guidelines](../CONTRIBUTING.md)
- [CLI Usage](guides/cli-usage.md)

## Historical compatibility material

The following documents predate the signed-event refoundation. Each carries a
warning and is retained for migration context only:

- [Legacy Architecture](ARCHITECTURE.md)
- [Legacy Deployment Guide](DEPLOYMENT.md)
- [Legacy Performance Baseline](PERFORMANCE.md)
- [Legacy Quick Start](QUICKSTART.md)
- [Legacy Nix Guide](NIX.md)
- [Legacy Getting Started Guide](guides/getting-started.md)

Do not use historical E-tier progression, mutable verification counts, mock
proof paths, or old production-readiness claims as current protocol behavior.
