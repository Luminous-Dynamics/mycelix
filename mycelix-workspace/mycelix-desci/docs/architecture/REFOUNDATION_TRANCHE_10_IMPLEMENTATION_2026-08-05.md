# Refoundation Tranche 10 — Governed Database Epochs

Date: 2026-08-05

## Purpose

Tranche 10 makes PostgreSQL primary promotion and disaster recovery part of the
scientific authority protocol. It prevents infrastructure operators from
silently changing the authoritative database without a threshold-governed,
cryptographically committed transition.

## Implemented

- Versioned promotion intents for initial activation, planned failover, and
  disaster recovery.
- Critical-risk governance action for an exact promotion intent.
- Dedicated signed database-epoch certificates.
- Exact authority-state commitments under an exclusive SQL barrier.
- Hash-chained epoch history with deployment continuity.
- Multi-person bounded emergency-recovery ceremonies.
- Exact signed recovery reconciliation after PITR or disaster recovery.
- Signed publication outbox linkage for epochs and reconciliation evidence.
- Organization-diverse, immutable-reference delivery acknowledgements.
- Pluggable authority signer interface for HSM/KMS integration.
- Readiness gates for initial epoch, disaster-recovery reconciliation, and
  independent delivery acknowledgement.
- PostgreSQL schema v3 and read-only relational recovery checks.
- Exact retry semantics for epoch certificates, recovery reconciliations, and delivery acknowledgements.
- Upgrade preconditions evaluated before domain DDL, leaving blocked migrations untouched.

## Security properties

- Epoch keys cannot overlap actor, receipt, acceptance, witness, or delivery
  keys.
- A promotion certificate must exactly match an executed governance proposal.
- State capture and epoch persistence exclude cooperating authority writers.
- Replay verifies every database row, predecessor hash, signed certificate,
  linked outbox envelope, and indexed publication field.
- Disaster-recovery targets cannot postdate incident declaration.
- At least two named ceremony participants must be actual governance approvers.
- Recovery reconciliation must match the exact governed primary, system,
  timeline, LSN, recovery target, operator, epoch, and authority-state heads.

## Deliberate limits

The SQL advisory barrier coordinates this application's writers only. It does
not implement PostgreSQL fencing or consensus. The signer trait is an adapter
boundary, not a supplied HSM integration. No Rust compilation or PostgreSQL
integration test was possible in the patch-generation environment.

## Next work

- Compile and run all workspace and SQL integration tests.
- Generate cross-language golden vectors for epoch, reconciliation, and
  acknowledgement codecs.
- Implement and review a production HSM/KMS signer.
- Exercise planned failover, PITR, split-brain containment, and backup restore
  against real PostgreSQL clusters.
- Add externally cosigned database-epoch checkpoints and formal operational
  ceremony tooling.
