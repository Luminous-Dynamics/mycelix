# Refoundation Tranche 11 — External Write Fencing and Remote Signing

Date: 2026-08-05

## Objective

Prevent an isolated former PostgreSQL primary from continuing authoritative
writes merely because its local database process and advisory locks remain
available. Add an explicit hardware/remote signer path and define deterministic
failover and PITR validation evidence.

## Implemented

- Versioned canonical `AuthorityWriteLease` protocol.
- Domain-separated Ed25519 signatures and independent signature verification.
- Strict lease expiration with no post-expiry clock-skew grace.
- Deployment, primary, PostgreSQL system identifier, timeline, generation,
  database epoch, scope, activation, and expiration binding.
- Static and atomically replaceable file lease providers.
- Durable per-deployment generation and lease-hash memory in PostgreSQL.
- Fencing of scientific, credential, governance, epoch, recovery,
  acknowledgement, mirror, outbox, and schema-migration writes.
- Schema-v4 migration and read-only structural verification.
- PostgreSQL readiness status exposing active lease identity and expiry.
- Fenced offline credential/governance import.
- CLI signing of unsigned lease documents.
- Unix-domain remote `AuthoritySigner` with timeouts, response limits, key-ID
  binding, public-key binding, and caller-side Ed25519 verification.
- Production examples with runtime migration disabled and a directory-mounted
  atomically rotated lease.
- Deterministic failover/PITR validation campaign.

## Security invariants

- Lease-issuer keys are distinct from every other authority domain.
- Expiration is absolute.
- A generation cannot move backward or fork into different signed bytes.
- Ordinary writes after the first database epoch bind the exact latest epoch.
- Epoch promotion binds the exact candidate certificate and database identity.
- Runtime DDL requires an explicit migration scope before any domain DDL.
- PostgreSQL deployments require fencing by default.

## Deliberate boundary

This tranche does not claim to provide an external lease consensus service or a
vendor-specific PKCS#11/KMS daemon. The Unix signer adapter is a narrow verified
client boundary. Live PostgreSQL, partition, failover, PITR, HSM, and outbox
integration remain mandatory before release.
