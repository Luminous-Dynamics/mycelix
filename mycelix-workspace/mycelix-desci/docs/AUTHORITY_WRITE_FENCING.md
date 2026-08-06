# Externally Signed PostgreSQL Write Fencing

PostgreSQL advisory locks serialize cooperating writers, but they cannot stop an
isolated former primary. Mycelix-DeSci therefore requires a short-lived,
externally signed write lease for every production authority mutation.

The lease is a public authorization artifact, not a secret. It binds one:

- deployment identifier;
- candidate or active primary identifier;
- PostgreSQL system identifier and timeline;
- monotonic lease generation;
- bootstrap state or exact governed database epoch number and hash;
- set of permitted mutation scopes; and
- activation and strict expiration time.

A former primary can continue writing only until its last independently issued
lease expires. Clock-skew allowance may admit a lease slightly before
`not_before`; it never extends `expires_at`.

## Trust boundary

The lease issuer must be operationally independent from the PostgreSQL primary.
Do not store the lease-issuer private key on the API host or database host. A
production issuer should make renewal conditional on independent primary
membership, timeline, epoch, and incident-state observations.

The reference CLI signs an already prepared lease:

```bash
cargo run --locked --release \
  --package mycelix-desci-core \
  --bin mycelix-desci -- \
  authority-write-lease-sign \
  --lease-file ./runtime/authority/unsigned-write-lease.json \
  --signing-key-file /secure/authority-write-lease.seed \
  --output ./runtime/authority/current-write-lease.json
```

The output must be replaced atomically. When containerized, mount the containing
directory rather than an individual file so inode replacement is visible.

## Independent verification

A separate Python verifier reimplements the canonical v1 codec and does not call
into the Rust service:

```bash
python3 scripts/verify-authority-write-lease.py \
  /run/authority/current-write-lease.json \
  --trust-file ./config/authority-write-lease-trust.json \
  --deployment-id mycelix-desci-production \
  --primary-id postgres-primary-a \
  --database-system-identifier "$(psql -Atc 'SELECT system_identifier FROM pg_control_system()')" \
  --postgres-timeline "$(psql -Atc 'SELECT timeline_id FROM pg_control_checkpoint()')" \
  --scope scientific_event
```

It requires Python's `cryptography` package and verifies exact canonical bytes,
Ed25519 signature, issuer trust, strict lifetime, generation floor, optional
epoch binding, and requested scope. The golden vector lives in
`tests/vectors/authority-write-lease-v1.signed.json`.

## Required environment

```text
DESCI_REQUIRE_AUTHORITY_WRITE_FENCING=true
DESCI_AUTHORITY_DEPLOYMENT_ID=mycelix-desci-production
DESCI_AUTHORITY_WRITE_LEASE_FILE=/run/authority/current-write-lease.json
DESCI_AUTHORITY_WRITE_LEASE_TRUST_FILE=/app/config/authority-write-lease-trust.json
DESCI_AUTHORITY_WRITE_LEASE_CLOCK_SKEW_SECONDS=30
```

The trust file is a nonempty JSON array of 64-character Ed25519 public-key hex
strings. Lease keys must not overlap scientific actor, receipt, acceptance,
outbox, epoch, or witness keys.

## Scopes

The protocol currently defines independent scopes for:

- scientific-event commits;
- credential-registry mutations;
- credential-governance mutations;
- database-epoch promotion;
- recovery reconciliation;
- delivery acknowledgement;
- checkpoint mirrors;
- outbox delivery; and
- schema migration.

A lease authorizes only its listed scopes. Runtime DDL additionally requires the
`schema_migration` scope. Production deployments should normally apply schema
migrations through a controlled job and set `DESCI_POSTGRES_AUTO_MIGRATE=false`.

## Bootstrap and epoch leases

Before the first governed database epoch, only a bootstrap lease is valid. It
must omit epoch number and hash while binding the actual PostgreSQL system
identifier and timeline.

After an epoch is committed, every ordinary write lease must bind the exact
latest epoch number and hash. Promotion itself uses a candidate epoch lease that
binds the signed certificate's candidate primary, system identifier, timeline,
epoch number, and computed epoch hash.

## Failover sequence

1. Stop renewing the old primary's lease.
2. Promote the candidate through the governed database-epoch ceremony.
3. Issue a candidate lease permitting only the required promotion operation.
4. Commit the signed epoch certificate.
5. Issue a higher-generation epoch lease for the new primary and normal scopes.
6. Atomically publish the new lease file to API and worker processes.
7. Confirm readiness reports the new primary, timeline, epoch, generation, and
   future expiration.
8. Keep the old primary isolated until its prior lease has strictly expired.

A lease generation may increase. It may not decrease, and one generation may
never refer to two different signed leases for the same deployment.

## Fail-closed conditions

Authority mutation is rejected when any of the following is true:

- the lease is absent, malformed, untrusted, not active, or expired;
- the requested scope is absent;
- deployment, primary, system identifier, timeline, or epoch does not match;
- the generation is lower than durable state;
- the same generation is reused with different signed bytes; or
- the configured lease key overlaps another authority domain.

## Current boundary

The repository provides the signed lease protocol, file provider, CLI signer,
SQL enforcement, generation memory, and readiness reporting. It does not ship an
independently operated lease consensus service. Production release therefore
still requires a real issuer, PostgreSQL integration tests, network-partition
fault injection, and observed stale-primary rejection after strict expiry.
