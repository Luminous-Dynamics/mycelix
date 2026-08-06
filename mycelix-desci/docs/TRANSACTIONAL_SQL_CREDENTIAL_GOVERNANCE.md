# Transactional SQL Credential and Governance Authority

> **v0.9 upgrade precondition:** drain or explicitly publish every pending schema-v1 outbox row before applying schema v2. Those rows predate signed delivery envelopes and cannot be retroactively signed without changing their historical acceptance semantics. Delivered v1 rows may remain as historical transport records.

## Boundary

When all three authority backends are `postgres`, PostgreSQL is authoritative
for:

- scientific credential records;
- threshold-governance records;
- atomic proposal execution that optionally mutates the credential registry;
- scientific events and authority receipts; and
- signed publication-outbox rows.

A proposal execution that changes credentials commits the credential record,
the governance execution record, and both publication messages in one
serializable transaction. Advisory transaction locks establish a global order
between credential and governance writers. Durable heads are reread inside the
transaction; stale processes fail instead of overwriting history.

## Startup configuration

```text
DESCI_SCIENTIFIC_EVENT_BACKEND=postgres
DESCI_CREDENTIAL_REGISTRY_BACKEND=postgres
DESCI_CREDENTIAL_GOVERNANCE_BACKEND=postgres
DESCI_POSTGRES_URL=postgres://...
DESCI_AUTHORITY_RECEIPT_SIGNING_KEY_FILE=/run/secrets/receipt.key
DESCI_AUTHORITY_OUTBOX_SIGNING_KEY_FILE=/run/secrets/outbox.key
```

Selecting PostgreSQL for scientific events while selecting file or memory for
credential/governance state is rejected to prevent split-brain authority.

## Offline cutover

Network genesis remains forbidden. Bootstrap and validate the credential and
governance journals with the offline file commands, stop every writer, then run:

```bash
mycelix-desci credential-authority-import-postgres \
  --database-url "$DESCI_POSTGRES_URL" \
  --registry ./data/scientific-credentials.json \
  --governance ./data/scientific-credential-governance.json \
  --bootstrap-trust-file ./config/credential-bootstrap-trust.json \
  --acceptance-signing-key-file ./config/authority-receipt-signing.key \
  --outbox-signing-key-file ./config/authority-outbox-signing.key
```

The destination credential and governance tables must be empty. Both histories
and all generated signed outbox messages commit in one transaction. Keep the
source journals read-only until PostgreSQL startup, replay, and checkpoint
verification have succeeded.

## Read synchronization

Each authority-sensitive API read refreshes credential state first and
threshold-governance state second. This preserves their dependency order across
multiple API replicas. Writes independently resynchronize and then rely on
serializable database head checks.
