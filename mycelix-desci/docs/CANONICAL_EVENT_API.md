# Canonical Scientific Event API

**Status:** authoritative refoundation write path

The canonical API accepts events that were constructed and signed by the client. A bearer JWT authenticates the HTTP session, but the JWT does not replace the scientific signature. A write is committed only when all of the following hold:

1. The bearer token has the configured issuer and audience, is unexpired, and its subject exactly equals `event.envelope.actor`.
2. The Ed25519 signature verifies over the canonical event bytes.
3. The signing key is active for that actor in the configured authority registry.
4. Any acting organization is present in the resolved actor profile.
5. The actor has the role and relationship required for the event type.
6. The event sequence, previous hash, event ID, idempotency key, evidence references, and projection invariants are valid.
7. The server signs a receipt-time authority snapshot and atomically commits the event, receipt, and publication-outbox row when PostgreSQL is selected. The file reference backend retains its recoverable prepare/append/finalize sequence.

Receipt time is server-observed. Neither an HTTP client nor a direct caller of the governed log can backdate authority evaluation; the governed boundary replaces any supplied storage timestamp with its own current time.

## Endpoints

### `POST /api/v1/scientific/events`

Protected by bearer authentication. The request body contains an optimistic sequence and the complete signed event:

```json
{
  "expected_sequence": 0,
  "event": {
    "envelope": {
      "protocol": "mycelix-desci",
      "protocol_version": 1,
      "codec": "mycelix-canonical-binary-v1",
      "schema_version": 3,
      "event_id": "00000000-0000-4000-8000-000000000001",
      "stream_id": "00000000-0000-4000-8000-000000000002",
      "sequence": 0,
      "previous_hash": null,
      "actor": "did:key:alice",
      "acting_organization": "ror:alice-lab",
      "occurred_at": "2026-08-04T15:00:00Z",
      "idempotency_key": "alice-create-claim-0001",
      "payload": {
        "type": "claim_proposed",
        "research_object": {
          "id": "00000000-0000-4000-8000-000000000003",
          "title": "Registered study result",
          "object_type": "manuscript",
          "persistent_identifier": "doi:10.example/study"
        },
        "claim": {
          "id": "00000000-0000-4000-8000-000000000002",
          "research_object_id": "00000000-0000-4000-8000-000000000003",
          "statement": "Treatment X changes endpoint Y in population Z",
          "scope": "Preregistered endpoint and population"
        }
      }
    },
    "signer_public_key": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "signature": [
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    ]
  }
}
```

The zero values above are placeholders and will not verify. Clients should use the core library's `ScientificEventEnvelope` and `SignedScientificEvent::sign` until independent codec implementations have passed the golden-vector suite.

A successful response contains the append receipt, the signed authority receipt, and the deterministic projection after the commit. Optimistic concurrency and duplicate idempotency keys return HTTP 409. PostgreSQL commits event, receipt, and outbox atomically; the file reference backend reconciles interrupted receipt finalization before readiness can pass.

`legacy_claim_imported` is intentionally rejected by this public endpoint. Legacy migration is an offline operator action.

### `GET /api/v1/scientific/claims/{claim_id}`

Returns the current deterministic projection and its versioned evidence assessment.

### `GET /api/v1/scientific/claims/{claim_id}/events`

Query parameters:

- `from_sequence` defaults to `0`.
- `limit` defaults to `100` and is capped at `500`.

Returns the signed source events. Projections are disposable and can always be rebuilt from this history.

### `GET /api/v1/scientific/claims/{claim_id}/authority-receipts`

Returns the ordered committed receipt chain for a claim plus every event in the stream that lacks a receipt and its classified authority status. This is the preferred compact export for independent receipt-chain auditing.

### `GET /api/v1/scientific/events/{event_id}`

Returns one signed event, its canonical event hash, its signed authority receipt when present, and an explicit authority status: `receipt_attested`, `pending_reconciliation`, `legacy_unattested`, `unsafe_unattested`, or `unknown`. A contiguous pre-receipt prefix is `legacy_unattested`; a missing receipt at or after a stream’s first receipt is `unsafe_unattested` and cannot be waived.

### `GET /api/v1/scientific/events/{event_id}/authority-receipt`

Returns the independently persisted receipt-time authority evidence and receipt hash. The receipt binds the event hash, actor key, active key interval, organization membership, roles, authorization action, policy identifier and version, decision reason, server receipt time, and previous receipt hash.

## Scientific credential registry

Set `JWT_SECRET`, `JWT_ISSUER`, and `JWT_AUDIENCE` for bearer authentication. Tokens issued for another service are rejected even when they use the same HMAC secret.

Canonical identity authority comes from the append-only scientific credential registry:

- `DESCI_CREDENTIAL_REGISTRY_BACKEND=file`
- `DESCI_CREDENTIAL_REGISTRY_PATH=./data/scientific-credentials.json`
- `DESCI_CREDENTIAL_BOOTSTRAP_TRUST_FILE=./config/credential-bootstrap-trust.json`
- `DESCI_MIN_CREDENTIAL_REGISTRY_ADMINS=2`
- `DESCI_CREDENTIAL_GOVERNANCE_BACKEND=file`
- `DESCI_CREDENTIAL_GOVERNANCE_PATH=./data/scientific-credential-governance.json`
- `DESCI_MIN_CHECKPOINT_WITNESS_ORGANIZATIONS=2`

The bootstrap file is a JSON array of 32-byte public keys encoded as 64 hexadecimal characters. It authorizes only registry genesis. Genesis is deliberately unavailable through HTTP and must be created with the offline `credential-registry-init` command.

After at least two administrators exist, initialize threshold governance offline. Subsequent actor registration, key rotation, role changes, organization membership, compromise notices, service-key changes, witness authority changes, and revocations are signed proposals. A governed risk policy snapshots tier-specific administrator thresholds, organization diversity, and activation delays. The direct credential endpoint returns HTTP 410 after governance initialization. See [Threshold Credential Governance](THRESHOLD_CREDENTIAL_GOVERNANCE.md), [Risk-Tiered Scientific Credential Governance](RISK_TIERED_CREDENTIAL_GOVERNANCE.md), and [Scientific Credential Registry](SCIENTIFIC_CREDENTIAL_REGISTRY.md).

Set `DESCI_AUTHORITY_RECEIPT_SIGNING_KEY_FILE` to a distinct mode-0600 Ed25519 seed and `DESCI_SCIENTIFIC_AUTHORITY_AUDIT_PATH` to the independent authority journal. For receipt-key rotation, keep prior public keys in `DESCI_AUTHORITY_RECEIPT_TRUST_FILE`.

Authority-receipt protocol v2 binds the exact credential-registry head used to resolve the actor. A valid event signature is rejected when the key was not active for that actor, the actor lacked the required role, or the claimed acting organization was absent at that revision.

## Schema compatibility

- Schema v2 native events remain verifiable and replayable.
- New events are emitted as schema v3.
- `legacy_claim_imported` requires schema v3.
- Adding an event payload does not change the canonical bytes of existing payload types because every event type has an explicit numeric codec tag.

## File-backend trust boundary

The reference event backend verifies signatures, stream identifiers, filenames, file-size bounds, event IDs, idempotency keys, sequence continuity, hash chaining, and deterministic projection rebuilding. The separate authority journal uses prepared and committed receipt files so interrupted writes can be reconciled. Startup verifies trusted receipt-service signatures, service/event key separation, exact event binding, event-time skew, key validity at `received_at`, organization membership snapshots, policy identity, bounded receipt fields, and per-stream receipt-chain continuity. The journal rejects parallel receipt forks at the same stream sequence before event commit.

Current key or role changes do not retroactively invalidate a legitimate historical acceptance: the receipt preserves the exact authority snapshot used at receipt time. Conversely, old events without receipts remain explicitly unattested. Set `DESCI_ALLOW_UNATTESTED_AUTHORITY_HISTORY=true` only as a bounded acknowledgement of a contiguous pre-receipt prefix; it does not manufacture missing evidence and never permits an unsafe receipt gap after journaling begins.

## PostgreSQL and checkpoint-mirror endpoints

Set `DESCI_SCIENTIFIC_EVENT_BACKEND=postgres` and `DESCI_POSTGRES_URL` to use
the multi-process canonical authority backend. Direct SQL event append is
disabled; all writes pass through governed authorization and the atomic commit
interface.

- `POST /api/v1/scientific/authority/governance/checkpoint-mirrors` records one
  client-signed mirror observation. The JWT subject must match its witness actor.
- `GET /api/v1/scientific/authority/governance/checkpoint-mirrors/{hash}` exports
  all observations for a checkpoint with their current compromise-aware validity.

The mirror URI must be HTTPS, immutable, contain the checkpoint hash in its
path, and omit credentials, query parameters, and fragments. External verifiers
must fetch and hash the document;
recording an observation alone does not prove continued availability.

An exact replay of an already committed signed event returns its original append
receipt without creating a second event. Clients should retain and retry the same
signed envelope, including its event ID; changing the event ID creates a distinct
command and does not receive retry semantics.

The PostgreSQL outbox is delivered at least once when
`DESCI_AUTHORITY_OUTBOX_WEBHOOK_URL` is configured. Subscribers must deduplicate.

## Governed database epoch endpoints

PostgreSQL authority deployments expose a separate promotion and recovery
surface. Database-epoch writes are not ordinary infrastructure administration;
they require signed protocol objects already authorized through critical
threshold governance.

- `GET /api/v1/scientific/authority/database-epochs`
- `GET /api/v1/scientific/authority/database-epochs/state-commitment`
- `GET /api/v1/scientific/authority/database-epochs/{epoch_number}`
- `POST /api/v1/scientific/authority/database-epochs`
- `POST /api/v1/scientific/authority/recovery-reconciliations`
- `POST /api/v1/scientific/authority/delivery-acknowledgements`
- `GET /api/v1/scientific/authority/deliveries/{delivery_id}/acknowledgements`

The state-commitment endpoint returns a checkpoint-bound candidate. Any
committed authority change makes that candidate stale. The epoch commit path
recomputes the state under an exclusive SQL barrier and fails on disagreement.
See `AUTHORITY_DATABASE_EPOCHS.md`.
