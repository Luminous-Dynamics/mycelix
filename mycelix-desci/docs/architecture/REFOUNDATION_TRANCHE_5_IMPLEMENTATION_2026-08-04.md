# Refoundation Tranche 5: Governed Scientific Credential Registry

**Date:** 2026-08-04
**Protocol release:** 0.6.0
**Status:** implementation tranche; Rust build validation still required

## Purpose

Tranche 4 made each accepted scientific event independently auditable through a
signed authority receipt. Its remaining weakness was the source of the actor,
key, role, and organization snapshot: mutable startup JSON could be replaced
without leaving a governed history.

Tranche 5 makes authority itself append-only. Startup configuration now contains
only a narrow genesis trust set and the current authority-service key. Every
scientific identity or authorization change is represented by a signed,
replayable credential event.

## Security goals

The tranche is designed to make the following statements independently
verifiable:

1. a named registry administrator signed each credential transition;
2. that administrator key was active when the server accepted the transition;
3. the server-observed acceptance time was signed by a separate trusted service;
4. every transition extended exactly one prior recorded registry revision;
5. scientific authority receipts name the exact registry revision used for
   acceptance;
6. later key rotation or role revocation cannot rewrite historical acceptance;
7. one actor key cannot be shared or rebound to another actor;
8. governance cannot accidentally remove every usable recovery administrator.

This does not claim distributed consensus or threshold governance. The current
reference backend remains a single-process append-only file store.

## Credential protocol

The new canonical protocol is implemented in
`src/core/src/scientific_credentials.rs`.

### Signed event envelope

A `ScientificCredentialEnvelope` contains:

- protocol, protocol version, codec, and schema version;
- event ID and monotonic sequence;
- the previous **record hash**;
- administrator actor ID;
- actor-observed occurrence time;
- actor-scoped idempotency key;
- one typed credential transition.

The explicit canonical binary codec is domain separated from scientific events,
authority receipts, and credential acceptance records. Unknown JSON fields are
rejected before signature verification so wire-level extensions cannot be
silently discarded and replayed as the same signed object.

### Transition types

- `registry_initialized`
- `actor_registered`
- `actor_key_authorized`
- `actor_key_revoked`
- `actor_key_compromised`
- `role_granted`
- `role_revoked`
- `organization_membership_granted`
- `organization_membership_revoked`

Genesis is signed by a key in the local bootstrap trust set. Every successor is
signed by an actor who resolves to an active `registry_admin` at the
server-observed acceptance time.

## Independently signed acceptance records

A signed administrator event does not prove when the registry accepted it.
Tranche 5 therefore persists a `RecordedScientificCredentialEvent` containing:

- the signed credential event;
- server-generated `received_at`;
- authority-service public key;
- authority-service signature over the event hash and `received_at`.

The credential actor key and authority-service key must differ. All historical
service public keys needed by the registry must remain in the authority trust
set.

The record hash commits to the administrator event, acceptance time, service
public key, and acceptance signature. A successor signs this record hash as its
predecessor. This protects acceptance timestamps for both historical and tail
records rather than merely storing an unsigned timestamp beside a signed event.

Acceptance timestamps must be monotonic. Historical identity resolution replays
only the prefix accepted at or before the requested time, preserving the exact
knowledge state used for receipt-time authorization.

## Deterministic projection and invariants

Registry replay reconstructs the complete actor authority state and rejects:

- invalid event or acceptance signatures;
- untrusted acceptance-service keys;
- event/acceptance key reuse;
- unsupported protocols or unknown fields;
- future-skewed actor events;
- backward acceptance timestamps;
- duplicate event IDs;
- duplicate actor-scoped idempotency keys;
- sequence or record-hash discontinuity;
- mutations by unregistered or unauthorized administrators;
- duplicate actor registration;
- lifetime reuse of an actor signing key by another identity;
- registration of any trusted authority-service key as an actor key;
- revocation before key validity begins;
- transitions that remove every active administrator;
- transitions that leave no active administrator key without scheduled expiry
  or revocation.

Every resolved actor profile receives the registry head record hash as its
`authority_revision`.

## Authority receipt protocol v2

`ScientificAuthorityReceipt` now supports protocol versions 1 and 2.

- v1 is retained only for historical receipts created by the former in-memory
  startup resolver.
- v2 requires `credential_registry_revision` and includes it in both the signed
  receipt bytes and the authority snapshot hash.

New registry-backed scientific writes therefore prove the precise authority
history used to accept them. Changing the revision after signing invalidates the
receipt.

## API cutover

The API now owns an `Arc<ScientificCredentialRegistry>` and uses it as the
`ScientificIdentityResolver` for all governed scientific writes.

New endpoints:

| Method | Path | Purpose |
|---|---|---|
| `POST` | `/api/v1/scientific/authority/events` | Append one signed credential transition |
| `GET` | `/api/v1/scientific/authority` | Registry status and head revision |
| `GET` | `/api/v1/scientific/authority/events` | Export paged recorded transitions |
| `GET` | `/api/v1/scientific/authority/events/{id}` | Retrieve one recorded transition |
| `GET` | `/api/v1/scientific/authority/actors/{actor}` | Resolve the current actor profile |

The JWT subject must exactly match the credential event actor. Registry genesis
is deliberately rejected by the network API and remains an offline bootstrap
operation.

Readiness now requires:

- initialized durable credential storage;
- configured credential-acceptance signing;
- the configured minimum active administrator count, default two;
- at least one continuing administrator recovery key;
- durable scientific events;
- configured authority-receipt signing;
- a reconciled authority journal without pending or unsafe gaps.

## Administrative CLI

Two offline administrative commands are added.

### `credential-registry-init`

Creates:

- the durable registry;
- the signed genesis event and acceptance record;
- a public bootstrap trust file.

It refuses to overwrite existing outputs, rejects symbolic-link or overly broad
secret-key files, and requires separate administrator and authority-service
keys.

### `credential-actor-register`

Registers a new actor with:

- one initial public key;
- one or more scientific roles;
- optional organization memberships.

The transition is signed by an existing active administrator and independently
accepted by the authority service. Historical authority-service public keys may
be supplied during key rotation.

## Governed legacy migration

The migration command no longer constructs a temporary identity profile. It
opens and verifies the durable credential registry, then requires the migration
actor to:

- exist in the current registry projection;
- hold `migration_service`;
- control the supplied active signing key;
- belong to the requested acting organization, when one is supplied.

The same historical authority trust set verifies both credential acceptance
records and scientific authority receipts. Newly imported legacy events remain
explicitly unassessed and must finish with complete authority receipts.

## Deployment configuration

Removed:

- `DESCI_SCIENTIFIC_AUTHORITY_FILE`
- the mutable scientific-authorities example file

Added:

- `DESCI_CREDENTIAL_REGISTRY_BACKEND`
- `DESCI_CREDENTIAL_REGISTRY_PATH`
- `DESCI_CREDENTIAL_BOOTSTRAP_TRUST_FILE`
- `DESCI_MIN_CREDENTIAL_REGISTRY_ADMINS`

The current `DESCI_AUTHORITY_RECEIPT_SIGNING_KEY_FILE` is domain-separated and
used for both credential acceptance signatures and scientific authority
receipts. Historical public keys remain configured through
`DESCI_AUTHORITY_RECEIPT_TRUST_FILE`.

## Validation performed in this environment

The implementation was checked with:

- patch whitespace validation;
- Rust delimiter and structural scans across the source tree;
- duplicate enum-variant and duplicate adjacent `#[async_trait]` scans;
- JSON parsing for repository configuration;
- Docker Compose YAML parsing;
- static call-site review for registry construction, successor creation,
  authority-receipt construction, and actor-profile literals.

A Rust toolchain is not present in the execution environment, and the standalone
archive still omits the sibling `../crates/mycelix-zkp-core` dependency.
Accordingly, this tranche has **not** been validated by `cargo fmt`, `cargo
clippy`, `cargo test`, or a complete build.

## Remaining work

The next authority tranche should add:

1. threshold or quorum approval for high-impact credential transitions;
2. explicit proposals, approvals, expiry, and cancellation rather than one-step
   administrator mutation;
3. governed authority-service key rotation instead of an operator-maintained
   public trust set;
4. transparency-log or external checkpoint anchoring;
5. durable per-event storage with cross-process writer exclusion;
6. DID/ORCID and institutional credential proof adapters;
7. signed cross-language golden vectors and client SDKs;
8. complete generated OpenAPI schemas for canonical endpoints.
