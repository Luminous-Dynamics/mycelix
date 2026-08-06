# Governed Authority Database Epochs

A PostgreSQL primary promotion changes which database instance may serialize
scientific authority. Mycelix-DeSci therefore treats promotion as a governed
protocol transition rather than an infrastructure-only action.

## Epoch certificate

A signed database-epoch certificate binds:

- one threshold-governed promotion intent;
- the executed governance proposal and its durable record hash;
- deployment, candidate primary, PostgreSQL system identifier, timeline, and
  replay LSN;
- the previous epoch hash;
- a governed transparency-checkpoint anchor;
- the exact credential, governance, scientific-event, and authority-receipt
  heads observed under an exclusive database barrier;
- the promotion mode and signed promotion time, with a separate server acceptance time; and
- a dedicated epoch-signing public key.

The epoch key is domain separated from actor, receipt, credential-acceptance,
transparency-witness, and outbox-delivery keys.

## Promotion modes

### Initial activation

Initial activation creates epoch one. It has no predecessor or recovery target.
It should be executed only after the SQL authority history and current
transparency checkpoint have been independently verified.

### Planned failover

A planned failover extends the current epoch chain and commits the new primary,
system identifier, timeline, LSN, and exact authority state. The signed intent
must execute through the critical governance quorum before promotion.

### Disaster recovery

Disaster recovery additionally requires:

- a declared incident;
- a recovery target not later than the incident declaration;
- a bounded ceremony window of at most seven days;
- at least two unique named participants;
- at least two of those participants among the governance approvers; and
- an exact post-recovery reconciliation before readiness passes.

The ceremony records the recovery objective, acknowledged data-loss window, and
coordination reference. It does not weaken the normal critical quorum.

## Database barrier

All cooperating scientific, credential, and governance SQL writers acquire a
shared advisory transaction lock. Epoch state capture and commit acquire the
exclusive form of the same lock. This prevents an epoch certificate from
committing a state that changed while its heads were being measured.

The barrier coordinates Mycelix-DeSci writers; it is not a substitute for
PostgreSQL fencing, synchronous replication policy, or infrastructure-level
split-brain prevention.

## Publication and acknowledgement

The epoch row references exactly one signed transactional-outbox delivery.
Replay verifies the delivery signature, indexed topic, aggregate, sequence,
creation time, payload hash, and embedded certificate.

Independent governed witnesses may acknowledge a published delivery. An
acknowledgement is valid only when:

- the outbox delivery has been published;
- its topic and signed delivery hash match;
- the witness key and organization were active at acknowledgement time; and
- the durable HTTPS reference contains the immutable delivery hash and has no
  credentials, query, or fragment.

Readiness may require acknowledgements from multiple organizations. New acknowledgements are serialized against the governance journal and exact retries return the original accepted record even after later witness revocation.

## Canonical endpoints

Read-only:

- `GET /api/v1/scientific/authority/database-epochs`
- `GET /api/v1/scientific/authority/database-epochs/state-commitment`
- `GET /api/v1/scientific/authority/database-epochs/{epoch_number}`
- `GET /api/v1/scientific/authority/deliveries/{delivery_id}/acknowledgements`

Authenticated writes:

- `POST /api/v1/scientific/authority/database-epochs`
- `POST /api/v1/scientific/authority/recovery-reconciliations`
- `POST /api/v1/scientific/authority/delivery-acknowledgements`

The JWT subject must match the signed operator or witness actor exactly.

## Remaining production work

The current implementation defines the protocol, PostgreSQL schema, replay
checks, and API boundary. Before production use it still requires compilation,
SQL integration tests, multi-primary adversarial testing, cross-language signed
vectors, actual failover exercises, and an independently reviewed HSM/KMS
adapter.
