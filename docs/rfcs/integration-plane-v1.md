# Mycelix Integration Plane v1

Status: Draft RFC

Profile: `MYCELIX-INTEGRATION-1`

## Purpose

This RFC defines the v1 boundary for integrating Mycelix with external enterprise, financial, logistics, industrial, and SaaS systems without making those systems the source of Mycelix institutional truth.

The integration plane is a native/runtime boundary. Holochain zomes and Mycelix domain logic MUST NOT directly call vendor APIs or treat vendor records as authoritative domain state.

The core architectural split is:

- **Connector** — vendor/protocol semantics and authentication.
- **Projection** — deterministic translation into provider-neutral Mycelix semantics.
- **Domain** — business truth, authority, validation, and accepted transitions.

External observations are evidence. External side effects require independent authority. Neither direction bypasses Mycelix policy.

## 1. Architectural placement

The integration plane SHOULD use an explicit namespace rather than extending `mycelix-core` or overloading existing `mycelix-bridge-*` crates:

```text
crates/
  mycelix-integration-core/
  mycelix-integration-runtime/
  mycelix-integration-authority/
  mycelix-integration-testkit/

connectors/
  mycelix-connector-stripe/
  mycelix-connector-odoo/
  mycelix-connector-epcis/
  mycelix-connector-mercury/
```

The intended dependency direction is:

```text
External systems
      |
      v
connector implementations
      |
      v
mycelix-integration-runtime
      |
      v
mycelix-integration-core
      |
      +----> deterministic projections
      |
      +----> authority adapter
                  |
                  v
             Mycelix domains
                  |
                  v
             evidence layer
```

`mycelix-integration-core` MUST remain transport-neutral and provider-neutral. It MUST NOT depend on vendor SDKs, Holochain HDK/HDI, HTTP clients/servers, SQL drivers, secret managers, or a particular async runtime.

The integration authority layer MUST consume existing Mycelix authority/decision/execution evidence where semantically appropriate. It MUST NOT create a parallel capability or governance system merely for integrations.

## 2. Non-negotiable invariants

The following are normative v1 invariants.

### I-01 External observation is not domain truth

A provider event, even when correctly authenticated, is an observation about an external system. It MUST NOT directly mutate Mycelix domain state.

```text
IntegrationEvent
  -> Projection
  -> ProposedDomainCommand
  -> Domain validation / authority
  -> AcceptedDomainTransition
  -> Evidence
```

### I-02 Provider identity is not Mycelix identity

External identities and object identifiers MUST be represented through explicit bindings. Provider identifiers MUST NOT become first-class Mycelix identities merely because a connector observed them.

### I-03 Authentication precedes decoding

Inbound webhook or callback authentication MUST operate over the original request bytes and security-relevant request metadata whenever the provider signature contract binds those bytes or metadata.

The required order is:

```text
raw bytes
  -> authenticate
  -> authenticated envelope
  -> decode
  -> normalize
  -> canonical IntegrationEvent
```

Parsing and reserializing signed JSON before authentication is forbidden when doing so could change the signed representation.

### I-04 Projection is deterministic

Projection from a canonical integration event to proposed Mycelix consequences MUST be deterministic for the same versioned inputs.

Projection MUST NOT read wall-clock time, randomness, mutable global configuration, network state, or hidden database state. Any required contextual input MUST be explicit and versioned in `ProjectionContext`.

### I-05 Side effects require independent authority

A connector MUST NOT execute a write-capable external command until an authority adapter has consumed the relevant Mycelix decision/authority evidence and emitted an integration-specific approved command or equivalent sealed artifact.

A semantic claim such as `Allowed` MUST NOT by itself be interpreted as an execution capability.

### I-06 Ambiguous execution is first-class

Transport failure MUST NOT be equated with external failure.

If a provider may have committed a side effect before the integration plane loses the response, the outcome MUST be represented as `Ambiguous` (or semantically equivalent) and MUST enter reconciliation before any unsafe re-execution.

### I-07 Retry must not mint effects

Retry is not authority and MUST NOT amplify an operation. Irreversible or non-provably-idempotent operations MUST reconcile before re-execution.

### I-08 Durable intent precedes execution

For side-effecting outbound work, durable outbox commitment MUST occur before execution begins. Crash recovery MUST be able to distinguish at least:

- approved but not durably queued;
- queued but not claimed;
- claimed/executing;
- confirmed;
- rejected;
- ambiguous;
- reconciled/finalized.

### I-09 Raw secrets and unnecessary PII stay off shared DHT state

Shared Mycelix evidence SHOULD contain commitments, references, provenance, policy-relevant facts, and minimum necessary claims rather than raw provider secrets, access tokens, or unnecessary external payloads.

### I-10 No silent state invention

Connector failure may delay convergence. It MUST NOT silently invent provider state, silently drop unresolved obligations, or convert uncertainty into success/failure without evidence.

## 3. Canonical integration semantics

`mycelix-integration-core` SHOULD define provider-neutral equivalents of the following concepts:

```text
IntegrationEvent
ExternalSystemId
ConnectorInstanceId
ExternalObjectRef
ExternalActorRef

ExternalIdentityBinding
ExternalObjectBinding
MappingProfileId
SchemaVersion

ExternalTrustState
VerificationEvidence
ContentCommitment

IntegrationCommand
ApprovedExternalCommand
ExternalExecutionOutcome
ExternalReceipt

ReconcileCursor
ReconciliationHint
ReconciliationResult

CorrelationId
CausationId
IdempotencyKey
SideEffectClass
```

Provider-specific names such as `StripeCustomerId`, `MercuryTransactionId`, and `OdooInvoiceId` MAY exist inside connector crates. They MUST be normalized before crossing into integration-core.

A canonical external object reference SHOULD have semantics equivalent to:

```rust
pub struct ExternalObjectRef {
    pub system: ExternalSystemId,
    pub object_type: ExternalObjectType,
    pub external_id: ExternalOpaqueId,
}
```

The provider identifier is opaque to the core. Domain interpretation happens through explicit bindings and mapping profiles.

## 4. Execution outcome model

The v1 execution contract MUST distinguish confirmed rejection from transport/protocol ambiguity.

A reference shape is:

```rust
pub enum ExternalExecutionOutcome {
    Confirmed(ExternalReceipt),
    Rejected {
        reason: ExternalRejection,
    },
    Ambiguous {
        operation: ExternalOperationRef,
        reconciliation_hint: ReconciliationHint,
    },
}
```

A connector execution API SHOULD return an outcome distinct from connector-local errors:

```rust
async fn execute(
    &self,
    command: &ApprovedExternalCommand,
) -> Result<ExternalExecutionOutcome, ConnectorError>;
```

`ConnectorError` is for failures that prevent producing a meaningful provider outcome. It MUST NOT collapse a provider-side uncertainty into `Rejected`.

## 5. Side-effect classes

Every outbound command MUST declare or derive a side-effect class:

```rust
pub enum SideEffectClass {
    ReadOnly,
    Reversible,
    Compensatable,
    Irreversible,
}
```

The runtime MUST be allowed to apply stricter execution/retry policy from this class.

Reference policy:

```text
ReadOnly      -> bounded normal retry
Reversible    -> idempotent retry when connector contract proves safety
Compensatable -> durable saga / compensation semantics
Irreversible  -> reconcile before re-execution unless exact provider idempotency proof permits replay
```

Classification MUST NOT weaken domain authority. It constrains runtime execution; it does not grant permission.

## 6. Inbound state machine

The inbound path SHOULD be modeled explicitly rather than as booleans:

```text
Received
  |-- authentication failure --> Rejected
  v
Authenticated
  |-- duplicate -------------> Duplicate
  v
Decoded
  v
Normalized
  v
Persisted
  v
Projected
  |-- domain/policy denial ---> DomainRejected
  v
DomainAccepted
  v
Reconciled
  v
Finalized
```

A duplicate authenticated provider event MUST NOT create a second semantic consequence.

Out-of-order events MUST either converge deterministically or remain explicitly unresolved until sufficient evidence exists.

## 7. Outbound state machine

The outbound path SHOULD be modeled as:

```text
Proposed
  v
AuthorityChecked
  |-- denied ----------------> Rejected
  v
Approved
  v
OutboxCommitted
  v
Executing
  |------------+-------------+
  v            v             v
Confirmed    Rejected      Ambiguous
  |                          |
  |                      reconcile
  |                          |
  +-------------+------------+
                v
            Reconciled
                v
             Finalized
```

The `OutboxCommitted -> Executing` transition is crash-sensitive and MUST be durably observable.

## 8. Inbound authentication boundary

Connectors SHOULD expose a raw envelope equivalent to:

```rust
pub struct RawInboundEnvelope {
    pub body: Bytes,
    pub headers: CanonicalHeaders,
    pub method: HttpMethod,
    pub target: RequestTarget,
    pub received_at: Timestamp,
}
```

Authentication SHOULD produce a distinct `AuthenticatedInbound` artifact that binds:

- connector instance;
- authentication method/profile;
- provider or principal identity when established;
- exact content commitment;
- replay-relevant metadata;
- verification evidence;
- receive time and allowed freshness window where policy requires it.

Authentication success MUST NOT imply semantic acceptance of the decoded event.

## 9. Deterministic projection boundary

Reference projection contract:

```rust
pub trait IntegrationProjection {
    fn project(
        &self,
        event: &IntegrationEvent,
        context: &ProjectionContext,
    ) -> Result<ProjectionResult, ProjectionError>;
}
```

`ProjectionContext` MUST contain only explicit, versioned inputs. Projection output SHOULD include enough provenance to answer:

- which event caused this proposal;
- which mapping profile and version were used;
- which schema version was interpreted;
- which external identity/object bindings were consulted;
- which deterministic projection implementation/profile was used.

A future replay of the same event under the same context MUST produce the same semantic result.

## 10. Authority composition

The integration authority boundary exists to translate already-qualified Mycelix authority into a connector-safe execution subject.

It SHOULD bind at least:

- exact proposed domain/external operation;
- exact side-effect class;
- exact connector instance/system scope;
- exact external object or operation subject when known;
- exact decision/authority/evidence references required by the owning domain;
- idempotency/replay policy;
- expiry/freshness bounds where relevant;
- mapping/profile versions that affect execution semantics.

The adapter MUST fail closed if required authority evidence is absent, stale, revoked, scope-mismatched, or semantically indeterminate.

It MUST NOT treat the connector as an authority oracle.

## 11. Durable runtime boundary

`mycelix-integration-runtime` SHOULD own:

- durable inbound inbox;
- authenticated-event deduplication;
- transactional outbound outbox;
- work claiming/leases with crash recovery;
- bounded retry/backoff;
- dead-letter or quarantine state;
- reconciliation cursors/checkpoints;
- ambiguity recovery;
- audit reconstruction;
- secret access through an injected provider;
- connector lifecycle/health state.

A reference storage abstraction is:

```rust
#[async_trait]
pub trait IntegrationStore {
    async fn insert_inbound(
        &self,
        event: PersistableInbound,
    ) -> Result<InsertDisposition>;

    async fn enqueue_outbound(
        &self,
        command: ApprovedExternalCommand,
    ) -> Result<OutboxEntryId>;

    async fn claim_outbox(
        &self,
        worker: WorkerId,
        limit: usize,
    ) -> Result<Vec<ClaimedOutboxEntry>>;

    async fn record_execution(
        &self,
        entry: OutboxEntryId,
        outcome: ExternalExecutionOutcome,
    ) -> Result<()>;

    async fn checkpoint_reconciliation(
        &self,
        connector: ConnectorInstanceId,
        cursor: ReconcileCursor,
    ) -> Result<()>;
}
```

The core semantics MUST NOT depend on SQLite, PostgreSQL, Kafka, or another particular storage/queue product.

Kafka or another distributed log MAY be used by deployments, but is not required by `MYCELIX-INTEGRATION-1`.

## 12. Reconciliation

Every connector that can miss events, observe eventual consistency, or produce ambiguous outcomes MUST implement reconciliation semantics.

Reconciliation SHOULD support:

- point lookup of an exact external operation/object;
- bounded historical scan from a cursor when the provider supports it;
- missed-event recovery;
- ambiguous execution recovery;
- out-of-order convergence;
- evidence that records how the final external state was established.

An irreversible `Ambiguous` operation MUST NOT be automatically executed again until reconciliation proves one of:

1. the original operation definitely did not occur; or
2. the provider contract proves replay of the exact idempotency identity cannot create a second effect.

## 13. Idempotency

Idempotency is a semantic contract, not merely a string field.

The connector contract MUST state:

- idempotency scope;
- provider retention horizon;
- whether identical keys with changed payloads are rejected;
- whether the key survives provider failover/retries;
- what exact operation identity it protects;
- whether reconciliation can recover the operation from that identity.

The runtime MUST NOT assume indefinite idempotency from providers that only guarantee a bounded key-retention window.

## 14. Mapping and schema provenance

All normalization/projection that can change business meaning MUST be versioned.

`MappingProfileId` and `SchemaVersion` MUST be carried far enough downstream to reconstruct how an external record became a Mycelix proposal or accepted transition.

Mapping changes MUST NOT silently reinterpret already-finalized historical evidence.

Schema drift SHOULD produce an explicit unsupported/quarantined state rather than best-effort field guessing for security- or finance-relevant operations.

## 15. Privacy and content commitments

Raw external payloads MAY be retained in a local/private integration store when operationally required and legally permitted.

Shared Mycelix evidence SHOULD prefer:

- cryptographic content commitments;
- minimal normalized facts;
- immutable references;
- source/provider provenance;
- authentication/verification evidence;
- mapping/schema profile identity;
- reasoned redaction/minimization.

Secrets, bearer tokens, API keys, private webhook signing keys, and equivalent credentials MUST NOT be committed to shared DHT state.

## 16. Connector conformance profile

`mycelix-integration-testkit` SHOULD expose a reusable conformance suite for connector factories.

A v1 connector MUST pass, where applicable:

- C01 deterministic normalization
- C02 inbound authentication
- C03 replay resistance
- C04 inbound idempotence
- C05 outbound idempotence
- C06 credential least privilege
- C07 missed-event recovery
- C08 out-of-order convergence
- C09 schema drift behavior
- C10 secret/DHT exclusion
- C11 capability/authority enforcement
- C12 evidence linkage
- C13 outage/crash recovery
- C14 audit reconstruction
- C15 connector substitutability

A testkit API MAY support a pattern such as:

```rust
integration_conformance_suite!(StripeConnectorFactory);
```

A conformant report SHOULD identify the exact connector implementation/version, profile version, test corpus version, and evidence artifact for the run.

Conformance MUST NOT be self-certified solely by a boolean in connector code.

## 17. Threat model

The v1 design MUST consider at least:

- forged webhook requests;
- validly signed but semantically malicious/irrelevant provider events;
- replayed provider events;
- duplicate deliveries;
- missing deliveries;
- out-of-order deliveries;
- schema changes;
- stale identity/object mappings;
- compromised or overprivileged connector credentials;
- connector compromise;
- integration runtime crash between provider commit and local receipt;
- crash between local approval and execution;
- 429/5xx/provider outage;
- network partitions;
- partial multi-step workflows;
- malicious or buggy projection code;
- stale/revoked Mycelix authority;
- idempotency-key expiry;
- side-effect reconciliation races;
- clock skew where provider freshness checks rely on time;
- PII/secrets leaking into shared evidence;
- provider account takeover or provider-side fraud.

A connector MUST NOT be considered a trusted source of institutional truth merely because transport authentication succeeds.

## 18. Failure model

The integration plane MUST preserve uncertainty instead of collapsing it.

Reference principles:

```text
Timeout != Failure
SignedEvent != AcceptedDomainFact
Retry != NewAuthority
ProviderRecord != MycelixIdentity
Observation != Consequence
Decision != Capability
AllowedClaim != ExecutableAuthority
Ambiguous != Rejected
Reconciled != NecessarilySuccessful
```

The runtime SHOULD expose operator-visible unresolved states instead of forcing premature finality.

## 19. First provider ordering

Provider implementation SHOULD follow the architecture rather than drive it.

Recommended order after INT-01 through INT-04 are green:

1. Stripe — payment/refund/dispute and webhook/idempotency pressure.
2. Odoo — product/order/inventory/invoice/accounting mapping pressure.
3. EPCIS 2.0.1 — open supply-chain event interoperability and semantic round-trip pressure.
4. Mercury — narrow fiat settlement/reconciliation pressure.
5. Federated procure-to-pay — cross-organization composition under failure injection.

No money-moving connector SHOULD be merged before the core/runtime/authority invariants are independently executable.

## 20. Procure-to-pay release gate

The first end-to-end industrial demonstrator SHOULD include two sovereign organizations and cover:

```text
Buyer A
  -> request
  -> authority
  -> purchase order

Supplier B
  -> acceptance
  -> inventory
  -> EPCIS shipment

Buyer A
  -> receipt
  -> invoice binding
  -> cryptographic three-way match
  -> governed payment approval

External rail
  -> settlement

Mycelix
  -> webhook authentication
  -> reconciliation
  -> accounting/domain consequences
  -> evidence closure
```

The release gate MUST include adversarial execution, including:

- process kill/restart;
- duplicate messages;
- reordered messages;
- lost webhooks;
- revoked capabilities/authority;
- stale mappings;
- 429/500 responses;
- network partitions between organizations;
- provider timeout after a successful external payment;
- crash after outbox claim but before local outcome persistence.

A successful UI demo without these convergence properties is not sufficient evidence for `MYCELIX-INTEGRATION-1` readiness.

## 21. Implementation tranches

### INT-01 — Integration Plane RFC

Hard gate:

- architecture, threat, trust, privacy, authority, failure, retry, side-effect, and evidence models are normative;
- no provider implementation code required.

### INT-02 — `mycelix-integration-core`

Hard gate:

- zero provider/network/Holochain/storage/runtime dependencies;
- canonical provider-neutral IDs/events/commands/receipts/bindings;
- deterministic canonical serialization/hashing for protocol subjects;
- explicit ambiguous outcome and side-effect class;
- projection purity tests.

### INT-03 — `mycelix-integration-runtime`

Hard gate:

- durable inbox/outbox;
- deduplication;
- crash recovery;
- reconciliation;
- ambiguity handling;
- DLQ/quarantine;
- reference local durable backend;
- chaos tests proving no silent duplicate irreversible effect.

### INT-04 — `mycelix-integration-authority`

Hard gate:

- fail-closed adapter from existing Mycelix authority/evidence artifacts to exact integration execution subjects;
- no new generic governance/capability system;
- stale/revoked/scope-mismatched authority rejected;
- exact connector/system/operation/side-effect binding;
- authorization remains separate from connector execution.

INT-01 through INT-04 form one security/kernel milestone.

## 22. Required properties before first money-moving connector

Before Stripe or Mercury can move real funds in a production profile, the kernel MUST demonstrate:

1. **No direct vendor-to-domain mutation.**
2. **No execution without exact current authority.**
3. **No blind retry after ambiguous irreversible execution.**
4. **Crash-safe durable outbox semantics.**
5. **Deterministic replay of projection semantics.**
6. **Explicit schema/mapping provenance.**
7. **Duplicate inbound delivery does not duplicate consequence.**
8. **Missed events can be reconciled.**
9. **Secrets and unnecessary PII do not enter shared DHT evidence.**
10. **Audit reconstruction can explain each external consequence from exact source evidence and authority.**

## 23. Non-goals

This RFC does not:

- make Holochain a synchronous enterprise message bus;
- require Kafka;
- define a universal ERP ontology;
- make Stripe, Mercury, Odoo, EPCIS, or any provider authoritative for Mycelix state;
- permit connectors to bypass domain policy;
- define one universal governance model;
- make a successful cryptographic authentication equivalent to truth;
- claim exactly-once delivery across arbitrary external systems;
- hide partial failure behind a single success/failure boolean.

## 24. Central claim boundary

If `MYCELIX-INTEGRATION-1` is satisfied, the defensible claim is:

> Mycelix can govern causal relationships and externally visible side effects across mutually sovereign organizations and legacy systems while preserving explicit authority, provenance, uncertainty, reconciliation, and domain-owned truth.

It is not a claim that external providers are trusted, that every workflow is globally atomic, or that transport reliability can eliminate institutional uncertainty.
