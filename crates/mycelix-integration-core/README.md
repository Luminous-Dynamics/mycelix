# mycelix-integration-core

`mycelix-integration-core` is the transport-neutral semantic kernel for the Mycelix integration plane.

It intentionally contains no vendor SDK, HTTP client/server, Holochain HDK/HDI, database/queue driver, secret-manager client, async runtime, provider credential logic, or current execution-authority minting.

## Owned semantic vocabulary

The crate owns provider-neutral external identities/bindings, schema/mapping/semantic profile identity, SHA-256 content commitments, deterministic canonical v1 preimages for selected external objects/events/commands, pure projection contracts, side-effect/retry classes, exact provider execution outcomes, reconciliation vocabulary, and inbound/outbound state machines.

Current outbound execution vocabulary is intentionally causal:

```text
AuthorityChecked
  |-- local denial ------------------------> AuthorityDenied -> Finalized
  v
Approved
  v
OutboxCommitted
  v
AttemptPrepared
  v
DispatchStarted
  |-- proven provider pre-commit rejection -> RejectedBeforeCommit -> Finalized
  |-- confirmed effect --------------------> Confirmed ----+
  |-- commit uncertainty ------------------> Ambiguous ----+--> Reconciled -> Finalized
```

Therefore:

```text
AuthorityDenied != RejectedBeforeCommit
RejectedBeforeCommit != transport failure
RejectedBeforeCommit != CommitUnknown
IntegrationCommand != execution capability
Allowed claim != execution capability
Authenticated provider event != accepted Mycelix fact
Retry != authority
```

## Serialization is not semantic identity

Many semantic structs/enums derive `Serialize`/`Deserialize` for ordinary interchange and testing. **Their Serde representation is not, by itself, a frozen durable protocol representation.**

The crate currently owns explicit canonical v1 preimages only where `CanonicalEncodeV1` is implemented. A persistence layer MUST NOT assume that:

```text
serde enum name
    == durable wire identity

crate version 0.1.0
    == semantic protocol identity

same SQL integer
    == same semantic state across revisions
```

The split from the former generic outbound `Rejected` state to distinct `AuthorityDenied` and `RejectedBeforeCommit` demonstrates why this matters: the old state was reachable from both authority denial and post-dispatch provider rejection, so an old durable `Rejected` value cannot later be relabeled as one specific cause without additional provenance.

Persistence implementations therefore need an explicit **integration semantic producer/profile identity** in addition to their structural storage schema version. When a semantic revision changes the meaning or decomposition of durable state, that semantic identity/version must change even when the table/JSON shape could still deserialize.

Required migration rule:

```text
old representation many-to-one
    !=
one arbitrarily chosen new semantic cause

semantic migration underdetermined
    -> fail closed / quarantine / explicit legacy-underdetermined state
```

Syntactic readability is not semantic equivalence.

## Exact operation identity

`ExternalOperationRef` identifies one logical command/connector operation and may later acquire a provider operation identifier.

Provider-operation identity must refine monotonically:

```text
provider_operation None -> Some(A)      may refine
Some(A) -> Some(A)                       compatible
Some(A) -> Some(B)                       conflict / reject
```

A later less-specific observation must not erase an already established exact provider-operation binding.

## Claim boundary

This crate does **not** authenticate providers, execute connectors, persist an inbox/outbox, grant institutional authority, reconcile provider state, or make an external observation into domain truth.

The future execution-authority layer must combine one exact runtime attempt with fresh current execution authority and a qualified provider execution profile before provider payload materialization or dispatch.

```text
DurableIntent != ExecutablePayload
QueueClaim != CurrentExecutionAuthority
provider receipt != world postcondition
```

## Source status

This is an INT-02 source candidate stacked on the Integration Plane RFC. It should remain draft until hosted qualification proves formatting, compile, tests, Clippy, dependency boundaries, canonical-vector stability, state-machine invariants, and the semantic-versioning/migration boundary required by `MYCELIX-INTEGRATION-1`.
