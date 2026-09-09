# mycelix-integration-runtime

`mycelix-integration-runtime` is the durable causal-state layer for the Mycelix integration plane.

It is deliberately **not** a connector, provider SDK wrapper, authority oracle, or domain-state engine.

## v0.1 source boundary

The first reference implementation uses SQLite to make the failure semantics executable before adding distributed infrastructure.

It owns:

- durable normalized inbound insertion;
- duplicate-vs-identity-collision detection;
- durable post-authority outbound intent storage;
- atomic outbox claiming with worker leases;
- crash recovery;
- explicit `Confirmed | Rejected | Ambiguous` outcome persistence;
- reconciliation persistence;
- reconciliation cursor checkpoints;
- reconciliation-before-finalization enforcement;
- restart persistence tests.

It does **not** own:

- provider HTTP clients;
- webhook authentication;
- Holochain zome calls;
- Mycelix institutional authority;
- capability minting;
- provider execution;
- domain acceptance;
- a Kafka requirement.

## Critical crash rule

A worker lease expiring while a side-effecting operation is `Executing` does not mean the provider failed.

The reference runtime therefore moves that operation to `Ambiguous` and removes it from the executable queue. It cannot be claimed again until a separate reconciliation path establishes what happened.

```text
OutboxCommitted
      |
      v
   Executing
      |
      +-- receipt ----------> Confirmed
      +-- explicit reject --> Rejected
      +-- uncertain --------> Ambiguous
      +-- worker dies ------> Ambiguous   (side-effecting)
                                  |
                                  v
                              Reconciled
                                  |
                                  v
                               Finalized
```

Read-only operations may be requeued after an expired lease because replay cannot create an external side effect. This is a runtime recovery rule, not a grant of authority.

## Authority boundary

`DurableOutboundIntent.authority_commitment` records the authority subject/provenance used by the higher layer. It is **not self-authenticating** and this crate exposes no connector execution API.

INT-04 must bind the exact integration command to independently authenticated Mycelix authority/decision evidence before any provider execution component is allowed to consume it.

This preserves the repository's current authority invariant:

```text
Allowed claim != execution capability
Decision != authority
Stored intent != permission to execute
```

## Why SQLite first

The v1 contract is durability and convergence, not a particular infrastructure vendor. SQLite gives the project a small, auditable reference backend for:

- transactional inbox/outbox state;
- crash/restart tests;
- lease recovery;
- local deployments;
- single-node connector agents.

A PostgreSQL, distributed-log, or replicated implementation can satisfy the same semantics later. Kafka remains optional.

## Qualification target

Before INT-03 is considered green, hosted tests should prove at least:

1. identical inbound redelivery is idempotent;
2. the same provider event identity with different content is rejected as a collision;
3. the same command identity cannot be mutated after enqueue;
4. execution begins only after durable outbox commitment;
5. a side-effecting worker crash produces `Ambiguous`, not retry or failure;
6. ambiguous work cannot be claimed again;
7. confirmed work cannot finalize before reconciliation;
8. outcome/reconciliation records cannot be attached to a different operation;
9. reconciliation checkpoints survive reload;
10. queued state survives store reopen.
