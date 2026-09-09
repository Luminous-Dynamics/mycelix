# mycelix-integration-core

`mycelix-integration-core` is the transport-neutral semantic kernel for the Mycelix integration plane.

It intentionally contains no:

- vendor SDK;
- HTTP client/server;
- Holochain HDK/HDI;
- database or queue driver;
- secret-manager client;
- async runtime;
- provider credential logic.

The crate currently owns:

- provider-neutral external system/object/actor references;
- explicit external identity/object bindings;
- schema/mapping/semantic profile identity;
- SHA-256 content commitments;
- deterministic canonical v1 preimages for external objects, integration event envelopes, and typed integration commands;
- explicit external trust progression;
- pure deterministic projection contract;
- side-effect classes and minimum retry policy classes;
- first-class `Confirmed | Rejected | Ambiguous` execution outcomes;
- reconciliation hints/results/cursors;
- explicit inbound and outbound state-machine vocabulary.

## Claim boundary

This crate does **not** authenticate providers, execute connectors, persist an inbox/outbox, grant authority, verify Mycelix governance, reconcile provider state, or make an external observation into domain truth.

In particular:

```text
IntegrationCommand != ApprovedExternalCommand
Allowed claim != execution capability
Authenticated provider event != accepted Mycelix fact
Ambiguous transport outcome != provider rejection
Retry != authority
```

The future `mycelix-integration-authority` layer must consume the repository's existing qualified authority/decision/execution artifacts and bind them to one exact integration operation. The runtime must require that qualified authority before executing a side effect.

## Source status

This is an INT-02 source candidate stacked on the Integration Plane RFC. It should remain draft until hosted qualification proves formatting, compile, tests, Clippy, dependency boundaries, canonical-vector stability, and the state-machine invariants required by `MYCELIX-INTEGRATION-1`.
