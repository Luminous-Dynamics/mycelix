# mycelix-business-ingress

Read-only external witness ingress contracts for the Mycelix Business Fabric.

This crate is deliberately separate from `mycelix-bridge-common`. Bridge Common routes trusted in-ecosystem Holochain/zome calls; Business Ingress describes evidence arriving from external systems such as POS, inventory, accounting, sensors, or supplier feeds.

The ingress contract is structurally one-way:

`external event -> immutable witness -> deterministic normalization -> shadow observation`

There is no outbound mutation type, provider command, credential container, refund/order/payment operation, or execution authority in this crate.

## Rules

- adapter identity, source schema, and mapping semantics are digest-bound;
- raw provider payloads are represented by digest only at this boundary;
- one provider event identity cannot silently change payload or schema;
- normalized observations must retain the exact source event and payload lineage;
- adapters may emit only input classes they declared;
- duplicate events within a batch fail closed rather than depending on arrival order;
- credentials and transport clients live outside this contract and must not be serialized into evidence records;
- an accepted ingress record is evidence from a witness, not authoritative business truth.
