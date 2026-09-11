# mycelix-business-adapter-delimited

Strict read-only ingestion for headered CSV/TSV-style business exports.

The adapter exists for the common pilot case where a POS, inventory, accounting, sensor, or supplier system can export rows but should not receive commands from Mycelix. It converts a pinned file shape into `mycelix-business-ingress` witness records.

## Safety boundary

The crate contains no provider credential, network transport, mutation request, refund/order/payment command, or authority type. Its only direction is:

`delimited export -> canonical row witness -> deterministic fixed-point mapping -> ingress batch`

Accepted rows remain evidence from an external source. They do not become authoritative business truth merely because they parsed successfully.

## Qualification semantics

- exact ordered headers are bound into `source_schema_digest`;
- mapping semantics are independently bound into `mapping_digest`;
- each parsed row receives a SHA-256 canonical-record witness digest;
- each normalized observation receives a deterministic identity derived from source, event, input, metric, and mapping digest;
- batch identity binds adapter digest plus ordered source-event witnesses;
- schema drift, duplicate event IDs, excess precision, invalid timestamps, unsupported mappings, and excessive batch size fail closed;
- decimal values are converted directly to integer mantissas without floating-point conversion.

The adapter digest itself is release/build evidence supplied by the adapter owner. The crate does not claim to derive a code-build identity from its own source at runtime.

## Scope

A real restaurant can create a mapping preset for its export columns without changing the universal Business or Hospitality contracts. Vendor-specific names remain in that preset rather than becoming core enums.

Passing unit tests establishes parser/contract behavior only. It does not establish field qualification, semantic correctness of a particular POS export, causal business benefit, or authority to execute business actions.
