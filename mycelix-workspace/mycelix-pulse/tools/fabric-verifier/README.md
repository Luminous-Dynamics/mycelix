# Pulse Fabric Verifier v0.1

This is an independent, read-only verifier for the redacted trace format in
[PULSE-FABRIC-QUAL-001](../docs/PULSE_COMMUNICATIONS_FABRIC_QUALIFICATION_V0_1.md).
It is intentionally outside the Pulse application's send, authorization,
transport, and receipt decision paths.

## Run

From the public workspace checkout:

```bash
cargo test --manifest-path mycelix-workspace/mycelix-pulse/tools/fabric-verifier/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-pulse/tools/fabric-verifier/Cargo.toml -- \
  verify path/to/trace.json --expected-source-sha <40-character-source-sha>
```

The expected source SHA is an optional external pin. Without a trusted value
supplied out of band, a syntactically valid SHA in the trace is only a claim
made by the trace producer.

Exit codes: 0 means the checked evidence structure is internally consistent;
2 means invalid JSON or invalid evidence; 64 means usage error; 66 means input
file error. A valid trace with terminal disposition `UNKNOWN` or
`INCOMPLETE` exits 0 because it can be a valid record of an unresolved
outcome. Consumers must inspect `terminal_disposition`; verifier success is
not message-delivery success.

## v0.1 invariants

- Required run, scenario, event, evidence-reference, and redaction fields exist
  with the expected basic types.
- Source SHA and supplied digests have the expected lowercase hexadecimal
  shape; an optional external source pin must match exactly.
- Event identifiers are unique; each causal predecessor resolves; relation
  types are enumerated; the graph is acyclic.
- Same-clock causal predecessors cannot have a later monotonic timestamp.
  Wall-clock timestamps and timestamps from different clock domains are never
  used to invent a global ordering.
- Attempt-bearing event classes require a concrete attempt identifier.
  Attempt IDs cannot be reused across logical operations.
- Evidence roles are constrained for host, domain, source-chain, integrity,
  DHT, transport, recipient, receipt, recovery, and UI observations.
- One logical operation cannot produce multiple recipient inbox commits.
- A `SUCCESS_VERIFIED` UI projection must have a causal path to a separately
  attributed receipt verifier, which in turn has causal ancestors for recipient
  inbox commit, recipient decryption verification, and receipt creation. The
  content commitment must match across those events, and the verifier must
  reference a recipient receipt or signed statement.
- Obvious plaintext and secret-bearing field names are rejected.

## Explicit non-goals and limitations

This prototype does **not**:
- implement the complete JSON Schema Draft 2020-12 specification;
- validate every schema constraint or reject every unknown JSON property;
- verify signatures, hashes of external artifacts, capability grants, Holochain
  source-chain lineage, DHT validation, cryptographic protocol correctness, or
  transport truth;
- prove that the trace producer told the truth or that actor references
  correspond to real independent principals;
- establish forward secrecy, post-compromise security, anonymity, availability,
  Byzantine safety, or production readiness.

The `recipient_receipt` / `signed_statement` digest is a reference, not a
cryptographic verification. The receipt verifier's actor identity and
independence must be anchored by a separately authenticated trust policy in a
future qualification layer. This binary checks evidence-graph consistency;
it is not a trust root.

The full Draft 2020-12 schemas remain normative for shape validation and must
be run through a conforming schema engine in the dedicated validation gate.
This crate's checks are additive semantic checks, not a replacement.

## Test intent

The unit corpus includes controls for valid receipt-backed success, source SHA
mismatch, transport-ACK-only false success, UNKNOWN delivery, dangling causal
edges, causal cycles, role confusion, missing attempt IDs, duplicate inbox
effects, secret-bearing fields, and recipient/verifier identity collision.
