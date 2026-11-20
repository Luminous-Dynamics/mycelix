# Pulse Fabric Verifier v0.1

This is an independent, read-only verifier for the redacted trace format in
[PULSE-FABRIC-QUAL-001](../docs/PULSE_COMMUNICATIONS_FABRIC_QUALIFICATION_V0_1.md).
It is intentionally outside the Pulse application's send, authorization,
transport, and receipt decision paths.

## Run

From the public workspace checkout:

```bash
cargo fmt --manifest-path mycelix-workspace/mycelix-pulse/tools/fabric-verifier/Cargo.toml -- --check
cargo test --manifest-path mycelix-workspace/mycelix-pulse/tools/fabric-verifier/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-pulse/tools/fabric-verifier/Cargo.toml -- \
  verify path/to/trace.json --schema mycelix-workspace/mycelix-pulse/docs/pulse-fabric-trace.schema-v0.1.json \
  --expected-source-sha <40-character-source-sha>
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

The CLI first meta-validates the supplied schema and validates the trace with
JSON Schema Draft 2020-12, then applies the causal and lifecycle checks below.
The schema engine is pinned to `jsonschema = 0.61.2`; default features are
disabled so the verifier cannot resolve remote schema references at runtime.
Keep the schema self-contained. The direct dependency is pinned, but a
package-specific Cargo.lock has not yet been generated and committed, so
transitive dependency resolution is not yet frozen.

## v0.1 invariants

- Required run, scenario, event, evidence-reference, and redaction fields exist
  with the expected basic types.
- Source SHA and supplied digests have the expected lowercase hexadecimal
  shape; an optional external source pin must match exactly.
- Event identifiers are unique; each causal predecessor resolves; relation
  types are enumerated and checked against supported event/role/identity
  combinations; the graph is acyclic.
- Same-clock causal predecessors cannot have a later monotonic timestamp.
  Wall-clock timestamps and timestamps from different clock domains are never
  used to invent a global ordering.
- Attempt-bearing event classes require a concrete attempt identifier.
  Attempt IDs cannot be reused across logical operations.
- Evidence roles are constrained for host, domain, source-chain, integrity,
  DHT, transport, recipient, receipt, recovery, and UI observations.
- One logical operation cannot produce multiple recipient inbox commits.
- A `SUCCESS_VERIFIED` UI projection must have one causally connected
  lifecycle: user intent, selected crypto profile, host admission, domain
  authorization, source-chain authorship, integrity acceptance, durable outbox,
  DHT projection observation, one transport attempt, recipient admission,
  inbox commit, decryption verification, receipt creation, and separately
  attributed receipt verification.
  scope and exact cell. The authored source-action hash must match the DHT
  projection's action hash. Receipt creation and verification must reference
  the same recipient-receipt digest and scope. These are consistency checks on
  supplied references, not signature or host-state verification.
- Attempt-scoped authority, authorship, validation, transport and recipient
  events must bind to the same concrete attempt ID. Logical intent, crypto
  selection, outbox commit and DHT observation remain operation-scoped.
- One recipient identity and one content commitment must persist through
  admission, commit, decryption, receipt creation and receipt verification.
  The verifier must reference a recipient receipt or signed statement.
- Obvious plaintext and secret-bearing field names are rejected.

## Explicit non-goals and limitations

This prototype does **not**:
- implement the JSON Schema Draft 2020-12 engine itself; it delegates schema
  validation to the pinned `jsonschema` crate, whose Draft 2020-12 keyword
  compliance is not assumed to be universal;
- independently establish that schema keywords beyond the checked-in schemas'
  exercised subset are correctly implemented by the selected engine;
- verify signatures, hashes of external artifacts, capability grants, Holochain
  source-chain lineage, DHT validation, cryptographic protocol correctness, or
  transport truth. Matching evidence digests and scope strings do not prove
  that the referenced artifacts are authentic or that the referenced actors
  are independent;
- prove that the trace producer told the truth or that actor references
  correspond to real independent principals;
- establish forward secrecy, post-compromise security, anonymity, availability,
  Byzantine safety, or production readiness.

The matching `recipient_receipt` digests are references, not cryptographic
verification. The receipt verifier's actor identity and independence must be
anchored by a separately authenticated trust policy in a future qualification
layer. This binary checks evidence-graph consistency; it is not a trust root.

The full Draft 2020-12 schema is supplied at the CLI and checked before the
semantic verifier runs. The selected engine advertises Draft 2020-12 support,
but this does not establish compliance for every keyword in the specification;
the exact checked-in schemas and tested scenarios remain the claim boundary.

## Test intent

The unit corpus checks the three checked-in Draft 2020-12 schemas/data pairs,
catalog uniqueness and all seven scenario families, plus controls for valid
receipt-backed success, source SHA mismatch, transport-ACK-only false success,
UNKNOWN delivery, dangling causal edges, causal cycles, same-clock ordering,
role confusion, missing attempt IDs, cross-operation attempt reuse, duplicate
inbox effects, secret-bearing fields, mismatched commitments, changed recipient
identity, missing recipient admission, recipient/verifier identity collision, semantically invalid resolved causal
relations, host/domain grant mismatch, DHT/source-action hash mismatch, and
recipient receipt mismatch. The profile, scenario catalog, and benchmark profile
are validated against their checked-in schemas by tests. The benchmark remains
a pre-registered artifact, not measured results.
