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
cargo test --manifest-path mycelix-workspace/mycelix-pulse/tools/fabric-verifier/Cargo.toml --test cli
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
Keep the schema self-contained. A package-specific Cargo.lock is not
committed. The draft-safe workflow generates a run-specific lockfile, uses
--locked for tests and Clippy, and uploads the generated lockfile as a 14-day
artifact. Resolution is frozen within each run, but transitive dependencies
are not yet pinned across separate runs by source control.

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
  attributed receipt verification. Each required transition must be represented
  by the specified direct causal edge; a list of unrelated events connected
  through generic ancestry is not sufficient.
- Host admission and domain authorization must reference the same grant hash,
  scope and exact cell. The authored source-action hash must match the DHT
  projection's action hash. Receipt creation and verification must reference
  the same recipient-receipt digest and scope. These are consistency checks on
  supplied references, not signature or host-state verification.
- Attempt-scoped authority, authorship, validation, transport and recipient
  events must bind to the same concrete attempt ID. Logical intent, crypto
  selection, outbox commit and DHT observation remain operation-scoped.
- One recipient identity and one content commitment must persist through
  admission, commit, decryption, receipt creation and receipt verification.
  The verified event must reference the same recipient-receipt digest.
- Obvious plaintext and secret-bearing field names are rejected.

## Group fan-out verification

The verify-fanout command checks a parent manifest against the exact
bytes of every supplied recipient child trace:

~~~bash
cargo run --manifest-path mycelix-workspace/mycelix-pulse/tools/fabric-verifier/Cargo.toml -- \
  verify-fanout fanout-manifest.json \
  --manifest-schema mycelix-workspace/mycelix-pulse/docs/pulse-group-fanout-manifest.schema-v0.1.json \
  --trace-schema mycelix-workspace/mycelix-pulse/docs/pulse-fabric-trace.schema-v0.1.json \
  --manifest-sha256 EXTERNALLY_PINNED_64_HEX_SHA256 \
  --child recipient-a.trace.json \
  --child recipient-b.trace.json
~~~

The external manifest SHA-256 pin is mandatory. Each child trace's exact
byte digest is embedded in the pinned manifest. The verifier checks that the
manifest's frozen recipient commitments map one-to-one to child logical
operation IDs, that no child is missing or duplicated, that all child traces
bind the same group, epoch, membership-snapshot digest, recipient-set
commitment and content commitment, and that each child passes the v0.1
recipient-level causal verifier. The only v0.1 aggregation policy is
ALL_REQUIRED.

A fully receipt-backed child set is reported as
ALL_REQUIRED_RECEIPT_CLAIMS_CONSISTENT, not COMPLETE_VERIFIED. This is
deliberate: v0.1 verifies structure, causal consistency and exact
content-addressed inputs. It does not validate receipt signatures, authenticate
the manifest author, establish that the membership snapshot is true, or prove
deadline compliance. An external digest pin establishes which bytes were
checked; the trust policy that selects that pin must be supplied independently.
A later trust layer must verify signatures, principal independence, membership
authority and trusted time before using a cryptographically strong
group-completion claim.

The recipient-set commitment is deterministic and domain-separated:
sort the 64-character lowercase recipient commitments lexicographically,
then SHA-256 the byte prefix
`PULSE-GROUP-FANOUT-RECIPIENT-SET-V0\\0`, followed by each commitment's
UTF-8 bytes prefixed by its unsigned 64-bit big-endian byte length. A golden
vector is covered by the Rust unit tests. This encoding makes ordering and
concatenation unambiguous; it does not make low-entropy identifiers private.

Do not put raw member identifiers or the roster in public manifests. Recipient
commitments must be opaque, scoped pseudonyms or keyed commitments generated
under an independently governed secret; do not use bare SHA-256 of an email
address, phone number or other low-entropy identifier. Group sizes and
commitment reuse can themselves leak metadata.

PARTIAL is a recipient-level terminal disposition for incomplete but
positively observed progress. UNKNOWN remains distinct: the verifier must
not turn an absent or ambiguous outcome into a success.

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

The Rust unit tests validate the checked-in profile, trace, scenario and
benchmark schemas/data pairs; confirm scenario ID uniqueness and all seven
scenario families; and exercise positive and negative semantic controls for
source identity, role assignment, attempt identity, causal resolution,
acyclicity, clock order, duplicate logical effects, sensitive fields, receipt
bindings, grant-reference consistency, source-action/DHT hash continuity and
complete verified-delivery lifecycle requirements.

The CLI integration tests consume three actual checked-in JSON traces:
1. Complete receipt-backed success, expected to be EVIDENCE_CONSISTENT.
2. Transport ACK only with terminal disposition UNKNOWN, a valid unresolved
   evidence record.
3. An ACK incorrectly promoted to SUCCESS_VERIFIED, expected to be rejected.

A fourth CLI integration test checks rejection of an externally supplied
source-SHA mismatch. The benchmark profile and schema are validated by tests,
but remain pre-registered artifacts, not measured results.
