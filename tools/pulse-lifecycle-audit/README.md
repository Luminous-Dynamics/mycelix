# Pulse Lifecycle Evidence Auditor

A standalone Rust verifier for redacted, six-plane Pulse authorization traces.

This is deliberately independent of the TLA+ models and independent of application coordinator code. It consumes a trace exported by an instrumented conductor/client test and checks whether the recorded planes are internally consistent.

## Run

From the repository root:

    cargo run --manifest-path tools/pulse-lifecycle-audit/Cargo.toml -- \
      --input tools/pulse-lifecycle-audit/examples/safe-trace.json

Write a machine-readable report:

    cargo generate-lockfile --manifest-path tools/pulse-lifecycle-audit/Cargo.toml
    cargo run --manifest-path tools/pulse-lifecycle-audit/Cargo.toml --locked -- \
      --input path/to/trace.json --output report.json

Exit codes:
- 0: PASS — no structural invariant violation or missing evidence detected.
- 1: FAIL — one or more safety invariants are violated.
- 1: INCOMPLETE — required evidence is missing or comes from the wrong evidence plane.
- 1: malformed input / tool execution error (the CLI reports the underlying error on stderr).

The example trace is synthetic. PASS on that fixture means only that the verifier accepts the intended structural contract.

## Trace planes

The schema at mycelix-workspace/mycelix-pulse/docs/security/pulse-lifecycle-trace-v1.schema.json is strict (additionalProperties: false).

- **Host authority:** exact grant ActionHash, grantor/grantee, assigned scope, grantor-scoped idempotency key, revoke ActionHash, and conductor authority-order index.
- **Private claim:** exact grant hash and recipient identity; claim must occur inside the grant lifetime.
- **Host call:** exact grant hash, caller, function, conductor authority-order index, host state at decision and host disposition.
- **Domain effect:** logical operation ID, attempt ID, durable effect ID and source-chain ActionHash.
- **Projection read:** observed authority epoch, minimum requested epoch, freshness decision and whether projection was used as authority.
- **Remote signal:** transport sender, payload-claimed sender, trust class, backing record reference, record author, canonical payload SHA-256, verifier build, trusted-state mutation and authorization flags.
- **Caller observation:** acknowledged, rejected, committed, not-committed or unknown. Unknown does not mean no effect.

## Safety properties checked

- one `(grantor, idempotency_key)` identity cannot mint multiple grant identities; different grantors may reuse the same operation ID;
- grant revocation is monotonic, references the exact original grant hash, and is authored by the original grantor;
- claim recipient and claim hash are bound to the exact assigned grant and fall within its lifetime;
- conductor authority-order indices are strictly increasing in the exported trace;
- a host allow cannot follow revocation, use the wrong grantee/function, precede an assigned private claim, or rely on a projection;
- a domain allow requires a host allow; a durable domain effect requires both host and domain allow;
- each logical operation ID has at most one durable effect, even across multiple attempts;
- checkpoint-constrained stale reads return UNKNOWN; unconstrained reads older than a known revocation are explicitly STALE_ADVISORY or UNKNOWN;
- projections are never used as host authority;
- a signal never authorizes an operation, and an unverified/advisory signal never mutates trusted state;
- caller outcomes reconcile per exact logical operation ID plus attempt ID; a not-committed/rejected attempt cannot contradict an effect from that same attempt, and a different retry attempt is not conflated;
- every projection read references an exact issued grant; a view older than a revocation already present in the trace cannot be labeled trusted-active/trusted-revoked, and an unconstrained stale view must be explicitly STALE_ADVISORY or UNKNOWN;
- remote signal event types are bounded identifiers; a VerifiedRecord disposition requires observer-plane evidence, matching claimed/transport sender and a backing ActionHash; an AuthenticatedPeerHint binds the sender but remains advisory; LocalOnly cannot claim remote sender metadata;
- missing or misclassified evidence becomes INCOMPLETE, not PASS.

## Evidence semantics and limits

A runtime trace must export a monotonically increasing **conductor authority-order index** at the point where grant issue, revoke and host decision are linearized. A trace bundle is currently treated as one ordered conductor history; do not merge raw authority indexes from independent conductors into one bundle without an explicit per-conductor identity and cross-conductor causal join. Wall-clock timestamps and source-chain order are not substitutes for this value. If Holochain does not expose the required event boundary, instrument a test build or mark the trace incomplete; do not infer order.

Every event names an evidence source and a receipt reference. Coordinator assertions cannot stand in for conductor host decisions. A remote peer may prove who sent a hint, but it cannot prove the referenced record is true. A VerifiedRecord trace disposition must include an observer-plane record author, exact backing ActionHash, SHA-256 digest of the canonical payload comparison and verifier build. The auditor checks structural consistency only; it does not authenticate the verifier or receipt. The verifier does not itself authenticate receipt signatures, inspect a conductor database, verify a DHT validation receipt, or prove that the exporter is honest. Its report therefore has the fixed claim ceiling STRUCTURAL_ONLY.

Never include capability secrets, message plaintext, private keys, raw authentication tokens or unnecessary personal identity data in the trace. Hashes and identifiers should be salted/pseudonymized where cross-run correlation is not required.

## Next qualification step

Connect the exporter to the two-conductor experiments in issues #5772 and #5781. Preserve:
- source commit and exact binary/DNA digests;
- per-conductor host authority-order index;
- source-chain action hashes and author identity;
- validator acceptance/rejection evidence;
- observer projection state and checkpoint;
- raw event receipt plus this verifier's exact version/input digest/report.

A runtime trace PASS is still not a general proof of Byzantine correctness, liveness, privacy, linearizability or production readiness. It is a reproducible structural cross-check of observed evidence.
