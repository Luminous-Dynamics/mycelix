# Independent Go reference verifier for SAP payment saga v1

Status: synthetic software-conformance implementation; not a production payment component.

This package independently implements the effects-disabled SAP payment saga contract in Go's standard library. It does not import, bind to, or call the Rust crate. Its purpose is to expose semantic disagreement between two implementations and to run a frozen adversarial scenario pack.

## What it checks

- Sender-scoped sender DID plus client idempotency key identity with length-prefixed operation IDs.
- Exact same-intent retry versus same-key / changed-intent conflict.
- Checked principal-plus-fee arithmetic before journal creation.
- One local atomic-payment transaction evidence subject, binding debit, credit, Payment, Receipt and fee outbox intent.
- Separate Treasury fee effect, gated on the local transaction and bound to its exact Payment action and stable outbox ID.
- Stable effect IDs across prepared, indeterminate, resumed and committed states.
- Typed evidence subject matching, canonical digest/reference shape, exact receipt signer/profile, and same-evidence idempotency.
- Explicit Indeterminate, FeeDistributionPending, Completed and Rejected dispositions.

## Run

    go test -race -count=1 ./...
    go vet ./...

The tests consume fixtures/sap-payment-saga-conformance-v1.json, a frozen 20-case synthetic corpus. It contains no banking credentials, private account data or live value.

## Comparison with industry practice

The aspiration is not simply to retry with the same key. The provider contract is part of the retry theorem:

- Adyen documents key uniqueness within a company account, 7–14 day key retention, regional endpoint caveats, and a distinct transient-error signal for retrying an in-progress duplicate. Mycelix therefore needs provider-profile-specific scope, retention and retry semantics rather than assuming one universal key guarantee: https://docs.adyen.com/development-resources/api-idempotency
- Holochain documents zome-function writes as an atomic source-chain transaction; a competing stale source-chain write fails with a moved-chain-head error, and the caller is the retry boundary. This does not create atomicity across agents or external rails: https://developer.holochain.org/concepts/3_source_chain/
- SARB's QR+ Standard 1.2 describes payment initiation data; the selected underlying rail executes the payment. A QR+/PayShap initiation payload is not settlement evidence: https://www.resbank.co.za/en/home/publications/publication-detail-pages/other-publications/2026/qr-1-2-overview
- ISO's Bank-to-Customer Cash Management set distinguishes CAMT.052 account reports, CAMT.053 statements and CAMT.054 debit/credit notifications; the exact pinned message set and bank MUG govern their evidence semantics: https://www.iso20022.org/iso-20022-message-definitions?search=Bank-to-Customer+Cash+Management

## Canonical effect-evidence digest v1

The Go implementation recomputes evidence_digest over every typed field using the same domain-separated MXSAP-EFFECT-EVIDENCE-V1 profile as Rust:

- UTF-8 strings are four-byte unsigned big-endian length-prefixed and limited to 4,096 bytes.
- Minor-unit values are eight-byte unsigned big-endian integers.
- Common identity/value fields precede a one-byte local-payment or Treasury-fee variant tag and fixed variant-specific fields.
- Payment status and optional outbox identity have explicit one-byte tags.
- The digest field is excluded from its own preimage. Output is lowercase sha256: plus 64 lowercase hex digits.

Both languages load the same known-answer file at fixtures/sap-payment-evidence-v1.kat. The fixed Treasury vector is sha256:5d4182a07eaa54dc3c5a358a8cee0b98019d333e8d9eae8cd1fd759c635461d9.

This is a project-specific canonical binary profile, not a claim of RFC 8785/JCS or CBOR conformance. It detects typed-field mutation, but it does not prove that the fields describe authentic Holochain actions or a real ledger movement.

## Anchored append-only journal replay

`replay.go` adds a standard-library-only reducer for the same typed event stream as Rust `src/replay.rs`. It reconstructs a fresh `Journal` from create, authorize/reject, effect-prepared, indeterminate and effect-committed events.

Each frame binds a sequence number, prior-frame digest, versioned event encoding and event digest. Event text fields are capped at 16 KiB before hashing. Replay verifies sequence continuity, the hash chain, operation identity, external event-count/tail anchor and legal journal transitions. Committed effects re-run typed evidence and canonical digest validation. The Go verifier also rejects non-canonical event shapes (unused identity, intent, leg, effect-ID or evidence fields) so payload omitted from the digest cannot be smuggled into an otherwise valid event.

Both languages consume `fixtures/sap-payment-journal-replay-v1.kat`, which pins the local and Treasury typed-evidence digests, all eight event-frame digests for the full local-payment/Treasury-recovery history, and its final tail anchor. The Go tests also replay a full local-payment/Treasury recovery with an indeterminate fee result, and reject tail truncation, reordered/tampered events, impossible transitions and invalid committed evidence.

This is a reference replay model, not a durable Holochain event store. The anchor must be held independently to detect tail truncation, and the anchor itself is not authenticated by this model. A fully rewritten stream plus rewritten anchor can be rehashed. Production still requires independently protected checkpoints, authenticated source records and ActionHash/predecessor verification.

## Replay scale benchmark

`replay_benchmark_test.go` measures pure in-memory journal validation/reconstruction for exact 100, 1,000 and 10,000 frame histories. Each history contains the same local-payment and Treasury legs; variable depth comes from repeated `Indeterminate → same-effect Prepared` recovery cycles. Fixture construction is outside the timed region. The benchmark reports elapsed time and allocation metrics.

Run with the Go version declared by `go.mod` (1.27.0):

```sh
go test -run 'a^' -bench '^BenchmarkReplayJournal' -benchmem -benchtime=200ms -count=30
```

The CI qualification captures the Go version, OS/architecture and CPU model, retains the 30 raw samples per history size in the job log, and fails the summary step unless all three groups contain exactly 30 samples. It computes nearest-rank p50/p95/p99 latency plus mean bytes and allocations per operation. For publishable cross-language comparisons, preserve the exact commit SHA, hardware, build mode, fixture bytes, and full raw output. This is a synthetic in-memory replay microbenchmark: it excludes Holochain DHT retrieval, source-chain validation, conductor restart, storage fsync, remote peers, and real monetary effects. No performance result is claimed until executed on the exact head.

## Claim ceiling

A passing test suite demonstrates agreement of this Go implementation with its own frozen synthetic scenario expectations. It does not establish equivalence to Rust unless the same scenarios are executed against the Rust model; Holochain source-chain singleton uniqueness, DHT convergence, cross-agent exclusivity or fork exclusion; Ed25519 verification, ActionHash resolution or predecessor authorisation; actual SAP account movement or Treasury receipt; provider idempotency, external bank settlement, recipient availability or legal finality; live performance, regulatory compliance or production readiness.

The Go registry's mutex test only demonstrates in-process serialization. It is not a distributed exactly-once theorem.


## Paired Rust / Go replay benchmark

The replay benchmark now measures the same deterministic synthetic histories in both implementations on one CI runner:

- exact sizes: 100, 1,000 and 10,000 event frames;
- identical history shape: Created, Authorized, local Payment prepared/committed, Treasury prepared, repeated Indeterminate → same-effect Prepared recovery cycles, then Treasury committed;
- the fixture and evidence construction are outside the timed region;
- each implementation captures 30 raw samples per size, using a 200 ms sample window;
- the Rust harness reports nanoseconds per replay; the Go harness reports nanoseconds, bytes and allocations per replay;
- a single summary step refuses to report percentiles unless every language/size has exactly 30 samples, then computes nearest-rank p50/p95/p99 from sorted copies.

The Rust harness is at tools/finance/sap-payment-saga-model/benches/replay_benchmark.rs. Both benchmarks run in the same Actions job and print the Rust/Go toolchain versions, OS/architecture and CPU model. Raw sample output remains in the job log.

This is a matched in-memory microbenchmark, not a production performance claim. It excludes conductor scheduling, DHT retrieval/validation, source-chain writes, fsync or database durability, network latency, provider APIs and real settlement. Sequential language runs may experience clock/thermal variation; preserve raw samples and rerun on controlled hardware before making a performance claim. Go allocation counts and Rust latency are not directly interchangeable metrics.


## Prefix-by-prefix crash-window reconstruction

Rust and Go tests now reconstruct the live journal projection after every valid prefix of the local-Payment/Treasury-recovery path: Created, Authorized, local Prepared, local Committed, Treasury Prepared, Treasury Indeterminate, same-effect Treasury Prepared, and final Treasury Committed. Each prefix is replayed against its own event-count/tail anchor and compared with the live in-memory journal. This tests intermediate recovery states rather than only the terminal Completed state.

These are deterministic state-machine prefix checks. They do not inject process termination into a real conductor, storage engine, message broker or provider API; those remain a separate runtime qualification layer.

## Owner-scoped journal checkpoint subject v1

The Go checkpoint module independently encodes the Payments/Treasury checkpoint subject using the fixed MXSAP-JOURNAL-CHECKPOINT-SUBJECT-V1 binary profile. It binds operation and intent identity, owner domain, event-log identity/count/tail, owner-local checkpoint sequence and prior digest, source action/frontier, issuer DID and verification method, an authority-grant reference, and issuance time.

The shared fixture fixtures/sap-payment-journal-checkpoint-v1.kat pins canonical intent bytes/digest and checkpoint-subject bytes/digest. ValidateAgainstIntent recomputes MXSAP-PAYMENT-INTENT-V1 from sender, recipient, client idempotency key, payload fingerprint, principal, fee and currency, then checks the sender-scoped operation ID. Tests mutate every checkpoint dimension and immutable-intent field and reject operation/digest substitution, malformed digests, empty/oversized fields, unknown owners, zero counters/timestamps and inconsistent prior-checkpoint sequencing.

This remains a reference subject encoder/verifier. The digest is not authorship or authorization proof. A native Holochain Create action and its author must be validated by the production integrity boundary; owner authorization and exact source ActionHash/frontier checks remain separate. The reference model does not establish DHT convergence, fork exclusion, physical effect, external settlement or legal finality.


## Deterministic saga trace profile v1

`trace.go` implements the language-neutral MXSAP-SAGA-TRACE-V1 input profile. Rust and Go use identical SplitMix64 seed derivation, draw order, bounded scenario templates, and fixed-width big-endian encoding. The 20-byte command records bind opcode, owner, fault, reserved-zero byte, subject nonce and value.

Both implementations strictly decode complete traces and reject unknown opcodes/owners/faults, nonzero reserved bytes, truncated inputs and trailing bytes. They consume the shared fixture at `../sap-payment-saga-model/fixtures/saga-trace-v1.kat`; Rust embeds the same fixture in its unit test. The trace generator and decoder are input infrastructure only: the commands are not yet executed against the saga state machine, and the initial 4,096-seed smoke corpus is not the million-trace qualification. See `tools/finance/sap-payment-saga-model/SAGA_TRACE_V1.md` for the frozen grammar, seed algorithm and current claim ceiling.


## Effect-level financial compensation V2

The independent Rust and Go reference models add a separate V2 compensation profile without changing Trace V1 or Oracle V1:

- `../sap-payment-saga-model/src/compensation.rs` and `compensation.go` model a new compensating effect linked to one positively settled forward effect.
- The exact forward source/destination, funding account, beneficiary account, asset, reversible ceiling, plan digest, owner-scoped grant, liquidity reservation and outcome subject are bound into canonical domain-separated SHA-256 profiles.
- Partial refunds share cumulative plan and capability ceilings. Capability IDs cannot be reused with changed scope; reservation IDs are consumed once.
- Indeterminate and observed-unsettled outcomes retain encumbrance. An authoritative absence can permit only the same stable compensation effect to be retried while the same capability remains live.
- Cancellation releases capability use and liquidity encumbrance only before dispatch or after an authoritative absence.
- Rust and Go compare all seven canonical subject digests against the shared `../sap-payment-saga-model/fixtures/sap-compensation-v2.kat` vectors.

The current books use indexed reservation/capability maps and incrementally maintained settled/encumbered totals. Go's book serializes the in-memory transitions with a mutex. Rust relies on exclusive mutable access. A shared in-memory total is not a production account hold and is not proof of distributed serialization.

Run the compensation conformance and race tests with:

```sh
go test -race -count=1 ./...
go vet ./...
```

The 10,000-refund regression checks the indexed admission path. `compensation_benchmark_test.go` records single-threaded and parallel in-memory admission throughput, allocations and bytes:

```sh
go test -run 'a^' -bench '^BenchmarkCompensationReserve(V2|ParallelV2)$' -benchmem -benchtime=200ms -count=10
```

This benchmark includes canonical hashing, map-index checks, aggregate cap checks and mutex contention. It excludes durable storage, Holochain validation, external account holds, provider calls and settlement. No latency or throughput claim is made until the exact-head runner has completed and the hardware/toolchain/raw samples are retained.

See `../sap-payment-saga-model/COMPENSATION_V2.md` for the state machine, safety invariants, industry comparison, benchmark protocol and the required provider-signature / durable-reservation / actual-SUT qualification stages.
