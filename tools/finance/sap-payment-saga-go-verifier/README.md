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
