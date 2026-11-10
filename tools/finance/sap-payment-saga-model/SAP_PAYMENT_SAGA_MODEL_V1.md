# SAP Payment Saga Model v1

**Status:** pure Rust reference model; not wired to production Finance and not evidence of distributed atomicity or a live payment system.

## Correct transaction boundary

The reviewed Finance send_payment implementation calls debit_sap and credit_sap as direct Rust functions in the same payments coordinator invocation. They are not separate host zome calls. Holochain documents that source-chain writes made during one zome-function call are atomic: all commit together or all roll back, and a stale concurrent transaction can fail with a chain-head-moved error. The model therefore does **not** claim that the debit can commit while the credit fails inside that local invocation.

The Treasury path is different: payments invokes treasury::receive_compost through an inter-zome call using CallTargetCell::Local. The current payments code catches an error from that call, logs it, and continues to create a Payment marked Completed. The call boundary and swallowed error require a separate recovery contract and conductor-level qualification. Until that integration is proven, the model treats Treasury delivery as a separately retryable effect.

Holochain references:
- [Zome functions — atomic, transactional commits and chain-head movement](https://developer.holochain.org/build/zome-functions/).
- [Calling zome functions — inter-zome calls within the same hApp](https://developer.holochain.org/build/calling-zome-functions/).

## Paradigm: atomic local payment + durable fee outbox

The reference protocol deliberately groups the local payment write set into one effect:

1. stable sender-scoped operation claim and payer authorisation;
2. one local atomic transaction containing sender debit, recipient credit, Payment record, signed Receipt and a durable Treasury-fee outbox intent;
3. separate Treasury fee delivery using the outbox-derived stable effect ID;
4. reconciliation of the Treasury receipt against that exact outbox ID and Payment action;
5. overall operation Completed only after all required legs are confirmed.

The local Payment record's Completed status means that the **local principal-transfer transaction** committed. The journal's overall status can still be FeeDistributionPending or Indeterminate while the required Treasury fee is outstanding. This avoids conflating principal transfer with fee allocation.

Every local retry uses the same sender/key/intent identity. Every Treasury retry uses the same outbox/effect identity. A timeout never creates a new effect ID. A duplicate effect with changed subject/evidence conflicts.

## Modeled effect evidence

A local-transaction commit carries typed evidence for both sides of the balance movement and the Payment/Receipt subjects:

- sender account, recipient counterparty, debit direction and exact principal + fee;
- recipient account, sender counterparty, credit direction and exact principal;
- Payment action reference and Completed local Payment status;
- distinct Receipt action reference, exact sender signer, v1 receipt profile and signature-evidence reference;
- stable Treasury outbox ID when the fee is non-zero.

Treasury commit evidence must identify the global fee pool, the source sender, the exact fee amount, the exact Payment action, the original outbox/effect ID, and a source/treasury action pair.

The 17 Rust unit tests cover sender/key retry, immutable-intent conflict, amount overflow, local commit as one indivisible subject, exact debit/credit matching, Payment/Receipt status and signature-profile gates, Treasury outbox prerequisites, wrong pool/amount/payment/key, fee timeout recovery, zero-fee behaviour, same-evidence retry, changed-evidence conflict, pre-effect rejection, in-memory concurrency and operation-ID delimiter resistance.

## Claim ceiling for typed evidence

The model checks the **shape and semantic agreement** of caller-supplied evidence. It does not recompute that an evidence digest matches evidence bytes, verify Ed25519, fetch real Holochain records, prove the Payment predecessor is valid, verify Treasury's actual balance transition, or authenticate the caller's capability/authority. A SHA-256-shaped string or action-reference-shaped string is not proof of authenticity. Production integrity code must resolve and validate the source records and cryptographic proofs.

## Canonical effect-evidence digest v1

The reference model no longer accepts a merely SHA-256-shaped evidence_digest. It recomputes the digest over the complete typed evidence subject using the domain-separated profile MXSAP-EFFECT-EVIDENCE-V1 followed by a NUL byte.

The byte profile is fixed and versioned:

- UTF-8 strings use a four-byte unsigned big-endian byte length followed by the exact UTF-8 bytes.
- Minor-unit values use eight-byte unsigned big-endian integers.
- Common fields are ordered as operation ID, effect ID, leg, sender DID, recipient DID, principal, fee and currency.
- The local-transaction and Treasury-fee variants use separate one-byte type tags and fixed field orders. Payment status and optional outbox identity also have explicit one-byte tags.
- Every string field is limited to 4,096 UTF-8 bytes. Invalid or over-limit fields are rejected.
- evidence_digest is excluded from its own preimage. The output is lowercase sha256: plus 64 hexadecimal digits.

Rust and Go consume the same known-answer fixture at tools/finance/sap-payment-saga-go-verifier/fixtures/sap-payment-evidence-v1.kat. The fixed Treasury vector produces sha256:5d4182a07eaa54dc3c5a358a8cee0b98019d333e8d9eae8cd1fd759c635461d9.

This is a project-specific canonical binary profile, not a claim of RFC 8785 JSON Canonicalization Scheme or deterministic CBOR conformance. Any field/order/encoding change requires a new profile version and new known-answer vectors.

The digest now detects changes to typed evidence fields even if the caller recomputes the digest after mutation. It still does not authenticate the source: production code must resolve the cited Holochain actions, verify Ed25519 and predecessor authority, and establish that the signed evidence actually corresponds to committed records. signature_digest remains only a typed digest reference in this model; signature verification is not implemented here.

## Anchored append-only journal replay

The reference model now includes `src/replay.rs`, a separate event-stream representation and reducer for reconstructing the mutable `PaymentJournal` projection.

The log records a typed creation event, authorization/rejection, stable effect preparation, indeterminate outcomes, and typed effect commits. Each frame binds:
- a monotonically increasing sequence number;
- the previous frame's digest;
- a versioned, domain-separated event digest over fixed-width integer and length-prefixed UTF-8 fields (event text fields are capped at 16 KiB);
- the exact event variant and its immutable subject.

`replay_journal` verifies the sequence, previous-digest chain, event digest, externally supplied event-count/tail anchor, operation identity, and legal saga transitions before returning a newly reconstructed journal. The committed evidence event binds both the recomputed typed evidence digest and the caller-supplied digest; ordinary commit validation still rejects a stale or fabricated digest.

This adds a distinct integrity question beyond "does the current mutable projection look valid?": **can this exact operation state be reconstructed from the retained ordered history, and does that history still match an independently retained tail anchor?** Tests cover full local-payment/Treasury recovery replay, tail truncation, event reordering, event-payload tampering, impossible transition order, and invalid committed evidence.

The Go standard-library verifier now implements the same event framing and a separate replay reducer in `tools/finance/sap-payment-saga-go-verifier/replay.go`. Both languages consume `tools/finance/sap-payment-saga-go-verifier/fixtures/sap-payment-journal-replay-v1.kat`, which pins the local and Treasury typed-evidence digests, all eight event-frame digests of the full local-payment/Treasury-recovery sequence, and its final tail anchor. Rust includes seven replay-specific tests for known answers, full Treasury recovery replay, tail truncation, reordering, tampering, impossible transition order and invalid committed evidence. Go includes those seven scenarios plus two non-canonical event-shape negative-control tests covering six event variants; its closed checks mirror the Rust enum's one-variant-only representation. The Go implementation is a second implementation in the same repository, not organizationally independent. A Go microbenchmark now constructs exact 100/1,000/10,000-frame synthetic histories with repeated Treasury indeterminate/same-effect retry cycles and measures in-memory replay cost (allocations and elapsed time). Its procedure is documented in the Go verifier README; it has not yet been run, and it is not a Holochain or real-settlement performance result.

The event stream is a reference model, not yet the production durable store or Holochain record format. The anchor must be retained outside the stream (for example in an independently protected operation receipt/checkpoint) to detect tail truncation. The model does not authenticate that anchor, authenticate event authors, resolve Holochain ActionHashes, prove distributed append-only storage, or establish monetary movement. A party able to rewrite the entire stream and its anchor can recompute this hash chain; production needs an authenticated, independently retained checkpoint and source-record verification. Event sequence is local logical order, not a global wall-clock or cross-agent total order.

## Distributed-systems and production limitations

- The Mutex concurrency test proves only that this in-memory registry serializes its own threads.
- It does not prove Holochain source-chain claim uniqueness, cross-agent uniqueness, DHT convergence, compare-and-swap, or fork exclusion.
- Holochain's per-zome-call atomicity and chain-head-moved rollback are stronger than a naïve multi-write model, but they do not replace a durable sender/key idempotency record. A caller receiving a lost response must query the original key and retry the same identity, not invent a new one.
- The nested Treasury inter-zome call's atomic relationship to the enclosing payment invocation must be covered by a SweetConductor test. The current source swallows Treasury errors, so the fee-delivery state must not be silently represented as fully complete.
- The model does not implement the actual ledger, outbox worker, Treasury fee reconciliation, compensation/reversal, bank settlement, custody, or legal finality.

Production implementation must compose with #4491 (sender-scoped operation identity), #4559 (balance cause/conservation validation), #627 (SAP mint provenance/raw-credit removal), #4965 (durable outbox recovery), #5716 (receipt authenticity) and #5740 (SAP-only direct-payment boundary).

## Exact tests and next gate

The model has **20 unit tests authored**, including two shared canonical-digest known-answer vectors, typed-field mutation check and over-limit field rejection. The dedicated workflow pins Rust 1.96.0 and runs formatting, tests and Clippy. A queued run is not PASS; tests have not been locally compiled in the authoring environment because no Rust/Cargo/Nix toolchain is available.

Next production gates:
- same-sender concurrent zome-call test: one transaction commits; stale chain-head attempt leaves no partial local write set;
- lost response after local commit: query by original sender/key and return the committed Payment/Receipt without another debit;
- Treasury call failure/timeout: durable outbox remains pending and repeated worker calls use the same effect ID;
- Treasury side effect succeeds but acknowledgement is lost: retry resolves the same Treasury receipt and does not credit the pool twice;
- mutation tests for wrong sender, payee, currency, amount, fee, Payment action, outbox identity and Treasury pool;
- independent review of the Holochain transaction boundary and the cross-zome outbox protocol.

Run commands:

~~~sh
cargo fmt --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml
cargo clippy --manifest-path tools/finance/sap-payment-saga-model/Cargo.toml --all-targets -- -D warnings
~~~


## Owner-scoped journal checkpoint subject v1

The checkpoint module defines the canonical subject for independently replayable Payments and Treasury journal checkpoints. It binds:

- operation ID and immutable-intent digest;
- owner domain (Payments or Treasury) and event-log identity;
- event count and exact tail digest;
- owner-local checkpoint sequence and optional prior-checkpoint digest;
- source action hash and source-chain frontier;
- issuer DID, verification-method identity and owner-authority grant reference;
- issuance time in Unix milliseconds.

The binary profile is MXSAP-JOURNAL-CHECKPOINT-SUBJECT-V1: fixed owner tag, u32 big-endian length-prefixed UTF-8 strings, u64 big-endian counters/timestamp, and an explicit option tag for the prior checkpoint. Each text field is bounded to 4,096 bytes. Sequence one must have no prior checkpoint; later sequences must name a prior digest. The checkpoint validator also recomputes MXSAP-PAYMENT-INTENT-V1 over sender, recipient, client idempotency key, payload fingerprint, principal, fee and currency, and checks that the operation ID is the sender-scoped ID derived from that exact intent. The shared Rust/Go KAT pins the exact intent bytes/digest and checkpoint-subject bytes/digest.

This is a subject-identity/reference increment, not a signature or Holochain proof. In the native same-DNA deployment, the signed Holochain Create action should bind the committed entry to its action author. The integrity validator must separately prove that author-to-DID relationship, owner-domain authorization, source ActionHash/entry type, exact operation scope and predecessor/frontier relationship. The authority-grant reference is not itself an authorization proof. Missing, unavailable, stale, forked or conflicting source evidence remains unqualified/indeterminate. No settlement or legal-finality claim follows from this digest.
