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

## Distributed-systems and production limitations

- The Mutex concurrency test proves only that this in-memory registry serializes its own threads.
- It does not prove Holochain source-chain claim uniqueness, cross-agent uniqueness, DHT convergence, compare-and-swap, or fork exclusion.
- Holochain's per-zome-call atomicity and chain-head-moved rollback are stronger than a naïve multi-write model, but they do not replace a durable sender/key idempotency record. A caller receiving a lost response must query the original key and retry the same identity, not invent a new one.
- The nested Treasury inter-zome call's atomic relationship to the enclosing payment invocation must be covered by a SweetConductor test. The current source swallows Treasury errors, so the fee-delivery state must not be silently represented as fully complete.
- The model does not implement the actual ledger, outbox worker, Treasury fee reconciliation, compensation/reversal, bank settlement, custody, or legal finality.

Production implementation must compose with #4491 (sender-scoped operation identity), #4559 (balance cause/conservation validation), #627 (SAP mint provenance/raw-credit removal), #4965 (durable outbox recovery), #5716 (receipt authenticity) and #5740 (SAP-only direct-payment boundary).

## Exact tests and next gate

The model has **17 unit tests authored**. The dedicated workflow pins Rust 1.96.0 and runs formatting, tests and Clippy. A queued run is not PASS; tests have not been locally compiled in the authoring environment because no Rust/Cargo/Nix toolchain is available.

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
