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

## Claim ceiling

A passing test suite demonstrates agreement of this Go implementation with its own frozen synthetic scenario expectations. It does not establish equivalence to Rust unless the same scenarios are executed against the Rust model; Holochain source-chain singleton uniqueness, DHT convergence, cross-agent exclusivity or fork exclusion; evidence-digest recomputation, Ed25519 verification, ActionHash resolution or predecessor authorisation; actual SAP account movement or Treasury receipt; provider idempotency, external bank settlement, recipient availability or legal finality; live performance, regulatory compliance or production readiness.

The Go registry's mutex test only demonstrates in-process serialization. It is not a distributed exactly-once theorem.
