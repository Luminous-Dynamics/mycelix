# SAP Owner-Authenticated Balance Execution v0

Status: Draft implementation profile

Related:
- AC-099 / #4123 — owner-authenticated SAP balances
- AC-092 / #4110 — SAP conservation / typed justification
- AC-095 / #4114 — issuance identity and exact-once serialization
- AC-115 / #4180 — non-governance issuance migration
- AC-116 / #4181 — cross-hApp settlement reconciliation
- AC-117 / #4183 — hearth SAP pool settlement

## 1. Core invariant

For every SapBalance create or update:

    member_did == DID derived from the signed action author

Creation is restricted to the owner's zero-balance initialization state.

A positive balance delta requires a valid, addressable justification and an exact amount/recipient match.

## 2. Transfer protocol

Sender:

    SapTransferIntent
    + exact sender pre-balance ActionHash
    + sender-authenticated balance debit

Recipient:

    SapTransferClaim
    + exact intent ActionHash
    + exact recipient pre-balance ActionHash
    + recipient-authenticated balance credit

A reused claim is rejected when its bound pre-balance action is no longer the balance update being attempted.

The sender transfer fee is charged only after the transfer intent is confirmed absent, so an idempotent retry does not charge the fee twice.

## 3. Governance issuance

Governance creates an immutable SapMintRecord authorization containing:

- recipient;
- amount;
- source/proposal identity;
- authorized_by_did;
- basis_id;
- authorization timestamp.

The recipient then creates SapMintClaim and performs the owner-authenticated balance increase.

Direct recipient credit from governance has been retired.

## 4. Raw credit boundary

credit_sap is retained only as an explicit compatibility failure.

Synchronous SAP send_payment is retired.

Legacy callers must migrate to typed authorization/claim flows rather than bypass the owner-authenticated balance invariant.

## 5. Cross-hApp payment boundary

A successful SAP transfer now means:

    sender intent/debit committed
    != recipient settlement complete

The bridge therefore keeps CrossHappPayment in Processing and emits PaymentAwaitingRecipientClaim.

## 5a. Durable compost settlement

Fee and demurrage redistribution are durable-first. Balance mutations persist a pending compost-delivery obligation instead of performing treasury side effects inside the monetary transaction.

Each pending queue link has a stable Holochain link-creation ActionHash. That hash is passed as the treasury delivery identity. Treasury records the corresponding CompostReceival under an append-only delivery-identity index and returns the existing receival on replay.

Therefore:

    fee/demurrage debit
    -> durable pending-delivery link
    -> treasury delivery
    -> queue-link deletion

does not rely on queue-link deletion as the replay guard. A successful treasury delivery followed by a failed queue-link deletion is replay-safe.

Queue presence is not completion evidence: the demurrage API reports redistributed=false until treasury delivery is actually observed as successful.

Settlement reconciliation:

    exact payment reference
    -> deterministic transfer id
    -> exact SAP transfer claim
    -> recipient + amount verification
    -> sender-authored payment update
    -> Completed

Only the payment source-chain author may finalize the bridge payment record.

## 6. Index integrity

The following indexes are validated against their canonical subjects:

- SapBalance DID index;
- transfer-intent index;
- transfer-claim index;
- mint-claim index.

Conflicting balance or transfer roots are rejected instead of selected by collection ordering.

## 7. Holochain dependency model

Validation resolves monetary dependencies through addressable hashes. Missing validation dependencies remain unresolved rather than being replaced with current mutable state.

The sender-side intent/debit and recipient-side claim/credit each rely on Holochain source-chain transaction atomicity within their respective zome calls. Cross-agent settlement remains a protocol boundary rather than a single global transaction.

## 8. Remaining qualification ceilings

This implementation does not yet prove:

- global issuance-basis exact-once serialization;
- external collateral or banking truth;
- legal ownership or redemption capacity;
- reserve solvency;
- hearth-pool serialization/typed settlement;
- complete migration of thermodynamic/collateral/fiat issuance adapters;
- independent exact-head CI qualification.

Those remain explicit downstream boundaries.

## 9. Candidate

Current implementation branch:

    feat/ac-099-owner-authenticated-sap-balance

The exact candidate commit must be read from the branch/PR at qualification time. Draft/queued/skipped/cancelled CI is not a PASS.
