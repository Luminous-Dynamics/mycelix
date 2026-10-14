# SAP Owner-Authenticated Balance Profile v0

Status: Draft implementation profile  
Related: AC-092 (#4110), AC-095 (#4114), AC-096 (#4116), AC-097 (#4117), AC-099 (#4123)

---

## 1. Principle

A final SAP account balance should be writable only by the agent who owns that account.

The desired invariant is:

    SapBalance.member_did
        ==
    DID derived from action.author

except for the exact-zero genesis initialization state.

This makes the agent's source chain part of the monetary authorization boundary.

Holochain describes each agent source chain as a chronological journal of that agent's actions. Concurrent writes to the same source chain serialize: one succeeds and a competing write observes the moved chain top and must retry. Holochain also permits multiple source-chain records to be committed atomically within one zome function call. citeturn863462search5turn863462search2

The architecture should use that native property rather than treating every account as globally shared mutable state.

---

## 2. Current problem

The current payments implementation allows an agent to perform:

    credit_sap(other_did, amount)

and therefore lets sender-side code mutate the receiver's balance.

That forces the balance itself to become a shared mutable object and creates unnecessary global ordering problems.

The intended replacement is:

    immutable authorization
        ↓
    recipient-local claim
        ↓
    recipient-authored balance update

The sender or issuer creates an obligation. The recipient owns the final account mutation.

---

## 3. Transfer protocol

### 3.1 Sender phase

The sender creates:

    SapTransferIntent {
        transfer_id,
        from_did,
        to_did,
        amount,
        authorization_reference,
        created_at,
        expiry
    }

The sender source-chain operation atomically:

1. verifies caller == from_did;
2. applies any due demurrage;
3. checks sufficient effective balance;
4. debits the sender;
5. records the immutable transfer intent.

This makes the sender-side debit authoritative on the sender's own source chain.

### 3.2 Receiver phase

The receiver invokes:

    claim_sap_transfer(transfer_id)

The receiver-side operation:

1. verifies caller == to_did;
2. resolves the immutable transfer intent;
3. verifies amount and recipient binding;
4. verifies the transfer has not already been claimed/finalized;
5. credits the receiver;
6. records the claim/finalization.

The receiver's balance is therefore written by the receiver.

### 3.3 Finality

Recommended state machine:

    Proposed
       ↓
    Debited
       ↓
    Claimed
       ↓
    Final

Failure states:

    Expired
    Cancelled
    Indeterminate
    Disputed
    Reconciled

A transfer in Debited but not Claimed state must never disappear from accounting.

---

## 4. Monetary meaning of an in-flight transfer

An unclaimed transfer is not new SAP and is not destroyed SAP.

Instead:

    sender spendable balance  -= A

while:

    receiver spendable balance += 0

and:

    outstanding settlement claim += A

The amount is temporarily immobilized between the two accounts.

This matters for reserve accounting.

### Spendable supply

    Q_spendable

counts SAP that holders can currently deploy.

### Outstanding claim supply

    Q_claims

counts SAP obligations that have been issued/debited but are not yet reflected in the recipient's spendable balance.

### Total monetary obligation

    Q_obligation = Q_spendable + Q_claims

A reserve report must not silently replace Q_obligation with Q_spendable.

If an in-flight claim remains legally/protocolically payable, it remains part of the liability base even though the corresponding SAP is not currently spendable.

---

## 5. Mint protocol

Issuance should use the same owner-authenticated pattern.

### Issuer phase

An authorized issuer creates an immutable:

    SapMintAuthorization

containing:

- canonical issuance ID;
- issuance class;
- recipient DID;
- exact amount;
- qualified basis reference;
- authority/quorum evidence;
- policy version;
- expiry;
- issuance status.

The authorization is not itself spendable balance.

### Recipient phase

The recipient invokes:

    claim_sap_mint(issuance_id)

The recipient source chain:

1. proves caller == recipient;
2. verifies the authorization;
3. verifies the issuance is final/claimable;
4. verifies exact recipient and amount;
5. verifies the authorization has not already been consumed;
6. atomically records consumption + balance increase.

The positive SAP delta is therefore authored by the account owner.

---

## 6. Exact-once semantics

Owner-authentication solves per-account ordering, but it does not solve issuer-side duplication.

Two distinct protections are required.

### Account serialization

The recipient source chain serializes competing claims against that account.

This prevents:

    same recipient
    + same authorization
    + concurrent claim
    = two balance increases

### Issuance uniqueness

The issuance authority must prevent:

    same basis
    + two independent authorizations
    = two legitimate issuances

This remains the AC-095 problem.

The separation is important:

    AC-099
        protects account state

    AC-095
        protects issuance identity / authority state

---

## 7. Why this fits Holochain better

Holochain's capability system is agent-centric: every call is signed, and author grants apply when the caller shares the callee's agent key. Remote calls can execute work from the callee's perspective. citeturn863462search0turn863462search1

A receiver-owned account therefore gives the protocol a natural security boundary:

    caller identity
        +
    action author
        +
    account owner
        =
    one coherent authorization domain

The protocol does not need to pretend that the DHT is a globally serialized database.

---

## 8. Integrity enforcement

For every non-genesis SapBalance creation/update:

    member_did == did_for_author(action.author())

must hold.

For every positive delta:

- justification must be explicit;
- justification must be addressable;
- referenced action must be valid;
- referenced object type must be expected;
- recipient/member binding must match;
- exact positive delta must match the authorization amount;
- authorization must be final under its declared protocol;
- authorization may not be consumed twice.

Holochain validation explicitly supports deterministic addressable dependencies through `must_get_valid_record`. This should be preferred over mutable collection scans for monetary authorization. citeturn863462search6turn863462search8

---

## 9. Atomic source-chain commits

Where sender-side or receiver-side operations create multiple records for one local state transition, those records should be committed in one source-chain operation.

For example, recipient mint claim can atomically commit:

    MintClaimRecord
    +
    updated SapBalance

Likewise the sender can atomically commit:

    TransferIntent
    +
    debited SapBalance

Holochain documents that multiple source-chain records written by one zome function can commit atomically. citeturn863462search5

This does not make two agents' source chains a single transaction. The protocol must still represent the cross-agent boundary explicitly.

---

## 10. Recovery protocol

Cross-agent settlement must have explicit recovery.

If sender debit succeeds but receiver claim is unavailable:

    state = Debited

If sender later learns that the transfer expired before claim, recovery must be deterministic:

    Debited
       ↓
    Expired
       ↓
    Compensated / Released

The compensation itself must be represented as a new typed monetary event, never as an invisible balance rewrite.

If claim outcome is ambiguous:

    state = Indeterminate

and:

    strict reserve qualification = fail closed

until reconciliation establishes the exact outcome.

---

## 11. Reserve accounting model

The reserve layer should maintain separate quantities:

    Q_spendable
    Q_claims
    Q_indeterminate

and at minimum:

    Q_obligation
        =
    Q_spendable
      + Q_claims
      + Q_indeterminate

where inclusion of each class in legal liabilities is determined by the monetary instrument's enforceability.

A reserve report must therefore publish both:

    immediately spendable SAP

and:

    outstanding SAP claims / obligations

This avoids the dangerous accounting shortcut:

    "not spendable"
        =
    "not a liability"

---

## 12. Monetary finality root

The canonical supply evidence should eventually distinguish:

- finalized owner balances;
- finalized transfer claims;
- unclaimed but valid claims;
- indeterminate/disputed claims;
- finalized issuance events.

A future `SupplyRoot` should therefore contain at least:

    finalized_balance_root
    finalized_issuance_root
    finalized_transfer_root
    outstanding_claim_root
    indeterminate_root
    sequence

This makes reserve coverage reproducible without forcing the account ledger into one global mutable record.

---

## 13. Adversarial qualification corpus

Minimum cases:

1. foreign agent creates a positive SapBalance;
2. foreign agent updates another member's balance;
3. sender directly credits receiver;
4. receiver claims a transfer twice;
5. two concurrent receiver claims race;
6. transfer amount differs from intent;
7. transfer recipient differs from intent;
8. expired transfer is claimed;
9. sender debit succeeds, receiver unavailable;
10. receiver claim succeeds, final acknowledgement is lost;
11. mint authorization claimed twice;
12. mint authorization claimed by wrong DID;
13. issuer creates two authorizations for one basis;
14. stale/invalid authorization is presented;
15. indeterminate transfer is counted as finalized supply;
16. reserve report uses spendable balance while omitting outstanding claims.

Green qualification requires exact-head evidence for these cases under the declared authority and recovery model.

---

## 14. Implementation sequence

1. Add owner binding to SapBalance integrity validation.
2. Introduce immutable SapTransferIntent / SapTransferClaim records.
3. Refactor sender debit to atomically create the transfer intent.
4. Introduce receiver claim operation.
5. Remove sender-side receiver balance mutation.
6. Introduce SapMintAuthorization and recipient claim.
7. Make SapMintRecord final issuance evidence, not direct balance mutation.
8. Add explicit pending/indeterminate accounting.
9. Update SupplyRoot to include outstanding claims.
10. Re-qualify AC-092 and AC-095 adversarial corpora.

The key principle is:

> other agents may authorize value for your account, but only your account owner may finalize your account state.

---

## 15. Qualification boundary

This profile can establish strong local account integrity and cross-agent settlement semantics.

It still does not prove:

- external collateral truth;
- legal ownership;
- physical measurement truth;
- market valuation;
- governance legitimacy;
- real-world redemption capacity.

Those remain separate evidence domains under AC-093, AC-094, AC-096, AC-097, and AC-098.
