# SAP Issuance Conservation Profile v0

Status: Draft implementation profile  
Related: AC-092 (#4110), AC-095 (#4114), AC-091 (#4109)  
Scope: Mycelix Finance / SAP monetary state

---

## 1. Purpose

This profile closes the gap between a provenance record and a reserve-grade monetary state transition.

Required pipeline:

    issuance basis
        ↓
    typed authorization
        ↓
    authenticated authority
        ↓
    canonical issuance identity
        ↓
    serialized consumption
        ↓
    balance delta
        ↓
    finalized supply state

A balance is not final monetary supply merely because a coordinator successfully wrote an entry.

The reserve standard therefore distinguishes authorization, attribution, conservation, serialization, and finality.

## 2. Critical distinction: provenance is not serialization

SapBalance.justified_by is necessary but insufficient.

A validator can deterministically inspect an action and explicit hash-addressed dependencies. It cannot safely implement a global uniqueness constraint by reading a mutable collection and assuming that no concurrent writer will create another matching event.

Therefore:

    no matching mint found right now

must not be treated as equivalent to:

    this issuance basis has been consumed exactly once globally

The first is an observation. The second requires an authority/serialization protocol.

This follows Holochain's validation model: validation can depend on explicit addressable DHT data, while collections are intentionally unsuitable for deterministic validation. Source-chain writes are serialized per author, but that is not a global serialization mechanism for independently-authored monetary state.

## 3. Monetary state objects

### 3.1 SapMintIntent

An intent to create new SAP.

Required fields:
- unique intent ID;
- issuance class;
- authorized issuer set;
- recipient;
- exact amount in μSAP;
- issuance basis reference;
- source-domain identity;
- policy/version identifier;
- expiry;
- created timestamp.

A mint intent is not supply.

### 3.2 SapMintRecord

The immutable monetary issuance event.

Required fields:
- canonical issuance ID;
- intent/reference;
- exact amount;
- recipient;
- source;
- authority/quorum proof;
- canonical sequence/epoch;
- finalized timestamp.

A SapMintRecord is the only object that can justify a final positive SAP supply delta.

### 3.3 SapTransferRecord

The immutable conservation event for moving existing SAP.

Required fields:
- unique transfer ID;
- sender;
- recipient;
- exact amount;
- authorizing action/receipt;
- transfer sequence or operation identity;
- created timestamp.

A transfer record changes ownership, not aggregate supply.

### 3.4 SapSupplyFinality

A canonical snapshot or sequence boundary that identifies the set of finalized issuance and transfer events accepted into reserve-grade supply accounting.

This is what reserve reports should consume rather than raw mutable balance observations.

## 4. Required state machine

### Mint lifecycle

    Proposed
       |
       v
    Authorized
       |
       v
    Serialized
       |
       v
    Consumed
       |
       v
    Final

Terminal failure states:

    Rejected
    Expired
    Revoked
    Conflicted

Only Final issuance may increase reserve-grade circulating supply.

### Transfer lifecycle

    Proposed
       |
       v
    Authorized
       |
       v
    Committed
       |
       v
    Final

A transfer must never be interpreted as issuance.

## 5. Authority model

The preferred Mycelix design is threshold authorization plus canonical serialization, rather than trusting a single unrestricted minting key.

### Authorization

An issuance class defines a threshold:

    N authorities
        →
    K-of-N certificate

The certificate commits to intent ID, recipient, amount, issuance class, basis hash/reference, policy version, expiry, and authority identities.

No certificate may be reused for a different amount or recipient.

### Serialization

A separate canonical issuance sequence assigns domain/namespace, monotonic epoch, sequence number, and issuance ID.

The serialization authority may be:
1. a dedicated monetary-authority agent for a small deployment;
2. a threshold-controlled issuance service;
3. a deterministic federation reconciler for multiple domains.

The profile must explicitly name which mode is active. There is no hidden assumption that the DHT itself provides global ordering.

## 6. Why a single global mutable counter is insufficient

The existing annual mint counter is useful as an operational guard, but it cannot by itself establish globally serialized supply.

Two authorized writers can observe the same prior state and both produce locally valid next states.

Therefore the authoritative question is not:

    Does the counter currently have room?

It is:

    Does this canonical issuance sequence contain this issuance,
    and what is the finalized cumulative amount at its sequence boundary?

The counter should eventually become a derived index of canonical issuance records.

Where a fast counter remains operationally useful, it must be treated as an optimization whose correctness is checked against canonical issuance state.

## 7. Balance justification rules

Integrity should enforce:

### Creation

A new SapBalance is valid only when:

    balance == 0
        AND justified_by == None

or:

    balance > 0
        AND justified_by references a valid finalized issuance/transfer event

### Update increase

For a positive balance delta, require:
- justification is present;
- referenced event exists;
- referenced event type is permitted;
- recipient matches member_did;
- amount equals the exact positive delta;
- event is final under the declared authority/serialization model;
- event cannot be reused as a second positive delta.

### Update decrease

A decrease does not create aggregate supply and may be justified by transfer, demurrage, fee/sink, redemption/burn, or explicit protocol adjustment.

Each reduction class should remain typed so that monetary accounting is explainable.

## 8. Transfer conservation

For a finalized transfer:

    Δsender = -A
    Δreceiver = +A

therefore:

    Δaggregate_supply = 0

The transfer record must be created before the dependent balance increases so that the receiver's positive delta always references an existing immutable event.

Because Holochain does not expose a global multi-agent transaction, implementations must also define crash recovery.

Safe transfer protocol:

    create transfer intent
        ↓
    authorize
        ↓
    commit sender debit
        ↓
    commit receiver credit
        ↓
    mark final

If execution stops between legs, the transfer remains non-final and is reconciled rather than silently treated as complete.

## 9. Mint conservation

For issuance:

    Δaggregate_supply = +A

The corresponding final issuance event must be unique.

A bridge or productive-capacity claim may supply the basis, but it does not itself become money.

Final relation:

    basis
      → qualified claim
      → mint authorization
      → canonical issuance
      → SAP supply

This prevents oracle quote → mint and sensor reading → mint from becoming implicit monetary rules.

## 10. Exact-once consumption

The strongest practical interpretation of one-shot is:

> A mint basis may cause at most one final issuance in the canonical supply sequence.

This is stronger and more precise than requiring a DHT collection to contain one link.

Required duplicate classes:
- same basis + same amount, repeated;
- same basis + different amount;
- same basis + different recipient;
- same intent + multiple issuance IDs;
- same quorum certificate + multiple canonical sequence positions;
- conflicting finalized records for one basis.

Unknown or conflicting consumption state must be non-final and not reserve-eligible rather than optimistically aggregated.

## 11. Genesis / productive-capacity issuance

The current thermodynamic-genesis path should evolve from:

    ThermodynamicGenesis
        →
    payments.credit_sap

to:

    ThermodynamicGenesis
        ↓
    qualified evidence object
        ↓
    mint intent
        ↓
    authority / policy verification
        ↓
    canonical SapMintRecord
        ↓
    payments balance mutation

The evidence object should include or reference measurement/proof digest, sensor/source identity, measurement window, quantity, beneficiary/owner relationship, replay identity, and verification state.

A non-empty proof byte vector is not sufficient evidence of cryptographic validity.

## 12. Collateral issuance

Collateral issuance should follow:

    collateral deposit
        ↓
    authenticated collateral identity
        ↓
    custody/control state
        ↓
    qualified valuation snapshot
        ↓
    haircut policy
        ↓
    issuance authorization
        ↓
    canonical SapMintRecord
        ↓
    SAP credit

The deposit record must not itself be counted as reserve value and SAP issuance simultaneously.

This is the connection between AC-092 and AC-094: AC-092 protects supply attribution; AC-094 protects asset identity and anti-double-counting.

## 13. Oracle boundary

Reserve valuation should follow:

    Observation
        ↓
    source authentication
        ↓
    qualification
        ↓
    canonical valuation snapshot
        ↓
    reserve eligibility

The valuation snapshot, not a mutable live oracle number, is the dependency for a finalized reserve decision.

Floating-point market observations may remain analytical inputs, but final monetary calculations must use exact fixed-point/integer representations with explicit rounding rules.

## 14. Supply states for reserve reporting

Reserve reporting should publish at least:

    Q_final
      = finalized spendable SAP

    Q_pending
      = authorized but not finalized SAP

    Q_conflicted
      = disputed or duplicate monetary state

For strict reserve coverage, the denominator is Q_final only when Q_pending is not spendable.

Otherwise the conservative denominator is Q_final + Q_pending + Q_conflicted, with the last two categories subject to explicit liability treatment.

The reserve standard should never hide unresolved monetary state.

## 15. Canonical supply root

A reserve-grade report should bind to a canonical SupplyRoot containing domain ID, policy version, sequence, finalized supply, finalized issuance root, finalized transfer root, pending supply, conflict count, and generation time.

The root should be immutable once published. New state creates a new root.

This produces a clean evidence chain:

    reserve report
      ↓
    SupplyRoot
      ↓
    canonical issuance/transfer set
      ↓
    individual SAP deltas

A verifier can therefore reproduce the reported supply rather than trusting a dashboard total.

## 16. Failure and recovery

No monetary operation should have an ambiguous successful outcome.

Required states:

    pending
    committed
    final
    failed
    compensated
    conflicted

A crash between dependent writes must leave an inspectable state that can be reconciled deterministically.

Recovery must be idempotent:

    recover(recover(x)) = recover(x)

Recovery must never silently manufacture a new issuance or erase an unresolved liability.

## 17. Reserve-eligibility consequences

Eligible:
- canonical finalized supply;
- authenticated and qualified reserve instruments;
- unique, unencumbered collateral;
- valuation snapshots inside freshness windows;
- documented liquidity facilities.

Zero strict value / non-final:
- unresolved asset identity;
- conflicting ownership;
- duplicated collateral claims;
- stale valuation;
- unfinalized monetary issuance;
- unresolved transfer outcome;
- unknown supply reconciliation.

This lets the reserve system remain conservative without pretending that every observed record is economically equivalent.

## 18. Implementation order

Recommended sequence:

1. Remove the public raw credit_sap capability.
2. Add an immutable typed SapTransferRecord.
3. Require SapBalance.justified_by for every positive delta.
4. Route send_payment and bridge transfers through the typed transfer path.
5. Make governance issuance create and consume a typed SapMintRecord.
6. Migrate collateral and fiat bridge issuance to typed mint sources.
7. Convert thermodynamic genesis into a qualified productive-capacity mint basis.
8. Add canonical issuance serialization / sequence handling.
9. Derive annual supply counters from canonical issuance state.
10. Add SupplyRoot snapshots for reserve reporting.
11. Integrate AC-093 valuation snapshots.
12. Integrate AC-094 canonical collateral identity.

Steps 1–6 close the immediate monetary-creation surface.

Steps 7–12 turn that closed supply ledger into a reserve-grade evidence substrate.

## 19. Adversarial qualification set

The minimum qualification corpus should include unrestricted caller invokes old credit capability; positive balance created with no justification; wrong entry type; wrong recipient; amount mismatch; supply-increasing transfer; duplicate transfer replay; duplicate mint basis under concurrent issuers; conflicting basis amounts; certificate reused at two sequence numbers; annual cap race; crash after sender debit; crash after receiver credit; replayed bridge message; replayed productive-capacity evidence; changed oracle snapshot; duplicated collateral identity; encumbered collateral; stale supply root; conflicting supply roots.

A green result requires exact-head evidence that the declared authority, serialization, and recovery model were actually executed.

## 20. Qualification ceiling

A successful AC-092 / AC-095 implementation can establish deterministic balance attribution, conservation semantics, authority binding, explicit serialization semantics, conflict handling, replay resistance within the declared authority model, and reproducible supply snapshots.

It cannot independently prove that external assets exist, a sensor measurement is truthful, a legal claim is enforceable, a market valuation is economically correct, a governance decision is socially legitimate, or SAP will be accepted at par outside the system.

Those are separate evidence domains.

## 21. Monetary architecture conclusion

The resulting architecture is intentionally different from both a gold standard and an algorithmic stablecoin.

SAP is the settlement/accounting unit.

Reserve instruments are separately typed.

Collateral is a backing relationship, not money itself.

Oracle observations are evidence, not issuance authority.

Liquidity facilities are contingent capacity, not reserve assets.

Governance is an authorization domain, not an automatic supply entitlement.

The monetary backbone is therefore:

    singleness
        +
    integrity
        +
    explicit elasticity/liquidity
        +
    auditable supply finality

The system should only claim a reserve-grade property at the layer for which it can produce reproducible evidence.

---

### References

- Holochain validation: https://developer.holochain.org/concepts/7_validation/
- Holochain validation build guidance: https://developer.holochain.org/build/validation/
- Holochain source-chain concurrency: https://developer.holochain.org/concepts/3_source_chain/
- Holochain DHT operations: https://developer.holochain.org/build/dht-operations/
- BIS, The next-generation monetary and financial system: https://www.bis.org/publications/aer-2025/next-generation-monetary-financial-system
- IMF, Special Drawing Rights: https://www.imf.org/en/about/factsheets/sheets/2023/special-drawing-rights-sdr
---

## 22. Current implementation findings (2026-10-05)

The repository inspection found three concrete gaps that this profile now treats as prerequisites rather than theoretical risks.

### AC-092 / #4110 — raw balance credit

payments::credit_sap is still a public extern that can increase an arbitrary DID balance. SapBalance.justified_by exists but is not enforced, and governance/bridge/genesis paths currently reach the balance mutation through that raw capability.

This means the implementation does not yet make the issuance record and supply delta inseparable.

### AC-096 / #4116 — collateral issuance before confirmation

bridge::deposit_collateral creates a Pending CollateralBridgeDeposit and then immediately credits SAP. Confirmation happens later.

That permits spendable SAP to exist before the collateral position reaches Confirmed status.

The target ordering is:

    collateral evidence
        ↓
    confirmed deposit
        ↓
    typed issuance
        ↓
    SAP becomes spendable

### AC-097 / #4117 — redemption before SAP consumption

bridge::redeem_collateral currently updates the collateral deposit to Redeemed before the corresponding payments::debit_sap call.

That means a debit failure can leave the persisted collateral state looking released while the associated SAP remains outstanding.

The target ordering is:

    redemption request
        ↓
    SAP consumption
        ↓
    collateral release
        ↓
    final Redeemed state

Because cross-zome effects are not globally atomic, both issuance and redemption need explicit intermediate/recovery states.

These findings reinforce the core principle of this profile:

    economic state transition
        =
    evidence + authority + conservation + serialization + recovery

not merely the presence of a signed ledger entry.

### Reserve qualification consequence

Until AC-092, AC-095, AC-096, and AC-097 are closed with exact-head execution evidence, SAP supply should be treated as prototype monetary state rather than reserve-qualified supply.

Reserve dashboards should expose unresolved issuance/redeem states instead of collapsing them into one balance total.
