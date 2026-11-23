# SAP Effect-Caused Balance Transition v0.1

**Status:** design contract and migration inventory only  
**Audit snapshot:** `e949664ceb2f9ddde26704c70e23a19380f33dde`  
**Related:** [writer inventory PR #6561](https://github.com/Luminous-Dynamics/mycelix/pull/6561), [SAP invariant parent #654](https://github.com/Luminous-Dynamics/mycelix/issues/654), [FIN-SAFE-012 PR #657](https://github.com/Luminous-Dynamics/mycelix/pull/657), [Hearth containment PR #6566](https://github.com/Luminous-Dynamics/mycelix/pull/6566)

## 1. The change in primitive

The current SAP balance shape is a number plus an optional `justified_by: ActionHash`. That is insufficient to establish why a monetary delta happened, which exact predecessor it advances, who owns the effect, whether the cause has already been consumed, or whether the evidence is complete.

The proposed primitive is an **effect-caused balance transition**:

```text
stable operation identity
    + canonical semantic payload fingerprint
    + explicit settlement profile
    + authenticated owner / signer
    + exact predecessor
    + one typed economic cause
    + checked integer delta
    + policy digest
    + independently verifiable evidence
    -> one immutable balance successor
```

The balance successor is not a place to store a mutable number and an optional pointer. It is a compact claim about one economic event. Its validity depends on loading and verifying the actual predecessor and referenced cause records at the integrity boundary.

The JSON Schema in `benchmarks/sap/sap-effect-cause-transition.v0.1.schema.json` describes the serializable contract. The 16-writer mapping in `sap-effect-cause-migration-map.v0.1.json` links existing entrypoints to target cause kinds and migration requirements. Both are design artifacts; neither is production enforcement.

## 2. Settlement profile is explicit and immutable

Two distinct authority models must not be silently blended:

| Profile | Authority model | Required proof |
|---|---|---|
| `FinanceLedgerAtomicV1` | Finance is a specifically delegated ledger authority for the defined effect. | One Finance-authored ledger transition commits the explicitly authorized sender/recipient effects inside its local transaction boundary. |
| `BilateralOwnerClaimV1` | Each balance owner controls their own balance successor. | Payer authors the intent/debit; recipient independently authors a claim/credit bound to the same operation and sender evidence. |

The profile ID and version are included in the operation fingerprint and durable operation record. A retry cannot change profile. If a profile is unavailable, the system rejects or returns Indeterminate; it does not fall back to another authority model.

A profile does not by itself prove a transaction has settled. Local commit, remote effect, beneficiary observation and reconciliation remain distinct states.

## 3. Cause algebra

Version 0.1 defines the following economic causes:

| Cause kind | Meaning | Minimum binding evidence |
|---|---|---|
| `zero_genesis` | Owner's exact zero-balance root. | No predecessor, zero before/after/amount, owner-authored root and unique genesis policy. |
| `external_issuance_claim` | SAP issuance backed by a non-governance external source. | Authenticated issuer/custodian evidence, exact asset/amount/owner/value date, one-use receipt. |
| `governance_issuance_claim` | Issuance under an approved governance action. | Exact SapMintRecord / governance authorization, recipient, amount, policy and one-use consumption. |
| `bilateral_transfer_sender_debit` | Payer's debit under bilateral settlement. | Owner-authored intent/debit, beneficiary, amount, asset and operation ID. |
| `bilateral_transfer_recipient_claim` | Beneficiary's independent credit claim. | Beneficiary-authored claim referencing exact payer intent/debit, beneficiary, amount and same fingerprint. |
| `finance_ledger_transfer_effect` | Delegated Finance-ledger transfer. | Exact Finance authority grant, sender/recipient deltas, policy and atomic local effect. |
| `directed_debit` | Owner-authorized debit for a named obligation or redemption. | Exact operation/obligation, amount, owner, asset and authorization. |
| `demurrage` | Formula-derived decrease over an explicit interval. | Formula version, exact interval, exemption state, predecessor, computed deduction and associated compost liability. |
| `staking_lock` | Member SAP moves into a specific stake position. | Stake/escrow position, owner, amount and one-use lock identity. |
| `staking_unlock` | SAP is released from a specific stake position. | Exact prior lock, release authorization, owner claim and one-use unlock identity. |
| `hearth_contribution` | Member contributes to an exact Hearth pool. | Owner debit, pool credit, explicit membership evidence and operation identity. |
| `hearth_withdrawal` | Hearth pool pays a member. | Pool debit, beneficiary claim/credit, pool identity, membership evidence and operation identity. |
| `metadata_only` | Non-monetary balance metadata changes. | Separate metadata authorization; balance is identical and prior monetary cause lineage is preserved. |

An unrecognized cause kind is invalid. “Reason” text is descriptive metadata, not evidence. A cause action hash is not sufficient by itself: the verifier must load the referenced record and compare its semantic fields with the transition.

### One successor, one monetary cause

A successor may represent exactly one economic cause. Do not combine a demurrage deduction and a transfer credit into one balance delta with one cause pointer. When both are due in a transaction, stage two predecessor-linked transitions, each with its own cause. If they are in the same owner cell and one zome invocation, Holochain's local call transaction can commit or roll back those writes together; that local property does not create a transaction across independent cells or owners.

Metadata-only updates must preserve the existing monetary balance and historical cause. Revoking an exemption must not reset the prior monetary cause to `None`.

## 4. Transition invariants

Let (B_p) be the exact predecessor balance, (B_s) the successor balance, (A) the non-negative integer amount, and (D) the declared direction.

- Credit: (B_s = B_p + A), checked for overflow.
- Debit: (B_s = B_p - A), checked for underflow.
- None / metadata-only: (B_s = B_p) and (A = 0).
- Zero genesis: (B_p = B_s = A = 0), with no predecessor.
- All other transitions require one exact predecessor ActionHash.
- The actual Holochain `Action.author` is the signer input. A caller-supplied DID or public-key string is never accepted as proof of authorship.
- The owner DID must be bound to the actual signer through the selected identity/authority policy.
- Cause owner, beneficiary, counterparty, amount, asset, operation ID and profile must match the exact balance transition.
- A cause may justify one effect only. Reuse is rejected unless the protocol explicitly defines a shared cause with an independently proven allocation; version 0.1 does not.
- An effect ID reused with the same canonical fingerprint resolves to the original result. Reuse with a different fingerprint is a conflict.
- Replay checks are over complete, authoritative ancestry/effect history. A caller-supplied partial ancestor list is not sufficient.
- Competing successors from the same predecessor are a conflict. Do not choose a winner by DHT link order, timestamp, first/last result or arrival order.
- If predecessor ancestry, DHT completeness, revocation freshness or cause evidence is unavailable, return `Indeterminate`. It is not permission, absence, settlement or a reason to retry under a new operation ID.

The JSON Schema validates shape and selected profile/cause combinations. Cross-field arithmetic, author derivation, predecessor existence, signature/evidence validity, replay uniqueness and DHT completeness require deterministic verifier and HDI logic; those cannot be established by JSON Schema alone.

## 5. Incoming-transfer compatibility and the migration trap

The current public `transfer_sap` implementation calls `debit_sap` and then `credit_sap` from the sender's invocation. Therefore the credit successor on the recipient's balance can be signed by the sender's agent key. That record does not prove the recipient consented to receive or control the credit.

A simplistic rule “the latest balance record must be authored by the beneficiary” can consequently reject valid legacy incoming transfers. That is an intentional fail-closed response, but it is not a migration strategy and can block withdrawal availability.

The target bilateral flow is:

1. Sender authors an immutable transfer intent with stable effect ID, canonical payload fingerprint, beneficiary and exact amount.
2. Sender's chain records the debit against its exact predecessor.
3. Beneficiary receives the intent and independently authors a one-use recipient claim on their own chain.
4. Beneficiary's balance transition references both the sender evidence and the beneficiary claim.
5. Both sides reconcile to the same effect ID, fingerprint and profile.
6. Missing acknowledgement leaves the state pending/indeterminate; sender-side data is not promoted to beneficiary consent.

For legacy sender-authored credits, define a migration/claim entry that references the exact historical credit and requires recipient authentication before establishing an owner-accepted successor. Do not silently rewrite historical authorship, accept sender signatures as beneficiary claims, or strand valid credits without a recovery path.

The separate `FinanceLedgerAtomicV1` profile can support a delegated ledger model, but only with explicit delegation and a defined authority boundary. It must not be used as an implicit fallback when bilateral membership/claim evidence is unavailable.

## 6. Migration map across the 16 writers

The companion JSON maps every inventoried class. At a glance:

| Writer classes | Target cause(s) |
|---|---|
| Zero genesis | `zero_genesis` |
| Amber grant / revoke | `metadata_only` |
| Demurrage | `demurrage` |
| Raw credit | `external_issuance_claim` or `bilateral_transfer_recipient_claim`; then remove raw public API |
| Owner debit | `directed_debit` |
| Transfer / send payment | Bilateral sender debit + recipient claim, or explicitly delegated Finance-ledger effect |
| Governance mint | `governance_issuance_claim` |
| Hearth contribution / withdrawal | `hearth_contribution` / `hearth_withdrawal` |
| Collateral / fiat / physical issuance | `external_issuance_claim` with independent issuer/custodian evidence |
| Staking lock / unlock | `staking_lock` / `staking_unlock` |
| Membership gate | Authorization evidence, not a monetary cause |

Do not migrate one writer and enable global integrity rejection while other legitimate writers still create untyped transitions. This is a dependency-coherent cutover.

## 7. Implementation sequence

**Phase A — pure deterministic verifier.** Build a no-HDK Rust kernel with explicit `Valid`, `Invalid`, `Conflict` and `Indeterminate` outcomes. Add checked arithmetic, canonical encoding, cause/owner/profile rules and complete operation fingerprinting. Build known-answer tests and a separate Go verifier over the same corpus.

**Phase B — HDI enforcement.** Derive signer from actual `Action.author`; resolve DID authority; load the exact predecessor and typed cause; verify semantic agreement, replay consumption and fork state. Coordinator preflight alone is not the enforcement boundary.

**Phase C — migrate all 16 writer classes.** Add beneficiary claim and legacy-credit recovery before tightening recipient authorship. Remove raw public `credit_sap` only after all valid producers have a typed path. Use staged feature/versioned migration with a closed list of accepted transition versions; never make an undocumented bypass.

**Phase D — remote effects and recovery.** Commit local operation + outbox in one local transaction. Deliver at least once. Remote owner consumes the stable operation ID and returns an independently authored receipt. Sender reconciles through the original effect ID/fingerprint after timeout or process restart; no new identity is minted to “retry”.

**Phase E — qualification and retirement.** Run model checking, adversarial corpus, real multi-agent Sweettests, stale/incomplete DHT injection, chain-head races, response loss, restart, and mutant controls. Retire legacy raw writers after positive and negative controls prove no untyped path remains.

## 8. Industry comparison: improve the semantics, not the slogans

| Industry pattern | What it solves | What this design adds |
|---|---|---|
| Payment API idempotency keys | Deduplicate retries over a published retention window and, in some systems, scope keys to the account or region. | Durable owner-authored economic identity, payload conflict, explicit recipient claim and reconciliation beyond request-cache expiry. |
| Transactional outbox | Atomically records local state + outgoing message; delivery is commonly at least once and consumers must be idempotent. | Bind the outbox to the exact source-chain cause and require an independent remote-owner receipt. |
| Database/ledger transaction | Strong local atomicity inside a defined database transaction. | Explicitly distinguish that local atomicity from cross-cell or bilateral ownership; never call a multi-owner effect settled because one local call committed. |
| Holochain source-chain transaction | All local writes in one zome call are transactional; concurrency can cause chain-head-moved rollback. | Preserve that native local guarantee while representing cross-owner and cross-cell effects as protocol state, not an assumed distributed transaction. |

The target is not “more decentralized” by assertion. It is explicit ownership of each effect, reproducible evidence, durable retries and safe unknown outcomes.

## 9. Qualification targets — not achieved results

Safety gates precede throughput:

| Metric | Target |
|---|---:|
| Balance successors with missing/wrong typed cause | 0 |
| Accepted signer/owner or beneficiary mismatch | 0 |
| Replayed cause producing second balance effect | 0 |
| Same effect ID + changed fingerprint accepted | 0 |
| Demurrage hidden in principal movement transition | 0 |
| Metadata update changes balance or erases cause | 0 |
| Raw untyped positive-credit path reachable after cutover | 0 |
| Incomplete DHT/fork view promoted to success/absence | 0 |
| 100 concurrent same-ID/same-payload attempts | At most one owner effect |
| 1,000 same-ID/changed-payload attempts | 100% hard conflicts |
| 10,000 duplicate outbox deliveries | At most one remote effect per ID |
| 100,000 generated schedules with crashes, forks, replay, stale views and timeouts | Zero unauthorized effects, duplicate settlements, negative balances or false finality |

Only after safety gates pass, report p50/p95/p99 latency, sustainable throughput, CPU, storage/DHT growth per transition, verifier read amplification, outbox age, reconciliation lag, recovery time and cost per 1,000 correctly authorized effects. Match effect semantics, key retention, retry load, topology and failure injection against a ledger transaction baseline and an outbox-based service baseline. A candidate research target is ≥20% lower p95 cost or ≥20% higher sustainable throughput at equal p99/resource budgets. This is not a measured result.

## 10. Claim ceiling

This PR's artifacts are a serializable design contract, a source-level migration map and structural CI. They do not implement SAP cause enforcement, validate external custodians, prove global DHT completeness, establish money-supply conservation, qualify a recipient-claim migration, or demonstrate runtime behavior. The separate Hearth PR #6566 is a containment patch; its tests and scope must not be described as the full SAP transition theorem.
