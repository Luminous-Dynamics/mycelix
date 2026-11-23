# SAP Balance-Cause Writer Inventory v0.1

**Audit date:** 23 November 2026  
**Status:** source-level inventory and migration contract; no production code changed  
**Base:** public \`main\` commit \`e949664ceb2f9ddde26704c70e23a19380f33dde\`

## Executive finding

\`SapBalance.justified_by: Option<ActionHash>\` exists, but the public-main integrity callback does not validate it. The type comment explicitly says the field is not yet enforced and that producers currently write \`None\`. \`validate_sap_balance\` currently validates DID shape and Amber metadata, then accepts the entry; both create and update callbacks call this same shape validator without proving a predecessor-to-successor economic transition.

This is not merely a missing property on one mint function. The Payments coordinator contains several distinct writers with different economic meanings:

- zero-balance initialization;
- issuance / raw credit;
- owner debit;
- transfer / payment;
- demurrage;
- Hearth contribution and withdrawal;
- Amber exemption metadata.

They cannot safely share one generic \`credit_sap\` / \`debit_sap\` cause. In particular, a balance update that applies demurrage and an incoming credit in one successor combines two economic causes while the schema offers only one \`justified_by\` reference.

**No production code changed and no Rust / Holochain test result is claimed.** This document inventories source paths and defines the dependency-coherent migration order required before making integrity enforcement mandatory.

## Audited source snapshot

| File | Blob SHA |
|---|---|
| \`mycelix-finance/zomes/payments/coordinator/src/lib.rs\` | \`7df81b5374526e4c145a4b7506e7295aa20d079c\` |
| \`mycelix-finance/zomes/payments/integrity/src/lib.rs\` | \`b551f1ea439f6899d67c0ba55519b12cbee1fe7e\` |
| \`mycelix-finance/types/src/lib.rs\` | \`354a377c88a57e8a5612a518d8c8b8b519fb67e2\` |
| \`mycelix-finance/zomes/bridge/coordinator/src/lib.rs\` | \`78c54d124fb0b4089a2155e3265ea015891bfbe8\` |
| \`mycelix-finance/zomes/staking/coordinator/src/lib.rs\` | \`4192013b800b7a1403c5193421b31565a98af86f\` |
| \`mycelix-finance/zomes/currency-mint/coordinator/src/lib.rs\` | \`98ba66730b645c9f8efce3d98442bab5c25a92ef\` |

The canonical Finance files above were read from one exact public-main commit. Workspace mirrors and mock/UI projections are not counted as independent production writers.

## Current integrity gap

\`SapBalance\` at Payments integrity lines 116–144 stores:
- \`member_did\`;
- raw integer \`balance\`;
- \`last_demurrage_at\`;
- \`exemption\`;
- \`justified_by: Option<ActionHash>\`.

The field comment at lines 127–142 explicitly states it is "NOT yet enforced" and that every producer currently writes \`None\`.

\`validate_sap_balance\` at lines 704–747 checks DID length / prefix and the structural Amber issuer, self-issue, cap and expiry constraints. It does not load the original balance predecessor, compute the delta, resolve \`justified_by\`, check cause ownership, verify exact amount / currency / purpose / recipient, or prevent replay. The \`CreateEntry\` and \`UpdateEntry\` callbacks at lines 304 and 325 both call this same helper.

A caller can therefore submit a syntactically valid balance successor with an economically unsupported value increase. A mint record existing elsewhere in the DHT does not bind that balance delta unless the integrity boundary resolves the exact record and verifies the transition.

## Writer inventory

### A. Zero-balance initialization — legitimate no-cause state

**Writer:** \`initialize_sap_balance\`, Payments coordinator lines 23–64.  
**Current behavior:** owner-caller check; creates balance zero and \`justified_by: None\`.  
**Required rule:** permit only an owner-authored, exact-zero genesis balance with no prior predecessor. Any nonzero first balance must use a typed issuance or transfer claim; raw credit must not auto-create a positive balance.

### B. Amber exemption grant / revoke — metadata-only successor

**Writers:** \`grant_amber_exemption\` lines 97–159; \`revoke_amber_exemption\` lines 161–178.  
**Current behavior:** update a SapBalance entry while leaving the monetary balance unchanged. Grant uses struct update syntax and retains existing fields. Revoke explicitly writes \`justified_by: None\`, clearing the prior cause pointer even though no monetary transition occurred.  
**Required rule:** metadata-only updates must keep the balance exactly unchanged and preserve the historic cause pointer / predecessor lineage. They need a separate metadata authorization and must not overwrite or erase monetary evidence.

### C. Demurrage — owner balance decrease with a distinct cause

**Writer:** \`apply_demurrage\`, Payments coordinator lines 180–260.  
**Current behavior:** computes a deduction and persists a lower balance and new timestamp; the update keeps the previous \`justified_by\` through \`..bal\`. Compost distribution happens after the balance change using Treasury calls / a queue.  
**Required rule:** demurrage is its own typed transition with exact formula version, elapsed interval, exemption state, calculated deduction and owner predecessor. The deduction and the compost liability/outbox need explicit linked evidence. It must not be hidden inside an unrelated credit/debit successor.

### D. Raw \`credit_sap\` — public nonzero balance increase

**Writer:** \`credit_sap\`, Payments coordinator lines 432–556.  
**Current behavior:** public \`#[hdk_extern]\`; accepts \`member_did\`, \`amount\`, \`reason\`; does not bind the caller to the target member or require an authorization / cause reference. If the balance does not exist, it creates a nonzero SapBalance with \`justified_by: None\`. Otherwise it updates the balance using \`post_demurrage + amount\` and preserves the previous cause pointer. Its arithmetic is unchecked. The code comment explicitly calls this a known mint-any-DID hole.  
**Required rule:** remove raw public credit as an economic capability. Split into non-public typed internal operations and purpose-bound claim entrypoints. Issuance must consume an exact independent issuance authorization; transfers must be beneficiary-authored claims; pool withdrawals must cite a pool debit. Use checked integer arithmetic and reject before any mutation.

### E. Owner debit — public owner-authenticated decrease

**Writer:** \`debit_sap\`, Payments coordinator lines 557–659.  
**Current behavior:** requires caller == \`member_did\`, applies pending demurrage, checks sufficient effective balance and updates the balance. The \`reason\` is a free-form string; no typed operation / debit cause ActionHash is required. If demurrage is applied in the same successor, there are two economic causes in one balance mutation.  
**Required rule:** retain owner authorization, but bind the debit to one typed operation: transfer intent, redemption, staking lock, Hearth contribution, fee or other enumerated cause. Stage demurrage as a separate predecessor-bound update if due. Never treat a free-form reason as evidence.

### F. \`transfer_sap\` — two member changes without two owner claims

**Writer:** \`transfer_sap\`, Payments coordinator lines 654–705.  
**Current behavior:** validates from/to and amount, calls debit_sap then credit_sap. Both helpers execute as Rust functions in the same Payments zome invocation; same-source-chain writes are expected to commit / roll back together. However, the beneficiary balance successor is authored by the sender's agent and the credit carries no exact transfer ActionHash / justification. The existing comment about a debit remaining committed if a local credit helper fails should be reviewed against Holochain's same-call atomicity contract.  
**Required rule:** select a declared settlement profile. Under bilateral ownership, sender intent/debit is payer-authored and recipient claim/credit is beneficiary-authored on its own chain. Under a Finance-authoritative local ledger profile, document the delegation and use a distinct profile ID. Never silently switch profiles.

### G. Governance mint — mint receipt exists but is not threaded into balance cause

**Writer:** \`mint_sap_from_governance\`, Payments coordinator lines 710–815.  
**Current behavior:** checks governance authorization and tier / caps; creates immutable SapMintRecord and retains its ActionHash locally; then calls \`credit_sap\` with only member, amount and reason. The created mint ActionHash is not passed into the balance update, and integrity does not verify it.  
**Required rule:** bind exact mint ActionHash, source, recipient, amount, policy version and operation identity into the recipient's owner-authored claim. The same receipt must be consumed once and cannot support a second balance successor.

### H. Direct payment — local sender debit / recipient credit and Treasury fee side effect

**Writer:** \`send_payment\`, Payments coordinator lines 1170–1335.  
**Current behavior:** sender debit and receiver credit are called as local Rust functions within one Payments zome invocation; Payment and Receipt are created later in that local call. A separate Treasury inter-zome call occurs before the Finance invocation finishes; Treasury may commit independently. Treasury errors are logged and the main Payment path continues as \`Completed\`. Balance changes do not carry the Payment ActionHash / operation cause; the Payment ID is timestamp-derived.  
**Required rule:** stable operation identity and typed sender/recipient causes. If using bilateral profile, beneficiary claim is separately authored. Commit a durable fee liability / outbox in the payer transaction and dispatch after commit. Treasury has its own idempotent receipt and reconciliation state.

### I. Hearth contribution — direct personal-balance debit

**Writer:** \`contribute_to_hearth_pool\`, Payments coordinator lines 2020–2095.  
**Current behavior:** verifies Hearth membership (but helper can fail open, see section below); directly subtracts from caller's SapBalance then updates the HearthSapPool. This bypasses debit_sap, does not set a new \`justified_by\` cause, checks raw balance rather than demurrage-adjusted effective balance, and computes pool sums with unchecked addition.  
**Required rule:** treat contribution as an exact owner-authored transfer/claim to the Hearth pool with predecessor-bound cause, checked arithmetic, effective-balance policy, exact counterpart pool credit and one operation identity. A missing / unavailable Hearth membership verifier must not authorize value movement.

### J. Hearth withdrawal — pool debit may return success without member credit

**Writer:** \`withdraw_from_hearth_pool\`, Payments coordinator lines 2110–2162.  
**Current behavior:** debits the pool and increments \`total_withdrawn\`, then credits the caller's SapBalance only inside \`if let Some(find_sap_balance_record(...))\`. If no personal balance exists, no credit is created and the function still returns \`Ok(pool)\`. The member addition and pool counter addition are unchecked.  
**Required rule:** resolve or initialize the recipient account before debiting; or create a durable recipient claim. Absence of a balance must reject / remain Pending, never silently skip the value credit. Use exact, cause-bound, owner-authored successor and checked arithmetic. Tracked separately in [#6560](https://github.com/Luminous-Dynamics/mycelix/issues/6560).

### K. Hearth membership helper — fail-open authorization for value movement

**Helper:** \`verify_hearth_membership\`, Payments coordinator lines 2378–2428.  
**Current behavior:** rejects a decoded false and a decode error, but unexpected ZomeCallResponse and call error (including Hearth unavailable) return \`Ok(())\` under a "standalone mode" comment. This helper gates both contribution and withdrawal.  
**Required rule:** production value-moving membership checks fail closed. A development-only standalone capability must be explicit and cannot be selected by an absent role, network error or unexpected response. Tracked with the Hearth movement work in [#6560](https://github.com/Luminous-Dynamics/mycelix/issues/6560).

### L. External issuance producers — raw-credit calls do not carry independently verified causes

| Producer | Current call | Cause / source issue |
|---|---|---|
| Collateral bridge deposit | Bridge coordinator ~line 663 calls payments credit_sap | #6505: credits while deposit is Pending and no independent custody receipt is bound. |
| Fiat deposit verification | Bridge coordinator ~line 2417 calls credit_sap after updating FiatBridgeDeposit | #6505: confirmation is not an independently authenticated bank/custodian receipt. |
| Thermodynamic genesis | Currency-mint coordinator ~line 179 calls credit_sap | #6505: uses a claimed kWh value converted from f64; actual STARK proof verification is a future integration. |
| Staking lock / return | Staking coordinator ~line 344 calls debit_sap; ~line 379 calls credit_sap | These are lock/unlock transfers, not new issuance; require exact staking escrow cause and recipient-owned balance claim. |

A caller-supplied reason string does not satisfy an issuance proof. Independent evidence adapters, source/issuer authority and one-time consumed-evidence checks remain separate from arithmetic.

## Design correction: one balance successor, one economic cause

A single \`justified_by\` reference cannot justify a successor which silently combines:
- a demurrage deduction;
- a principal debit or credit;
- an Amber / metadata mutation.

The target protocol should make each monetary successor represent exactly one enumerated economic cause and bind:
1. exact owner / author;
2. exact predecessor ActionHash and prior balance;
3. exact successor balance and checked delta;
4. cause kind, cause ActionHash, operation ID and policy/version;
5. exact asset / denomination and recipient / counterparty where relevant;
6. one-time consumption / replay ancestry;
7. the exact source-chain lineage and fork disposition.

When a user transaction also needs demurrage, stage a demurrage successor and a separate movement successor in the same owner zome-call transaction (if the implementation uses one local source-chain profile). Each successor has its own cause and exact predecessor; the whole same-chain call commits / rolls back together. This preserves one-cause-per-transition without hiding demurrage.

For bilateral transfers, the sender debit and recipient credit are separate owner-authored commits; the recipient claim is not authored by the payer.

Metadata-only updates preserve balance, predecessor and the historical monetary cause. They use their own metadata authorization, not \`justified_by: None\` to erase the prior economic reference.

## Existing FIN-SAFE-012 kernel: what it does and does not yet cover

I also read the current public source on [PR #657](https://github.com/Luminous-Dynamics/mycelix/pull/657), specifically:
- `finance-sap-balance-effect/src/author_bound.rs`, blob `2f28f5970e45f80b049dd184ab7d0b8932cb3c56`;
- `finance-sap-balance-effect/src/lib.rs`, blob `7a5362fa6075a4db70285a69f34452de30661503`.

The pure V2 kernel currently does provide:
- exact-zero, owner-authored genesis;
- exact collateral-issuance positive delta with checked addition;
- exact recipient / deposit / amount / `justified_by` binding;
- no hidden demurrage timestamp or Amber exemption change inside an issuance update;
- sequential receipt-replay rejection against the supplied predecessor / ancestor justification references;
- conflict classification when distinct successor ActionHashes are supplied for one exact predecessor;
- an author-bound wrapper that rejects third-party action-author DIDs before observations participate in conflict classification.

It does **not yet** constitute the production `SapBalance` integrity theorem:

1. The pure wrapper receives `action_author_did: String`; HDI must derive this from the actual Holochain `Action.author`. A caller-supplied DID is not authoritative.
2. The replay check receives an `ancestor_justification_references` slice. HDI must load the exact predecessor and prove the ancestry / list completeness. An incomplete slice means incomplete replay protection.
3. The conflict classifier receives an observations slice. HDI must define the authoritative enumeration / completeness boundary; unavailable or delayed DHT observations remain Indeterminate. Link count or first/last link is not a completeness proof.
4. The current API is collateral-issuance-specific plus genesis / fork classification. It does not yet cover owner debit, demurrage, governance issuance, bilateral transfer claim, staking, Hearth movement or metadata-only balance transitions.
5. The live `SapBalance` StoreEntry / StoreRecord callbacks still call shape-only `validate_sap_balance`; the pure kernel is not yet wired into HDI validation.
6. The PR branch is 36 commits ahead / 194 behind current main, and the PR notes the tracked Finance Cargo.lock is stale. Rebase and exact-head locked fmt / test / clippy / HDI / Tryorama qualification are still required.

This is why the writer inventory is not a proposal to simply set `justified_by` on existing credit calls. The integration must preserve the strong collateral theorem while adding typed causes for every other economic path, and it must enforce owner authorship / predecessor binding at the real HDI boundary.

## Migration order (fail-closed, dependency-coherent)

1. Freeze / gate unqualified issuance and raw credit entrypoints; do not leave a mint-any-DID capability available while migration is underway.
2. Adopt the declared settlement profile and typed balance-effect / issuance authorization schema from FIN-SAFE-010/#651 and FIN-SAFE-012/#654/#657. Rebase and re-audit stale branches before integration.
3. Inventory every producer above and introduce typed operations: zero genesis, external issuance claim, governance issuance claim, bilateral recipient transfer claim, owner debit, demurrage, staking lock/unlock, Hearth contribution/withdrawal, and metadata-only update.
4. Create exact cause evidence and enforce owner-authorship, predecessor binding, checked delta, exact cause reference and single-use replay at the Holochain integrity boundary. Do not rely on coordinator checks or DHT link cardinality.
5. Migrate all producers and tests together; a half-migrated writer is a bypass.
6. Add negative controls by temporarily omitting each guard in a test oracle and prove the exact attack is caught.
7. Add real-agent Sweettests / Tryorama for same-owner concurrent updates, source-chain-top races, duplicate cause, alternate key, beneficiary claim, fork conflict, restart and DHT partition.
8. Only then remove / make private raw \`credit_sap\`, close the optional justification path and promote the integrity gate.

## Proposed hard gates

| Metric | Target |
|---|---:|
| Positive balance deltas without exact typed cause | 0 |
| Negative balance deltas without exact typed cause | 0 |
| Causes reused for a second delta | 0 |
| Owner / beneficiary author mismatches accepted | 0 |
| Demurrage hidden in credit/debit successor | 0 |
| Metadata updates that change balance or erase prior cause | 0 |
| Raw positive \`credit_sap\` path reachable from an unauthorized caller | 0 |
| Overflowing amount / pool arithmetic accepted | 0 |
| Missing Hearth membership verifier treated as authorization | 0 |
| Hearth withdrawal success without exact member claim / credit | 0 |
| Exact valid transitions rejected | 0 in the frozen positive-control suite |

Performance is measured only after all safety gates pass: p50/p95/p99 balance update latency, predecessor reads per transition, cause-evidence reads, DHT growth per effect, recovery time after unknown outcomes, and cost per 1,000 correctly authorized effects. Do not compare raw TPS before semantics match.

## Claim limits

This is a source-level writer inventory and proposed migration contract. It does not implement the integrity theorem, prove independent custody, authenticate external evidence, establish global uniqueness, prove numeric monetary supply conservation, or demonstrate live Holochain behavior. The relevant implementation and integration obligations remain open under [#6505](https://github.com/Luminous-Dynamics/mycelix/issues/6505), [#654](https://github.com/Luminous-Dynamics/mycelix/issues/654), [PR #657](https://github.com/Luminous-Dynamics/mycelix/pull/657), and [#6560](https://github.com/Luminous-Dynamics/mycelix/issues/6560).
