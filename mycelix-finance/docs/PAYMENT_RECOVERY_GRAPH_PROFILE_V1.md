# Payment Recovery Graph Profile v1

**Status:** synthetic reference contract and Pix comparator. No live account tracing, PSP communication, holds, notifications or movement of funds.

## Industry baseline: Brazil Pix MED v4.2

The Banco Central do Brasil's *Guia de implementação dos procedimentos de devolução no Pix*, version 4.2, is dated 11 May 2026. It defines a root-transaction recovery graph that identifies subsequent transfers suspected of receiving diverted funds and allows linked notifications to reach the PSPs of downstream recipient accounts.

Pix-specific operating targets in that guide include: immediate recovery-case opening (recommended within 10 minutes; 95% within 30 minutes), an 80-day root transaction window, 7 calendar days for receiving PSP analysis, up to 72 hours for the payer PSP to request returns after assessment, sequential receiving-PSP return steps of up to 6 hours each, and monitoring/partial return up to 90 days. These are **Pix-specific comparator values**, not South African rules or targets.

The Pix guide also distinguishes fraud recovery from ordinary mistaken-payee payments. A wrong destination selected by the payer is not reclassified as an operational flaw merely to enter the fraud MED process. Keep mistaken payee, suspected fraud, unauthorised payments, operational duplicates and merchant disputes separate.

The 3 September 2026 notice for Pix User Experience Manual v7.4 says its changes take effect on **1 March 2027**. As of 6 November 2026, these are future-effective changes, not current rules.

## Paradigm shift: trace topology, not presumed money ownership

A graph edge means an observed transaction relationship or candidate continuity from one account to another. It does **not** prove that the same fungible money moved along that edge. Commingling, pre-existing balances, unrelated credits, incomplete ledgers and multiple fraud roots can make exact provenance unknowable.

Mycelix therefore keeps four different things separate:

1. **Observed graph:** uniquely identified transactions, accounts, PSPs, timestamps and evidence-backed links.
2. **Candidate exposure:** downstream transactions connected to the root under a declared graph snapshot. This is not legal attribution or recovered value.
3. **Institutional action:** notifications, acknowledgements, decisions and account-specific holds/blocks under an authority basis. A notification or hold is not a return.
4. **Actual return:** a unique successor economic event with an independent return receipt and independent ledger reconciliation.

The evaluator reports unique graph-node counts and per-root reconciled returns. It deliberately does not sum all downstream edge values into a "recovered" or "attributable" amount: split paths, convergence and re-spending make such sums double-count-prone.

## Value conservation and commingling

For a complete account-ledger snapshot, require exact integer reconciliation:

`opening balance + root inflows + other inflows = observed outflows + other outflows + closing balance`

An account with partial or unknown ledger coverage stays partial or unknown. Missing opening balance, missing external edges or unknown available balance must never become zero.

Holds are account-scoped observations. A hold amount must be supported by valid authority and a local available-balance observation, and may not exceed the requested amount or observed available balance. Multiple holds across PSPs are **not summed as recovered value**.

Actual returns are summed by root in integer minor units. A unique return reference can only be counted once. Reversals are separate successor events that reduce net return and restore residual loss. For each root:

`net returned = independently reconciled returns - independently reconciled return reversals`

`residual loss = original root amount - net returned`

No notification, candidate path, account hold or block changes these equations.

## Benchmark gates

- 0 candidate edges described as proof of ownership.
- 0 graph nodes or notifications counted as returns.
- 0 holds counted as returns.
- 0 duplicate economic return references.
- 0 per-root net returns above the root amount.
- 0 graph cycles or unresolved parent links silently treated as complete.
- 100% complete account snapshots reconcile exactly with checked integer arithmetic.
- 100% partial/unknown ledger windows remain visibly partial/unknown.
- 100% deterministic graph traversal and unique transaction identity.
- 100% returned-value accounting tied to a root, currency, receipt and independent reconciliation.

Measure graph reachability precision/recall only against synthetic ground truth. In live studies, separately report graph completeness, unresolved external paths, time to open a recovery case, time to first PSP acknowledgement, time to hold, time to return, partial return, residual loss, false-positive holds and appeal/release outcomes. Denominators and scheme versions must be explicit.

## Claim ceiling

This v1 proves only that the reference evaluator can validate a synthetic graph, account conservation, authority-scoped holds and unique return accounting. It does not prove legal ownership of fungible funds, authority to block an account, actual recoverability, a PayShap implementation or any South African service-level target.


## Shared-account reservation conservation (2026-11-06)

A single downstream account can receive candidate flows associated with more than one recovery root. Each notification's requested amount may be individually within the local available balance while the **sum of simultaneous active holds** exceeds that balance.

The v1 corpus includes RG-025: two roots converge on one downstream account with 9,000 minor units available. The valid fixture applies 6,000 and 3,000 active holds, exactly consuming the observed available amount. Its negative control raises the second hold to 4,000: each hold remains individually below 9,000, but the aggregate 10,000 must fail.

The evaluator keys snapshots and reservation totals by account plus currency, requires each hold to bind the notified transaction's currency, uses checked integer addition, and counts only valid-authority `applied`/`partial` holds as active. Released and contested historical records do not consume current reserved balance. This is synthetic value-conservation qualification; actual freezing authority and prioritisation remain governed by the relevant scheme, participant and jurisdiction.
