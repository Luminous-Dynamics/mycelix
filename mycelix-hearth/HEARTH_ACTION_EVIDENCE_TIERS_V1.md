# Hearth Action Evidence Tiers V1

**Status:** evidence contract and resident-facing copy update. No validation-receipt read API or DHT propagation UI is wired by this change.

## Core paradigm: status must say what was observed

Hearth previously treated a successful decodable zome response as a generic "confirmed" result. That language obscures Holochain's stages. According to the official build guide, a zome workflow writes actions to the author's source chain and local DHT store, starts DHT publication and post-commit workflows in separate threads, and then returns the zome function's result. Peer validation receipts are separate observations received as validators receive, validate, store, and begin serving the author's DHT operations.

A successful local response therefore supports **SourceChainCommitted**. It does not by itself prove that a peer has received or validated the corresponding DHT operation, and no finite number of peer receipts proves that every peer has the same current view.

References:
- [Holochain zome functions](https://developer.holochain.org/build/zome-functions/) — source-chain/local DHT writes, asynchronous publish and post-commit workflows, result return.
- [Holochain validation receipts](https://developer.holochain.org/build/validation-receipts/) — validators send receipts after receiving, validating, storing, and serving DHT operations.
- [Holochain signals](https://developer.holochain.org/build/signals/) — signals are send-and-forget and do not wait for receiver confirmation or persist until an unavailable receiver comes back.
- [Holochain validation](https://developer.holochain.org/build/validation/) — validation is distinct and can have unresolved dependencies.

## Status and evidence dimensions

Do not collapse operation lifecycle, replication evidence, and observed state into one boolean.

| Evidence/status | What is observed | What it does not prove |
|---|---|---|
| Preview only | Local sample/demo state changed | No sync or live write |
| Not sent | A local preflight guard blocked dispatch | No remote request was sent |
| Outcome unknown | The operation may or may not have committed | Not a rejection; not permission to retry |
| Source-chain committed | Successful decodable zome response returned a record | No proof of peer validation/integration or global synchronization |
| Peer validation receipt observed | A receipt says a particular validating peer received, validated, stored, and started serving an operation | No proof every peer has it or that the entire DHT is globally consistent |
| Current state observed | A correctly scoped read returned the expected current state | Only the queried view at that point in time; other peers or views may differ |
| Explicitly rejected | A typed domain contract proves the operation was rejected without applying the effect | Must not be inferred from generic timeout/transport/decode errors |

The first four statuses can be used now in feedback. The current client action path does **not** read validation receipts in this tranche, so it must not emit Peer validation receipt observed from a successful action return. A future propagation status requires a real API/evidence source.

## Resident-facing copy rules

- **Preview only:** "Preview only: the action changed local demo state; nothing was synced."
- **Not sent:** identify the missing requirement and say no request was sent.
- **Outcome unknown:** ask the resident to check current state before retrying.
- **Source-chain committed:** "Saved to your Holochain source chain. Sharing with other peers may still be in progress." Keep this compact; expose the technical explanation in detail UI/help.
- **Peer validation receipts observed:** show the actual observed receipt count only if an authoritative API returns it. Explain that this is evidence from those peers, not universal synchronization.
- **Current state observed:** name the read/source in details and say this confirms the current query view only.
- **No observed propagation evidence:** say pending/unknown, not failed or synchronized.

A local signal is a notification, not a receipt. A single validation receipt is evidence about that validator, not a global synchronization theorem. A local current-state query should not be labeled "all members synced". A remote timeout must remain OutcomeUnknown and must not trigger automatic resend.

## Implementation in the current tranche

- Replaces the generic `Confirmed` feedback kind with `SourceChainCommitted` throughout the pure lifecycle and journal protocol models.
- Changes the successful Hearth zome-call feedback to say that a source-chain record was saved while peer sharing may still be in progress.
- Adds feedback variants for receipt counts and scoped reads, with explicit limitations in their copy. These variants are a contract only; production code must not invoke them until it has the stated evidence.
- Adds negative-control tests for a single validation receipt, zero receipts, and a scoped read; none may be described as global synchronization.
- Preserves PreviewOnly, NotSent, and OutcomeUnknown semantics and keeps the journal dispatch gate fail-closed.
- Does not add a validation-receipt query, network convergence monitor, or automatic retry.

## Benchmarks (targets, not results)

### Truthfulness

1. 100% of action feedback messages map to an explicit evidence tier.
2. Zero messages equate a successful zome response with peer validation, DHT integration, or global synchronization.
3. Zero labels of globally synchronized based on local signals, one peer receipt, or a plain local read.
4. 100% of ambiguous errors remain OutcomeUnknown and recommend reconciliation before retry.
5. 100% of future propagation labels cite the actual observation source, freshness, and scope.

### Network evidence, after a real observer API exists

1. The UI displays no peer-receipt number unless it came from an actual receipt read for the relevant action/operation.
2. Stale, unavailable, incomplete, or unsupported receipt data is rendered as unknown/pending—not zero peers, failed, or synchronized.
3. Measure time to first validation receipt and receipt-count distribution across multiple local multi-agent scenarios; report percentile and sample size. No fixed receipt count is described as universal consistency.
4. Use partition, delayed publication, invalid action, missing dependency, conductor restart, and peer churn scenarios.
5. Every positive network-evidence result is paired with negative controls: zome return without a receipt; local signal without remote receipt; receipt from one peer while another is offline; stale read; rejected DHT op.

### Human comprehension and accessibility

- At least 90% of study participants distinguish "saved to source chain" from "available to peers".
- At least 90% understand that a validation receipt from one peer does not establish that all peers have the change.
- 100% of uncertainty messages provide a safe next step without implying failure or success.
- Screen readers announce status changes without moving focus unnecessarily, consistent with [WCAG 2.2 SC 4.1.3](https://www.w3.org/WAI/WCAG22/Understanding/status-messages.html).

All numeric gates are targets; this change does not claim any have passed. The pure copy tests do not establish live network behavior or human comprehension.

## Next engineering tranche

1. Keep the immediate zome return labelled SourceChainCommitted.
2. Audit the current Holochain version and zomes for a supported validation-receipt query / observation surface before writing adapter code.
3. If receipt monitoring is supported, add an explicit operation/action-hash-to-DHT-op mapping with freshness and partial-result semantics; do not assume a source-chain ActionHash alone implies all expected DHT ops are integrated.
4. Model lifecycle outcome and propagation evidence independently in a journal/event record.
5. Qualify with a deterministic multi-agent Tryorama corpus, including no-peer, one-peer, lagging-peer, invalid-op, restart, and partition cases.
6. Keep the status UI useful when propagation cannot be measured: honest "saved locally; sharing may still be in progress" is better than a false green "synced".

## Nonclaims

This is not a distributed consensus protocol, a network-wide synchronization proof, a complete DHT health indicator, a delivery SLA, or a claim that other devices have observed the change. Only a future source-qualified runtime integration can report measured peer receipts, and even that should state its scope rather than promise global convergence.
