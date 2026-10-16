# Hearth Action Outcomes v1

**Scope:** resident-facing Leptos action feedback.
**Status:** design/implementation proposal; hosted and browser qualification is still required.

## Core paradigm

The UI must report only what it knows. A request is not a confirmed change; a local demo mutation is not a synchronized household event; and a transport/decode error does not necessarily prove that the remote operation failed.

Use four states:

| State | Meaning | Copy rule |
|---|---|---|
| Preview only | Local sample/demo state changed | Explicitly say local-only and not synced |
| Confirmed | A successful remote response was received | Say the action was confirmed |
| Outcome unknown | The app cannot establish whether a remote action took effect | Tell the resident to check current state before retrying |
| Not sent | Required target or identity was missing before dispatch | Say no request was sent and name the missing requirement |

Do not derive a "rejected" result from an untyped transport error. Until a structured error taxonomy exists, ambiguous errors stay Outcome unknown.

## Implemented in this tranche

- Added a pure Rust copy-policy module with tests for Preview only, Confirmed, Outcome unknown, and Not sent.
- Replaced ordinary success phrasing in mock-only branches for bond tending, gratitude, votes, presence, care completion, and invitation with explicit preview/local-only copy.
- Removed the live care path's unconditional "task completed" feedback; it now checks the call result.
- Fixed a wire-contract mismatch found by inspecting the actual zome signatures: the five mutation zomes return a Record, while the frontend was deserializing their return into unit (()). The Leptos calls now decode a WireRecord.
- Fixed the matching input shape for those zomes: ActionHash/AgentPubKey fields are decoded from strict base64 form into validated 39-byte Holochain hash byte vectors before dispatch. Invalid/missing identities are Not sent, not guessed.
- Added resident-facing unknown-outcome feedback for live gratitude, vote, presence, care, and bond failures.
- If the bond update succeeds but the follow-up read fails, feedback says the write was confirmed while the displayed details may remain stale.
- Live invitation now fails closed: the current function accepts only a display name but would previously submit it in a field requiring an AgentPubKey. It sends no live zome call until the UI/API accepts a verified recipient identity.
- The household data-state notice now also identifies deliberate MockOnly mode as Preview. When fallback mode separately raises a connection alert, the notice communicates the resulting local-data state.

## What this does not solve

- Does not add automatic retries or claim idempotency.
- Does not implement an action journal, remote reconciliation, cancellation, undo, or operation IDs.
- Does not prove source completeness, current active-household membership, signer readiness, or backend authorization.
- Does not change zome behavior, physical-device dispatch, or adapter authority.
- Does not convert errors into definite rejection when the outcome is ambiguous.

A later action-lifecycle tranche should use an operation ID/idempotency key where the source contract supports it, persist submitted/acknowledged/rejected/unknown/reconciled transitions, and allow recovery after app restart. It must not blindly replay non-idempotent operations after a timeout.

## Benchmarks (targets, not results)

- 100% of tested action outcomes map to truthful user-facing feedback.
- Zero false-success messages across the targeted action corpus.
- Zero live invitation calls without a verified recipient identity.
- 100% of mock-mode mutation feedback states that it is local-only and unsynchronized.
- 100% of ambiguous remote errors state that the outcome is unknown and guide users to verify before retry.
- At least 4/5 user-test participants choose "check current state" rather than immediately resubmitting after an outcome-unknown message.
- No success toast before a successful remote call result.

## Verification required before merge

1. Run the Leptos 0.8 wasm check at the exact head.
2. Execute the five pure helper tests and existing Hearth frontend qualification.
3. Test success, explicit preflight block, remote rejection, response loss after possible commit, decode failure, and failed post-write refresh.
4. Inspect preview/demo wording and keyboard/screen-reader announcement behavior.
5. Record exact commit SHA and execution result; queued/skipped/unexecuted is not PASS.

## Research references

- [Apple Human Interface Guidelines: Undo and redo](https://developer.apple.com/design/human-interface-guidelines/undo-and-redo) — people need to predict reversal and see its effect.
- [Apple Human Interface Guidelines: Alerts](https://developer.apple.com/design/human-interface-guidelines/alerts) — non-intrusive labels can communicate common connection problems; alerts should match the importance and reversibility of actions.
- [Home Assistant: Local voice assistant](https://www.home-assistant.io/voice_control/voice_remote_local_assistant/) — demonstrates local processing for privacy-preserving home interaction.
- [AWS Builders' Library: Timeouts, retries, and backoff with jitter](https://builder.aws.com/content/3EumjoZascWd1oZiEgL8ORlv3qE/timeouts-retries-and-backoff-with-jitter) and [Stripe idempotent requests](https://docs.stripe.com/api/idempotent_requests) — explain why side effects can have ambiguous outcomes after a lost response and how server-supported idempotency keys can make retries safer.
- [W3C WCAG 2.2, Success Criterion 4.1.3](https://www.w3.org/WAI/WCAG22/Understanding/status-messages.html) — status messages should be exposed to assistive technology without unnecessarily moving focus.
