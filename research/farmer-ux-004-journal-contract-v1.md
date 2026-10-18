# Farmer UX-004: restart-safe operation journal and offline outbox

**Status:** Contract proposal; no durable journal implementation is claimed.  
**Tracking:** [Issue #5202](https://github.com/Luminous-Dynamics/mycelix/issues/5202) · [Farmer Today prototype PR #5201](https://github.com/Luminous-Dynamics/mycelix/pull/5201)

## Paradigm shift

Offline-first is not a Wi-Fi badge or an in-memory form cache. It is the ability to preserve user intent through crashes and connection loss, accurately expose uncertainty, and avoid repeating operations whose remote effect is unknown.

The first use case is a farmer recording an irrigation problem and optionally requesting help. Keep local transport status separate from the remote business lifecycle:

`local draft → request receipt → quote offered/expired → quote accepted → booking confirmed/cancelled → provider reports completion → farmer accepts OR disputes/requests rework`

A successful RPC may prove that the request was received. It does not automatically prove that a quote exists, a booking was made, work is complete, or the farmer accepted it.

## Reuse before implementing

[Hearth PR #5059](https://github.com/Luminous-Dynamics/mycelix/pull/5059), branch `hth-ux-004-action-lifecycle`, currently has a pure lifecycle model at `mycelix-hearth/apps/leptos/src/action_lifecycle.rs`, source blob `1af673d1a38ea7d69ca51379088f07f088b5d080`. It defines eight statuses, an 8×8 transition oracle, fail-closed dispatch eligibility, and recovery of a persisted Submitted attempt as OutcomeUnknown. The durable browser journal is separately tracked in [HTH-UX-005 / issue #5060](https://github.com/Luminous-Dynamics/mycelix/issues/5060).

Both are open/unmerged. Do not assume the code exists on `main`, and do not fork a second competing generic status enum. Pin and review the exact owner head first. This farmer-specific layer defines adapter mapping, business-state distinctions, scope/consent policy, persistence fault cases and UI language.

## Write-before-send invariant

1. Validate the allowlisted operation kind, payload version, selected farmer/site scope, and permission.
2. Commit a `Prepared` envelope and any required payload reference to durable local storage.
3. Commit `Submitted` durably before sending. If either durable write fails or cannot be confirmed, do not send.
4. Send once and record only a typed, decodable result.
5. On timeout, transport loss, decode failure, browser crash/restart, or result-write failure, recover an unresolved Submitted attempt as `OutcomeUnknown`.
6. Never automatically resend OutcomeUnknown. Reconcile only through an authoritative read or a separately qualified server-side idempotency contract. Without a safe resolver, keep it unknown and offer a human contact path.

This rule is intentionally conservative: a crash after Submitted is persisted but before the network call may recover as unknown even if the call never went out. False uncertainty is preferable to a false success or duplicate economic/physical effect.

Client operation IDs and local digests do not provide server-side idempotency, source authenticity, or exactly-once distributed effects. A backend must define idempotency scope, canonical request binding, duplicate-key behavior, and authoritative reconciliation separately.

## Minimal envelope and privacy

The persistent index holds only schema version, opaque operation ID, allowlisted operation kind, opaque target/scope references, attempt status, safe payload/result references, timestamps and retention metadata. Keep free text, photos, exact coordinates, contact details and financial terms out of routine indexes/logs; if locally stored at all, define access, retention, export and deletion behavior.

Do not retarget a pending operation when the farmer changes selected site or farm. A digest is not encryption. Do not silently fall back to transient memory when durable storage is unavailable while claiming “saved on this device” or permitting remote dispatch.

## Farmer-facing language

- `DraftLocal`: “Saved on this device. Not sent.”
- `Submitted` without receipt: “Waiting for a response. The provider has not been confirmed to have received it.”
- request receipt: “Request received. This is not a quote or booking.”
- unresolved attempt: “We could not confirm whether the request went through. We will not send it again automatically.”
- booking: “Booking confirmed. Work has not yet been completed.”
- provider completion: “The provider reports the work is complete. Accept it, report a problem, or request rework.”

Do not show internal enum names/hashes to farmers. If no authoritative reconciliation exists, preserve unknown and offer a phone/cooperative/adviser path.

## Proposed gates (targets, not results)

- 100% of remote sends have a durable Submitted record committed before the call.
- Zero sends after failed or unverified durable writes.
- 100% of restarted Submitted attempts without an authoritative result recover as OutcomeUnknown before recovery controls appear.
- Zero automatic resends for OutcomeUnknown.
- Zero locally acknowledged records lost in the declared crash/restart/reconnect suite.
- Zero silently overwritten farmer/provider commitments.
- Zero sensitive payloads, exact farm coordinates or raw identities in synthetic fixtures and routine diagnostics.
- 10,000 deterministic crash/reload schedules with zero blind replay or false success promotion.
- At least 90% of tested users correctly distinguish local save, send, request receipt, booking, provider-reported completion and farmer acceptance. This is a proposed comprehension target, not a measured result.

## Machine-readable state mapping and failure corpus

- [Local-attempt to farmer-service mapping](farmer-ux-004-state-mapping-v1.json) and [schema](farmer-ux-004-state-mapping-v1.schema.json) define eight reusable operation-attempt statuses separately from eleven farmer-service states, with nine guarded state promotions and explicit non-goals.
- [Failure-injection corpus](farmer-ux-004-journal-cases-v1.json) and [schema](farmer-ux-004-journal-cases-v1.schema.json) freeze 16 crash, storage, scope, consent, response, duplicate, and service-mapping schedules.

Add an implementation against an independently authored expected-outcome oracle; do not derive expected state from production code. The corpus is synthetic and is not a test result.

Build sequence: (1) review/pin the reusable Hearth transition model, (2) deterministic fake durable store and crash injection, (3) browser/WASM persistence after schema/quota/privacy review, (4) Farmer Today outbox UI binding, (5) owning-service receipt/reconciliation adapters, (6) constrained-device tests and consenting user evaluation.

No autonomous payment, dispatch, actuation or agronomic diagnosis; no second service lifecycle; no real farm data in fixtures; and no field-impact claims from synthetic tests.
