# FARMER-UX-006 — channel-neutral request continuity and relay provenance

**Status:** Contract proposal; synthetic fixtures only.  
**Tracking:** [Issue #5207](https://github.com/Luminous-Dynamics/mycelix/issues/5207)  
**Parent contract:** [FARMER-UX-004 / PR #5203](https://github.com/Luminous-Dynamics/mycelix/pull/5203)  
**Canonical service evaluator:** [Symtropy PR #1906](https://github.com/Luminous-Dynamics/symtropy/pull/1906) and [Symtropy issue #1936](https://github.com/Luminous-Dynamics/symtropy/issues/1936).

## Paradigm shift

**One service workflow, many ways to participate.** A farmer's intention must survive app, SMS, phone, extension-worker, cooperative/broker, paper and later-digitized channels without losing its origin, being duplicated, or turning a relay into a provider commitment. The app is an interface into a service network, not the network itself.

This is a hypothesis and design contract to test—not an assertion that each intermediary is reliable or every digital platform is harmful. A 2025 case study of Farm Digital in Ghana reports that an app-first tractor-hire marketplace was repeatedly outperformed by informal brokerage and argues for building on existing institutions (https://doi.org/10.1080/27685241.2025.2531122). A June 2026 study of rural African agricultural platforms reports uneven, short-lived usage and identifies peer networks, extension services and low-tech tools as important alternatives/complements (https://doi.org/10.1016/j.jrurstud.2026.104210). FAO's Digital Services Portfolio adapts themes and content to national and local context (https://www.fao.org/digital-services/about/).

## Identity: three different keys

Do not conflate these:

1. **Logical request reference** identifies the farmer's underlying service request across channels.
2. **Event ID** identifies one assertion, decision or receipt in the event history.
3. **Remote operation ID/idempotency receipt** identifies a mutating attempt at the authoritative service boundary, if and only if that backend contract supports it.

A local UUID is not a server-side idempotency guarantee. A digest detects changed bytes against a declared digest; it is neither authenticity nor proof of truth. If two channels present possibly duplicate requests but lack a shared request reference, never auto-merge them by fuzzy matching. Mark a suspected duplicate and ask an authorized human to reconcile identity without dropping either request from the audit denominator.

## Minimal normalized envelope

- schema/protocol version, stable opaque `request_ref`, unique `event_id`, event type and canonical payload digest;
- channel: app, SMS, voice/phone, extension-assisted, cooperative/broker-assisted, paper-digitized or import;
- **asserting actor role** separately from **recording/relaying actor role**;
- authority basis and consent/mandate reference when another person acts for the farmer;
- opaque scope reference, immutable once a remote request is submitted;
- capture/claimed time and its source/precision separately from received/verified time;
- evidence state: self-reported, relayed, authoritative provider receipt, conflicting, stale, or unknown;
- optional `supersedes_event_id` for corrections. Prior assertions remain auditable.

Do not put names, phone numbers, raw agent/member keys, precise farm coordinates, free-text observations, photos or financial details in this generic metadata envelope. Keep those payloads in a separately authorized domain store with explicit retention/export/deletion rules.

## Authority ladder and allowed claims

- `CapturedLocally`: a participant/device recorded an event. Nothing is implied about transmission.
- `RelayedWithAuthority`: an intermediary forwarded a farmer request within recorded authority scope. Provider receipt is not implied.
- `ProviderReceipt`: authoritative service evidence acknowledges the exact `request_ref`. It does not imply a quote or booking.
- `QuoteOffered`: the owning service evaluator validates complete quote terms and source authority.
- `BookingConfirmed`: explicit farmer acceptance, valid unexpired quote, scope match and authoritative booking confirmation.
- `ProviderReportedComplete`: provider completion assertion only.
- `FarmerAccepted` or `DisputedOrRework`: distinct explicit farmer decisions.
- `OutcomeUnknown`: remote effect cannot be established. No automatic resend; reconcile through an authoritative source or preserve uncertainty.

An intermediary may record that a farmer verbally accepted only when authority/consent rules allow it. The event remains attributed as an agent-recorded account of the farmer's decision—not a direct cryptographic farmer signature. The acceptance method and assurance class must be visible to downstream policy.

## Replay, conflict and ordering rules

1. Same event ID + same canonical digest: idempotent duplicate; keep one effect and return/reference the original event.
2. Same event ID + different digest: equivocation/conflict; quarantine and alert an authorized resolver. Never overwrite.
3. Same logical request reference across channels: append to the same request history.
4. Separate request references with a plausible match: create a suspected-duplicate relation; do not merge, delete or silently choose a winner.
5. Delayed/out-of-order events: retain origin/capture/received/verification times separately and apply only transitions permitted by the service contract. Client timestamps alone do not establish trusted order.
6. Paper digitization: preserve claimed original date/source separately from digitization/receipt time. Never backdate server receipt.
7. Consent revoked before forwarding: block new onward disclosure; preserve prior receipt/audit history subject to retention rules. Do not retry/share automatically.
8. Scope changes cannot retarget submitted requests. A new target requires a new explicit request and authority path.
9. Unknown remote outcome or lost journal metadata never proves “not sent”.
10. Channel change never counts as a second request or second provider dispatch by itself.
11. No-provider, refusal, cancellation, no-show, pending acceptance and disputes remain visible in service denominators.
12. No default channel ranking. Analyze by channel only with disclosed purpose, consent and adequate cohort size.

## Frozen scenario corpus

The machine-readable [v1 cases](farmer-ux-006-channel-cases-v1.json) and [schema](farmer-ux-006-channel-cases-v1.schema.json) freeze seventeen synthetic scenarios. The [envelope schema](farmer-ux-006-channel-envelope-v1.schema.json) validates normalized metadata structure; it cannot prove actor identity, real consent, source authenticity, durable persistence, provider receipt, or truth.

## Proposed benchmarks (targets, not results)

- **100%** of valid channel handoffs preserve the same logical request reference.
- **Zero** duplicate logical effects from exact same-event replay, and **zero** automatic merges when separate request references only look similar.
- **100%** of relayed decisions retain asserting role, relaying/recording role, authority basis, source channel and time provenance.
- **Zero** intermediary statements promoted to provider receipt, booking or farmer acceptance without required authority/evidence.
- **Zero** same-event-ID/different-digest conflicts silently overwritten.
- **100%** of ambiguous submissions recover as unknown, never success/failure or automatic-retry permission.
- **100%** of paper/offline entries preserve claimed event time separately from receipt/digitization time.
- **Zero** unfulfilled requests, no-provider outcomes, cancellations, no-shows, unresolved outcomes or disputes silently removed from evaluation denominators.
- **10,000** deterministic duplicate/reorder/clock-skew/consent-revocation schedules with zero duplicate logical effects and zero unauthorized promotions.
- At least **90%** of formative participants correctly explain who asserted a fact, who recorded/relayed it, what is confirmed, and what remains unknown. Any false belief that a relay is a confirmed booking is a stop-and-revise failure.
- Report request completion, time-to-authoritative-receipt, cancellation/no-provider/no-show rate, correction rate, farmer acceptance inside the declared useful window, and human-agent workload separately. Do not set a universal time target before a local task/window baseline exists.

## Reuse boundaries

Symtropy owns the canonical service evaluator and denominators; this adapter never implements a second quote/booking/completion engine. Hearth owns the candidate generic local journal and service-side idempotency work; this contract adds neither another journal nor a backend deduplication guarantee. Mycelix owns consent, request references, provenance and cross-channel interaction. A channel/agent adapter must be authorized before it sends or claims receipt.

## Non-goals

No live SMS/telephony integration, payment, provider dispatch, physical control, identity-verification provider, autonomous fuzzy merge, forced smartphone onboarding, public personal/farm metadata, or agricultural impact claim. Schema/fixture success is not evidence of field accessibility or better farming outcomes.