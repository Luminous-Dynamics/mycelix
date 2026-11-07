# Maritime Provider Webhook Ingress Profile v0.1 — Dockwa research candidate

**Status:** synthetic research contract; not a Dockwa API specification, integration certification, production adapter, or permission to connect.
**Parent:** Mycelix #5920 / #5916
**Implementation issue:** Mycelix #5925
**Fixture:** `webhook-conformance-cases-v0.1.json`

## 1. Purpose and claim ceiling

This profile defines a narrow, inbound-only adapter boundary for existing marina-management systems. Dockwa is the first research candidate because its public Help Center documents configurable HTTPS webhooks for marina account events.

The intended result is a read-only, bounded projection of provider-originated notifications into existing Mycelix evidence/event machinery. The provider remains authoritative for inventory, bookings, prices, customer relationships, charges, cancellations, refunds and operational status.

A webhook is not:
- a fresh availability query;
- a reservation hold or commitment;
- permission to create, amend or cancel a booking;
- payment settlement evidence;
- proof that a service or physical resource was delivered;
- navigation, port-entry or safety authority.

This v0.1 profile intentionally does not invent Dockwa's private-dashboard payload fields. Exact event payload schemas and samples must be captured by an authorized marina administrator from Dockwa's Event Catalog before any field-level adapter is implemented.

## 2. Publicly documented provider surface

Dockwa's official "Set Up Webhooks in Dockwa" Help Center article, retrieved 7 November 2026, documents:
- webhook subscriptions are enabled by a marina administrator and are not on by default;
- a receiving system supplies an HTTPS endpoint that receives POST requests;
- an Event Catalog inside the marina dashboard provides field lists and sample payloads;
- up to 12 created/updated event names are listed across six record types:
  - `customer.created`, `customer.updated`;
  - `lead.created`, `lead.updated`;
  - `reservation.created`, `reservation.updated`;
  - `sale.created`, `sale.updated`;
  - `space.created`, `space.updated`;
  - `space_group.created`, `space_group.updated`;
- reservation events cover transient and contract/month-to-month-contract reservations and include dockage, charges and payments;
- space events refer to slips, moorings and dock/storage spaces;
- space-group events describe named groups of spaces with shared booking rules;
- the dashboard exposes endpoint delivery logs and activity and permits endpoint pause/deletion;
- Dockwa instructs receivers to use HTTPS and verify requests using the endpoint signing secret; it points to Svix's general receiving/verification guidance.

Authoritative references:
- Dockwa Help Center: https://mayday.dockwa.com/en/articles/17710897-set-up-webhooks-in-dockwa
- Svix verification overview: https://docs.svix.com/receiving/verifying-payloads/why
- Svix receiving overview: https://docs.svix.com/receiving/introduction

These public documents do **not** establish:
- exact Dockwa HTTP header names and versions for a real endpoint;
- the full Dockwa webhook payload schema or sample values;
- event ordering or at-least-once/exactly-once delivery guarantees;
- the configured retry schedule, retention or redelivery controls;
- whether deletions generate separate events;
- a provider read-back API, availability query, hold/commit API or booking write API;
- the full provider revision or event-time semantics.

Do not infer any of those. Capture them from an authorized test endpoint and current provider documentation.

## 3. Candidate event subscription policy

| Event family | Documented meaning | Default subscription |
|---|---|---|
| Reservation created/updated | Dockage booking, contract type, charges/payments | Candidate, after minimum field allowlist |
| Space created/updated | Slip, mooring or dock/storage space | Candidate, after minimum field allowlist |
| Space-group created/updated | Group of spaces and shared booking rules | Optional, only if necessary |
| Customer created/updated | Contact information, address and membership details | Disabled by default |
| Lead created/updated | Service inquiry and its status | Disabled by default |
| Sale created/updated | Point-of-sale transaction; may include financial data | Disabled by default |

The first pilot should subscribe only to reservation and space events needed for the agreed use case. A marina administrator must explicitly approve the exact event types and fields.

## 4. Verification and admission pipeline

Keep each disposition separate:

1. **Endpoint enrolled:** local provider identity and endpoint purpose are authorized by the marina and recognized locally.
2. **Raw bytes captured:** exact HTTP request body and required headers are retained within a protected ingress boundary before parsing.
3. **Transport verified:** provider signature verifies against the exact raw bytes, the endpoint-specific signing secret/key version and the provider-confirmed header contract.
4. **Freshness checked:** the signed delivery timestamp satisfies the configured replay/freshness policy. The policy must state its clock source and tolerance; do not silently inherit an assumed Dockwa setting.
5. **Delivery deduplicated:** repeated delivery identity with identical bytes is treated as a retry, not a second logical event. Reuse of an identity with changed bytes is a conflict/quarantine.
6. **Provider profile matched:** event name and payload schema match the exact pinned Event Catalog capture/profile. Unknown or changed shapes are quarantined.
7. **Locally authorized:** the endpoint, event family and field projection remain within the current local capability, purpose and disclosure policy.
8. **Projection created:** only explicitly allowlisted fields are mapped into existing Mycelix event/evidence types. Preserve original values, source times and mapping loss.
9. **Currentness resolved:** out-of-order or contradictory notifications remain conflicted/unknown until a separately authorized provider reconciliation establishes the current provider state.
10. **Retention enforced:** raw PII-bearing payloads are protected, access audited and deleted/redacted according to purpose and retention policy.

A successful HMAC/signature check establishes transport origin/integrity under the configured endpoint secret. It does not establish that the payload is factually correct, that the data subject authorized a new use, that inventory is current, or that a reservation is legally/operationally valid.

## 5. Svix-compatible verification is a hypothesis to confirm

Dockwa's public instructions point to Svix's receiving/verification guide and expose an endpoint signing secret. Svix's published guidance describes a delivery ID, attempt timestamp and signature headers; the delivery identifier is retained across retries in the documented Svix model.

For implementation research, record candidate header names and semantics, but do not hard-code them as Dockwa facts until a marina administrator supplies an authorized test delivery and the receiver verifies it using the provider's own instructions or supported library.

Regardless of the exact envelope:
- verify the signature over the exact raw body before JSON parsing;
- use constant-time signature comparison;
- bind the key to the exact provider endpoint and environment;
- support controlled secret rotation and revocation;
- enforce a declared timestamp replay window with synchronized clock assumptions;
- keep transport delivery identity separate from provider object identity and logical event identity;
- keep delivery attempts separate from logical provider changes.

Never put a real secret, signed payload, customer record or endpoint URL into Git.

## 6. Ordering, missed events and reconciliation

Do not sort events into truth by webhook arrival time.

Preserve, where actually supplied:
- provider event time;
- provider object update/revision time or sequence;
- webhook delivery/attempt timestamp;
- local receipt time;
- provider object reference;
- exact source profile and payload digest.

If the provider does not supply a reliable monotonic revision or event sequence, two updates to the same object may be incomparable. Report a conflict or NeedsReconciliation rather than choosing "last received".

A missing webhook does not prove that no change occurred. If Dockwa provides no authorized read-back for the pilot, reconciliation may require marina staff to consult the authoritative Dockwa dashboard and issue a new, explicitly sourced confirmation. Do not simulate a read API.

## 7. Privacy and security

- Do not subscribe to customer events by default.
- Do not ingest full customer addresses, phone/email, precise vessel journeys, payment-card data or full transaction details unless there is a separately authorized and necessary purpose.
- Apply field allowlists before downstream projections; unknown sensitive fields fail closed.
- Encrypt and tightly restrict raw-body storage; apply short retention and auditable deletion.
- Keep raw body, verified delivery metadata and minimized semantic projection in distinct stores/retention classes.
- Never publish webhook secrets, raw payloads, home addresses or customer profiles to a public DHT.
- Revoked endpoint authorization or key status stops future admissions; it does not retroactively rewrite the historical fact that an event was received.
- A provider can pause/delete the endpoint. Treat this as integration availability state, not proof the marina closed or withdrew all service offers.

## 8. Conformance corpus

The companion JSON contains 24 synthetic scenarios. They cover positive deliveries, signature failures, exact-byte verification, duplicate/replayed deliveries, conflicting identity reuse, out-of-order notifications, unknown schema, privacy violations, endpoint revocation, durable-inbox failure, and the critical negative control that a reservation notification is not live availability or a new commitment.

The corpus is declarative and **not yet executed**. It contains no real Dockwa payload, customer data, signing secret, signature or integration result.

## 9. Proposed qualification targets

- 100% acceptance of valid synthetic known-answer signatures.
- 100% rejection of invalid, altered, stale, wrong-key and unauthorized deliveries.
- 100% same-delivery/same-body retry deduplication.
- 100% changed-body reuse of a delivery identity rejected/quarantined.
- Zero PII fields outside the explicit projection allowlist.
- Zero webhook events promoted to live availability, booking commitment, payment finality or physical fulfilment.
- 100% preservation of source and received time fields when supplied.
- 100% deterministic projection/replay for frozen cases.
- Exact-head Rust format/test/Clippy and security evidence before any implementation maturity claim.

Report throughput, p50/p95 ingress latency, durable-inbox delay, retry/replay rate, schema coverage, quarantine rate, out-of-order conflict rate and operator reconciliation time separately.

## 10. Claim ceiling

Passing the synthetic conformance harness establishes only the declared adapter behavior over the exact synthetic inputs and pinned profile. It does not establish Dockwa certification, partnership, a public API, complete event coverage, source truth, live inventory, reservation authority, payment finality, physical fulfilment, legal compliance or production readiness.
