# Mycelix Hearth × Commons Transit × HomeOps
## Household authority, consent and privacy contract

**Status:** Research/design contract v0.1; not deployed capability  
**Date:** 2026-11-15  
**Tracking:** Mycelix #6264  
**Related work:** Mycelix #6258 (Commons logistics/HomeOps design), #6259 (revive dormant Commons transport)

## 1. Decision

Integrate Hearth as the optional household governance, consent and care-coordination boundary for Transit and HomeOps. Do not make Hearth a logistics engine, robot controller, public inventory service or universal model of family authority.

The division of responsibility is:

| System | Responsibility | Explicit non-responsibility |
|---|---|---|
| Mycelix Hearth | Household membership, household decisions, scoped delegation, care coordination, optional shared-resource agreements and emergency communication | Public freight matching, vehicle routing, high-rate telemetry, physical actuation |
| Mycelix Commons / Transit | Shared vehicles and assets, transport offers, bookings, freight, hubs, cooperative services, service agreements and community coordination | Household-private inventory, occupancy inference, robot safety |
| NestPort / HomeOps | Local receiving, package state, exact inventory, containerization, storage, resident access and local device policy | Granting community ownership or overriding household consent |
| Mycelix Civic Robotics Dispatch | Robot/device identity, mission record, authorization envelope, fleet-level status and recall | Motor, manipulator or flight control |
| Symthaea | Perception, intent interpretation, task decomposition, planning, uncertainty estimation and recovery proposals | Granting permission, changing safety limits or asserting delivery without evidence |
| Symtropy | Physical digital twin, coupled material-flow simulation, deterministic replay and scenario qualification | Certifying a real building, aircraft, robot or installation |
| Sol-Atlas | Site/geospatial/environmental evidence for homes, hubs, routes, docks and charging | Land-title determination, flight permission or engineering certification |

This is an integration of existing systems, not a new household ledger or second asset registry.

## 2. Existing Hearth capabilities that fit

The current Mycelix Hearth README describes a household/kinship hApp with 11 domain zomes and a unified bridge. Its domains include:

- Kinship and membership: household creation, invitations, membership, roles and relationship bonds.
- Decisions: proposals, votes, quorum and recorded outcomes.
- Care: recurring care tasks, task swaps, meal plans and schedules.
- Autonomy: graduated autonomy profiles and capability requests/approvals.
- Emergency: household emergency plans, alerts and check-ins.
- Resources: shared household resources, lending, returns and budget categories.
- Rhythms: household rhythms and presence.
- Stories, gratitude and milestones: cultural and relational functions that should remain outside logistics by default.

The Hearth hApp manifest provisions a separate cloned Hearth cell/DHT per household. Its manifest comment says that the provisioned template/control cell must not receive household records. This is a useful isolation boundary, but it is not a reason to publish granular inventories, presence or private schedules to the household DHT: DHT records are still shared with the peers of that household cell.

The older MYCELIX_HEARTH_DESIGN.md describes a media/publishing commons. The current mycelix-hearth README and happ.yaml describe household/kinship coordination. Treat the current hApp implementation and manifest as the integration reference, and reconcile the older design-document naming separately.

## 3. Non-negotiable authority and privacy rules

1. **Membership is not blanket consent.** Being related to, living with, caring for or working for a person does not automatically authorize delivery access, collection, entry, inventory access or disclosure.
2. **Roles are not capabilities.** Hearth roles and graduated-autonomy tiers may inform a household policy decision, but they must not be treated as generic robot permissions or substituted for an explicit action-specific grant.
3. **Consent is purpose-bound.** A grant names what action is allowed, by whom, for which endpoint or service, during what window, and under which policy version.
4. **Delegation cannot amplify authority.** A delegate may only exercise a subset of the original grant, with an earlier or equal expiry. No onward delegation unless explicitly granted.
5. **Revocation is prospective and checked at use time.** Revoking a grant prevents new actions. Previously shared DHT facts or receipts must not be assumed erasable; disclose as little as possible in the first place.
6. **Household inventory remains local.** Exact contents, item-level stock, package photos, occupancy, device sensor streams and room-level maps stay in local HomeOps storage unless a resident explicitly authorizes a specific disclosure.
7. **No presence surveillance.** Hearth rhythms or Home/Away state must not be forwarded to couriers, operators, fleet services or Commons. A local receiver may expose only a delivery endpoint's current availability when required.
8. **Emergency notification is not entry authority.** An alert may trigger a notification or a resident-approved escalation path. It does not silently unlock a door, grant a service provider access or bypass a local safety interlock.
9. **The household model is opt-in and plural.** Nuclear, extended, chosen, blended, multigenerational, intentional, co-pod and custom hearth types should not force one universal kinship or governance model. Cooperative, tenant, guest, caregiver and service-provider authority must remain separately represented.
10. **No network dependency in the safety loop.** A robot must never require a live Hearth/Commons DHT call to avoid a collision, stop safely, enforce a guarded zone or resolve an actuator fault.

## 4. Versioned contracts

The following are proposed interface concepts, not existing deployed zome entry types.

### 4.1 Household logistics grant

A HouseholdLogisticsGrant is a narrow authorization artifact issued by an authorized household member or the household's explicit decision process. It should include:

- schema version and unique grant identifier;
- household-cell reference and policy version;
- issuer and intended beneficiary/principal;
- the exact delegate identity, service-provider identity or device identity;
- allowed action set, such as accept-at-external-vestibule, collect-a-returnable-tote, hand-over-to-named-courier, or share-a-delivery-window;
- the exact receiver, dock or endpoint identifier;
- cargo/handling constraints only where needed for the authorized operation;
- audience and stated service purpose;
- not-before and expiry times;
- one-shot or bounded-use semantics;
- optional parent-grant reference for constrained delegation;
- nonce / idempotency identifier and revocation reference;
- a digest/reference to the underlying consent receipt, not a copy of private household discussion.

A grant must not contain an unrestricted home key, a reusable universal unlock token, a full household inventory, resident presence, room maps or an unrestricted robot command.

### 4.2 Minimal delivery intent

A household delivery intent contains only what the selected service needs:

- opaque shipment or request identifier;
- requested service class and delivery window;
- permitted external receiving endpoint;
- allowed package handling class and relevant constraints;
- grant identifier or grant digest;
- a returnable-container requirement, if applicable;
- a minimal acknowledgement destination.

The provider does not receive the household's internal destination, pantry stock, item history, care schedule, presence status or other deliveries.

### 4.3 Minimal custody receipt

A custody receipt records the operational facts necessary to resolve responsibility:

- opaque shipment and container identifiers;
- event kind (offered, accepted, rejected, transferred, held, returned or reconciled);
- releasing and receiving custodian identities;
- time and appropriately coarse location evidence;
- seal/condition and required environmental observations;
- referenced grant and asset capability digests;
- event identity, evidence digest and acceptance/rejection reason.

A receipt must not embed raw camera footage, full manifest contents, biometric data, household schedule or unrelated sensor telemetry. High-volume evidence is stored locally or in a separately controlled evidence store; a receipt carries only its digest and authorized reference.

### 4.4 No shared inventory API

HomeOps may maintain a detailed local inventory. Hearth may optionally maintain a small shared-resource register for items that the household has deliberately chosen to lend or govern together. Commons may maintain service-level inventory for community hubs and shared assets.

These are three different scopes. Do not mirror a household pantry or room-level inventory into Hearth or Commons merely because an inventory API exists.

## 5. Authorization and delivery sequence

1. A resident creates a local delivery request or explicitly approves one through Hearth.
2. Hearth checks membership and the household's configured decision/delegation rules, then records a narrow grant and its consent evidence.
3. HomeOps receives the grant reference and independently verifies issuer, beneficiary, endpoint, scope, policy version, time window, revocation state and local device policy.
4. Commons verifies the service provider, booked service, carrier/robot capability and mission eligibility. It receives only the minimal intent and grant proof needed for the service.
5. Symthaea may propose sorting, route or storage plans. The planner cannot widen the grant, change package eligibility or choose a restricted destination.
6. A local HomeOps controller checks actual package identity, seal, mass/dimensions, sensor freshness, compartment capacity, temperature and mechanical interlocks before physical transfer.
7. Releasing and receiving parties acknowledge the custody handoff. An absent or conflicting acknowledgement leaves custody unresolved.
8. HomeOps commits the local result. Commons stores the minimal service receipt. Hearth receives only the household-relevant completion or exception event that the resident has authorized.
9. If the outcome is unknown, the system reconciles evidence before retrying a physical mutation. It must not equate a timeout with failure or success.

A service may be scheduled while the household is offline, but a new physical action must still be checked against a valid local grant and local safety policy. A cached grant is valid only within its explicit scope and expiry; it is not a substitute for checking revocation when connectivity is available.

## 6. Data visibility matrix

| Data | HomeOps local device | Hearth household cell | Commons / provider |
|---|---|---|---|
| Item-level pantry and storage inventory | Full local record | Not by default | Never by default |
| Delivery window | Full | Only when household authorizes | Minimum required window |
| External receiving endpoint | Full local configuration | Grant/reference if authorized | Endpoint token/reference only |
| Household presence and rhythms | Local, if enabled | Hearth member-controlled | Never |
| Care schedule / meal plan | Local or Hearth according to household policy | Household-scoped | Only explicit, minimal fulfillment request |
| Package image and raw sensor stream | Local/evidence store | Not by default | Not by default |
| Custody completion | Full local evidence | Minimal household event | Minimal service receipt |
| Community hub asset availability | No need to mirror | Optional view | Commons authoritative |
| Household emergency plan | Local/Hearth per policy | Household-scoped | Never; only explicit alert escalation |

The table describes intended boundaries, not existing deployed enforcement.

## 7. Cross-hApp bridge contract

Hearth already exposes cross-cluster dispatch functions for Commons and uses a routing registry to define allowed targets. The integration must not assume that the presence of a bridge function proves every desired route is enabled or correctly protected.

Before an implementation is relied upon:

- Freeze the exact hApp role names, DNA hashes, zome names and function names used by the integration.
- Test the precise Hearth-to-Commons and Commons-to-Hearth allowlists against the deployed manifests.
- Prefer typed, narrow adapter functions over generic arbitrary-zome dispatch at the application boundary.
- Bind caller identity to the actual Holochain agent; never trust a client-supplied issuer or reviewer identity.
- Verify grant audience, household, endpoint, service and policy version on every authorization use.
- Make one-shot grants consume-once and all retryable requests idempotent.
- Treat expired, revoked, malformed, wrong-audience, wrong-household and unavailable authorization as denied.
- Never send high-frequency robot or aircraft telemetry through Hearth DHT records or Commons transport zomes.
- Use a bounded telemetry channel for operational status where appropriate; store signed, rate-limited summaries and evidence references in durable records.
- Keep bridge outages from changing local stop, interlock, safe-state or manual-recovery behaviour.

The older Mycelix ecosystem review flags allowlist drift and partial adoption of the shared routing registry as a cross-cluster risk. Re-verify the current code and add regression tests rather than treating that historic audit as proof of either current failure or current correctness.

## 8. Integration with other Luminous systems

| System | Integration contract |
|---|---|
| Mycelix Commons Housing / Property | Resolve the authorized property, facility or receiving endpoint. Tenure, ownership and household consent remain separate claims. |
| Mycelix Provision / Food | Turn an explicitly approved meal or replenishment intent into a minimal food-hub request. Keep dietary and health-sensitive details local; disclose only the operational constraint necessary to fulfill an order. |
| Mycelix SupplyChain | Reuse provenance/checkpoint vocabulary for container and shipment custody only after caller binding, sender/recipient authorization and escrow-release paths are independently re-audited. |
| Mycelix Pulse | Optional low-latency operational status and device-health signal; never the source of physical safety authority or a raw household sensor export. |
| Mycelix Identity / credentials | Authenticate people, providers, devices and service agents; identity is not itself authorization. |
| Mycelix Civic Robotics Dispatch | Register robotic assets, authorize mission envelopes, record telemetry summaries and support recall. Dispatch permission is not actuator permission. |
| Mycelix Finance / Treasury | Later service settlement, deposits or cooperative cost allocation after explicit authorization and verified service evidence. Do not connect payment mutation to unverified delivery claims. |
| AeroWeft | Aircraft, propulsion, dock, energy and flight-performance capability evidence. A household grant cannot authorize flight or airspace access. |
| Symthaea | Interpret the request and propose plans under the exact capabilities provided; expose uncertainty and ask for review rather than inventing contents or permission. |
| Symtropy | Replay expected and failed delivery sequences, interlocks, load transfers, thermal excursions, human/animal interactions and recovery operations. |
| Sol-Atlas | Supply provenance-tagged geographic, terrain, climate, access, hazard, charging and site evidence. A map result is not a legal or engineering approval. |

The existing SupplyChain and Finance audit history is a reason to make their integration conditional on a fresh source-level authorization review. Do not route custody or money through those systems merely because an API exists.

## 9. Failure and threat model

The contract and eventual implementation must cover:

- forged or stale household membership and delegated grants;
- replay, duplicate delivery, duplicate acceptance and counterparty substitution;
- wrong household, wrong receiver, wrong audience or wrong service;
- grant expiry or revocation during a network partition;
- a provider or robot attempting to broaden its scope;
- household departure, guest/caregiver role changes, loss of a device and identity recovery;
- an offline or unavailable Hearth/Commons bridge;
- conflicting package identity, damaged seal, unknown contents or a custody disagreement;
- stale or contradictory temperature, weight, position and door/interlock sensors;
- a resident, child, guest, animal or responder entering the transfer zone;
- DHT disclosure beyond the intended household audience;
- emergency escalation being misused as a blanket access or unlock mechanism.

Failure handling follows one rule: **unknown outcome means no new physical mutation until reconciliation**. The machine may enter a locally safe state, retain custody, alert the resident and provide manual recovery. It must not silently mark the package delivered, consume another grant or retry an irreversible transfer.

## 10. Qualification plan

### P0 — Source and manifest truth

- Reconcile the active Hearth README, hApp manifest, DNA manifest and older Hearth design-document naming.
- Verify current role names, clone provisioning, zome names and bridge allowlists from the exact branch/commit.
- Confirm Commons transport remains dormant until its own WASM build, DNA/hApp pack and validator audit gates pass.
- Record actual source/commit identifiers; distinguish source inspection from executed tests.

### P1 — Pure contract and property tests

Build the grant, delivery-intent and receipt types in a small dependency-light Rust contract crate or the appropriate existing shared types crate. Add validation and state-machine tests for expiry, replay, delegation, revocation, audience, household binding, endpoint binding, stale policy and idempotency.

Property-based tests should generate arbitrary event sequences and verify that no illegal path reaches an accepted-delivery state. Negative controls must demonstrate that removing each critical guard makes the corresponding test fail.

### P2 — Holochain multi-agent tests

Use at least two household members, a delegated caregiver, a service provider, a Commons coordinator and a device identity. Verify both permitted and forbidden cross-hApp calls. Assert that the service provider cannot query the household inventory, presence, rhythms or care schedule.

Run the tests against packed DNA/hApp artifacts and the intended conductor version. Ignored or single-agent tests do not qualify bilateral authorization.

### P3 — Local HomeOps adapter

Verify grants locally at the receiver before opening any external compartment or starting a transfer. Exercise expiry, revocation, sensor-stale, interlock, power loss, bridge loss and manual recovery. The local controller must remain safe with every network disconnected.

### P4 — End-to-end evidence

Run the physical logistics benchmark with separate ledgers for household consent, Commons booking, physical custody and local storage. No simulation, HIL or physical qualification claim may be inferred from another layer's result.

## 11. Interoperability and evidence references

- VDA 5050 version 3.0 is the current VDA mobile-robot communication recommendation. Its zone concept allows autonomous robots to plan local routes while fleet control retains bounded influence over zones and traffic. Use it as an adapter target for mixed-fleet task/status interoperability; do not treat it as a safety certification.
- GS1 EPCIS 2.0 is a useful event vocabulary for interoperable visibility and custody facts across organizations. Map only the minimal authorized logistics events; do not export household-private context.
- ISO 3691-4:2023 is relevant to driverless industrial trucks and their systems. Applicability must be assessed for the actual vehicle and use case.
- ISO 13482 is relevant to service robots and human-contact hazards. Check the edition and applicable jurisdiction at the time of design review; it does not replace a product-specific risk assessment or aviation requirements.

These references guide interoperability and assurance. This document does not claim conformance.

## 12. Explicit non-goals

This contract does not authorize:
- unattended entry into a home or living space;
- autonomous opening of unknown packages;
- disclosure of complete household or family inventories to Commons;
- continuous household tracking or presence inference for providers;
- robot movement based solely on a Hearth or Commons record;
- autonomous aircraft dispatch, launch, landing or airspace access;
- automatic payment release from an unverified delivery status;
- an AI system overriding a resident, property holder or local safety controller.

## 13. Completion criteria

The integration is ready for implementation review only when:
1. the role and bridge contract is frozen and checked against exact manifests;
2. the grant and receipt schemas have a versioning and migration strategy;
3. all negative authorization cases are executable tests;
4. household-private inventory and presence are unreachable from provider-facing interfaces;
5. local physical actions remain safe during bridge outage, revocation uncertainty and sensor faults;
6. two-sided custody is unresolved until both parties or an independently authorized reconciliation process provide evidence;
7. every claim is labelled as designed, source-checked, unit-tested, multi-agent-tested, simulated, HIL-tested or physically qualified.

The objective is not to make every household transparent to its logistics network. It is to make household participation possible with the smallest safe disclosure and a clear, revocable chain of authority.
