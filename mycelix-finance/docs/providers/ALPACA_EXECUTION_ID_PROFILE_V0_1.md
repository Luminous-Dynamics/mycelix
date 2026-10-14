# FIN-ALPACA-EXECID-001 — Activity SSE identity profile v0.1

## Status

Frozen provider-mapping profile and synthetic corpus only.

```text
NOT IMPLEMENTED
NOT NETWORK-EXECUTED
NOT QUALIFIED
NOT PASS
```

Tracks #2789 and composes with FIN-ADAPT-EXECID-001 / #2774.

## Provider semantic family reviewed

Provider: Alpaca, US region.

Shared activity surface family:

```text
GET /v2beta1/events/activities
```

Current Alpaca documentation exposes this endpoint family under **both** Trading API and Broker API documentation. The exact deployment/access profile is therefore separate from the semantic field-mapping profile.

Reviewed deployment profiles:

```text
Trading API / paper
  host: paper-api.alpaca.markets

Trading API / live
  host family: api.alpaca.markets

Broker API / sandbox
  host: broker-api.sandbox.alpaca.markets

Broker API / production
  host family: broker-api.alpaca.markets
```

The Trading API Activity SSE reference currently shows the paper host and Trading API key headers. The Broker API Activity SSE reference currently shows the broker sandbox host and Broker credentials. Alpaca's current authentication documentation separately identifies the corresponding live/production host families.

The `/v2beta1/` path is profile-significant because Alpaca documents it as beta and subject to a future GA path change.

## Identity roles

Current Activity SSE documentation states:

```text
event_id
= ULID replay/publication cursor
!= economic execution identity

ref_id
= stable provider activity identity
= execution ID for trades under this schema family

previous_id
= predecessor ref_id for correction/bust lineage

details.order_id
= order correlation
!= execution identity
```

Consumers are instructed to persist `event_id` for replay and deduplicate activities by `ref_id`.

No adapter may synthesize execution identity from order ID, quantity, price, timestamp, event cursor, or a hash/proximity heuristic.

## Fill vs adjustment identity

The provider schema uses `activity_type = TRD` with `details.execution_type` values including `fill`, `trade_correct`, and `trade_bust`.

Mycelix must not collapse those into one positive economic type.

### Fill

```text
TRD + fill + exact ref_id
-> candidate fill execution identity
```

### Correction

```text
TRD + trade_correct + exact ref_id + exact previous_id
-> correction activity identity + predecessor relation
```

The correction `ref_id` is not silently promoted into a replacement positive fill until a later FIN-MKT adjustment-semantic mapping qualifies that interpretation.

### Bust

```text
TRD + trade_bust + exact ref_id + exact previous_id
-> bust activity identity + predecessor relation
```

The bust activity is not a new positive fill.

## Deployment-profile firewall

The same Activity SSE semantic family is available through distinct Alpaca access/deployment profiles. Those profiles have different hosts and authentication/trust contexts.

Until a cross-surface equivalence theorem is separately qualified, portable identity remains scoped by exact deployment profile:

```text
same account_id + ref_id under TradingPaper
!= automatically same identity theorem under TradingLive

same account_id + ref_id under TradingPaper
!= automatically same identity theorem under BrokerSandbox

same account_id + ref_id under TradingLive
!= automatically same identity theorem under BrokerProduction
```

This is deliberately conservative. If a later provider theorem proves that one economic execution is represented by the same `ref_id` across two Alpaca surfaces, Mycelix may add an explicit cross-surface alias/equivalence layer rather than silently merging today.

Every admitted positive identity therefore binds at minimum:

```text
provider
region/schema profile
deployment profile
account_id
ref_id
```

Credential bytes, API secrets, bearer tokens, cookies, and session material never enter portable execution/activity identity.

## Fail-closed outcomes

Conceptually:

```text
StableFillExecutionIdentityAdmitted
StableCorrectionActivityIdentityAdmitted
StableBustActivityIdentityAdmitted
ExecutionIdentityUnavailable
ExecutionIdentityAmbiguous
AdjustmentLineageUnavailable
NotMarketExecution
ProviderProfileMismatch
MalformedIdentityEvidence
```

Unknown future `details.execution_type` values do not auto-map to a known FIN-MKT class.

Missing `ref_id` cannot be repaired from `event_id` or `details.order_id`.

## Replay and gap boundary

`event_id` is the stream replay cursor. On reconnect, `since_id` may be used to resume/replay; replayed activities are deduplicated by `ref_id`.

A slow-consumer/drop indication means frontier completeness must be withheld until separately repaired/reconciled.

```text
stream connected
!= frontier complete

execution identity admitted
!= observation frontier complete/current
```

The stream-control state machine is **not** part of the pure identity mapper. The corpus contains one stream-control case only to freeze this separation.

## FIN-MKT mapping boundary

Do not deserialize Alpaca payloads directly into canonical FIN-MKT observations.

```text
Alpaca Activity SSE deployment
-> bounded provider identity evidence
-> FIN-ALPACA identity mapper
-> later exact economic observation mapper
-> FIN-MKT-002A observation
-> FIN-MKT-002B0 economic execution frontier
-> FIN-MKT-002B observed projection
```

The identity mapper establishes only provider identity roles and predecessor lineage. It does not establish quantity semantics, price semantics, instrument identity, side, fees, currentness, positions, settlement, authority, or account ownership.

## Conformance sequencing

Preferred first network conformance remains **Trading API paper**, because the Activity SSE reference is exposed directly on the paper Trading API host and it gives us a low-effect retail-accessible environment.

A separate Broker API sandbox conformance can then test that the mapper is not Trading-API-shaped.

```text
Trading-paper conformance PASS
!= Broker-sandbox conformance PASS
!= live/production truth
!= order authority
```

The first networked tranches remain read/observe/reconcile only. Order submission is separately authorized.

## Corpus

`fixtures/FIN_ALPACA_EXECID_001_V0_1.json` freezes sanitized cases for:

1. admitted Trading API paper fill;
2. replay with a different `event_id` and the same `ref_id`;
3. equal economics with distinct `ref_id` values;
4. correction activity + predecessor mapping;
5. bust activity + predecessor mapping;
6. missing execution identity;
7. order ID not promoted to execution ID;
8. unknown trade execution type;
9. non-TRD activity;
10. missing correction predecessor;
11. Trading paper/live scope separation;
12. Trading-vs-Broker deployment scope separation;
13. account scope separation;
14. malformed `ref_id` rejection;
15. slow-consumer completeness withholding at the stream-control layer.

## Claim ceiling

This profile/corpus establishes only a preregistered interpretation of current documented Alpaca Activity SSE fields for later implementation/qualification across exact deployment profiles.

It does not establish provider authenticity, authenticated account/correspondent ownership, cross-surface identity equivalence, complete/current history, live/production provider truth, broker execution quality, best execution, position ownership, settlement/finality, financial authority, suitability, tax/accounting correctness, regulatory compliance, or autonomous Symthaea authority.
