# Finance exposure reference model v1

**Status:** effects-disabled reference model and cross-language canonical snapshot profile. This is not a production liquidity control or a claim about available external funds.

## Purpose

The reference kernel keeps a conservative, owner-scoped upper bound on economic effects that have been reserved, authorized for dispatch, become indeterminate, or are committed but not finally settled.

It is intentionally separate from the payment saga model lifecycle. The exposure book answers a different question:

> What is the maximum plausible amount this owner/pool/destination/currency scope is currently exposed to, including unresolved effects?

## Scope and units

A book is bound to one exact tuple:

- owner ID;
- pool ID;
- destination ID;
- currency/instrument.

Amounts are unsigned integer minor units. Different currencies cannot be combined in one book. The model performs no FX conversion, netting, collateral valuation, legal set-off or credit-risk calculation.

An ExposurePolicyV1 binds a monotonic policy sequence, maximum total exposure, maximum single-effect exposure and a policy digest. The digest is an identifier, not proof that an owner authorized the policy. Production must verify the policy under the actual owner-native authority mechanism.

## Reservation lifecycle

| State | Counts against limit? | Meaning |
|---|---:|---|
| Reserved | Yes | Stable effect identity reserved, not yet dispatch-authorized |
| DispatchAuthorized | Yes | Owner's dispatch gate has been passed for this stable effect |
| Indeterminate | Yes | An effect may have happened; outcome is unresolved |
| CommittedUnsettled | Yes | Destination effect is evidenced, but final settlement is not |
| Settled | No | A matching, typed settlement-resolution subject was supplied |
| AuthoritativelyAbsent | No | A matching, typed authoritative-absence subject was supplied |
| CancelledBeforeDispatch | No | Reservation was cancelled before dispatch authorization |

The aggregate is the checked integer sum of every reservation in the first four states. A timeout, lease expiry or age threshold does not release exposure.

The reference also exposes a per-state integer breakdown: reserved, dispatch-authorized, indeterminate and committed-unsettled. Its total must equal the aggregate used for the limit check. This prevents a single headline number from hiding whether risk is queued, already dispatched or awaiting settlement.

The reserve operation is idempotent for an exact effect identity and exact immutable intent. Reusing that effect ID with changed scope, intent digest, payload digest or maximum exposure is a conflict. A new reservation is admitted only when both the per-effect and aggregate limits are respected.

A lower policy never deletes old reservations. The existing book may temporarily exceed its new limit; new risk-increasing reservations fail closed until the outstanding obligations are resolved.

## Resolution evidence

Final settlement and authoritative absence have different typed resolution kinds. A provider acceptance acknowledgement or a pending/accepted processing status is not final settlement evidence and must not release exposure. Both bind:

- exact effect ID;
- exact destination;
- exact payload digest;
- resolution profile digest;
- evidence digest.

The pure model checks canonical digest syntax and exact subject binding only. It does not authenticate the provider, validate a signature, establish query consistency, prove destination finality or determine whether an absence query is authoritative. Those checks belong to a provider-specific adapter and independent trust policy.

A final-settlement resolution moves CommittedUnsettled to Settled. An authoritative-absence result releases a DispatchAuthorized or Indeterminate reservation. These are separate typed kinds; neither can be substituted for the other. The reference checks the kind and exact subject fields but cannot prove that the referenced provider profile actually defines finality; that is a provider-adapter qualification obligation.

## Canonical snapshot

Domain: MXSAP-EXPOSURE-SNAPSHOT-V1 followed by a zero byte.

Integers are unsigned big-endian. Text fields are UTF-8, prefixed by a u32 big-endian byte length and bounded at 4,096 bytes.

The snapshot binds the scope, policy sequence and limits, policy digest, then the reservation count and all reservations sorted by effect ID. Each reservation binds operation ID, effect ID, immutable intent digest, payload digest, maximum exposure amount and state tag.

The shared fixture sap-payment-exposure-v1.kat pins a canonical snapshot digest. Rust and Go tests load this same fixture independently.

## Qualification boundary

The Rust and Go books are serial in-memory reference implementations. They do not prove atomic reservation under concurrent Holochain agents, source-chain forks, multi-process compare-and-swap, persistence, DHT convergence, bank liquidity, collateral sufficiency, provider settlement or legal finality.

The next qualification stages are:
1. deterministic transition and mutation corpus;
2. shared #6045 fault traces (new trace version only; V1 remains frozen);
3. atomic reservation under real Holochain source-chain contention;
4. restart/recovery and partial-DHT evidence;
5. provider-specific settlement, returns, reversals and authoritative-absence semantics;
6. stress comparison with the PostgreSQL control arm in #6046.

Safety is non-compensatory. Throughput cannot offset an over-limit dispatch, false absence, or silently released unresolved obligation.
