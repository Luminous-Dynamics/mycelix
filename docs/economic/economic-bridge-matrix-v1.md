# Economic Fabric Bridge Matrix V1

The bridge is deliberately loss-aware. A mapping is conformant only when semantic widening is impossible or explicitly represented as adapter policy.

| Source | Native concept | Fabric representation | Must preserve |
|---|---|---|---|
| Valueflows | EconomicEvent | EconomicEvent | action, provider/receiver, resource, quantity, time, correction/fulfillment links |
| Valueflows | Commitment | Obligation/Commitment | provider, receiver, promise, due/validity |
| Valueflows | Intent | Offer/Intent | proposal/planning status; not execution |
| Valueflows | Claim | Entitlement/Claim | originating event and claim semantics |
| Valueflows | EconomicResource | ResourceReference | resource identity, accountable agent, quantity/state |
| Integral | COS production observation | EconomicEvent | source observation, provenance, evidence, time |
| Integral | ITC | Instrument-specific entitlement | ITC semantics, issuer/domain, validity; not generic currency |
| Mutual credit | Credit instrument | Instrument + CreditLimit | issuer, limits, backing policy, obligations |
| Accounting | Journal entry | SettlementProjection | source events, accounts, debit/credit semantics, accounting period |

## Loss taxonomy

- Exact — all required source semantics represented.
- BoundedLoss — source semantics intentionally omitted under an explicit profile.
- Unmapped — no safe mapping; adapter must reject rather than guess.
- Conflict — source semantics contradict local policy; adapter must preserve conflict rather than normalize it away.

## Key rule

A bridge must never turn an observation into an entitlement, an intent into execution, a commitment into settlement, a foreign instrument into local issuance, an accounting projection into physical evidence, or a recognition decision into source-origin rewrite.
