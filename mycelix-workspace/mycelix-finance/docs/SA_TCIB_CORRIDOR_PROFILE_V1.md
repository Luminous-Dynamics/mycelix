# South African / SADC TCIB Corridor Qualification Profile v1

**Status:** effects-disabled, synthetic-only corridor qualification. This is not evidence of a live TCIB connection, participant admission, FX service or settlement guarantee.

## Why corridor qualification is a separate contract

SARB's October 2026 programme statement sets a future target to move qualifying regional payments up to R100,000 to TCIB by late April 2027 and an aim for an eight-country hub-and-spoke pilot by October 2027. The statement does not name the participating countries, confirm participant pairs or establish live limits, FX, liquidity, settlement assets, tariffs or legal finality.

The programme milestones are therefore represented as planned only. The R100,000 value is stored as a programme-target value in ZAR minor units, not as a product limit or executable amount ceiling.

## Corridor qualification graph

origin jurisdiction → destination jurisdiction → rails → participant pair → instrument/message profile → operating calendar → currency pair → executable FX quote → funding/liquidity → settlement asset/model → screening authority → legal finality → recourse

A corridor is not executable merely because both payment networks have technical connectivity, an ISO 20022 message parses, an FX rate is visible, an origin-side participant accepted an instruction or one settlement leg is complete. Each property must be sourced, current and scoped to the participant pair and transaction date.

## Readiness classes

- current_domestic_scope_only: a domestic rail/profile cannot be promoted to a cross-border route.
- planned_programme_target: a future programme milestone; not currently executable.
- unqualified_unknown: participant, currency, funding, settlement, legal or operating property is missing.
- synthetic_test_only: fully specified hypothetical route for evaluator positive controls. It never enables live effects.

## FX and settlement boundaries

Reuse #5310's provider-operated quote and execution contract. Preserve exact source/destination currencies, quote direction, rate numerator/denominator, observation time, expiry, spread, fee, rounding rule and FX evidence. Indicative or expired quotes cannot authorise conversion.

Require authoritative funding evidence and specify the settlement asset and model. A PvP claim requires an arrangement evidence reference. Both origin and destination legs need independent receipts before bilateral settlement can be qualified. A technical response, single-leg settlement or domestic finality rule cannot prove cross-border legal discharge.

## Operational and compliance boundaries

Operating hours and holidays are local to each jurisdiction and participant. Screening obligations and authority must bind the transaction and participants. Fraud status cannot be promoted by an intermediary without originating-source authority. Data sharing is bound to purpose and destination audience. Recourse is separate: recall, trace and hold events do not count as returned funds.

## Benchmark

Hard gates:
- no planned milestone promoted as live;
- no domestic rail promoted to cross-border;
- no participant pair without both-side admission evidence;
- no currency pair or FX quote assumed;
- no hidden fee or amount/currency substitution;
- no assumed prefunding, PvP, settlement asset or legal finality;
- no one-leg event promoted to bilateral settlement;
- no unqualified screening or cross-border data disclosure;
- no relay confirmation of an originating PSP's fraud status;
- no recall/hold promoted to returned funds;
- deterministic replay and explicit unknown/blocked dispositions.

Operational comparisons should later measure total cost as basis points of gross/net delivered value, FX spread and slippage, initiation-to-recipient-availability, time to qualified settlement, availability by local calendar, participant liquidity needs, unknown/late outcomes, residual loss, investigation time and user effort. Compare the same frozen corridor/currency-pair cohort; do not compare unmatched corridor prices or different FX reference windows.

## Claim ceiling

The corpus is synthetic. It establishes no current TCIB route, no eight-country membership list, no bank/provider integration, no FX availability, no pre-funding guarantee, no PvP arrangement, no settlement finality and no regulatory approval. Any real activation requires official rulebooks, actual participant agreements, current local regulatory requirements, FX-provider evidence, liquidity/funding arrangements, settlement receipts and legally reviewed customer remedy terms.
