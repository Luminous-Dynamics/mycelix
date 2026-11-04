# South African Request-to-Pay Limit Fairness Profile v1

**Status:** synthetic, effects-disabled conformance benchmark. This is not a live bank connector or an account-specific current limit statement.

## Paradigm shift: limits are typed policy, not a global scalar

Do not describe a request service with one `max_amount` or `max_requests_per_day`. Limits depend on participant, product, channel, route, address type, direction, counterparty aggregation, window semantics, amount scope and receiver-side controls.

A rate-limit claim is only valid when its full scope is source-backed:

`participant × product × direction × route × counterparty scope × window × rule version`

A source-backed "no daily count cap" is different from an unknown limit. Unknown is neither unlimited nor a reason to reject.

## Source snapshots (observed 4 November 2026)

- **Capitec:** FAQ states two requests in 24 hours to any South African bank for the requester account. Its product page explicitly lists R100 per request and R200 combined over two requests per 24 hours. The 24-hour wording is modelled as a rolling window, not a midnight calendar reset.
- **Absa:** current product page describes unlimited daily outbound total with at most five requests to the same recipient, and unlimited inbound total with at most five from the same sender. It distinguishes R5 million on-us account, R3,000 on-us ShapID, and R50,000 off-us per-request amounts; off-us may be subject to a lower receiving-bank limit. Absa-sent requests expire after 45 days.
- **Nedbank:** current FAQ says R50,000 per request per day, no daily count limit for sending or paying requests, a default expiry of 14 days, and sender selection up to 45 days. Declining closes the request.

These are provider-specific public snapshots; the effective date of each displayed product term is not stated in the sources. Do not generalise them into a universal PayShap rule.

## Rule schema

Count rules bind:
- inbound/outbound direction;
- account-wide or per-counterparty scope;
- rolling 24-hour, calendar-day or unspecified "per day" window;
- total and/or per-counterparty counts;
- aggregate-unlimited versus unknown state;
- source reference and qualification.

Amount rules bind:
- on-us account, on-us ShapID, off-us or generic route;
- per-request ceiling;
- explicit aggregate amount ceiling, only when the source states one;
- aggregate window and source.

Expiry is a separate rule. Default, minimum and maximum expiry are not collapsed.

## Synthetic cases

The corpus contains 30 scenarios:
- Capitec requester-account count and rolling 24-hour behavior, including midnight-crossing requests and combined amount boundary;
- Absa outbound same-recipient and inbound same-sender thresholds, unlimited aggregate count across distinct counterparties, and three different route-specific amount limits;
- Absa off-us receiving-bank lower limit, with the receiver profile itself required as evidence;
- Nedbank no-daily-count-cap behavior, per-request amount and 14/45-day expiry boundaries;
- unknown participant handling, wrong-direction rule application, calendar-day substitution and false-positive appeal review.

All amounts use integer ZAR minor units. Requester and counterparty identities are synthetic URNs. No customer transactions or personal data appear.

## Hard gates

- 0 count/amount/expiry rules inherited across participants;
- 0 account-wide count treated as per-recipient, or vice versa;
- 0 inbound/outbound inversion;
- 0 rolling-24h windows implemented as calendar-day resets;
- 0 on-us account / on-us ShapID / off-us collapse;
- 0 receiver lower-limit claims without a current receiver profile;
- 0 unknown limits converted to a rejection or source-backed no-cap;
- 0 aggregate caps inferred from per-request amount × count;
- 100% decisions bind the exact applied rule, scope and prior count/amount evidence;
- 100% rejected requests with an appeal state receive explicit disposition.

## Industry comparison and targets

Initial baseline: source-backed public Capitec, Absa and Nedbank rules over the same frozen synthetic scenarios. The objective is not the most permissive bank or the highest request throughput. It is correct participant-scoped enforcement with minimal wrongful exclusion.

Targets:
- **100%** correct applied-rule selection across the frozen corpus;
- **0** cross-participant inheritance or unknown-limit rejection;
- **0** direction, address-type or counterparty-scope inversions;
- **0** rolling-window/calendar-day substitutions;
- **100%** reproducible deterministic replay;
- later, with authorised participants: false-positive and false-negative limit-decision rates, appeal reversal, legitimate merchant requests blocked, payer harassment and p50/p95/p99 rejection latency.

Do not optimise acceptance or throughput without measuring scams, spam, payer pressure and legitimate small-merchant exclusion.

## Claim ceiling

A passing evaluator proves only conformance of this synthetic source register and reference model. It does not establish an individual account's live limits, a universal scheme rule, actual bank enforcement or real-world fairness.
