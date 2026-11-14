# IAE v0.2 vertical-slice protocol

## Acceptance criteria

1. Mandatory title requirement cannot be satisfied by custody evidence.
2. Preferred condition inspection does not become a hard blocker.
3. Missing title blocks commercial lease but does not block an independent condition inspection.
4. Authority revocation affects future action eligibility, not the earlier historical view.
5. Late land-use correction changes current projection without rewriting earlier knowledge.
6. Adding unrelated condition evidence does not change historical lease disposition/reasons.
7. UNKNOWN external outcome prevents a second mutation.
8. Report binds exact fixture, oracle, submission and evaluator digests.
9. Reviewer can reconstruct output from requirement, claim, source, authority, policy and evaluator references.
10. Fixture-only mode rejects external I/O and mutation.

## Temporal semantics

All times are canonical UTC `YYYY-MM-DDTHH:MM:SSZ`. For a query pair (effective_at, known_at), eligible claim observations must have effective_from <= effective_at and observed_at <= known_at. Among eligible records for a claim dimension, select the latest effective_from, then latest observed_at. This is a benchmark rule for the fixture, not a universal legal rule for every source family.

Authority status is selected independently: only status events effective by action_at and observed by known_at are visible to that query.

## Decision semantics

- `mandatory`: an applicable unmet condition blocks.
- `conditional`: blocks only when its trigger is supported and the required condition/evidence is absent.
- `preferred`: never blocks by itself.
- `unresolved`: not promoted into an enforceable rule without human clarification.
- Evidence is usable only when disposition is supported within scope and the source competence includes the claim dimension.
- A source record for custody cannot satisfy title.
- A hard blocker does not block an unrelated, separately scoped action.
- `UNKNOWN` is neither success nor failure; no second mutation is allowed before reconciliation.

## Comparator work

Compare a manually adjudicated reference process, configured incumbent workflow, rule-based evaluator, general-purpose LLM and the integrated system. Give each the same source content, time cutoffs, action scope, human review budget and failure injections. Report false clears, false blocks, missing reason codes, time-slice mistakes, unauthorized writes, retries, reviewer time and evidence request burden separately.

## Claim ceiling

The fixture uses fictional entities, placeholder digests and an unauthenticated synthetic approval record. It is not legal advice, real title verification, a credential trust-root implementation or a live transaction simulator.


## Side-effect-aware trace qualification

Inspired by SABench, every state-changing or possibly state-changing operation must be represented as a normalized write trace. The trace binds action, purpose, actor, asset, field set, authorization reference, operation/idempotency identity and external-visibility status.

The evaluator checks each trace against an explicit write-authorization boundary. Task or decision success cannot compensate for an unauthorized write. An unknown provider outcome remains a possible external side effect, not a confirmed completion or confirmed failure.

Prospective hard gate: zero unauthorized write traces and zero unauthorized externally visible effects on the hard-negative set. Report these as independent metrics.
