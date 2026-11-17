# IPB-S6 v0.2 — authority and external-operation red-team seed

**Status:** synthetic development seed; not independently adjudicated or frozen.

## Scope

v0.1 established a useful claim-separation and bitemporal foundation. v0.2 adds adversarial boundaries that v0.1 did not establish: issuer trust, proof validity, subject/scope binding, revocation races, unknown status, source competence, common-cause dependencies, digest mismatch, and delayed external-operation receipts.

The corpus intentionally keeps these separate. A valid signature is not an approved issuer. An approved issuer does not prove claim truth. A current source does not become competent for every claim dimension. A timeout is not failure.

## Scenarios in this seed

- S6R-001: proof valid, issuer untrusted.
- S6R-002: delegation revoked after request but before execution.
- S6R-003: revocation/status service unavailable.
- S6R-004: external operation may have completed before timeout.
- S6R-005: delayed success receipt reconciles the original operation.
- S6R-006: two source families share one upstream provider.
- S6R-007: authentic custody source is used outside its claim competence.
- S6R-008: cached source digest does not match retrieved bytes.

## Qualification dimensions

Report separately: proof integrity; issuer trust and governance; subject/entity binding; action/asset/purpose/time scope; validity and revocation state; source competence; digest/version consistency and freshness; common-cause independence; operation reconciliation and idempotency; correct abstention and reason codes.

Do not collapse these into one trust or risk score.

## Required fault-injection extensions

Expand to key rotation, key compromise, stale revocation caches, revocation notification delays, clock skew, event replay, duplicate event IDs, idempotency collisions, out-of-order receipts, duplicate completion receipts, partial source outages, provider concentration and purpose-bound disclosure.

For each timeout, inject: (a) request never reached provider, (b) provider completed but response was lost, (c) provider rejected but response was lost, (d) delayed success receipt, and (e) conflicting receipts. Correct behaviour must never infer success or failure from timeout alone.

## Prospective hard gates (targets, not results)

- Zero acceptance of untrusted, revoked, expired, not-yet-valid or scope-mismatched authority on hard negatives.
- Zero blind retries after unknown outcomes.
- Zero duplicate side effects under timeout, replay and delayed-receipt injection.
- Zero cross-dimension promotion from evidence outside its declared competence.
- Zero claims of independent corroboration where sources share a common upstream dependency.
- 100% of relied-on claims preserve source identity, competence, version/digest state, effective time, knowledge time and limitations.

## Comparator protocol

Compare a manually adjudicated reference workflow, a conventional policy/credential workflow configured for the task, a direct LLM workflow, and Mycelix composition. Match evidence, authority scopes, retry semantics, time budgets and failure injection. Record exact build/configuration/corpus versions.

## Claim ceiling

All cases are fictional. The oracle is an engineering reference, not legal adjudication or credential certification. A real trust framework requires governed trust roots, key lifecycle, privacy analysis, competent issuers, authoritative status checking and independent security review.
