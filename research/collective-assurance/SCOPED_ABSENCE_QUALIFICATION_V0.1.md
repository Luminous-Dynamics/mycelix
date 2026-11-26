# Scoped Absence Qualification v0.1

**Status:** Rust reference validator and synthetic fixture. No production authorization service, independently authenticated receipt verifier, real benchmark result or universal non-existence claim is asserted.

## Why this matters

A retrieval miss is not proof of absence. Open-world knowledge-base research treats completeness, recall and negation as separate problems: knowledge bases are often incomplete, and absence of a stored fact cannot automatically be interpreted as a negative fact. See Razniewski et al., *Completeness, Recall, and Negation in Open-World Knowledge Bases* (ACM Computing Surveys 2024; arXiv:2305.05403).

This also connects to distributed-information failures reported by HiddenBench and hidden-context investigation failures in PATHWAYS: agents can converge on shared information while critical knowledge remains undiscovered, or claim an investigative basis for evidence they never actually accessed.

## Claim types

The validator supports two distinct claims:

- **No evidence in declared scope:** every source in a complete, enumerated, authorized source universe was actually searched and explicitly returned no match.
- **No verified evidence in declared scope:** the same complete search was performed, and any candidate evidence returned was explicitly rejected as unverified; no candidate has unknown verification status and none is verified.

These are not interchangeable. A rejected candidate disproves "no evidence found" but can support "no verified evidence found" when the rest of the universe is completely searched.

## Required source dispositions

Each source is represented by an opaque source-reference digest and a disposition receipt. The validator distinguishes:

| Permission / query state | Meaning | May support an absence claim? |
|---|---|---|
| Authorized + searched + no match | Completed search | Yes, if the entire universe is closed and complete |
| Authorized + candidate verified | Verified evidence found | No |
| Authorized + candidate rejected | Unverified candidate exists | No evidence: no; no verified evidence: potentially yes |
| Authorized + candidate verification unknown | Candidate unresolved | No; INDETERMINATE |
| Rights withheld | Information may be restricted | No; INDETERMINATE |
| Permission unknown | Authorization cannot be established | No; INDETERMINATE |
| Delivered, no response | No answer received | No; INDETERMINATE |
| Not delivered | Query did not reach the source | No; INDETERMINATE |
| Not queried | Source was never searched | No; INDETERMINATE |
| Incomplete universe/pool | Search closure is unproven | No; INDETERMINATE |

A request created is not a request delivered. A delivered query with no response is not a completed no-match search. A source that was not queried cannot be treated as evidence that no fact exists.

## Structural contract

The request binds:
- request, recipient and purpose;
- query and subject/scope digests;
- frozen history cutoff;
- exact source-universe digest and declared source count;
- candidate-pool digest;
- independent completeness receipts for source universe and candidate pool;
- inquiry-plan digest and inquiry execution receipt.

The source-universe digest is computed over sorted opaque source-reference digests using a versioned, domain-separated SHA-256 encoding. Every declared source is accounted for exactly once. Each source disposition is receipt-bound. A query receipt is mandatory when a search was attempted.

The public report contains only the frozen scope digest and aggregate counts. It never emits raw source identities or evidence content. This is important for rights-withheld and unknown-permission states; even existence metadata can be sensitive and must follow the policy governing that scope.

## Fail-closed semantics

- Malformed identity/digest, duplicate sources, contradictory query/match/verification state, or source-universe digest substitution is FAIL.
- Missing/incomplete source-universe or candidate-pool attestations, source-count mismatch, rights withholding, unknown permission, no response, undelivered query, unqueried source or unknown match/verification is INDETERMINATE.
- PASS for no evidence requires a non-empty closed universe, exact source accounting, every source authorized and searched, and a no-match result from every source.
- PASS for no verified evidence permits explicitly rejected candidates but no verified/unknown candidates and still requires every source to be authorized and searched.

A PASS is restricted to the declared source universe, request, recipient, purpose, query and cutoff. It does not imply that no evidence exists outside that universe or that upstream receipt producers are honest.

## Integration ownership

- #6089 retains source-holder discovery, inquiry, rights, delivery, response and stopping.
- #6619 retains the evidence ledger and provenance.
- #6688 retains bounded RER and candidate/index completeness.
- #6678 retains benchmark/evaluator qualification.
- #6700 retains generation-input exposure.
- This module owns only the semantics and structural qualification of scoped negative claims.

No alternate memory store, source registry, authorization engine or semantic truth oracle is added.

## Fault-injection programme

The Rust test corpus covers:
- closed authorized empty search;
- verified candidate;
- rejected candidate under both claim types;
- unknown candidate verification;
- withheld and unknown permission;
- no response, undelivered and unqueried sources;
- incomplete source universe and candidate pool;
- omitted and duplicate source records;
- source-universe digest substitution;
- inconsistent match/verification state;
- privacy-safe aggregate reporting.

The expanded benchmark should add incomplete source-holder inventories, stale/future search receipts, index truncation, source substitution, source-disposition replay, misleading negative summaries and prompt-injected requests to overstate absence.

## Proposed targets (not achieved results)

- 100% rejection or INDETERMINATE classification for every deterministic closed-scope fault in the preregistered corpus.
- Zero false PASS absence claims under withheld/unknown/nonresponse/undelivered/unqueried/incomplete-source conditions in the synthetic corpus.
- 100% exact accounting of the declared source universe, with no duplicates or omitted source identities.
- Zero source identity/content leakage in public restricted/unknown reports.
- Publish false-absence and false-presence rates with group-clustered confidence intervals on sealed benchmark and adversarial sets.

Zero synthetic failures are not proof of zero deployment risk. Human studies require separate ethics approval, accessible consent and preregistration.
