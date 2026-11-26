# Audience × Applicability × Disclosure v0.1

**Status:** pure Rust reference qualifier and JSON Schema/fixture on a research branch. This is not an authorization engine, identity provider, memory store, redaction transform, production policy gate or authenticated receipt service.

## Why three boundaries?

For an entry and a concrete user-agent pair, evaluate three independent predicates:

Eligible(entry, pair, situation, policy) = Audience(entry, pair) AND Applicable(entry, pair, situation) AND Disclosable(entry, pair, policy)

A permission to read does not establish that a lesson applies. A lesson's usefulness does not grant permission to read it. A disclosure profile must not be inferred from either of those decisions.

## Selector algebra

- **Pair:** exact user + exact agent.
- **User:** one user across that user's agents.
- **Agent:** one agent across users in the declared collective.
- **Global:** all pairs in one exact, named collective.

A list of selectors is an explicit union within that one dimension. Eligibility is the intersection of the three dimensions. Empty lists mean missing evidence and return **INDETERMINATE**; they never mean wildcard or global. Global is still bound to a collective ID.

Applicability rules additionally bind an exact situation digest. Disclosure rules bind an exact policy digest. The current implementation uses exact string/digest equality; it does not infer semantic equivalence or group membership.

## Protected source stream and projection

A protected source cannot be consumed directly by this reference profile. It needs a projection receipt that binds the exact entry ID and revision, source and projected payload digests, transform commitment, target selector, situation and disclosure-policy digests, independent-review commitment, and issuance/expiry times.

The validator recomputes a domain-separated digest across these fields. This is only structural self-consistency. It does not authenticate the issuer, prove that a transform removed every secret, or establish that the reviewer was independent or authorized. Those require upstream authenticated authority and runtime observation.

## Dispositions

- **PASS:** all three scope predicates match, and protected content has an exact, structurally valid, in-window projection receipt.
- **FAIL:** a known mismatch, malformed scope, expired/replayed-to-another-context projection, or protected source without a projection is observed.
- **INDETERMINATE:** any required scope dimension is absent.

No aggregate score can offset an unauthorized disclosure. Report each predicate and outcome independently.

## Tests in this slice

The Rust test module covers Pair/User/Agent/Global matching, collective binding, the non-implication of audience → applicability and applicability → disclosure, missing-evidence handling, protected-stream projection, receipt mutation, target mismatch, wildcard/duplicate selectors and expiry.

This is a first deterministic unit suite, not the complete 64-case seeded panel in #6735. The 64-case corpus, independently authored oracle, source/user/agent/task-family holdouts and real benchmark adapters remain subsequent work.

## Integration boundaries

- #6710/#6711 own task-graph applicability and direct consumer handoffs.
- #6719/#6734 own transitive lineage, scope non-amplification and revocation closures.
- #6730 owns longitudinal metadata/existence leakage.
- #5460 owns generic observable-trace leakage and observer profiles.
- #6089 and #2824 own upstream source/rights/disclosure authority.

This module intentionally does not duplicate those engines. The caller must provide current, authenticated facts from their owner-native interfaces.

## Claim ceiling

A PASS qualifies only the exact in-memory declarations and projection commitment in this finite reference profile. It does not prove real identity, consent, policy authority, secret-free redaction, source truth, production enforcement, cache/index completeness, physical deletion, model unlearning, absence of side channels or legal compliance.
