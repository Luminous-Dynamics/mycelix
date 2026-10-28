# NCF-ALLOC-4 — Identity splitting breaks bounded-influence assumptions

**Status:** independent Python model executed locally, with five tests passing. Rust, distributed, cryptographic identity/admission, Holochain, and production qualification remain unrun.

## Research question

Can a single adversarial principal multiply its allocation by presenting the same fixed underlying work through more shard IDs? This test holds the attacker's true backlog at 4,000 units and holds 15 honest identities at 4,000 units each, while the attacker splits its backlog across 1, 4, 8, 12, 15, 16, 24 or 32 IDs. Each virtual identity reports 20× its own split backlog. The total attacker backlog remains 4,000; the test doesn't invent additional real work.

Six policies are compared under the same 95,000-unit spendable budget: equal split, raw demand-weighting, 50% equal floor with raw claims, 2×/4× lower-median caps, and an idealized true-backlog oracle.

## Executed results

| Attacker IDs | Total IDs | Equal-floor raw attacker allocation | Equal-floor 2× cap | Equal-floor 4× cap | Demand served, raw equal-floor | Attacker stranded capacity, raw equal-floor |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 16 | 31.70% | 9.01% | 13.65% | 100.00% | 26,112 |
| 4 | 19 | 39.10% | 27.92% | 36.33% | 96.65% | 33,144 |
| 8 | 23 | 45.97% | 43.21% | 45.97% | 86.45% | 39,669 |
| 12 | 27 | 50.80% | 50.80% | 50.80% | 79.28% | 44,260 |
| 15 | 30 | 53.58% | 53.58% | 53.58% | 75.16% | 46,900 |
| 16 | 31 | 54.38% | 54.38% | 54.38% | 73.96% | 47,665 |
| 24 | 39 | 59.34% | 59.34% | 59.34% | 66.60% | 52,377 |
| 32 | 47 | 62.63% | 62.63% | 62.63% | 61.73% | 55,495 |

Five local tests passed. The model confirms a significant boundary: caps help a single fixed ID, but as the same 4,000-unit work is split across more IDs, each report falls below the cap threshold while the equal-floor allocation multiplies. Once 12 attacker IDs are present in this profile, the 2× and 4× policies produce the same attacker share as the uncapped equal-floor policy. At 16 attacker IDs versus 15 honest identities, attackers control the lower median; capping no longer reduces the allocation share at all. At 32 IDs, one underlying attacker captures 62.63% of rights and strands 55,495 capacity units.

## Paradigm shift

This falsifies a tempting but unsafe policy claim: **a median cap alone is not manipulation resistance when identity creation is cheap.** The right control point is eligibility and identity, not a more clever cap multiplier.

The next design should bind an allocation subject to an externally governed principal/organization identity or explicitly scarce admission token, while preserving plural community governance. Demand evidence should be attached to a canonical eligible obligation digest and prevent the same underlying obligation from being counted under multiple shard IDs. Require deduplication scope, epoch, verifier/version, replay sequence, and auditable receipts. Privacy-preserving attestations can be researched later; they cannot substitute for defining who may claim the same work.

## Claim ceiling

This is a deterministic Python model on stable IDs. It does not model cryptographic identity, account admission economics, collusion-resistant governance, partition recovery, concurrent reservation writes, or Holochain. It is not production evidence. The Rust median-capped policy remains only a source/test definition until exact-head Rust qualification runs. No universal cap/floor is recommended.

## Reproduction

    python3 -m unittest -v test_ncf_alloc4_identity_splitting.py
    python3 ncf_alloc4_identity_splitting.py --output ncf-alloc4-identity-splitting-results.json
