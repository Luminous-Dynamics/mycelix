# SAP Reserve Valuation Snapshot Profile v0

**Status:** Design / qualification-boundary artifact  
**Qualification:** UNQUALIFIED  
**Related:** AC-093 (#4111), AC-107 (#4136), AC-108 (#4138), AC-109 (#4140), AC-110 (#4147), AC-102 (#4128)

## 1. Purpose

This profile separates the community price-discovery plane from the reserve valuation plane.

The existing price oracle is useful for community observation and operational analytics. It is not, by itself, a canonical reserve valuation authority.

A reserve issuance decision MUST consume an immutable, addressable valuation snapshot rather than invoking a mutable live-consensus query during issuance.

The target chain is:

`observation -> qualification -> aggregation -> immutable snapshot -> issuance authorization -> SAP`

The profile deliberately does not claim that the underlying observation is economically true, legally owned, sufficiently liquid, or externally time-synchronized.

## 2. Why the boundary exists

The current `price_oracle::get_consensus_price()` function combines computation with writes:

1. it reads a mutable link collection;
2. it selects current reports;
3. it may fall back to a previous consensus;
4. it updates reporter-accuracy state;
5. it creates a `PriceConsensus` entry;
6. it replaces the latest-consensus link;
7. it may trigger a TEND escalation side effect.

The reserve bridge currently calls this function while checking collateral valuation.

That creates three distinct authority problems:

- a monetary decision depends on a mutable query surface;
- the query itself can change persisted oracle state;
- the reserve decision does not receive a stable valuation identity that can later be proven to be the exact value consumed.

Holochain's validation model is intentionally deterministic: validation dependencies must be addressable, and mutable link collections are not suitable as validation evidence because their state can change over time. Addressable records retrieved by `must_get_valid_record` are the appropriate primitive for inductive validation. The source-chain model also gives each entry a signed creation action containing its author and timestamp.

## 3. Separation of planes

### 3.1 Community observation plane

The observation plane may remain open to qualified reporters.

A `PriceReport` records:

- canonical item;
- observed price;
- evidence reference;
- reporter identity;
- observed timestamp;
- report timestamp.

AC-109 binds reporter identity and report timestamp to the signed action.

### 3.2 Operational consensus plane

The existing `PriceConsensus` API may continue serving community analytics and TEND-oriented operations.

Its fallback behavior is an operational availability feature, not reserve evidence.

A degraded consensus MAY be shown to users or used by non-reserve analytics according to its own policy.

It MUST NOT be silently promoted to reserve valuation.

### 3.3 Reserve valuation plane

Reserve valuation consumes an immutable `ReserveValuationSnapshot` identified by its creation action hash.

The snapshot is a historical fact about a particular valuation computation under a declared profile. It is not a mutable "current price" record.

## 4. Canonical snapshot object

The eventual implementation SHOULD define a dedicated entry type equivalent to:

`ReserveValuationSnapshot`

Required semantic fields:

| Field | Requirement |
| --- | --- |
| `basis_id` | Immutable identifier for the valuation basis / issuance context |
| `asset_id` | Exact collateral or reserve instrument identifier |
| `unit` | Unit of measured quantity |
| `quote_unit` | Exact settlement/quote unit, e.g. SAP |
| `valuation` | Canonical fixed-point value; no direct float-to-μSAP conversion |
| `valuation_scale` | Explicit decimal/rational scale |
| `effective_window_start` | Start of source-observation window |
| `effective_window_end` | End of source-observation window |
| `source_commitment` | Canonical ordered commitment to the exact source records or certificate |
| `source_count` | Number of accepted independent sources |
| `aggregation_profile_id` | Frozen algorithm/profile identifier |
| `freshness_limit` | Maximum permitted age at consumption |
| `qualification_state` | Explicit qualified / degraded / unavailable state |
| `publisher_policy_id` | Authority policy used to publish the snapshot |
| `supersedes` | Optional prior snapshot reference |
| `created_at` | Snapshot action time; must be bound to the signed Create action |

A machine-readable schema manifest is published alongside this profile at `SAP_RESERVE_VALUATION_SNAPSHOT_PROFILE_V0.json`. The current implementation tranche freezes the structural fields and basic invariants; authority certificates and source-record revalidation remain subsequent steps.

## 5. Snapshot identity

The snapshot action hash is the primary historical identifier.

The issuance record MUST persist that identity rather than storing only a copied numeric rate.

Therefore:

`issuance -> snapshot_action_hash -> exact valuation artifact`

The following is insufficient:

`issuance -> copied median_price`

A copied rate without a source snapshot permits later ambiguity over which observations, methodology, freshness state, or authority produced it.

## 6. Source commitment

The source commitment MUST be independent of link-iteration order.

The current implementation shape uses a domain-separated BLAKE2b-256 digest over the sorted fixed-length Holochain action-hash bytes, prefixed by the canonical source-set count. This keeps the commitment bounded even at the maximum 200-source set.

The snapshot also carries the exact sorted action hashes. The digest is therefore a compact integrity commitment, not a substitute for the addressable source records.

The snapshot then references either:

1. the exact bounded set of report action hashes, or
2. an addressable quorum certificate that itself commits to the exact source set.

The source set MUST be bounded by policy. The current oracle advertises a maximum of 200 reporters per consensus window; the implementation currently preserves that ceiling for the snapshot source list and can choose a lower reserve-specific ceiling later.

No reserve verifier may infer source membership from the current contents or order of a mutable link collection.

## 7. Authority models

Two publication models are supported conceptually.

### 7.1 Qualified publisher

A specifically authorized valuation publisher creates the snapshot.

The publisher identity MUST be bound to the signed action and to an explicit valuation-authority registry/policy.

The empty registry MUST NOT imply authority.

The authority bootstrap problem is covered by AC-106.

### 7.2 Threshold-attested snapshot

A publisher creates the snapshot only after obtaining an explicit N-of-M attestation set.

Each attester signs or otherwise creates an addressable attestation bound to:

- snapshot basis;
- exact asset;
- exact source commitment;
- exact valuation result;
- profile version;
- freshness bounds.

The accepted threshold and attester set are part of the immutable snapshot evidence.

A bare count field such as `attester_count = 3` is not sufficient. The individual attestations or a cryptographically verifiable certificate must be resolvable.

## 8. Deterministic valuation

The reserve valuation layer MUST use the arithmetic profile frozen by AC-102.

Monetary conversion MUST NOT perform:

`f32/f64 -> cast -> μSAP`

Rates should be represented as canonical rational/fixed-point values with explicit rounding.

For any non-negative quantity:

`μSAP = round_policy(quantity × numerator / denominator)`

The rounding policy is part of the profile and snapshot identity.

Different rounding policies MUST produce different canonical profile identities; they cannot silently coexist under one snapshot schema.

## 9. Freshness semantics

Freshness is part of the reserve decision, not an informational field.

The verifier evaluates:

`consumption_action_time - effective_observation_time <= freshness_limit`

The exact time relationship MUST be based on action-addressable timestamps and the declared valuation policy.

Holochain action timestamps provide internal signed ordering, not independent external wall-clock truth.

Therefore the snapshot profile MUST NOT claim that an action timestamp proves when an off-chain market observation actually occurred.

External time synchronization, venue timestamps, or measurement timestamps require their own evidence and qualification layer.

## 10. Degraded state

A snapshot has an explicit qualification state.

Only `QUALIFIED` may be consumed for reserve-backed issuance.

The following states are non-issuable:

- `DEGRADED`;
- `UNAVAILABLE`;
- `UNRESOLVED`;
- `SUPERSEDED`;
- `EXPIRED`;
- `CONFLICTED`.

There is no "use the previous value" fallback at the reserve consumption boundary.

This preserves the distinction established by AC-107 and AC-108:

`oracle outage / degraded consensus -> reserve issuance suspended`

rather than:

`oracle outage -> caller or stale value -> issuance continues`

## 11. Supersession

Snapshots are immutable.

A new valuation can reference the prior snapshot using `supersedes`.

Supersession MUST NOT rewrite the historical valuation or silently change an issuance that already consumed it.

An issuance remains bound to its original snapshot unless a separate, explicit reconciliation process is defined.

## 12. Reserve consumption contract

The bridge SHOULD evolve from:

`deposit(collateral, claimed_rate)`

toward an input contract equivalent to:

`deposit(collateral, snapshot_action_hash, issuance_basis)`

The bridge then verifies, before any SAP issuance is committed:

1. snapshot exists and is valid;
2. snapshot action is the expected entry type;
3. snapshot is `QUALIFIED`;
4. snapshot is not expired/superseded for this basis;
5. asset and units exactly match the collateral;
6. source commitment satisfies the published policy;
7. aggregation profile matches the frozen reserve profile;
8. fixed-point valuation recomputes consistently;
9. the snapshot identity is carried into the typed mint authorization.

The caller MAY supply a claimed rate for local UX, but the caller value is comparison-only. It is never authority.

## 13. Relationship to collateral custody

Valuation does not prove custody.

The target chain remains:

`asset identity -> ownership/controller -> custody/control -> encumbrance -> valuation snapshot -> issuance`

AC-103 owns custody/control evidence.

The reserve value of an asset with no qualified custody/control proof remains zero for strict reserve accounting, regardless of how strong its price snapshot is.

## 14. Relationship to supply conservation

A valuation snapshot answers:

"What qualified valuation artifact was consumed?"

It does not answer:

"Was SAP issuance unique and conserved?"

That remains the role of AC-092, AC-095, AC-099, and AC-105.

The intended complete chain is:

`qualified collateral`
→ `qualified valuation snapshot`
→ `canonical issuance authorization`
→ `recipient-owned balance transition`
→ `supply finality`

## 15. Prohibited reserve shortcuts

The reserve path MUST NOT:

- call the mutable `get_consensus_price()` function as its final authority;
- accept a stale previous consensus because fresh reporters are unavailable;
- infer source completeness from link iteration;
- use a free-form evidence string as sole source proof;
- treat `median_price` alone as valuation provenance;
- treat `reporter_count` alone as a quorum certificate;
- accept a caller-selected rate when a snapshot is missing;
- overwrite a historical snapshot to reflect a later market revision;
- omit the consumed snapshot identity from issuance provenance.

## 16. Required adversarial corpus

At minimum:

1. snapshot hash points to a wrong entry type;
2. snapshot record is invalid or unavailable;
3. asset identifier substitution;
4. quote-unit substitution;
5. quantity-unit substitution;
6. source commitment reordered without semantic change;
7. source commitment changed while digest remains stale;
8. source report replaced by another valid report;
9. source cardinality below threshold;
10. duplicate source counts twice;
11. correlated sources represented as independent;
12. degraded/fallback state marked as qualified;
13. expired snapshot;
14. superseded snapshot consumed for a new issuance;
15. profile-version substitution;
16. rounding-policy substitution;
17. fixed-point overflow;
18. float-derived legacy rate injected into a new issuance;
19. snapshot identity omitted from mint provenance;
20. caller-supplied rate differs from snapshot;
21. two valid snapshots exist and issuance does not bind which one it consumed;
22. snapshot publisher is not authorized;
23. empty authority registry treated as authorization;
24. invalid threshold certificate;
25. historical snapshot mutated after issuance.

A future qualification harness SHOULD execute these against the exact candidate head and frozen runtime manifest.

## 17. Implementation sequencing

This profile intentionally stages the work.

### Stage A — preserve observations

Keep community reporting and operational consensus functionality available.

AC-109 makes observation identity and action-time relationships integrity-enforced.

### Stage B — introduce immutable snapshots

Add the dedicated snapshot entry type, canonical source commitment, fixed-point valuation, freshness, qualification state, and authority/certificate binding.

### Stage C — make reserve issuance consume snapshots

Change collateral issuance to require a snapshot action hash and carry it into the canonical mint authorization.

The reserve path no longer calls the mutable live consensus function for final authority.

### Stage D — close remaining monetary ordering gaps

Complete:

- AC-092 typed balance provenance;
- AC-095 canonical issuance serialization;
- AC-096 confirmed-collateral issuance ordering;
- AC-097 redemption conservation;
- AC-099 owner-authenticated balances;
- AC-103 custody/control evidence;
- AC-105 issuance idempotency;
- AC-106 governance bootstrap root.

Only after these boundaries converge should a reserve qualification attempt be executed.

## 18. Qualification ceiling

A successful AC-110 qualification can establish:

- an issuance consumed a specific immutable valuation snapshot;
- the snapshot's declared source commitment and policy were the ones verified;
- the snapshot was non-degraded and within its declared freshness bounds;
- the valuation arithmetic was deterministic under the frozen implementation;
- the snapshot publisher/certificate satisfied the frozen authority rule.

It does NOT establish:

- truthfulness of off-chain market observations;
- honesty of reporters or attesters;
- legal ownership of collateral;
- collateral custody;
- market liquidity;
- stable purchasing power;
- real-world time synchronization;
- official monetary or reserve-currency status.

## 19. Research references

- Holochain validation guidance: validation dependencies must be addressable and deterministic; mutable link collections are not suitable validation evidence.
- Holochain source-chain guidance: entry records are paired with signed creation actions; concurrent writes to one source chain are serialized and multi-record writes can commit atomically.
- BIS, *The next-generation monetary and financial system* (2025): monetary architecture is evaluated through singleness, elasticity, and integrity; tokenised central bank reserves, commercial bank money, and government bonds form a stronger institutional backbone than privately issued stablecoins.
- FSB, *High-level Recommendations for the Regulation, Supervision and Oversight of Global Stablecoin Arrangements* (2023): emphasizes robust legal claims on reserve assets, timely redemption, stabilisation mechanisms, and appropriate prudential/risk controls.

## 20. Final boundary

The reserve system is not allowed to collapse these distinct statements:

`a report exists`

`a consensus can be computed`

`a valuation snapshot is qualified`

`a reserve asset is controlled`

`SAP was issued`

`the issuance is conserved and redeemable`

Each requires its own evidence.

Until the final implementation chain is executed and independently qualified, the SAP reserve claim remains **UNQUALIFIED**.
