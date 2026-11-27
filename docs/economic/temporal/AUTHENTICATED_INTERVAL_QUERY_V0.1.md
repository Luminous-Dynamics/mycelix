# Authenticated Temporal Interval Query V0.1

Status: research specification and implementation plan. This profile is deliberately separate from RFC 9162 append-only logs and the experimental sparse assertion map.

## 1. The next query primitive

The current Economic OS evidence stack distinguishes:

- business validity time;
- verifier identity and accepted causal frontier;
- append-only receipt inclusion / consistency;
- exact assertion-reference membership in an experimental sparse map.

The unresolved query is stronger:

> For this exact verifier, policy epoch, namespace, subject, claim and accepted source frontier, return every assertion whose validity interval contains business instant V, and prove that no matching assertion in this exact index root was omitted.

An exact-key sparse Merkle proof cannot answer this. Hashing the lookup key destroys temporal order, and proving one assertion reference absent does not establish that no other assertion for the claim is active.

This increment owns only the authenticated interval-query data structure. It composes with the existing temporal assertion, absence/completeness, causal-frontier, checkpoint and witness owners; it does not create another time service, log, witness quorum, or global absence framework.

## 2. Normative query semantics

V0.1 uses a point query over exact normalized UTC instants. An interval is half-open:

```
matches(interval, V) :=
    interval.valid_from <= V
    && (interval.valid_to == OPEN || V < interval.valid_to)
```

Therefore:

- the exact start endpoint is included;
- the exact finite end endpoint is excluded;
- an open end has no finite upper boundary;
- nanoseconds are preserved; no implicit millisecond truncation is allowed.

The query scope is the exact tuple:

```
verifier_ref
policy_epoch_ref
namespace_ref
subject_ref
claim_id
source_frontier_digest
```

A point query additionally supplies V as a signed 64-bit Unix seconds value plus an unsigned nanoseconds value in [0, 1_000_000_000).

V0.1 does not coerce calendar dates, uncertain intervals, uncalibrated clocks, or unknown endpoints into exact instants. Those require a separately versioned profile and otherwise produce Indeterminate.

## 3. Result and claim ceiling

The low-level verifier may return only:

- `CompleteIntervalResultForRoot`: all matching intervals in this exact index root and exact scope were returned;
- `CompleteEmptyIntervalResultForRoot`: no matching interval exists in this exact index root and exact scope.

It must return `Indeterminate` for an unanchored or mismatched root, malformed proof, unsupported profile, unknown precision, scope mismatch, or unresolved evidence needed by a higher-level source-completeness policy.

A complete empty result is **not** universal non-existence. It does not prove:

- that every accepted receipt was projected into the index;
- that the index's source frontier is authentic or current;
- that a fact was never known by a verifier;
- that the underlying event or assertion is true;
- legal non-liability, non-ownership, or non-eligibility;
- permission to mutate a journal, payment, or external system.

## 4. Candidate V1 authenticated interval tree

### 4.1 Canonical record

An interval record contains:

- non-empty, bounded UTF-8 `assertion_ref`;
- `valid_from = (seconds: i64, nanos: u32)`;
- either `valid_to = Finite(Instant)` or `valid_to = Open`;
- lower-case `sha256:` receipt/evidence digest.

Finite intervals must satisfy `valid_to > valid_from`. Duplicate assertion references within one exact scope are rejected. Records are ordered lexicographically by `(valid_from, assertion_ref UTF-8 bytes)`. Equal starts are therefore deterministic.

### 4.2 Authenticated subtree summary

Every non-empty subtree commits to:

- `count`: number of descendant records;
- `min_valid_from`: minimum start instant among descendants;
- `max_valid_to`: maximum end instant among descendants, where Open is +infinity.

These summaries are authenticated as part of the node hash, not trusted as detached metadata.

A subtree is safely irrelevant to a point query at V only when one of these conditions is proved by its committed summary:

```
min_valid_from > V
OR
(max_valid_to is Finite && max_valid_to <= V)
```

The first condition proves every interval starts after V. The second proves every finite interval has ended by V. Open-ended descendants force `max_valid_to = Open` and cannot be pruned by the expiry condition.

If neither condition holds, the proof must expand that subtree. At a leaf, the exact interval predicate is checked and a matching record is returned.

### 4.3 Tree shape and hashing

V1 uses a deterministic balanced binary tree over the canonical sorted record sequence. The builder splits each non-leaf sequence at `floor(count / 2)`; both children are non-empty. The tree shape is therefore reproducible from the ordered records.

The Go reference prototype freezes this root hash profile. Every hash preimage begins with a one-byte tag followed by the exact domain bytes `MYCELIX-AUTHENTICATED-INTERVAL-QUERY-V0.1\\0`:

| Tag | Commitment |
|---|---|
| `0x30` | `SHA256(tag || domain || canonical_record)` leaf |
| `0x31` | `SHA256(tag || domain || u64be(count) || canonical_summary || left_hash || right_hash)` branch |
| `0x32` | `SHA256(tag || domain)` empty tree |
| `0x33` | `SHA256(tag || domain || canonical_scope || u64be(count) || tree_hash)` scoped root |

Canonical scope field order is verifier, policy epoch, namespace, subject, claim, source-frontier digest. Each is UTF-8 encoded with a u32be byte-length prefix. Canonical record order is assertion reference (same string encoding), valid-from instant, valid-to bound, receipt digest (same string encoding). An instant is i64be seconds followed by u32be nanoseconds. A bound is tag `0x00` plus an instant for Finite, or tag `0x01` alone for Open. A summary is minimum valid-from instant followed by maximum valid-to bound. The branch commits to its derived count and summary as well as ordered left/right child hashes.

No JSON serialization bytes are hashed. Hash comparisons are byte comparisons. The empty tree has a distinct domain-separated hash and is still wrapped in the scoped-root commitment; empty roots therefore differ across verifier, policy, namespace, subject, claim and frontier scopes.

The prototype currently verifies an in-memory proof tree. A canonical transport/wire encoding and bounded decoder are **not yet implemented**; the in-memory proof enum must not be mistaken for an interoperable VDP encoding.

### 4.4 Completeness proof

A proof tree has three node forms:

1. **Pruned subtree**: contains the authenticated subtree hash, count and summary. The verifier accepts it only if the summary proves one of the safe-pruning conditions above.
2. **Expanded branch**: contains both child proofs. The verifier derives count and summary from the children and reconstructs the branch hash.
3. **Returned leaf**: contains the canonical interval record. The verifier recomputes its leaf hash and requires that the interval matches V.

The verifier reconstructs the exact scoped root. A pruned node cannot hide a possible match because its authenticated summary must prove irrelevance. Every non-pruned branch is expanded until all matching leaves are returned. Consequently, a valid proof establishes completeness relative to the supplied root.

The proof contains no independent result list; returned records are derived from the verified proof tree, preventing a caller from appending unproved results. Reject malformed node variants, missing/extra children, duplicate assertion references, non-matching returned leaves, invalid summaries, overflow, and root mismatch.

This proof structure is intentionally simple and auditable. It may expand a large fraction of the tree for adversarial or highly overlapping interval distributions. That cost is a benchmark result, not something to hide with an optimistic asymptotic claim.

## 5. Source completeness is a separate theorem

The interval tree proves a complete answer only relative to its own committed leaves. The higher-level Economic OS query may call that result source-complete only after a separate verification step:

1. the interval root is bound into an authenticated verifier checkpoint;
2. the checkpoint identifies this exact profile/version, query namespace, verifier, policy epoch and source frontier;
3. the source frontier identifies every required owner cut and append-only receipt-log root;
4. an independent replay or qualified index-builder attestation establishes that every in-scope accepted receipt through that frontier was parsed and projected deterministically;
5. omitted, delayed, rejected, malformed, redacted, unavailable and unknown-source receipts remain visible in the coverage assessment;
6. checkpoint freshness, key lifecycle, witness agreement and non-equivocation are established by the existing CIV-009 / #5942 owners.

A self-declared `complete=true`, zero counter, successful index build, or valid interval proof is not evidence of source completeness. Until all required conditions hold, the higher-level disposition is Indeterminate.

Compose with #1289 for bounded absence semantics, #2631/#6670 for temporal assertions, #6680 for causal frontier semantics, #6727 for receipt inclusion/consistency, #6728 for exact-key index experiments, and the existing signed-head/witness owners. Do not duplicate those services.

## 6. Independent implementation and adversarial corpus

The first prototype is a dependency-free Go reference implementation under `tools/economic/temporal-range-index/reference/go/`. Its frozen four-record known-answer root is `a770c656c9afcafc3204006c5e8d217fc5ab38e66f90194e40c4cbdb3aabaac6`. The current tests exercise half-open endpoints, nanoseconds, open ends, scope/query/root substitution, unsafe pruning, malformed nodes, invalid intervals, empty roots, exact scan parity, deterministic tie-breaking and all-overlap results.

This is **one implementation**, not yet an independent cross-language qualification. A Rust implementation must reproduce the same canonical bytes, roots, proof outcomes and reason codes without importing or calling the Go candidate code. Fixed vectors must be independently derived before both implementations are considered conformant.

Frozen cases must include:

- exact start included; exact end excluded;
- open end; zero-length and reversed finite intervals rejected;
- nanoseconds immediately before, at and after a boundary;
- duplicate starts with stable assertion-reference ordering;
- adjacent, nested, overlapping, disjoint and all-overlap intervals;
- empty index and complete empty point query;
- all records expired and all records starting in the future;
- wrong verifier, policy epoch, namespace, subject, claim, frontier digest or query instant;
- changed record, digest, summary, count, child hash or root;
- missing/extra child, duplicate leaf, reordered leaf and non-matching leaf smuggled into a proof;
- false Open sentinel, finite-end overflow and malformed UTF-8/identifier lengths;
- malformed proof node with contradictory fields set simultaneously;
- root valid for a different checkpoint or stale source frontier;
- accepted source receipt omitted from the index;
- missing owner, declared sequence gap, delayed receipt, unresolved fork and correction branch;
- a benign empty result where the complete scoped index genuinely contains no candidate.

The last source-completeness cases must remain distinct from low-level interval-tree proof validity. They test the adapter, not just the data structure.

## 7. Benchmark programme

No performance result is claimed by this specification. The harness should compare:

- independent linear scan oracle;
- ordinary ordered index / conventional interval tree;
- augmented authenticated interval tree;
- authenticated Merkle B-tree / range-tree candidate;
- SQL Server temporal history and XTDB only through matched adapters and equivalent query semantics.

Scales: 1k, 10k, 100k and 1m intervals.

Distributions: uniform, clustered, nested, disjoint, all-overlap, heavily skewed starts/ends, duplicate starts and adversarial interleaved expiry.

Query selectivity: 0%, 0.001%, 0.1%, 1%, 10% and 100%.

Report build/update cost; proof-generation and proof-verification p50/p95/p99; proof bytes; expanded/pruned/visited node counts; memory and bytes per record; root recomputation; result cardinality; and exact oracle disagreements. Separate warm-cache and cold-cache runs, pin hardware and toolchain, and publish raw samples and fixture digests.

Hard gates:

- 100% independent-oracle parity;
- zero false-empty results;
- zero omitted matches;
- zero acceptance of malformed or scope-substituted proofs;
- proof-size and verification-cost results reported at every workload scale.

A provisional performance goal is p95 authenticated query cost within 3x of an ordinary in-memory interval index for low-selectivity queries at 100k records, while keeping zero false-empty results. This is a research target only; it is not a baseline result or a claim of beating SQL Server or XTDB. Large proof expansion, high overlap and adversarial distributions must be reported rather than excluded.

## 8. Formal qualification

A bounded state model should vary tree shapes, interval endpoint order, open ends, query points, proof pruning choices and one-node mutations. Required invariants:

- every returned leaf matches V;
- every matching leaf is returned;
- a pruned subtree contains no matching interval;
- proof reconstruction equals the exact scoped root;
- a malformed or scope-substituted proof never yields a complete disposition.

The formal model should compose with #6680's frontier model. This issue does not claim a TLA+/Alloy/model-checker result until the model is checked by its named independent tool and exact state bounds are published.

## 9. Industry and research baseline

This profile must be compared against mature temporal databases and established authenticated-data-structure techniques. It is not sufficient to beat a linear scan.

| Baseline | Existing strength | What this work must add / compare |
|---|---|---|
| SQL Server system-versioned temporal tables | Retain current/history row versions and support `FOR SYSTEM_TIME AS OF`, including `datetime2` precision | A signed, independently verifiable result certificate bound to verifier/frontier identity and accepted source coverage |
| XTDB | Built-in bitemporal system/transaction time plus valid time; supports point-in-time and range queries over closed-open periods | Causal-frontier qualification, evidence lineage and independently verifiable proof of complete interval-query results |
| Conventional interval tree / ordered index | Fast in-memory point-stabbing query without cryptographic proof overhead | Quantify the cost of authentication, proof construction, proof bytes and incremental root maintenance |
| Authenticated Merkle B-tree / ordered ADS | Established range-query completeness techniques, including boundary records and authenticated sibling subtrees | Benchmark as the principal cryptographic data-structure baseline; test interval summaries and open-ended validity semantics |
| Authenticated multidimensional range-query schemes | Research demonstrates authenticated range queries beyond simple point membership | Compare setup, proof size, query dimensions, update cost and verifier assumptions before claiming novelty |

SQL Server's documented `FOR SYSTEM_TIME` feature queries the system-versioned current/history tables; `AS OF` uses the declared `datetime2` precision and UTC interpretation. XTDB documents both system and valid time, with half-open periods and explicit `FOR VALID_TIME` / `FOR SYSTEM_TIME` query forms. These are temporal-semantics baselines; their documentation is not a claim that these products provide this proposal's verifier-relative causal-frontier proof contract.

Authenticated range-query research predates this prototype. In particular, ordered authenticated structures use boundary records and authenticated subtrees to establish that result sets contain no omissions or extraneous records. Multidimensional authenticated range-query schemes are also established research. The present augmented interval tree is therefore a candidate construction, not a claim of a new authenticated-data-structure primitive.

### What could actually move the field

The paradigm shift is not a different timestamp column. It is to make the answer itself carry independently checkable evidence:

```
valid-time interval proof
+ receipt inclusion / append-only consistency
+ verifier-accepted owner frontier
+ source-to-index projection completeness
+ signed checkpoint / non-equivocation
= verifier-relative temporal query certificate
```

The proof layers remain distinct. An interval proof must not claim log inclusion; an inclusion proof must not claim index completeness; a valid signed checkpoint must not claim the represented event was true.

The next architecture after this single-scope prototype is a **frontier-index forest**: each owner maintains an interval index rooted in its own accepted receipt prefix; the verifier's vector frontier commits to the required owner cuts and per-owner index roots. A query certificate carries one interval proof per covered owner, preserves concurrent owner histories as concurrent, and refuses a complete result if any required owner cut or projection coverage is unknown. This avoids manufacturing a global scalar event order or hiding missing owners behind one aggregate index root. It must compose with #6680 and the existing CIV checkpoint / witness owners.

A later dynamic authenticated B+ interval index is a serious competitor to the binary tree used here. Page fan-out (e.g. 8 / 16 / 32 / 64), persistence, incremental insert/delete, proof bytes and I/O count should be compared under the same fixed workload. A high-overlap workload may necessarily return large result sets; the performance target should be output-sensitive rather than promising constant-sized proofs.

## 10. Standards and interoperability route

RFC 9162 remains the append-only log proof layer: it supplies inclusion and consistency semantics, not arbitrary temporal interval-query completeness. RFC 9942 defines COSE receipt envelopes and VDS/VDP registries; the initial RFC 9162 SHA-256 profile does not make this custom interval tree an existing registered proof type.

The proposed interval profile must remain explicitly experimental. If it matures, specify exact proof encodings and register/standardize a VDS/VDP profile through the appropriate standards process. Do not label it RFC 9162 or claim RFC 9942 conformance prematurely.

## 11. Canonical wire encoding

The Go reference now provides a bounded V1 encoder/decoder in `tools/economic/temporal-range-index/reference/go/wire.go`.

- Frame magic: `MYCITPQ1`.
- Six exact query-scope strings, each UTF-8 with u32be length prefix.
- Query instant: i64be seconds, u32be nanoseconds.
- Proof record count: u64be.
- Node tags: `0x01` pruned subtree, `0x02` expanded branch, `0x03` returned leaf.
- A pruned node carries count, authenticated summary and subtree hash.
- A branch carries exactly its left and right children; its count, summary and hash are reconstructed.
- A leaf carries only its canonical assertion record; its summary/hash are reconstructed.
- A finite end is tag `0x00` plus an instant; an open end is tag `0x01` alone.

Decoder limits are 64 MiB per proof, 1,000,000 total records, 2,000,001 proof nodes, recursion depth 64 and 1,024 bytes per string. It rejects invalid UTF-8, malformed nanos/end tags, unsupported node tags, truncation, extra bytes and semantically malformed proof trees. The expected index root is intentionally not embedded in the wire frame; it must come from the separately authenticated checkpoint/frontier.

The wire format is an experimental Mycelix encoding, not a registered RFC 9942 VDP profile. Only the Go reference exists at this stage; Rust parity and interoperable profile registration remain outstanding.

## 12. Nonclaims

A passing tree verifier proves only a mathematically complete interval-stabbing answer relative to one supplied scoped root. It does not authenticate that root, establish the source frontier's completeness or truth, prove global absence, establish trusted time, or grant financial/operational authority.
