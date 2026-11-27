# Temporal Frontier Index Forest V0.1

Status: structural composition prototype. This is not a trusted checkpoint verifier, an assertion-semantic mapper, a source-completeness verifier, or production authority.

## 1. Purpose

Compose per-owner authenticated temporal interval indexes with the verifier-relative causal frontier.

A conventional temporal query answers which rows match a valid-time point. A verifier-relative query must also say which owner histories this verifier had accepted, through which owner-local cuts, under which policy epoch, and whether all required owners were covered.

The forest preserves one temporal-index root per owner. Owner identifiers are sorted for canonical encoding only; no cross-owner causal order is inferred from that sort.

## 2. Data model

A `TemporalFrontierIndexManifestV1` binds verifier identity, policy epoch, accepted frontier digest, interval-index profile, common query scope, and the canonical ordered owner binding list.

Each `OwnerIntervalIndexBindingV1` binds:

- owner reference and stable owner-history identity;
- owner-local sequence cut;
- receipt-log tree size, measured in Merkle leaf positions;
- owner receipt-log root digest;
- temporal interval-index root;
- projection-audit digest;
- explicit inclusive unresolved owner-sequence gaps.

### Owner sequence and receipt-log tree size

These are distinct coordinates and must never be treated as identical values. However, the current V1 owner-log contract has one receipt-log leaf for each owner sequence that is not declared as a gap. Thus the structural invariant is:

```
TreeSize + total_gap_cardinality = OwnerSequenceCut
```

For a receipt at owner sequence `s`, its zero-based log leaf index is:

```
leaf_index = s - 1 - number_of_gap_sequences_strictly_before(s)
```

The forest validates this cardinality and mapping contract. It does not assert that a source-chain sequence number itself is a Merkle index; the map is derived through explicit gaps. The owner log verifier must still authenticate that the actual leaf sequence corresponds to the signed owner tree head.

Owner entries must be strictly sorted by UTF-8 owner reference, without duplicates. Gap ranges must be positive, within the owner sequence cut, sorted, disjoint and non-adjacent (adjacent gaps have one canonical merged representation). Resource limits are 100,000 owners and 100,000 gaps per owner.

## 3. Canonical forest commitment

All integers use fixed-width big-endian encoding. Strings use UTF-8 with a u32be byte-length prefix. The hash is SHA-256.

The forest preimage consists of:
1. one-byte tag `0x40`;
2. domain `MYCELIX-TEMPORAL-FRONTIER-INDEX-FOREST-V0.1\0`;
3. verifier, policy epoch, frontier digest, interval profile, namespace, subject and claim strings in that order;
4. u32be owner count;
5. each strictly sorted owner binding: owner reference, owner-history reference, u64be owner sequence, u64be log tree size, owner log root digest string, 32-byte interval-index root, projection-audit digest string, u32be gap count, then each gap's u64be first and last sequence.

The forest root commits to the owner/index mapping. It is not itself authenticated. A caller must obtain the expected forest root and exact required owner set from a separately verified checkpoint/frontier layer.

## 4. Verifying a declared forest query

`VerifyTemporalFrontierQueryProofV1` requires:

- expected verifier and policy epoch;
- expected accepted-frontier digest;
- exact temporal query scope and point V;
- the exact owner set from the separately verified frontier;
- the expected forest root from the authenticated checkpoint;
- one interval proof for every owner binding, in canonical order.

The verifier rejects missing or substituted owners, duplicate/non-canonical manifest entries, wrong owner proof order, scope/frontier/profile substitution, inconsistent owner sequence/tree-size/gap cardinality, malformed roots, and failed per-owner interval proofs.

It unions matching interval records while preserving owner identity, history identity, owner sequence and log tree size. Cross-owner sequence values are never compared; concurrent cuts remain concurrent.

The only positive disposition at this layer is:

`CompleteForDeclaredIndexForest`

It means that every result relative to the exact declared owner-index roots was proved. It is not a source-completeness or accepted-frontier completeness result.

## 5. Full owner-log projection replay

`projection.go` verifies the entire owner-log leaf sequence, not merely the verifier-receipt subset. This distinction follows the owner-log construction used by #6727: ordinary owner-history events may occupy log positions, while canonical `MYCELIX-KNOWLEDGE-RECEIPT-V1` entries replace particular positions when verifier-acceptance receipts are present.

The inventory contains every leaf in order:
- zero-based `LeafIndex`;
- owner-local `OwnerSequence`;
- exact `EntryBytes`.

The verifier:
1. checks owner and history identity, owner sequence cut, tree size and claimed log root against the forest binding;
2. requires exactly `TreeSize` leaf entries with contiguous indices;
3. checks owner-sequence order and sequence-to-leaf mapping using the explicit gaps;
4. recomputes canonical receipt digests and the RFC 9162 SHA-256 Merkle Tree Hash over *all* owner-log leaves;
5. recognizes canonical knowledge-receipt leaves and requires exactly one typed projection disposition for each;
6. retains non-receipt owner-log leaves in the Merkle root and counts them separately;
7. rebuilds the temporal interval index from the supplied typed projections and checks its root;
8. commits the exact full log-leaf sequence, typed projection inputs, interval root, counts and gaps into deterministic inventory/audit digests.

A receipt leaf without a temporal or non-temporal projection is rejected. A non-receipt log leaf cannot carry a receipt projection. Malformed lookalike knowledge-receipt prefixes are rejected rather than treated as ordinary events.

### Claim ceiling

The replay disposition is `ProjectionMatchesSuppliedAcceptedInventory`. The result reports:

`ReceiptInventoryAuthentication = LogRootRecomputedButTrustExternal`

This establishes that the supplied ordered bytes reconstruct the claimed owner-log root. It does **not** authenticate the signed checkpoint/tree head or prove that the expected root is accepted by the verifier. That remains the responsibility of the existing signed-checkpoint/key/witness/frontier owners.

The result also reports:

`ProjectionMappingAuthentication = NotEvaluated`

The mapping from the canonical assertion digest/payload to `valid_from`, `valid_to` and the temporal/non-temporal classification is a separate semantic proof obligation. The current module accepts the explicit typed projection as input; it does not independently decode the assertion's domain payload or prove that those intervals are the correct interpretation.

The audit digest can be committed into the owner binding and forest root. It must not be promoted to a source-completeness theorem until the checkpoint/root trust and assertion-to-interval semantic mapping are separately qualified.

## 6. Composed forest projection replay

`forest_projection.go` runs the per-owner projection replay for the exact owner set and forest root supplied by the separately verified frontier/checkpoint layer.

It returns `ProjectionMatchesAllSuppliedOwnerInventories`, with:
- `ReceiptInventoryAuthentication = LogRootRecomputedButTrustExternal`;
- `ProjectionMappingAuthentication = NotEvaluated`.

This confirms that all declared owner interval roots match their supplied full owner-log inventories. It does not authenticate the checkpoint or prove the supplied forest is the complete accepted frontier. Missing owners are rejected against the external expected owner set.

The interval query verifier continues to return `ProjectionCompletenessNotEvaluated`; it cannot infer source completeness from the replay digest alone.

## 7. SOTA boundary

The forest is not a new Merkle structure claim. VeriTS (Yao et al., 28 August 2026 preprint) already presents authenticated aggregate interval trees, minimum-covering-set completeness, boundary records and sliding-window delta proofs for per-source ordered time-series streams.

The proposed research differentiator is the composition of:
- valid-time interval semantics and retrospective correction lineage;
- exact verifier/policy accepted frontier;
- owner-local cuts and explicit causal concurrency;
- full owner-log history including ordinary non-receipt events;
- per-owner authenticated temporal indexes;
- recomputed receipt-log roots and separately qualified assertion-to-interval projection;
- replayable query certificates that preserve uncertainty and provenance.

The temporal index must still be benchmarked against VeriTS-style AIT/MCS proofs, authenticated Merkle B+ trees, conventional interval indexes and applicable 2D authenticated range structures. Do not claim that a custom interval tree alone beats SOTA.

## 8. Formal and qualification status

The Go module is a structural reference implementation and adversarial test harness. The current branch includes a bounded TLA+ model at `tools/economic/temporal-range-index/formal/TemporalIntervalProof.tla` and its configuration. The model has been authored but TLC has not been run; no model-checking result is claimed.

The current prototype does not implement:
- signed checkpoint, key lifecycle or witness verification;
- authenticated frontier owner-set verification inside this module;
- assertion-payload-to-interval semantic mapping;
- canonical Rust implementation and cross-language parity;
- durable persistence or mutation authority.

No result establishes global absence, event truth, trusted time, legal effect, payment authority, or permission to mutate journals or external systems.

## 9. Standards references

- [RFC 9162 — Certificate Transparency Version 2.0](https://www.rfc-editor.org/rfc/rfc9162)
- [RFC 9942 — COSE Receipts](https://www.rfc-editor.org/rfc/rfc9942)
- [VeriTS: Verifiable Model-Enhanced Time-Series Queries on Blockchain Systems (arXiv, 28 August 2026)](https://arxiv.org/abs/2608.28318)
- [COSE Receipts for MMRs, draft-ietf-scitt-receipts-mmr-profile-00 (21 November 2026)](https://datatracker.ietf.org/doc/draft-ietf-scitt-receipts-mmr-profile/)

The MMR profile is an Internet-Draft, not an RFC. It concerns append-only log receipts, not temporal-index completeness or causal frontier composition.
