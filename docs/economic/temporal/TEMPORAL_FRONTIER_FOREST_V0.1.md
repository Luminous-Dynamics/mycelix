# Temporal Frontier Index Forest V0.1

Status: structural composition prototype. This is not a trusted checkpoint verifier, a source-completeness verifier, or production authority.

## 1. Purpose

Compose per-owner authenticated temporal interval indexes with the verifier-relative causal frontier.

A conventional temporal query answers which rows match a valid-time point. A verifier-relative query must also say which owner histories this verifier had accepted, through which owner-local cuts, under which policy epoch, and whether all required owners were covered.

The forest deliberately preserves one temporal-index root per owner. Owner identifiers are sorted for canonical encoding only; no cross-owner causal order is inferred from that sort.

## 2. Data model

A \`TemporalFrontierIndexManifestV1\` binds:

- verifier identity;
- policy epoch;
- accepted frontier digest;
- interval-index profile;
- common query scope;
- canonical ordered owner binding list.

Each \`OwnerIntervalIndexBindingV1\` binds:

- \`OwnerRef\`;
- stable \`OwnerHistoryRef\`;
- owner-local \`OwnerSequence\`;
- append-only receipt-log \`TreeSize\`;
- owner receipt-log root digest;
- temporal interval-index root;
- projection-audit digest;
- explicit inclusive unresolved gap ranges.

\`OwnerSequence\` and \`TreeSize\` are intentionally separate. A source-chain sequence is not a Merkle leaf count. The current prototype requires \`TreeSize <= OwnerSequence\`, but does not prove the relationship from owner-history receipts; that belongs to the frontier/log adapter.

Owner entries must be strictly sorted by UTF-8 owner reference, without duplicates. Gap ranges must be positive, within the owner sequence cut, sorted, disjoint and non-adjacent (adjacent gaps have one canonical merged representation). Resource limits are 100,000 owners and 100,000 gaps per owner.

## 3. Canonical forest commitment

All integers use fixed-width big-endian encoding. Strings use UTF-8 with a u32be byte-length prefix. The hash is SHA-256.

The preimage is:

1. one-byte tag \`0x40\`;
2. domain \`MYCELIX-TEMPORAL-FRONTIER-INDEX-FOREST-V0.1\\0\`;
3. verifier, policy epoch, frontier digest, interval profile, namespace, subject and claim strings in that order;
4. u32be owner count;
5. for each strictly sorted owner:
   - owner reference;
   - owner-history reference;
   - u64be owner sequence;
   - u64be log tree size;
   - owner log root digest string;
   - 32-byte interval-index root;
   - projection-audit digest string;
   - u32be gap count;
   - each gap's u64be first and last sequence.

The forest root commits to the owner/index mapping. It is not itself authenticated. A caller must obtain the expected forest root and exact required owner set from a separately verified checkpoint/frontier layer.

## 4. Verification contract

\`VerifyTemporalFrontierQueryProofV1\` requires:

- expected verifier and policy epoch;
- expected accepted-frontier digest;
- exact temporal query scope and point V;
- the exact owner set from the separately verified frontier;
- the expected forest root from the authenticated checkpoint;
- one interval proof for every owner binding, in the same canonical order.

The verifier rejects missing or substituted owners, duplicate/non-canonical manifest entries, wrong owner proof order, scope or frontier substitution, malformed cuts/gaps, invalid roots, and any failed per-owner interval proof.

It then unions matching interval records while preserving owner identity, history identity, owner sequence and tree size.

Cross-owner sequence values are never compared. A cut at owner A's sequence 100 and owner B's sequence 2 are not ordered by those magnitudes.

## 5. Receipt-to-index projection replay

`projection.go` adds a deterministic, independently recomputed projection audit over one supplied owner-local accepted-receipt inventory.

The supplied inventory has exactly `TreeSize` receipt entries, with zero-based leaf positions `0..TreeSize-1`. Every entry binds the leaf index, owner-local sequence, SHA-256 receipt digest, and a closed projection class:

- `ReceiptProjectsIntervalAssertion`: carries one exact assertion reference and validity interval;
- `ReceiptIsNonTemporal`: carries no interval fields and contributes no interval record.

The replay validates the owner/history identity and owner-sequence cut against the forest binding, requires contiguous log-leaf indices up to the separately declared log tree size, strictly increasing owner sequences within the cut, no supplied receipt inside a declared unresolved gap, well-formed receipt digests, canonical interval endpoints and duplicate assertion rejection. It never compares owner-sequence values with log tree size. It deterministically rebuilds the owner's interval tree and requires its root to equal the manifest's index root.

Two domain-separated commitments are produced:

- accepted-receipt inventory digest: commits to query scope, owner/history/cut, owner log root reference, ordered leaf indices, owner sequences, receipt digests, projection classes and interval payloads;
- projection audit digest: commits to the inventory digest, derived interval root, owner log root reference, counts and explicit unresolved gaps.

The projection audit digest can be placed in the owner binding and therefore in the frontier forest root. Fixed known-answer vectors cover the inventory digest, derived interval root and audit digest.

### Claim ceiling

The positive replay disposition is only `ProjectionMatchesSuppliedAcceptedInventory`. The result also explicitly returns `ReceiptInventoryAuthenticationNotEvaluated`.

This implementation does **not** authenticate receipt bytes or signatures, verify RFC 9162 inclusion/consistency paths, prove that the supplied inventory is the complete accepted owner-log prefix, authenticate the checkpoint, or establish that the frontier's owner set is complete. A caller that omits a receipt and supplies an incomplete inventory may still create a different internally consistent root; the separate log/checkpoint adapter must prove the supplied inventory exactly corresponds to the committed accepted prefix.

The forest query verifier therefore still returns `ProjectionCompletenessNotEvaluated`. A future `CompleteForAcceptedFrontier` result requires both this deterministic projection replay and an independently qualified log/frontier verifier that authenticates inventory coverage and owner completeness. The two obligations must not be collapsed.

## 5. Deliberately limited result

The only positive disposition emitted here is:

\`CompleteForDeclaredIndexForest\`

This means every result relative to the exact set of supplied owner-index roots has been proved by its per-owner interval proof.

The result always reports:

\`ProjectionCompletenessNotEvaluated\`

The module does not prove that every accepted owner receipt was parsed and projected into the index. It binds a projection-audit digest but does not validate that receipt or its issuer. It also preserves \`HasOwnerGaps\`, rather than silently interpreting missing sequences as no matching assertion.

A future \`CompleteForAcceptedFrontier\` disposition requires separate evidence that:

1. every owner in the verifier's accepted frontier is represented;
2. every owner cut and log root is authenticated and non-equivocating;
3. all gaps, forks, parent-frontier links, dependencies and correction branches are resolved under the query profile;
4. an independent receipt-to-index replay proves that every in-scope accepted receipt was projected into that owner's temporal index;
5. the query index root and the owner receipt-log root are bound in a signed, current checkpoint.

A self-asserted \`complete=true\`, a valid Merkle root or a digest-shaped audit reference is not sufficient.

## 6. SOTA boundary

The forest is not a new Merkle structure claim. VeriTS (Yao et al., 28 August 2026 preprint) already presents an authenticated aggregate interval tree over per-source ordered time-series streams, minimum-covering-set completeness, boundary records, and delta proofs for sliding windows. It assumes a trusted blockchain layer with a total order and does not target verifier-relative partial-order owner frontiers or per-owner receipt projection completeness.

The proposed differentiator is the composition of:
- valid-time interval semantics and retrospective correction lineage;
- exact verifier/policy accepted frontier;
- owner-local cuts and explicit causal concurrency;
- per-owner authenticated temporal indexes;
- independently checked receipt-to-index projection completeness;
- replayable query certificates that preserve uncertainty and provenance.

The temporal index must still be benchmarked against VeriTS-style AIT/MCS proofs, authenticated Merkle B+ trees, conventional interval indexes, and applicable 2D authenticated range structures. Do not claim that a custom interval tree alone beats SOTA.

## 7. Qualification status and nonclaims

The Go module is a structural reference implementation and adversarial test harness. Projection replay now detects omitted or altered interval records relative to a supplied accepted receipt inventory, but it does not authenticate that inventory. The current prototype does not implement:
- signed checkpoint or key lifecycle verification;
- RFC 9162 receipt proof verification inside this module;
- witness quorum or split-view detection;
- owner-history sequence/log-size reconciliation;
- projection-audit signature or source-inventory authentication / accepted-prefix completeness;
- formal model checking;
- a canonical wire decoder or interoperable VDS/VDP profile;
- durable persistence or mutation authority.

No result establishes global absence, event truth, trusted time, legal effect, payment authority or journal/external-effect permission.

## 8. Standards references

- [RFC 9162 — Certificate Transparency Version 2.0](https://www.rfc-editor.org/rfc/rfc9162)
- [RFC 9942 — CBOR Object Signing and Encryption (COSE) Receipts](https://www.rfc-editor.org/rfc/rfc9942)
- [VeriTS: Verifiable Model-Enhanced Time-Series Queries on Blockchain Systems (arXiv, 28 August 2026)](https://arxiv.org/abs/2608.28318)
- [COSE Receipts for MMRs, draft-ietf-scitt-receipts-mmr-profile-00 (21 November 2026)](https://datatracker.ietf.org/doc/draft-ietf-scitt-receipts-mmr-profile/)

The MMR profile is an Internet-Draft, not an RFC. It concerns append-only log receipts, not temporal-index completeness or causal frontier composition.
