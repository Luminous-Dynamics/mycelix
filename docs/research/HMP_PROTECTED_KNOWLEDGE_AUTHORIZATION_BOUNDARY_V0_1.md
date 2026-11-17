# HMP-001 — Mycelix Protected Knowledge and Authorization Boundary

**Status:** Architecture/security RFC; no protected-content runtime is implemented by this document  
**Date:** 2026-11-14  
**Applies to:** `mycelix-knowledge/zomes/claims`

> A permission label does not make published plaintext private. The system must protect the bytes and every path that can disclose or transform them.

## 1. Audit finding: the legacy claims zome is a public-data path

The current public claims integrity zome declares `Claim`, `ClassifiedClaim`, `Evidence` and related records as ordinary entry types without a `visibility = "private"` annotation. Holochain's documented default for an entry type is public; public entries are shared in the DHT. The coordinator's `submit_claim` writes `EntryTypes::Claim` through `create_entry`, and the zome exposes read paths including `get_claim`, author/tag retrieval, paginated retrieval and `search_claims`.

Consequences:

1. The legacy claim body (`content`) and public evidence body (`content`) must be treated as public once published.
2. Adding HMP access metadata or calling the pure `hmp_rights::authorize` kernel does not protect existing public bytes.
3. Removing an index link can reduce discoverability but is not confidentiality or erasure of already replicated plaintext.
4. A revocation can govern future system-mediated uses; it cannot recall copies already obtained by peers, exports, screenshots, model training or caches.
5. Existing public records must never be relabelled as restricted in a way that promises retroactive confidentiality.

This is a design boundary, not a claim that the entire Mycelix application is public. It applies to these current entry types and their current shared-data paths.

## 2. Holochain constraints that shape the design

Holochain's DHT documentation distinguishes public entries shared with peers from private entries kept on the author's source chain; source-chain actions themselves remain public. A private entry is therefore not a group-shared vault. A multi-party system still needs explicit cryptographic key distribution if several authorized participants must read protected content. [DHT overview](https://developer.holochain.org/concepts/4_dht/) · [Entry visibility](https://developer.holochain.org/build/entries/)

Integrity validation is deterministic and peer-reproducible. Holochain documents that validation cannot rely on changing link collections, current time or other non-deterministic state. Therefore the integrity callback must not be treated as a place to query a mutable revocation-link set and declare that no revocation exists. [Validation](https://developer.holochain.org/build/validation/) · [Callbacks and lifecycle hooks](https://developer.holochain.org/build/callbacks-and-lifecycle-hooks/)

This implies a clean separation:

- **Integrity layer:** validates immutable entry structure, author/signature binding and deterministic references.
- **Coordinator/read policy:** checks the requester's authenticated identity, purpose, operation and a fresh authority-signed permission/status proof.
- **Cryptographic data plane:** releases only a recipient-bound wrapped decryption key after authorization; plaintext must not be present in a public index or general DHT entry.
- **Community governance:** determines who may act as an authority, how authority changes, the decision process, appeal and benefit-sharing. A trust/reputation score or DHT consensus cannot substitute for that authority.

## 3. Protected-record architecture

Do not retrofit confidentiality onto the legacy `Claim` entry. Introduce a separately versioned protected-record format and a migration path under an explicit DNA/version decision.

### 3.1 Public descriptor — safe by construction

A public descriptor may contain only fields explicitly approved for public disclosure, for example:

- opaque record identifier;
- format/version identifier;
- salted content commitment or ciphertext digest;
- encryption/key-envelope identifier;
- coarse, reviewed discovery metadata;
- authority-key identifier and policy version;
- safe provenance pointers and correction state.

Do not put plaintext, plaintext excerpts, sensitive titles, protected tags, locations, participant identities, translation notes or embeddings into public fields by default. A hash is not anonymization: predictable/low-entropy content can be guessed, and metadata can identify a person or group.

### 3.2 Protected payload

Store protected content as authenticated ciphertext. The encryption scheme, key rotation and storage medium require a separate cryptographic design and security review. If ciphertext is replicated through a public DHT, all peers may obtain the ciphertext; confidentiality depends on strong content encryption and the handling of wrapped keys. Encryption must authenticate the payload and bind it to the record ID, version and algorithm suite to prevent substitution.

Never place a plaintext content-encryption key in the DHT, source-chain action, link tag, application log, telemetry, search index or experiment receipt.

### 3.3 Scoped capability / grant

A grant must be signed by an authority key that is valid under the record's community governance policy. Bind at minimum:

- record ID and immutable payload version/digest;
- grantee's cryptographic public key (not a caller-supplied DID string);
- allowed purpose and operation set;
- denied operation set (denial wins; contradictory policy is indeterminate);
- authority-key ID and governance/policy version;
- grant ID, monotonic authority epoch, issue time and expiry;
- recipient-bound wrapped-key reference;
- challenge/nonce or request binding where replay resistance is needed.

Operations must remain distinct: `cite`, `read`, `display`, `retrieve`, `index`, `summarize`, `translate`, `adapt`, `train`, `redistribute`, `export` and `preserve` are not interchangeable.

### 3.4 Fresh status and revocation

A signed grant proves that a grant was issued; it does not prove that the grant is still current. The reader needs a fresh, signed status proof bound to the grant ID and current authority epoch. A revocation or authority-key rotation advances the epoch.

Because a disconnected or stale reader cannot know that a newer revocation exists, the design must state its consistency guarantee rather than promising instantaneous revocation. For sensitive records, use short-lived status leases and require online refresh before releasing a wrapped key. If the authority/status service is unavailable, the proof is stale, the epoch is unknown, or the revocation state is indeterminate, release no key. The maximum stale-authorization window is bounded by the lease lifetime; previously decrypted copies cannot be recalled.

### 3.5 No plaintext side doors

The policy must cover the entire data flow, not just `get_claim`:

- read-by-ID, author/tag queries, pagination, full-text search and API handlers;
- indexing, embeddings, summarization, translation and training ingestion;
- caches, browser persistence, exports, backups and observability logs;
- cross-hApp bridges, agents, tools and delegated workers;
- corrections, withdrawal, key rotation, authority disputes and data-retention workflows.

Do not index protected plaintext. Index only explicitly approved metadata. Delegated agents must receive a fresh grant bound to their own recipient key and operation; another agent's permission must not transfer.

## 4. Explicit legacy migration rule

| Record class | Treatment |
|---|---|
| Existing public `Claim` / `ClassifiedClaim` / `Evidence` | Mark operationally as `legacy_public`; no retroactive privacy promise |
| New intentionally public knowledge | Keep the public record path, with explicit disclosure and provenance |
| New community-controlled or restricted content | Protected-record path only; encrypted payload and scoped key release |
| Metadata-only reference | Publish only reviewed metadata; no implication that underlying content may be read |
| Unknown or disputed authority | No protected-key release; retain only safe metadata where allowed |

Migration must not silently copy legacy plaintext into a supposedly protected record and then claim the old copies are gone. A migration can create a new encrypted copy for future use, but must separately document whether any public copy remains and whether the custodian is authorized to create the new copy.

## 5. Required security tests

The first end-to-end implementation should not be called complete until it tests at least:

- unauthenticated caller, caller/key mismatch and forged DID attribution;
- grant for a different record, version, digest, recipient or purpose;
- operation expansion (citation-only used for display, retrieval, translation, training or export);
- pending, denied, withdrawn, disputed or missing authority;
- stale status lease, unknown epoch, revocation and key rotation;
- expiry boundaries, replayed nonce and duplicate grant;
- signature mutation, malformed key envelope and ciphertext substitution;
- search/index/embedding leakage and every alternate read endpoint;
- stale cache, export path, delegated worker and cross-hApp path;
- coordinator outage and authority-service outage.

Property tests and adversarial integration tests should assert **zero protected key or plaintext release** for all denied/indeterminate cases. Measure authorized success separately so a system that denies everything cannot appear successful.

## 6. Benchmark and operational targets

These are proposed acceptance targets for the future protected-data release, not measured results:

| Metric | Initial target |
|---|---|
| Unauthorized protected-key/plaintext release | 0 in the full negative-control suite |
| Fail-closed behavior | 100% for missing, disputed, stale or unknown authorization state |
| Operation-scope enforcement | 100% of negative controls; no permission broadening |
| Cross-record / cross-recipient leakage | 0 |
| Revocation exposure | Explicitly bounded by a short, documented lease; target no more than 5 minutes for high-sensitivity online release, subject to threat-model review |
| Authorized operation success | Report separately by operation and availability condition |
| Audit completeness | Every release has a verifiable receipt: record/version, recipient, purpose, operation, grant, authority epoch, proof freshness and decision |
| Performance | p50/p95 authorization latency and key-release latency measured under normal, stale and partitioned-network conditions |

The five-minute target is a proposed risk budget, not a guarantee and not a replacement for key expiry or operational testing. Decryption keys and plaintext that have already escaped the controlled process cannot be revoked.

## 7. Integration sequence

1. **Keep the current HMP pure evaluator explicitly non-enforcing.** Do not wire a boolean permission check into only one endpoint.
2. Define the versioned protected-record, signed grant, authority epoch, status lease and key-envelope formats.
3. Add integrity validation for immutable schema, cryptographic signature and author/key binding; test deterministic validation behavior.
4. Add coordinator capability resolution and status-proof freshness checks; unknown state denies key release.
5. Add one read path for protected content behind the gate; do not expose a second plaintext endpoint.
6. Audit every query/index/export/training/bridge path before enabling protected records.
7. Add cryptographic key wrapping and rotation only after algorithm and implementation review.
8. Run threat-model-driven adversarial tests, independent review and a migration rehearsal before production.
9. Keep legacy-public and protected records visibly distinct.

## 8. Current implementation status

| Component | Status |
|---|---|
| HMP pure authorization kernel | Present on the HMP research branch; 18 tests authored, not executed here |
| Existing claims and evidence paths | Current public-entry model; read/search paths are not HMP-gated |
| Signed authority/grant entry types | Not implemented |
| Cryptographic signature validation for HMP grants | Not implemented |
| Fresh status lease / revocation epoch | Not implemented |
| Encrypted protected payload and recipient key wrapping | Not implemented |
| Protected read endpoint and full endpoint audit | Not implemented |
| Adversarial integration tests | Not implemented |
| Independent security review | Not performed |

**Release rule:** do not describe HMP rights enforcement as implemented until the cryptographic data plane and every relevant read/transform/export path enforce the same policy. A coordinator decision alone is not confidentiality.

## References

- Holochain, [The DHT: A Shared, Distributed Graph Database](https://developer.holochain.org/concepts/4_dht/).
- Holochain, [Entries and entry visibility](https://developer.holochain.org/build/entries/).
- Holochain, [Validation](https://developer.holochain.org/build/validation/).
- Holochain, [Callbacks and lifecycle hooks](https://developer.holochain.org/build/callbacks-and-lifecycle-hooks/).
- Mycelix current public branch: `mycelix-knowledge/zomes/claims/integrity/src/lib.rs` and `mycelix-knowledge/zomes/claims/coordinator/src/lib.rs`.
